"""Shared setup and result helpers for end-to-end aggregate-OD-route CG benchmarks."""

using StationSelection
using Dates
using Statistics

include(joinpath(@__DIR__, "..", "..", "scripts", "generate_zhuzhou_instance.jl"))

const BENCHMARK_BASELINE = (
    route_regularization_weight=10.0,
    walk_cost_weight=0.1,
    repositioning_time=20.0,
    max_wait_time=900.0,
    detour_factor=2.0,
    max_stops=10,
)

function benchmark_problem(study_dir::AbstractString, env_prefix::AbstractString,
        n_stations::Int, n_pairs::Int, n_scenarios::Int, seed::Int)
    project_root = normpath(joinpath(study_dir, "..", ".."))
    data_dir = get(ENV, "$(env_prefix)_DATA_DIR",
        normpath(joinpath(project_root, "..", "Data", "base_data")))
    data, meta = generate_zhuzhou_data(data_dir, n_stations, n_pairs;
        n_scenarios=n_scenarios, seed=seed)
    meta.n_stations_actual == n_stations || error(
        "benchmark instance has $(meta.n_stations_actual) stations, expected $n_stations"
    )
    meta.n_scenarios_actual == n_scenarios || error(
        "benchmark instance has $(meta.n_scenarios_actual) scenarios, expected $n_scenarios"
    )
    meta.pairs_per_scenario == fill(n_pairs, n_scenarios) || error(
        "benchmark instance has pair counts $(meta.pairs_per_scenario), expected " *
        "$(fill(n_pairs, n_scenarios))"
    )
    k = max(2, ceil(Int, n_stations / 2))
    return StationSelectionProblem(data, k; max_walking_distance=600.0), k, meta
end

function benchmark_output_dir(study_dir::AbstractString, env_prefix::AbstractString,
        study_slug::AbstractString)
    project_root = normpath(joinpath(study_dir, "..", ".."))
    benchmarks_root = joinpath(project_root, "benchmarks")
    default_output = joinpath(benchmarks_root, "experiments", "$(Dates.today())_$(study_slug)")
    output_dir = get(ENV, "$(env_prefix)_OUTPUT_DIR", default_output)
    mkpath(output_dir)
    return output_dir
end

"""
    benchmark_cg_solver(pricing_time_limit_sec; kwargs...) -> CGSolver

`pricing_time_limit_sec` is the *regular* pricing budget for one whole round, divided
equally across scenarios. `parallel_scenario_pricing` prices those scenarios concurrently
(needs `Threads.nthreads() > 1`); `threads` is Gurobi's own limit and is independent of it.
`certifying_pricing_time_limit_sec` (default `3600.0`) is the longer budget the loop
escalates to when a regular round returns no columns without exhausting -- only that
round can certify. `total_time_limit_sec` (default `Inf`) is a strict wall cap on the CG
loop: on expiry the run stops and reports `cg_stop_reason="total_budget"` with
`cg_converged=false`, so a censored job still writes a row instead of being killed by the
scheduler. The recovery MIP afterwards is bounded separately by `config.time_limit_sec`
(300 s), so budget a job's walltime for `total_time_limit_sec + 300 s` plus start-up.
"""
function benchmark_cg_solver(pricing_time_limit_sec::Real; recover_integer_solution::Bool=false,
        threads::Union{Nothing, Int}=nothing,
        certifying_pricing_time_limit_sec::Real=3600.0,
        total_time_limit_sec::Real=Inf,
        parallel_scenario_pricing::Bool=false)
    return CGSolver(
        config=SolverOptions(silent=true, time_limit_sec=300.0, threads=threads), max_iterations=1_000,
        reduced_cost_tol=1e-6, pricing_time_limit_sec=pricing_time_limit_sec,
        certifying_pricing_time_limit_sec=certifying_pricing_time_limit_sec,
        total_time_limit_sec=total_time_limit_sec,
        parallel_scenario_pricing=parallel_scenario_pricing,
        recover_integer_solution=recover_integer_solution,
    )
end

"""
    gap_ratio(z_lp, z_ip) -> Float64 or missing

Relative LP/IP gap `(z_ip - z_lp) / |z_ip|`. `missing` if either bound is absent or
`z_ip` is numerically zero.
"""
gap_ratio(z_lp, z_ip) = ismissing(z_lp) || ismissing(z_ip) || abs(z_ip) <= 1e-12 ? missing :
    (z_ip - z_lp) / abs(z_ip)

"""
    benchmark_lp_ip(result) -> (; z_lp, z_ip, gap, lp_termination_status)

Split a `recover_integer_solution=true` CG result into its pre-recovery LP bound and its
post-recovery integer objective.

`z_lp` is the LP value the CG loop converged to (`"cg_lp_objective_value"`), and is a
valid lower bound on the true optimum *only when pricing actually exhausted* -- check
`cg_converged`/`cg_pricing_exhausted` before treating it as one.

`z_ip` is the restricted-master heuristic's objective (see `CGSolver`'s docstring): a
feasible solution and a valid upper bound, but optimal only over the column pool CG
happened to generate. Two arms that are both exact can therefore certify the *same*
`z_lp` and still report *different* `z_ip`, because they discovered different pools.
Compare arms for equivalence on `z_lp`; treat `z_ip` and `gap` as measured outcomes.
"""
function benchmark_lp_ip(result)
    has_lp = haskey(result.metadata, "cg_lp_objective_value")
    z_lp = has_lp ? Float64(result.metadata["cg_lp_objective_value"]) : missing
    z_ip = something(result.objective_value, missing)
    return (z_lp=z_lp, z_ip=z_ip, gap=gap_ratio(z_lp, z_ip),
        lp_termination_status=has_lp ? "OPTIMAL" : "NOT_RECORDED")
end

"""
    benchmark_iteration_metrics(result) -> NamedTuple

Scalar per-iteration summaries derived from `metadata["cg_iteration_log"]` (see
`CGSolver`'s docstring for the log's own schema): how many columns each CG iteration
contributed, how long each took, and how that wall time split across the master LP
solve, pricing, and column insertion.

`lp_loop_sec` and `integer_recovery_sec` come straight from metadata and let the CG
loop be reported separately from the recovery MIP, which `OptResult.runtime_sec`
lumps together. Returns `missing` for every average when the log is empty.
"""
function benchmark_iteration_metrics(result)
    log = get(result.metadata, "cg_iteration_log", NamedTuple[])
    lp_loop_sec = Float64(get(result.metadata, "cg_lp_loop_sec", NaN))
    recovery_sec = Float64(get(result.metadata, "cg_integer_recovery_sec", NaN))
    if isempty(log)
        return (
            n_logged_iterations=0, total_columns_added=0,
            mean_columns_per_iteration=missing, median_columns_per_iteration=missing,
            max_columns_in_iteration=missing, mean_iteration_sec=missing,
            median_iteration_sec=missing, max_iteration_sec=missing,
            total_master_sec=missing, total_pricing_sec=missing,
            total_add_columns_sec=missing, pricing_share_of_loop=missing,
            lp_loop_sec=lp_loop_sec, integer_recovery_sec=recovery_sec,
        )
    end
    added = [Int(r.columns_added) for r in log]
    per_iter = [Float64(r.master_sec + r.pricing_sec + r.add_columns_sec) for r in log]
    total_master = sum(Float64(r.master_sec) for r in log)
    total_pricing = sum(Float64(r.pricing_sec) for r in log)
    total_add = sum(Float64(r.add_columns_sec) for r in log)
    accounted = total_master + total_pricing + total_add
    return (
        n_logged_iterations=length(log), total_columns_added=sum(added),
        mean_columns_per_iteration=mean(added),
        median_columns_per_iteration=median(added),
        max_columns_in_iteration=maximum(added),
        mean_iteration_sec=mean(per_iter), median_iteration_sec=median(per_iter),
        max_iteration_sec=maximum(per_iter), total_master_sec=total_master,
        total_pricing_sec=total_pricing, total_add_columns_sec=total_add,
        pricing_share_of_loop=accounted <= 0 ? missing : total_pricing / accounted,
        lp_loop_sec=lp_loop_sec, integer_recovery_sec=recovery_sec,
    )
end

"""
    benchmark_iteration_rows(result, identity::NamedTuple) -> Vector{NamedTuple}

The per-iteration log as tidy rows, each prefixed with `identity` (job/instance/arm
keys) so a study's per-iteration CSVs concatenate into one long-format frame.
"""
benchmark_iteration_rows(result, identity::NamedTuple) = NamedTuple[
    (; identity..., row...) for row in get(result.metadata, "cg_iteration_log", NamedTuple[])
]

function benchmark_cg_metrics(result, columns_key::Symbol)
    metadata = result.metadata
    pricing_stats = get(metadata, "cg_pricing_stats", Any[])
    return (
        runtime_sec=result.runtime_sec,
        cg_iterations=Int(get(metadata, "cg_iterations", 0)),
        cg_converged=Bool(get(metadata, "cg_converged", false)),
        cg_pricing_exhausted=Bool(get(metadata, "cg_pricing_exhausted", false)),
        n_columns=length(result.model[columns_key]),
        seed_columns_added=Int(get(result.counts.extras, "seed_columns_added", 0)),
        pricing_searches=length(pricing_stats),
        labels_generated=sum((s.labels_generated for s in pricing_stats); init=0),
        labels_rejected_by_dominance=sum((s.labels_rejected_by_dominance for s in pricing_stats); init=0),
        labels_removed_by_dominance=sum((s.labels_removed_by_dominance for s in pricing_stats); init=0),
        max_frontier_size=maximum((s.max_frontier_size for s in pricing_stats); init=0),
        max_live_labels=maximum((s.max_live_labels for s in pricing_stats); init=0),
    )
end
