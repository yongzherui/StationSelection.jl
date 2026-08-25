"""Shared setup and result helpers for end-to-end aggregate-OD-route CG benchmarks."""

using StationSelection
using Dates

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
    default_output = joinpath(project_root, "experiments", "$(Dates.today())_$(study_slug)")
    output_dir = get(ENV, "$(env_prefix)_OUTPUT_DIR", default_output)
    mkpath(output_dir)
    return output_dir
end

function benchmark_cg_solver(pricing_time_limit_sec::Real; recover_integer_solution::Bool=false)
    return CGSolver(
        config=SolverOptions(silent=true, time_limit_sec=300.0), max_iterations=1_000,
        reduced_cost_tol=1e-6, pricing_time_limit_sec=pricing_time_limit_sec,
        recover_integer_solution=recover_integer_solution,
    )
end

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
