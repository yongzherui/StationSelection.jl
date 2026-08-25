"""Run one independent Study 1 LP/IP-gap job from a `jobs.tsv` row."""

using StationSelection
using JuMP: relax_integrality
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

gap_ratio(z_lp, z_ip) = ismissing(z_lp) || ismissing(z_ip) || abs(z_ip) <= 1e-12 ? missing :
    (z_ip - z_lp) / abs(z_ip)

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 13 || error("expected 13 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
instance_id, comparison, variant, formulation_name = fields[2:5]
formulation_name in ("base", "joint") || error("formulation must be base or joint")
n_stations = parse(Int, fields[6])
n_pairs = parse(Int, fields[7])
n_scenarios = parse(Int, fields[8])
seed = parse(Int, fields[9])
max_stops = parse(Int, fields[10])
max_wait_time = parse(Float64, fields[11])
detour_factor = parse(Float64, fields[12])
pricing_time_limit_sec = parse(Float64, fields[13])

problem, k, instance_meta = benchmark_problem(@__DIR__, "STUDY1", n_stations, n_pairs, n_scenarios, seed)
output_dir = benchmark_output_dir(@__DIR__, "STUDY1", "study1_formulation_lp_ip_gap")
common = (
    route_regularization_weight=10.0, walk_cost_weight=0.1,
    repositioning_time=20.0, max_wait_time=max_wait_time,
    detour_factor=detour_factor, max_stops=max_stops,
)

base_fields = (
    job_id=[job_id], instance_id=[instance_id], comparison=[comparison], variant=[variant],
    formulation=[formulation_name], n_stations=[n_stations], n_pairs=[n_pairs],
    n_pairs_actual=[sum(instance_meta.pairs_per_scenario)],
    pairs_per_scenario=[join(instance_meta.pairs_per_scenario, ";")],
    n_scenarios=[n_scenarios], seed=[seed], k=[k], max_stops=[max_stops],
    max_wait_time=[max_wait_time], detour_factor=[detour_factor],
    pricing_time_limit_sec=[pricing_time_limit_sec],
)

if comparison == "formulation"
    # Both formulations' *native* solve strategy is exhaustive enumeration +
    # `DirectMIPSolver`, not `CGSolver` -- Base's own (`enumerate_aggregate_od_route_columns`)
    # and Joint's own (`enumerate_joint_routing_assignment_columns`,
    # `label_setting/joint_routing_assignment/exact/enumeration.jl`, which reuses Base's
    # physical-route DFS and combinatorially expands each route's *station subsets* -- see
    # that file's module docstring). Solve the exhaustive-pool master directly for the true
    # IP, then relax it in place (`JuMP.relax_integrality`) and resolve for the true LP bound
    # -- no CG duals or CG-restricted-recovery heuristic on either side, so this is a genuine
    # apples-to-apples comparison. Both `build_model` methods dispatch on `formulation`'s
    # concrete type, so the same code below works for either.
    formulation = formulation_name == "base" ?
        AggregateODRouteBaseFormulation(; common...) :
        AggregateODRouteJointRoutingAssignmentFormulation(; common...)
    solver = DirectMIPSolver(config=SolverOptions(silent=true, time_limit_sec=300.0))

    build = StationSelection.build_model(problem, formulation, solver)
    ip_result = StationSelection.optimize_model(build, solver)
    relax_integrality(build.model)
    lp_result = StationSelection.optimize_model(build, solver)

    z_ip = something(ip_result.objective_value, missing)
    z_lp = something(lp_result.objective_value, missing)
    n_columns_key = formulation_name == "base" ? "routes_enumerated" : "seed_columns_added"

    row = DataFrame((
        ; base_fields...,
        lp_termination_status=[string(lp_result.termination_status)],
        ip_termination_status=[string(ip_result.termination_status)], z_lp=[z_lp], z_ip=[z_ip],
        gap=[gap_ratio(z_lp, z_ip)], runtime_sec=[ip_result.runtime_sec + lp_result.runtime_sec],
        cg_iterations=[0],
        # No CG loop ran -- both enumerators throw rather than return a truncated pool (see
        # their own docstrings), so an OPTIMAL termination on both solves above already
        # certifies exactness; these two flags exist for `analyze.jl`'s certification gate,
        # which is solver-agnostic about *how* that exactness was reached.
        cg_converged=[true], cg_pricing_exhausted=[true],
        n_columns=[build.counts.extras[n_columns_key]], seed_columns_added=[0],
    ))
else
    formulation = AggregateODRouteJointRoutingAssignmentFormulation(; common..., pricing_mode=:exact)
    solver = benchmark_cg_solver(pricing_time_limit_sec; recover_integer_solution=true)

    result = run_opt(problem, formulation, solver)
    metadata = result.metadata
    has_lp = haskey(metadata, "cg_lp_objective_value")
    z_lp = has_lp ? Float64(metadata["cg_lp_objective_value"]) : missing
    z_ip = something(result.objective_value, missing)
    metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)

    row = DataFrame((
        ; base_fields...,
        lp_termination_status=[has_lp ? "OPTIMAL" : "NOT_RECORDED"],
        ip_termination_status=[string(result.termination_status)], z_lp=[z_lp], z_ip=[z_ip],
        gap=[gap_ratio(z_lp, z_ip)], runtime_sec=[metrics.runtime_sec],
        cg_iterations=[metrics.cg_iterations],
        cg_converged=[metrics.cg_converged], cg_pricing_exhausted=[metrics.cg_pricing_exhausted],
        n_columns=[metrics.n_columns], seed_columns_added=[metrics.seed_columns_added],
    ))
end

outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0'))_$(comparison)_$(variant).csv")
CSV.write(outfile, row)
println("Wrote $outfile")
