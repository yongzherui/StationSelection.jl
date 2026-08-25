"""Run one independent Study 1 LP/IP-gap job from a `jobs.tsv` row."""

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

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

problem, k = benchmark_problem(@__DIR__, "STUDY1", n_stations, n_pairs, n_scenarios, seed)
output_dir = benchmark_output_dir(@__DIR__, "STUDY1", "study1_formulation_lp_ip_gap")
common = (
    route_regularization_weight=10.0, walk_cost_weight=0.1,
    repositioning_time=20.0, max_wait_time=max_wait_time,
    detour_factor=detour_factor, max_stops=max_stops,
)
formulation = formulation_name == "base" ?
    AggregateODRouteBaseFormulation(; common...) :
    AggregateODRouteJointRoutingAssignmentFormulation(; common..., pricing_mode=:exact)
solver = benchmark_cg_solver(pricing_time_limit_sec; recover_integer_solution=true)

result = run_opt(problem, formulation, solver)
metadata = result.metadata
has_lp = haskey(metadata, "cg_lp_objective_value")
z_lp = has_lp ? Float64(metadata["cg_lp_objective_value"]) : missing
z_ip = something(result.objective_value, missing)
gap = ismissing(z_lp) || ismissing(z_ip) || abs(z_ip) <= 1e-12 ? missing :
    (z_ip - z_lp) / abs(z_ip)
columns_key = formulation_name == "base" ? :aggregate_od_route_base_theta :
    :joint_routing_assignment_columns
metrics = benchmark_cg_metrics(result, columns_key)

row = DataFrame((
    job_id=[job_id], instance_id=[instance_id], comparison=[comparison], variant=[variant],
    formulation=[formulation_name], n_stations=[n_stations], n_pairs=[n_pairs],
    n_scenarios=[n_scenarios], seed=[seed], k=[k], max_stops=[max_stops],
    max_wait_time=[max_wait_time], detour_factor=[detour_factor],
    pricing_time_limit_sec=[pricing_time_limit_sec],
    lp_termination_status=[has_lp ? "OPTIMAL" : "NOT_RECORDED"],
    ip_termination_status=[string(result.termination_status)], z_lp=[z_lp], z_ip=[z_ip],
    gap=[gap], runtime_sec=[metrics.runtime_sec], cg_iterations=[metrics.cg_iterations],
    cg_converged=[metrics.cg_converged], cg_pricing_exhausted=[metrics.cg_pricing_exhausted],
    n_columns=[metrics.n_columns], seed_columns_added=[metrics.seed_columns_added],
))

outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0'))_$(comparison)_$(variant).csv")
CSV.write(outfile, row)
println("Wrote $outfile")
