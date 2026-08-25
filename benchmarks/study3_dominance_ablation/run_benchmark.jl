"""Run one Study 3 `(instance, compensated_dominance)` job."""

using StationSelection
using CSV
using DataFrames

include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 9 || error("expected 9 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
instance_id = fields[2]
compensated_dominance = parse(Bool, fields[3])
n_stations = parse(Int, fields[4])
n_pairs = parse(Int, fields[5])
n_scenarios = parse(Int, fields[6])
seed = parse(Int, fields[7])
max_stops = parse(Int, fields[8])
pricing_time_limit_sec = parse(Float64, fields[9])

problem, k = benchmark_problem(@__DIR__, "STUDY3", n_stations, n_pairs, n_scenarios, seed)
output_dir = benchmark_output_dir(@__DIR__, "STUDY3", "study3_dominance_ablation")
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., pricing_mode=:exact, max_stops=max_stops,
    compensated_dominance=compensated_dominance,
)
result = run_opt(problem, formulation, benchmark_cg_solver(pricing_time_limit_sec))
metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)

row = DataFrame((
    job_id=[job_id], instance_id=[instance_id],
    compensated_dominance=[compensated_dominance], n_stations=[n_stations],
    n_pairs=[n_pairs], n_scenarios=[n_scenarios], seed=[seed], k=[k],
    max_stops=[max_stops], pricing_time_limit_sec=[pricing_time_limit_sec],
    termination_status=[string(result.termination_status)],
    objective_value=[something(result.objective_value, missing)],
    runtime_sec=[metrics.runtime_sec], cg_iterations=[metrics.cg_iterations],
    cg_converged=[metrics.cg_converged], cg_pricing_exhausted=[metrics.cg_pricing_exhausted],
    n_columns=[metrics.n_columns], seed_columns_added=[metrics.seed_columns_added],
    pricing_searches=[metrics.pricing_searches], labels_generated=[metrics.labels_generated],
    labels_rejected_by_dominance=[metrics.labels_rejected_by_dominance],
    labels_removed_by_dominance=[metrics.labels_removed_by_dominance],
    max_frontier_size=[metrics.max_frontier_size], max_live_labels=[metrics.max_live_labels],
))

mode = compensated_dominance ? "compensated" : "plain"
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0'))_$(mode).csv")
CSV.write(outfile, row)
println("Wrote $outfile")
