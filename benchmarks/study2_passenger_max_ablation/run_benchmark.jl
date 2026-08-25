"""Run one Study 2 `(instance, pricing_mode)` job from one `jobs.tsv` data row.

`runtime_sec` is copied only from `run_opt(...).runtime_sec`; it excludes data
generation and model construction and is never replaced by an outer timer.
"""

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 9 || error("expected 9 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
instance_id = fields[2]
mode_string = fields[3]
mode_string in ("exact", "darp") || error("pricing_mode must be exact or darp, got $mode_string")
n_stations = parse(Int, fields[4])
n_pairs = parse(Int, fields[5])
n_scenarios = parse(Int, fields[6])
seed = parse(Int, fields[7])
max_stops = parse(Int, fields[8])
pricing_time_limit_sec = parse(Float64, fields[9])

problem, k = benchmark_problem(@__DIR__, "STUDY2", n_stations, n_pairs, n_scenarios, seed)
output_dir = benchmark_output_dir(@__DIR__, "STUDY2", "study2_passenger_max_ablation")
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., pricing_mode=Symbol(mode_string), max_stops=max_stops,
)
solver = benchmark_cg_solver(pricing_time_limit_sec)

result = run_opt(problem, formulation, solver)
metadata = result.metadata
metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)
row = DataFrame((
    job_id=[job_id], instance_id=[instance_id], pricing_mode=[mode_string],
    n_stations=[n_stations], n_pairs=[n_pairs], n_scenarios=[n_scenarios],
    seed=[seed], k=[k], max_stops=[max_stops],
    pricing_time_limit_sec=[pricing_time_limit_sec],
    termination_status=[string(result.termination_status)],
    objective_value=[something(result.objective_value, missing)],
    runtime_sec=[metrics.runtime_sec], cg_iterations=[metrics.cg_iterations],
    cg_converged=[metrics.cg_converged], cg_pricing_exhausted=[metrics.cg_pricing_exhausted],
    n_columns=[metrics.n_columns], seed_columns_added=[metrics.seed_columns_added],
))

outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0'))_$(mode_string).csv")
CSV.write(outfile, row)
println("Wrote $outfile")
println("runtime_sec=$(result.runtime_sec) (from run_opt result)")
