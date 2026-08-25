"""Run one Study 3 `(instance, compensated_dominance)` job.

`runtime_sec` is `run_opt(...).runtime_sec`. With `recover_integer_solution=true` it
covers the CG loop *and* the integer-recovery solve, so it is not comparable to a run
made without recovery.

`z_lp`/`z_ip`/`gap` come from `benchmark_lp_ip` -- see its docstring for why the two
dominance arms are required to agree on `z_lp` but may legitimately differ on `z_ip`.
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
compensated_dominance = parse(Bool, fields[3])
n_stations = parse(Int, fields[4])
n_pairs = parse(Int, fields[5])
n_scenarios = parse(Int, fields[6])
seed = parse(Int, fields[7])
max_stops = parse(Int, fields[8])
pricing_time_limit_sec = parse(Float64, fields[9])

problem, k, instance_meta = benchmark_problem(@__DIR__, "STUDY3", n_stations, n_pairs, n_scenarios, seed)
output_dir = benchmark_output_dir(@__DIR__, "STUDY3", "study3_dominance_ablation")
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., pricing_mode=:exact, max_stops=max_stops,
    compensated_dominance=compensated_dominance,
)
solver = benchmark_cg_solver(pricing_time_limit_sec; recover_integer_solution=true)
result = run_opt(problem, formulation, solver)
metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)
bounds = benchmark_lp_ip(result)
iters = benchmark_iteration_metrics(result)

row = DataFrame((
    job_id=[job_id], instance_id=[instance_id],
    compensated_dominance=[compensated_dominance], n_stations=[n_stations],
    n_pairs=[n_pairs], n_scenarios=[n_scenarios], seed=[seed], k=[k],
    n_pairs_actual=[sum(instance_meta.pairs_per_scenario)],
    pairs_per_scenario=[join(instance_meta.pairs_per_scenario, ";")],
    max_stops=[max_stops], pricing_time_limit_sec=[pricing_time_limit_sec],
    lp_termination_status=[bounds.lp_termination_status],
    termination_status=[string(result.termination_status)],
    z_lp=[bounds.z_lp], z_ip=[bounds.z_ip], gap=[bounds.gap],
    runtime_sec=[metrics.runtime_sec], cg_iterations=[metrics.cg_iterations],
    cg_converged=[metrics.cg_converged], cg_pricing_exhausted=[metrics.cg_pricing_exhausted],
    n_columns=[metrics.n_columns], seed_columns_added=[metrics.seed_columns_added],
    pricing_searches=[metrics.pricing_searches], labels_generated=[metrics.labels_generated],
    labels_rejected_by_dominance=[metrics.labels_rejected_by_dominance],
    labels_removed_by_dominance=[metrics.labels_removed_by_dominance],
    max_frontier_size=[metrics.max_frontier_size], max_live_labels=[metrics.max_live_labels],
    lp_loop_sec=[iters.lp_loop_sec], integer_recovery_sec=[iters.integer_recovery_sec],
    n_logged_iterations=[iters.n_logged_iterations],
    total_columns_added=[iters.total_columns_added],
    mean_columns_per_iteration=[iters.mean_columns_per_iteration],
    median_columns_per_iteration=[iters.median_columns_per_iteration],
    max_columns_in_iteration=[iters.max_columns_in_iteration],
    mean_iteration_sec=[iters.mean_iteration_sec],
    median_iteration_sec=[iters.median_iteration_sec],
    max_iteration_sec=[iters.max_iteration_sec],
    total_master_sec=[iters.total_master_sec], total_pricing_sec=[iters.total_pricing_sec],
    total_add_columns_sec=[iters.total_add_columns_sec],
    pricing_share_of_loop=[iters.pricing_share_of_loop],
))

mode = compensated_dominance ? "compensated" : "plain"
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0'))_$(mode).csv")
CSV.write(outfile, row)
println("Wrote $outfile")

# Long-format per-iteration log: one row per CG iteration, carrying the same identity
# keys as the summary row above so every job's file concatenates into one frame.
iteration_rows = benchmark_iteration_rows(result, (
    job_id=job_id, instance_id=instance_id, compensated_dominance=compensated_dominance,
    n_stations=n_stations, n_pairs=n_pairs, n_scenarios=n_scenarios, seed=seed,
    k=k, max_stops=max_stops,
))
iterations_dir = joinpath(output_dir, "iterations")
mkpath(iterations_dir)
iterations_file = joinpath(iterations_dir, "job_$(lpad(job_id, 4, '0'))_$(mode)_iterations.csv")
CSV.write(iterations_file, DataFrame(iteration_rows))
println("Wrote $iterations_file ($(length(iteration_rows)) iterations)")
