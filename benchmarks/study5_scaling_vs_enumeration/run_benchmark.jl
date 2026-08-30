"""Run one single-threaded exact-CG Study 5 scaling job."""

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 12 || error("expected 12 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
cell_id, substudy = fields[2:3]
axis_value = parse(Int, fields[4])
fields[5] == "cg_exact" || error("Study 5 only supports method=cg_exact")
n_stations, n_pairs, n_scenarios, seed, max_stops = parse.(Int, fields[6:10])
time_limit_sec = parse(Float64, fields[11])
max_routes = parse(Int, fields[12]) # retained in the shared table schema; unused by CG

problem, k, instance_meta = benchmark_problem(
    @__DIR__, "STUDY5", n_stations, n_pairs, n_scenarios, seed,
)
output_dir = benchmark_output_dir(@__DIR__, "STUDY5", "study5_scaling_exact_cg")
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops, pricing_mode=:exact,
)
solver = benchmark_cg_solver(time_limit_sec; recover_integer_solution=true, threads=1)

let
status = "error"
termination_status = "NOT_RUN"
objective_value = missing
n_columns = missing
cg_iterations = missing
cg_converged = missing
cg_pricing_exhausted = missing
error_message = ""
result = nothing
t_start = time()
try
    result = run_opt(problem, formulation, solver)
    metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)
    termination_status = string(result.termination_status)
    objective_value = something(result.objective_value, missing)
    n_columns = metrics.n_columns
    cg_iterations = metrics.cg_iterations
    cg_converged = metrics.cg_converged
    cg_pricing_exhausted = metrics.cg_pricing_exhausted
    status = metrics.cg_converged && metrics.cg_pricing_exhausted ? "exhausted" : "incomplete"
catch err
    error_message = replace(sprint(showerror, err), '\n' => ' ')
end
wall_sec = time() - t_start

row = DataFrame((
    job_id=[job_id], cell_id=[cell_id], substudy=[substudy], axis_value=[axis_value],
    method=["cg_exact"], n_stations=[n_stations], n_pairs=[n_pairs],
    n_scenarios=[n_scenarios], seed=[seed],
    n_pairs_actual=[sum(instance_meta.pairs_per_scenario)],
    pairs_per_scenario=[join(instance_meta.pairs_per_scenario, ";")],
    k=[k], max_stops=[max_stops], time_limit_sec=[time_limit_sec], max_routes=[max_routes],
    status=[status], termination_status=[termination_status],
    objective_value=[objective_value], wall_sec=[wall_sec], n_columns=[n_columns],
    cg_iterations=[cg_iterations], cg_converged=[cg_converged],
    cg_pricing_exhausted=[cg_pricing_exhausted], error_message=[error_message],
))
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0'))_cg_exact.csv")
CSV.write(outfile, row)
println("Wrote $outfile (status=$status, wall_sec=$(round(wall_sec; digits=2)))")

# Long-format per-iteration log (master/pricing/add-columns seconds per CG
# iteration) -- absent when `result` is `nothing` (the try block errored before
# ever calling `run_opt`).
if result !== nothing
    iteration_rows = benchmark_iteration_rows(result, (
        job_id=job_id, cell_id=cell_id, substudy=substudy, axis_value=axis_value,
        n_stations=n_stations, n_pairs=n_pairs, n_scenarios=n_scenarios, seed=seed,
        k=k, max_stops=max_stops,
    ))
    iterations_dir = joinpath(output_dir, "iterations")
    mkpath(iterations_dir)
    iterations_file = joinpath(iterations_dir, "job_$(lpad(job_id, 4, '0'))_cg_exact_iterations.csv")
    CSV.write(iterations_file, DataFrame(iteration_rows))
    println("Wrote $iterations_file ($(length(iteration_rows)) iterations)")
end
end
