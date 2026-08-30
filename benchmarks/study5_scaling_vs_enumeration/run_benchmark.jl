"""Run one exact-CG Study 5 scaling job, on either the serial or the parallel arm.

The arms differ only in `CGSolver(parallel_scenario_pricing=...)`. Gurobi is pinned to one
thread in both, so scenario pricing is the only parallelism and the difference isolates it.
`pricing_time_limit_sec` is the same per-ROUND wall budget on both arms; serial splits it
across scenarios while parallel gives each scenario the whole round, so the parallel arm
fits up to `n_scenarios` x more search into the same round wall.

`scenario_search_sec_*` summarise the per-scenario label-search wall times recorded in
`metadata["cg_pricing_stats"]`. Their sum-versus-max is the direct evidence for the
serial/parallel gap: a serial round costs about the sum, a parallel round about the max.
"""

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 16 || error("expected 16 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
cell_id, substudy = fields[2:3]
axis_value = parse(Int, fields[4])
arm = fields[5]
arm in ("serial", "parallel") || error("unknown arm $(repr(arm)); expected serial|parallel")
fields[6] == "cg_exact" || error("Study 5 only supports method=cg_exact")
n_stations, n_pairs, n_scenarios, seed, max_stops, n_threads = parse.(Int, fields[7:12])
time_limit_sec = parse(Float64, fields[13])            # per-ROUND regular pricing budget
certifying_time_limit_sec = parse(Float64, fields[14]) # per-ROUND certifying budget
total_time_limit_sec = parse(Float64, fields[15])      # strict wall cap on the CG loop
max_routes = parse(Int, fields[16]) # retained in the shared table schema; unused by CG

# The parallel arm can only actually parallelize if Julia was started with the threads the
# job table asked for. Fail loudly rather than silently reporting a "parallel" run that ran
# serially -- that would look like a null result instead of a misconfiguration.
if arm == "parallel" && Threads.nthreads() < n_threads
    error("arm=parallel needs $n_threads threads but Julia has $(Threads.nthreads()); " *
          "submit with --cpus-per-task=$n_threads (submit_benchmark.sh derives " *
          "JULIA_NUM_THREADS from SLURM_CPUS_PER_TASK)")
end

problem, k, instance_meta = benchmark_problem(
    @__DIR__, "STUDY5", n_stations, n_pairs, n_scenarios, seed,
)
output_dir = benchmark_output_dir(@__DIR__, "STUDY5", "study5_scaling_exact_cg")
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops, pricing_mode=:exact,
)
solver = benchmark_cg_solver(
    time_limit_sec; recover_integer_solution=true, threads=1,  # Gurobi: 1 thread in BOTH arms
    certifying_pricing_time_limit_sec=certifying_time_limit_sec,
    total_time_limit_sec=total_time_limit_sec,
    parallel_scenario_pricing=(arm == "parallel"),
)

let
status = "error"
termination_status = "NOT_RUN"
objective_value = missing
n_columns = missing
cg_iterations = missing
cg_converged = missing
cg_pricing_exhausted = missing
cg_stop_reason = missing
cg_total_budget_exhausted = missing
cg_certifying_rounds = missing
scenario_search_sec_sum = missing
scenario_search_sec_max = missing
scenario_search_sec_min = missing
n_scenario_searches = missing
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
    cg_stop_reason = get(result.metadata, "cg_stop_reason", missing)
    cg_total_budget_exhausted = get(result.metadata, "cg_total_budget_exhausted", missing)
    cg_certifying_rounds = get(result.metadata, "cg_certifying_rounds", missing)
    # Sum vs max over every (iteration x scenario) label search in the run. Serial wall
    # tracks the sum; parallel wall tracks the max. Recorded for both arms so the ratio is
    # measurable from either side.
    searches = [Float64(st.search_sec) for st in get(result.metadata, "cg_pricing_stats", Any[])
                if hasproperty(st, :search_sec)]
    if !isempty(searches)
        scenario_search_sec_sum = sum(searches)
        scenario_search_sec_max = maximum(searches)
        scenario_search_sec_min = minimum(searches)
        n_scenario_searches = length(searches)
    end
    # "exhausted" = dual-certified optimal. "budget_exhausted" = the 6 h cap stopped the
    # loop: the incumbent is feasible but NOT certified, and z_lp is not a valid bound.
    # "incomplete" = stopped uncertified for some other reason (iteration cap, stalled
    # pricing, non-OPTIMAL master).
    status = if metrics.cg_converged && metrics.cg_pricing_exhausted
        "exhausted"
    elseif cg_total_budget_exhausted === true
        "budget_exhausted"
    else
        "incomplete"
    end
catch err
    error_message = replace(sprint(showerror, err), '\n' => ' ')
end
wall_sec = time() - t_start

row = DataFrame((
    job_id=[job_id], cell_id=[cell_id], substudy=[substudy], axis_value=[axis_value],
    arm=[arm], method=["cg_exact"], n_threads=[n_threads],
    julia_threads=[Threads.nthreads()],
    n_stations=[n_stations], n_pairs=[n_pairs],
    n_scenarios=[n_scenarios], seed=[seed],
    n_pairs_actual=[sum(instance_meta.pairs_per_scenario)],
    pairs_per_scenario=[join(instance_meta.pairs_per_scenario, ";")],
    k=[k], max_stops=[max_stops], time_limit_sec=[time_limit_sec],
    certifying_time_limit_sec=[certifying_time_limit_sec],
    total_time_limit_sec=[total_time_limit_sec], max_routes=[max_routes],
    status=[status], termination_status=[termination_status],
    objective_value=[objective_value], wall_sec=[wall_sec], n_columns=[n_columns],
    cg_iterations=[cg_iterations], cg_converged=[cg_converged],
    cg_pricing_exhausted=[cg_pricing_exhausted], cg_stop_reason=[cg_stop_reason],
    cg_total_budget_exhausted=[cg_total_budget_exhausted],
    cg_certifying_rounds=[cg_certifying_rounds],
    scenario_search_sec_sum=[scenario_search_sec_sum],
    scenario_search_sec_max=[scenario_search_sec_max],
    scenario_search_sec_min=[scenario_search_sec_min],
    n_scenario_searches=[n_scenario_searches], error_message=[error_message],
))
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0'))_$(arm).csv")
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
    iterations_file = joinpath(iterations_dir, "job_$(lpad(job_id, 4, '0'))_$(arm)_iterations.csv")
    CSV.write(iterations_file, DataFrame(iteration_rows))
    println("Wrote $iterations_file ($(length(iteration_rows)) iterations)")
end
end
