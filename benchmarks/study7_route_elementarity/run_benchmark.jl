"""Run one Study 7 job: certify a joint routing+assignment CG optimum at n=20, then write
the selected route columns out.

Unlike Studies 1-6, the artefact here is not the metrics row -- it is
`routes/job_NNNN/variable_exports/`, produced by `export_variables`. Every previous study
discarded the solution entirely: `result.solution` is `nothing` for every formulation
(no `extract_solution` method exists), the routes lived only in `result.model`'s
`joint_routing_assignment_{theta,columns}` dicts, and nothing serialized them, so
"which routes did the optimum pick" was unanswerable after the process exited.

The metrics row is still written, in a subset of Study 5's schema, so certification status
is recorded next to the exported routes: an uncertified cell's columns are an upper bound's
columns, not an optimum's, and the analysis has to be able to drop them.
"""

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 11 || error("expected 11 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
cell_id = fields[2]
n_stations, n_pairs, n_scenarios, seed, max_stops, n_threads = parse.(Int, fields[3:8])
time_limit_sec = parse(Float64, fields[9])             # per-ROUND regular pricing budget
certifying_time_limit_sec = parse(Float64, fields[10]) # per-ROUND certifying budget
total_time_limit_sec = parse(Float64, fields[11])      # strict wall cap on the CG loop

# Scenario pricing runs concurrently, so too few threads silently turns this into a slower
# serial run -- which here costs certifications, i.e. the very cells the study needs.
if Threads.nthreads() < n_threads
    error("job asks for $n_threads threads but Julia has $(Threads.nthreads()); submit " *
          "with --cpus-per-task=$n_threads (submit_benchmark.sh derives JULIA_NUM_THREADS " *
          "from SLURM_CPUS_PER_TASK)")
end

problem, k, instance_meta = benchmark_problem(
    @__DIR__, "STUDY7", n_stations, n_pairs, n_scenarios, seed,
)
output_dir = benchmark_output_dir(@__DIR__, "STUDY7", "study7_route_elementarity")
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops, pricing_mode=:exact,
)
# recover_integer_solution=true is load-bearing, not a default carried over from Study 5:
# without it `result.model` is the LP master and every exported theta is fractional, so
# there is no "selected route" to ask about elementarity of.
solver = benchmark_cg_solver(
    time_limit_sec; recover_integer_solution=true, threads=1,
    certifying_pricing_time_limit_sec=certifying_time_limit_sec,
    total_time_limit_sec=total_time_limit_sec,
    parallel_scenario_pricing=true,
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
routes_exported = false
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

# The study artefact. Its own subdirectory per job: `export_variables` always writes into
# `<dir>/variable_exports/`, so every job sharing one output_dir would overwrite the same
# files. Gated on an incumbent existing -- SOLVE_NOT_SOLVED leaves `JuMP.value` throwing,
# and SOLVE_FEASIBLE is exported deliberately (uncertified, but the metrics row says so).
if result !== nothing &&
        result.termination_status in (StationSelection.SOLVE_OPTIMAL, StationSelection.SOLVE_FEASIBLE)
    routes_dir = joinpath(output_dir, "routes", "job_$(lpad(job_id, 4, '0'))")
    mkpath(routes_dir)
    try
        export_variables(result, routes_dir)
        routes_exported = true
    catch err
        error_message *= " | export_variables failed: " *
            replace(sprint(showerror, err), '\n' => ' ')
    end
end

row = DataFrame((
    job_id=[job_id], cell_id=[cell_id], method=["cg_exact"],
    n_threads=[n_threads], julia_threads=[Threads.nthreads()],
    n_stations=[n_stations], n_pairs=[n_pairs], n_scenarios=[n_scenarios], seed=[seed],
    n_pairs_actual=[sum(instance_meta.pairs_per_scenario)],
    k=[k], max_stops=[max_stops], time_limit_sec=[time_limit_sec],
    certifying_time_limit_sec=[certifying_time_limit_sec],
    total_time_limit_sec=[total_time_limit_sec],
    status=[status], termination_status=[termination_status],
    objective_value=[objective_value], wall_sec=[wall_sec], n_columns=[n_columns],
    cg_iterations=[cg_iterations], cg_converged=[cg_converged],
    cg_pricing_exhausted=[cg_pricing_exhausted], cg_stop_reason=[cg_stop_reason],
    cg_total_budget_exhausted=[cg_total_budget_exhausted],
    routes_exported=[routes_exported], error_message=[error_message],
))
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0')).csv")
CSV.write(outfile, row)
println("Wrote $outfile (status=$status, routes_exported=$routes_exported, " *
        "wall_sec=$(round(wall_sec; digits=2)))")
end
