"""Run one Study 8 job: `exact` or `warm_start`, on Study 7's grid and budgets.

The arms differ in exactly one constructor argument,
`CGSolver(warm_start_pricing_mode=...)`. Everything else -- formulation, budgets, threads,
Gurobi settings, instance -- is identical, so the wall-clock difference isolates the
elementary-first phase and nothing else.

Per-phase columns come from `cg_iteration_log`, which carries `pricing_mode` per row, so a
warm-start run's phase 1 and phase 2 can be costed separately without a second run.
`warm_start_sec`/`warm_start_iterations` come from the solver directly (recorded at the
handoff) rather than being summed from the log, since the log covers only
master/pricing/add-columns time and a sum of it understates the phase.
"""

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 12 || error("expected 12 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
cell_id, arm = fields[2:3]
arm in ("exact", "warm_start") || error("unknown arm $(repr(arm)); expected exact|warm_start")
n_stations, n_pairs, n_scenarios, seed, max_stops, n_threads = parse.(Int, fields[4:9])
time_limit_sec = parse(Float64, fields[10])
certifying_time_limit_sec = parse(Float64, fields[11])
total_time_limit_sec = parse(Float64, fields[12])

if Threads.nthreads() < n_threads
    error("job asks for $n_threads threads but Julia has $(Threads.nthreads()); submit " *
          "with --cpus-per-task=$n_threads")
end

problem, k, instance_meta = benchmark_problem(
    @__DIR__, "STUDY8", n_stations, n_pairs, n_scenarios, seed,
)
output_dir = benchmark_output_dir(@__DIR__, "STUDY8", "study8_warm_start_speedup")
# Identical on both arms: the warm start is a CGSolver setting, not a formulation one, so
# `pricing_mode` stays :exact here and the warm-start arm's phase 2 uses exactly this pricer.
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops, pricing_mode=:exact,
)
solver = benchmark_cg_solver(
    time_limit_sec; recover_integer_solution=true, threads=1,
    certifying_pricing_time_limit_sec=certifying_time_limit_sec,
    total_time_limit_sec=total_time_limit_sec,
    parallel_scenario_pricing=true,
    warm_start_pricing_mode=(arm == "warm_start" ? :station_simple : nothing),
)

let
status = "error"
termination_status = "NOT_RUN"
objective_value = missing
lp_objective_value = missing
n_columns = missing
cg_iterations = missing
cg_converged = missing
cg_stop_reason = missing
cg_optimality_scope = missing
cg_final_pricing_mode = missing
warm_start_sec = missing
warm_start_iterations = missing
phase1_iterations = missing
phase2_iterations = missing
phase1_pricing_sec = missing
phase2_pricing_sec = missing
phase1_master_sec = missing
phase2_master_sec = missing
phase1_columns = missing
phase2_columns = missing
error_message = ""
result = nothing
t_start = time()
try
    result = run_opt(problem, formulation, solver)
    metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)
    termination_status = string(result.termination_status)
    objective_value = something(result.objective_value, missing)
    lp_objective_value = get(result.metadata, "cg_lp_objective_value", missing)
    n_columns = metrics.n_columns
    cg_iterations = metrics.cg_iterations
    cg_converged = metrics.cg_converged
    cg_stop_reason = get(result.metadata, "cg_stop_reason", missing)
    cg_optimality_scope = get(result.metadata, "cg_optimality_scope", missing)
    cg_final_pricing_mode = string(get(result.metadata, "cg_final_pricing_mode", missing))
    warm_start_sec = get(result.metadata, "cg_warm_start_sec", missing)
    warm_start_iterations = get(result.metadata, "cg_warm_start_iterations", missing)

    # Split the iteration log by the phase each row priced in. On the `exact` arm every
    # row is phase 2 by construction, so phase-1 columns come out zero -- which is the
    # right encoding, not a missing value: that arm genuinely ran no elementary phase.
    log = get(result.metadata, "cg_iteration_log", NamedTuple[])
    is_p1(row) = get(row, :pricing_mode, "") == "station_simple"
    p1 = [r for r in log if is_p1(r)]
    p2 = [r for r in log if !is_p1(r)]
    phase1_iterations = length(p1)
    phase2_iterations = length(p2)
    phase1_pricing_sec = sum((r.pricing_sec for r in p1); init=0.0)
    phase2_pricing_sec = sum((r.pricing_sec for r in p2); init=0.0)
    phase1_master_sec = sum((r.master_sec for r in p1); init=0.0)
    phase2_master_sec = sum((r.master_sec for r in p2); init=0.0)
    phase1_columns = sum((r.columns_accepted for r in p1); init=0)
    phase2_columns = sum((r.columns_accepted for r in p2); init=0)

    status = if metrics.cg_converged && metrics.cg_pricing_exhausted
        "exhausted"
    elseif get(result.metadata, "cg_total_budget_exhausted", false) === true
        "budget_exhausted"
    else
        "incomplete"
    end
catch err
    error_message = replace(sprint(showerror, err), '\n' => ' ')
end
wall_sec = time() - t_start

row = DataFrame((
    job_id=[job_id], cell_id=[cell_id], arm=[arm], method=["cg_exact"],
    n_threads=[n_threads], julia_threads=[Threads.nthreads()],
    n_stations=[n_stations], n_pairs=[n_pairs], n_scenarios=[n_scenarios], seed=[seed],
    k=[k], max_stops=[max_stops], time_limit_sec=[time_limit_sec],
    certifying_time_limit_sec=[certifying_time_limit_sec],
    total_time_limit_sec=[total_time_limit_sec],
    status=[status], termination_status=[termination_status],
    objective_value=[objective_value], lp_objective_value=[lp_objective_value],
    wall_sec=[wall_sec], n_columns=[n_columns], cg_iterations=[cg_iterations],
    cg_converged=[cg_converged], cg_stop_reason=[cg_stop_reason],
    cg_optimality_scope=[cg_optimality_scope],
    cg_final_pricing_mode=[cg_final_pricing_mode],
    warm_start_sec=[warm_start_sec], warm_start_iterations=[warm_start_iterations],
    phase1_iterations=[phase1_iterations], phase2_iterations=[phase2_iterations],
    phase1_pricing_sec=[phase1_pricing_sec], phase2_pricing_sec=[phase2_pricing_sec],
    phase1_master_sec=[phase1_master_sec], phase2_master_sec=[phase2_master_sec],
    phase1_columns=[phase1_columns], phase2_columns=[phase2_columns],
    error_message=[error_message],
))
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0')).csv")
CSV.write(outfile, row)
println("Wrote $outfile (arm=$arm, status=$status, wall_sec=$(round(wall_sec; digits=2)), " *
        "warm_start_sec=$warm_start_sec)")

if result !== nothing
    iteration_rows = benchmark_iteration_rows(result, (
        job_id=job_id, cell_id=cell_id, arm=arm,
        n_stations=n_stations, n_pairs=n_pairs, n_scenarios=n_scenarios, seed=seed,
        k=k, max_stops=max_stops,
    ))
    iterations_dir = joinpath(output_dir, "iterations")
    mkpath(iterations_dir)
    CSV.write(joinpath(iterations_dir, "job_$(lpad(job_id, 4, '0'))_iterations.csv"),
              DataFrame(iteration_rows))
end
end
