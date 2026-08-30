"""Run one single-threaded Study 6 method job (Joint+CG vs Base+Direct)."""

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 10 || error("expected 10 tab-separated fields, got $(length(fields))")
job_id = parse(Int, fields[1])
instance_id, method = fields[2:3]
method in ("cg_exact", "enumeration") || error("unknown method: $method")
n_stations, n_pairs, n_scenarios, seed, max_stops = parse.(Int, fields[4:8])
time_limit_sec = parse(Float64, fields[9])
max_routes = parse(Int, fields[10])

output_dir = benchmark_output_dir(@__DIR__, "STUDY6", "study6_exact_cg_vs_enumeration")
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0'))_$(method).csv")
if isfile(outfile)
    println("Skipping job $job_id: result already exists at $outfile")
    exit(0)
end

problem, k, instance_meta = benchmark_problem(
    @__DIR__, "STUDY6", n_stations, n_pairs, n_scenarios, seed,
)
# Study 6 compares the two *methods* as they are actually meant to be used:
#   cg_exact    -> Joint formulation solved by column generation (the production CG path)
#   enumeration -> Base formulation solved as one direct MIP over an exhaustively
#                  enumerated route pool (the baseline it is meant to displace)
# Both reach the same integer optimum (Study 1: mean z_ip identical to 1e-12), so the
# comparison isolates the solve algorithm. Before 2026-08-25 the cg_exact arm ran Base
# under CGSolver, which compared a solver against itself on a formulation nobody uses that
# way -- and was the only study exercising the Base+CG livelock fixed the same day.
cg_formulation() =
    AggregateODRouteJointRoutingAssignmentFormulation(; BENCHMARK_BASELINE..., max_stops=max_stops)
enumeration_formulation() =
    AggregateODRouteBaseFormulation(; BENCHMARK_BASELINE..., max_stops=max_stops)

let
status = "error"
termination_status = "NOT_RUN"
objective_value = missing
lp_objective_value = missing
n_columns = missing
cg_iterations = missing
cg_converged = missing
cg_pricing_exhausted = missing
error_message = ""
t_start = time()
try
    if method == "enumeration"
        solver = DirectMIPSolver(config=SolverOptions(
            silent=true, time_limit_sec=time_limit_sec, threads=1,
        ))
        build = StationSelection.build_model(
            problem, enumeration_formulation(), solver;
            max_routes=max_routes, time_limit_sec=time_limit_sec,
        )
        result = StationSelection.optimize_model(build, solver)
        termination_status = string(result.termination_status)
        objective_value = something(result.objective_value, missing)
        n_columns = build.counts.extras["routes_enumerated"]
        status = termination_status == "OPTIMAL" ? "exhausted" : "incomplete"
    else
        solver = benchmark_cg_solver(
            time_limit_sec; recover_integer_solution=true, threads=1,
        )
        result = run_opt(problem, cg_formulation(), solver)
        metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)
        bounds = benchmark_lp_ip(result)
        termination_status = string(result.termination_status)
        objective_value = bounds.z_ip
        lp_objective_value = bounds.z_lp
        n_columns = metrics.n_columns
        cg_iterations = metrics.cg_iterations
        cg_converged = metrics.cg_converged
        cg_pricing_exhausted = metrics.cg_pricing_exhausted
        status = metrics.cg_converged && metrics.cg_pricing_exhausted ? "exhausted" : "incomplete"

        # Long-format per-iteration log (master/pricing/add-columns seconds per CG
        # iteration) -- enumeration has no CG loop, so only this arm writes one.
        iteration_rows = benchmark_iteration_rows(result, (
            job_id=job_id, instance_id=instance_id, method=method,
            n_stations=n_stations, n_pairs=n_pairs, n_scenarios=n_scenarios, seed=seed,
            k=k, max_stops=max_stops,
        ))
        iterations_dir = joinpath(output_dir, "iterations")
        mkpath(iterations_dir)
        iterations_file = joinpath(iterations_dir, "job_$(lpad(job_id, 4, '0'))_cg_exact_iterations.csv")
        CSV.write(iterations_file, DataFrame(iteration_rows))
        println("Wrote $iterations_file ($(length(iteration_rows)) iterations)")
    end
catch err
    error_message = replace(sprint(showerror, err), '\n' => ' ')
    lower = lowercase(error_message)
    status = occursin("time limit", lower) ? "timed_out" :
        occursin("max_routes", lower) ? "route_limit" : "error"
end
wall_sec = time() - t_start

row = DataFrame((
    job_id=[job_id], instance_id=[instance_id], method=[method],
    n_stations=[n_stations], n_pairs=[n_pairs], n_scenarios=[n_scenarios], seed=[seed],
    n_pairs_actual=[sum(instance_meta.pairs_per_scenario)],
    pairs_per_scenario=[join(instance_meta.pairs_per_scenario, ";")], k=[k],
    max_stops=[max_stops], time_limit_sec=[time_limit_sec], max_routes=[max_routes],
    status=[status], termination_status=[termination_status],
    objective_value=[objective_value], lp_objective_value=[lp_objective_value],
    wall_sec=[wall_sec], n_columns=[n_columns], cg_iterations=[cg_iterations],
    cg_converged=[cg_converged], cg_pricing_exhausted=[cg_pricing_exhausted],
    error_message=[error_message],
))
CSV.write(outfile, row)
println("Wrote $outfile (status=$status, wall_sec=$(round(wall_sec; digits=2)))")
end
