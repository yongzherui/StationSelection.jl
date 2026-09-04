"""Run one Study 9 job: the `baseline` arm, or one cluster count `K` of the
relaxed-cluster certify-first arm.

The arms differ in exactly two coupled constructor arguments -- the formulation's
`relaxed_cluster_count = K` (which is what builds the station partition, once, at build
time) and `CGSolver(certification_pricing_mode = :relaxed_cluster)` (which is what makes
the loop consult it). Everything else -- budgets, threads, Gurobi settings, instance,
pricing mode -- is identical, so a wall-clock difference isolates the relaxation and
nothing else.

Three outcomes are worth distinguishing in the row and all are recorded:
`certified_by_relaxation` (the relaxation ended the solve, skipping the exhaustive
certifying round), `failed_certification_sec` (what the attempts that did *not* certify
cost), and the `refuted`/`inconclusive` split of those failures. A run can be slower than
baseline while still certifying, if the attempts were expensive; a run can be faster
without ever certifying, only by noise. And a run that never certifies is only
interpretable once you know whether its attempts were refuted (this arm's partition was
too coarse) or inconclusive (out of `certification_time_limit_sec`) -- the first is a fact
about the arm, the second a budget that could be raised.
"""

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 14 || error("expected 14 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
cell_id, arm = fields[2:3]
n_clusters = parse(Int, fields[4])
n_stations, n_pairs, n_scenarios, seed, max_stops, n_threads = parse.(Int, fields[5:10])
time_limit_sec = parse(Float64, fields[11])
certifying_time_limit_sec = parse(Float64, fields[12])
total_time_limit_sec = parse(Float64, fields[13])
certification_time_limit_sec = parse(Float64, fields[14])

(n_clusters == 0) == (arm == "baseline") || error(
    "arm $(repr(arm)) and n_clusters=$n_clusters disagree: n_clusters==0 encodes the " *
    "baseline arm and nothing else",
)
if Threads.nthreads() < n_threads
    error("job asks for $n_threads threads but Julia has $(Threads.nthreads()); submit " *
          "with --cpus-per-task=$n_threads")
end

problem, k, instance_meta = benchmark_problem(
    @__DIR__, "STUDY9", n_stations, n_pairs, n_scenarios, seed,
)
output_dir = benchmark_output_dir(@__DIR__, "STUDY9", "study9_relaxed_cluster_certification")
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops, pricing_mode=:exact,
    relaxed_cluster_count=(n_clusters == 0 ? nothing : n_clusters),
)
solver = benchmark_cg_solver(
    time_limit_sec; recover_integer_solution=true, threads=1,
    certifying_pricing_time_limit_sec=certifying_time_limit_sec,
    total_time_limit_sec=total_time_limit_sec,
    parallel_scenario_pricing=true,
    certification_pricing_mode=(n_clusters == 0 ? nothing : :relaxed_cluster),
    certification_time_limit_sec=certification_time_limit_sec,
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
certification_pricing_mode = missing
certified_by_relaxation = missing
certification_rounds = missing
certification_refuted_rounds = missing
certification_inconclusive_rounds = missing
certification_sec = missing
failed_certification_sec = missing
certifying_rounds = missing
cluster_sizes = missing
total_pricing_sec = missing
total_master_sec = missing
error_message = ""
result = nothing
t_start = time()
try
    result = run_opt(problem, formulation, solver)
    metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)
    certification = benchmark_certification_metrics(result)
    iteration_metrics = benchmark_iteration_metrics(result)
    termination_status = string(result.termination_status)
    objective_value = something(result.objective_value, missing)
    lp_objective_value = get(result.metadata, "cg_lp_objective_value", missing)
    n_columns = metrics.n_columns
    cg_iterations = metrics.cg_iterations
    cg_converged = metrics.cg_converged
    cg_stop_reason = get(result.metadata, "cg_stop_reason", missing)
    cg_optimality_scope = get(result.metadata, "cg_optimality_scope", missing)
    certification_pricing_mode = certification.certification_pricing_mode
    certified_by_relaxation = certification.certified_by_relaxation
    certification_rounds = certification.certification_rounds
    certification_refuted_rounds = certification.certification_refuted_rounds
    certification_inconclusive_rounds = certification.certification_inconclusive_rounds
    certification_sec = certification.certification_sec
    failed_certification_sec = certification.failed_certification_sec
    certifying_rounds = certification.certifying_rounds
    total_pricing_sec = iteration_metrics.total_pricing_sec
    total_master_sec = iteration_metrics.total_master_sec
    # The realized partition, not the requested count: k-medoids drops empty cells, and a
    # lopsided partition (one giant cell) is the first thing to look at when an arm never
    # certifies.
    if haskey(result.model.obj_dict, :joint_routing_assignment_station_clustering)
        cluster_sizes = join(
            station_cluster_sizes(result.model[:joint_routing_assignment_station_clustering]), '|',
        )
    end

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
    job_id=[job_id], cell_id=[cell_id], arm=[arm], n_clusters=[n_clusters],
    method=["cg_exact"], n_threads=[n_threads], julia_threads=[Threads.nthreads()],
    n_stations=[n_stations], n_pairs=[n_pairs], n_scenarios=[n_scenarios], seed=[seed],
    k=[k], max_stops=[max_stops], time_limit_sec=[time_limit_sec],
    certifying_time_limit_sec=[certifying_time_limit_sec],
    total_time_limit_sec=[total_time_limit_sec],
    certification_time_limit_sec=[certification_time_limit_sec],
    status=[status], termination_status=[termination_status],
    objective_value=[objective_value], lp_objective_value=[lp_objective_value],
    wall_sec=[wall_sec], n_columns=[n_columns], cg_iterations=[cg_iterations],
    cg_converged=[cg_converged], cg_stop_reason=[cg_stop_reason],
    cg_optimality_scope=[cg_optimality_scope],
    certification_pricing_mode=[certification_pricing_mode],
    certified_by_relaxation=[certified_by_relaxation],
    certification_rounds=[certification_rounds],
    certification_refuted_rounds=[certification_refuted_rounds],
    certification_inconclusive_rounds=[certification_inconclusive_rounds],
    certification_sec=[certification_sec],
    failed_certification_sec=[failed_certification_sec],
    certifying_rounds=[certifying_rounds], cluster_sizes=[cluster_sizes],
    total_pricing_sec=[total_pricing_sec], total_master_sec=[total_master_sec],
    error_message=[error_message],
))
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0')).csv")
CSV.write(outfile, row)
println("Wrote $outfile (arm=$arm, status=$status, wall_sec=$(round(wall_sec; digits=2)), " *
        "certified_by_relaxation=$certified_by_relaxation)")

if result !== nothing
    iteration_rows = benchmark_iteration_rows(result, (
        job_id=job_id, cell_id=cell_id, arm=arm, n_clusters=n_clusters,
        n_stations=n_stations, n_pairs=n_pairs, n_scenarios=n_scenarios, seed=seed,
        k=k, max_stops=max_stops,
    ))
    iterations_dir = joinpath(output_dir, "iterations")
    mkpath(iterations_dir)
    CSV.write(joinpath(iterations_dir, "job_$(lpad(job_id, 4, '0'))_iterations.csv"),
              DataFrame(iteration_rows))
end
end
