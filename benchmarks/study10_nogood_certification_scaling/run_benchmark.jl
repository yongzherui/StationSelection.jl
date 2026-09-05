"""Run one Study 10 job: the `baseline` arm, or one cluster count `K` of the
no-good-cut certify-first arm, at n=20 or n=30.

The arms differ in exactly two coupled constructor arguments -- the formulation's
`relaxed_cluster_count = K` (which builds the station partition, once, at build time) and
`CGSolver(certification_pricing_mode = :relaxed_cluster_nogood)` (which makes the loop
consult it). Everything else -- budgets, threads, Gurobi settings, instance, pricing mode
-- is identical, so a difference isolates the certification loop and nothing else.

Beyond Study 9's row, this one records the **no-good trace**: how many cut rounds each
attempt walked, how many cuts it placed, and how big the station subsets it searched
exhaustively were. That trace is what distinguishes the three ways an arm can fail --

  - `refuted`: an exhaustive subset search found a genuinely improving column, which is a
    true negative and says nothing against the relaxation;
  - `inconclusive` with `nogood_rounds == certification_max_rounds`: the round cap bound,
    so the partition needs more cuts than the cap allows;
  - `inconclusive` with fewer rounds: `certification_time_limit_sec` bound, so a subset
    search ran out of wall

-- and it is also how the one-shot mode Study 9 measured is read off for free: an attempt
that certified at `nogood_rounds == 1` is exactly one the one-shot mode would have
certified too.
"""

using StationSelection
using CSV
using DataFrames
using Statistics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
fields = split(strip(ARGS[1]), '\t')
length(fields) == 16 || error("expected 16 tab-separated fields, got $(length(fields))")

job_id = parse(Int, fields[1])
cell_id, arm = fields[2:3]
n_clusters = parse(Int, fields[4])
k_fraction = parse(Float64, fields[5])
n_stations, n_pairs, n_scenarios, seed, max_stops, n_threads = parse.(Int, fields[6:11])
time_limit_sec = parse(Float64, fields[12])
certifying_time_limit_sec = parse(Float64, fields[13])
total_time_limit_sec = parse(Float64, fields[14])
certification_time_limit_sec = parse(Float64, fields[15])
certification_max_rounds = parse(Int, fields[16])

(n_clusters == 0) == (arm == "baseline") || error(
    "arm $(repr(arm)) and n_clusters=$n_clusters disagree: n_clusters==0 encodes the " *
    "baseline arm and nothing else",
)
if Threads.nthreads() < n_threads
    error("job asks for $n_threads threads but Julia has $(Threads.nthreads()); submit " *
          "with --cpus-per-task=$n_threads")
end

problem, k, instance_meta = benchmark_problem(
    @__DIR__, "STUDY10", n_stations, n_pairs, n_scenarios, seed,
)
output_dir = benchmark_output_dir(@__DIR__, "STUDY10", "study10_nogood_certification_scaling")
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops, pricing_mode=:exact,
    relaxed_cluster_count=(n_clusters == 0 ? nothing : n_clusters),
)
solver = benchmark_cg_solver(
    time_limit_sec; recover_integer_solution=true, threads=1,
    certifying_pricing_time_limit_sec=certifying_time_limit_sec,
    total_time_limit_sec=total_time_limit_sec,
    parallel_scenario_pricing=true,
    certification_pricing_mode=(n_clusters == 0 ? nothing : :relaxed_cluster_nogood),
    certification_time_limit_sec=certification_time_limit_sec,
    certification_max_rounds=certification_max_rounds,
)

"""
    _nogood_trace_summary(result) -> NamedTuple

Reduce the per-(attempt x scenario) no-good rows on
`metadata["cg_relaxed_cluster_guide_stats"]` to the handful of numbers a result row can
carry. Rows written by *guided pricing* share that channel and carry no `nogood_outcome`,
so they are filtered out rather than assumed absent (this study never enables guided
pricing, but the schema is shared and a silent mis-read would be worse than a filter).

`max_rounds` is the diagnostic for whether `certification_max_rounds` bound;
`median_subset_size` against `n_stations` is the diagnostic for whether the exhaustive
half of each round was actually cheap.
"""
const EMPTY_NOGOOD_TRACE = (
    nogood_attempts=0, nogood_max_rounds=missing, nogood_median_rounds=missing,
    nogood_total_cuts=missing, nogood_median_subset_size=missing,
    nogood_certified_at_round_1=missing, nogood_outcomes=missing,
)

function _nogood_trace_summary(result)
    stats = get(result.metadata, "cg_relaxed_cluster_guide_stats", Any[])
    rows = [s for s in stats if hasproperty(s, :nogood_outcome)]
    isempty(rows) && return EMPTY_NOGOOD_TRACE
    rounds = [Int(r.nogood_rounds) for r in rows]
    cuts = [Int(r.nogood_cuts) for r in rows]
    subsets = [Int(r.subset_size) for r in rows if Int(r.subset_size) > 0]
    outcomes = [string(r.nogood_outcome) for r in rows]
    # A scenario certified on its FIRST relaxed search is one the one-shot mode
    # (`:relaxed_cluster`, Study 9's arm) would also have certified -- no cut was needed.
    certified_at_1 = count(i -> outcomes[i] == "certified" && rounds[i] <= 1, eachindex(rows))
    return (
        nogood_attempts=length(rows),
        nogood_max_rounds=maximum(rounds),
        nogood_median_rounds=median(rounds),
        nogood_total_cuts=sum(cuts),
        nogood_median_subset_size=isempty(subsets) ? missing : median(subsets),
        nogood_certified_at_round_1=certified_at_1,
        nogood_outcomes=join(sort(unique(outcomes)), '|'),
    )
end

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
non_certifying_certification_sec = missing
certification_harvested_columns = missing
certifying_rounds = missing
cluster_sizes = missing
total_pricing_sec = missing
total_master_sec = missing
nogood = EMPTY_NOGOOD_TRACE
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
    non_certifying_certification_sec = certification.non_certifying_certification_sec
    certification_harvested_columns = certification.certification_harvested_columns
    certifying_rounds = certification.certifying_rounds
    total_pricing_sec = iteration_metrics.total_pricing_sec
    total_master_sec = iteration_metrics.total_master_sec
    nogood = _nogood_trace_summary(result)
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
    k_fraction=[k_fraction],
    method=["cg_exact"], n_threads=[n_threads], julia_threads=[Threads.nthreads()],
    n_stations=[n_stations], n_pairs=[n_pairs], n_scenarios=[n_scenarios], seed=[seed],
    k=[k], max_stops=[max_stops], time_limit_sec=[time_limit_sec],
    certifying_time_limit_sec=[certifying_time_limit_sec],
    total_time_limit_sec=[total_time_limit_sec],
    certification_time_limit_sec=[certification_time_limit_sec],
    certification_max_rounds=[certification_max_rounds],
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
    non_certifying_certification_sec=[non_certifying_certification_sec],
    certification_harvested_columns=[certification_harvested_columns],
    certifying_rounds=[certifying_rounds], cluster_sizes=[cluster_sizes],
    nogood_attempts=[nogood.nogood_attempts],
    nogood_max_rounds=[nogood.nogood_max_rounds],
    nogood_median_rounds=[nogood.nogood_median_rounds],
    nogood_total_cuts=[nogood.nogood_total_cuts],
    nogood_median_subset_size=[nogood.nogood_median_subset_size],
    nogood_certified_at_round_1=[nogood.nogood_certified_at_round_1],
    nogood_outcomes=[nogood.nogood_outcomes],
    total_pricing_sec=[total_pricing_sec], total_master_sec=[total_master_sec],
    error_message=[error_message],
))
outfile = joinpath(output_dir, "job_$(lpad(job_id, 4, '0')).csv")
CSV.write(outfile, row)
println("Wrote $outfile (arm=$arm, n=$n_stations, status=$status, " *
        "wall_sec=$(round(wall_sec; digits=2)), " *
        "certified_by_relaxation=$certified_by_relaxation, " *
        "nogood_cuts=$(nogood.nogood_total_cuts))")

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
