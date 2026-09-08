using StationSelection
using CSV
using DataFrames
using Statistics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
f = split(strip(ARGS[1]), '\t')
length(f) == 17 || error("expected 17 fields, got $(length(f))")
job_id = parse(Int, f[1])
phase, arm = f[2], f[3]
n, p, s, seed, max_stops, n_threads, k0, kmax, guide_routes = parse.(Int, f[4:12])
barren_cache, cut_management = parse.(Bool, f[13:14])
pricing_limit, certifying_limit, total_limit = parse.(Float64, f[15:17])
arm in ("exact", "relaxed_k60", "relaxed_k80") || error("unknown arm $arm")
Threads.nthreads() == n_threads || error(
    "job requests $n_threads Julia threads, process has $(Threads.nthreads())")
(arm == "exact") == (k0 == 0) || error("exact iff cluster_count=0")
kmax == 0 || error("cluster refinement is excluded from Study 9")
!barren_cache || error("barren cache is excluded from Study 9")
!cut_management || error("cut management is excluded from Study 9")

problem, selection_k, instance_meta = benchmark_problem(@__DIR__, "STUDY9", n, p, s, seed)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops)
pricing = arm == "exact" ? CGPricingConfig(mode=:exact) : CGPricingConfig(
    mode=:relaxed_cluster,
    relaxed_cluster_count=k0,
    relaxed_cluster_max_count=(kmax == 0 ? nothing : kmax),
    relaxed_cluster_guide_routes=guide_routes,
    relaxed_cluster_barren_cache=barren_cache,
    relaxed_cluster_cut_management=cut_management,
)
solver = benchmark_cg_solver(pricing_limit;
    recover_integer_solution=true, threads=1,
    certifying_pricing_time_limit_sec=certifying_limit,
    total_time_limit_sec=total_limit,
    parallel_scenario_pricing=true, pricing=pricing)
outdir = benchmark_output_dir(@__DIR__, "STUDY9", "study9_relaxed_cluster_scalability")

function trace_summary(result)
    rows = [r for r in get(result.metadata, "cg_relaxed_cluster_guide_stats", Any[])
            if hasproperty(r, :nogood_outcome)]
    isempty(rows) && return (attempts=0, cuts=0, max_rounds=0,
        median_subset=missing, cache_hits=0, thread_ids="")
    subsets = [Int(x) for r in rows for x in r.nogood_subset_size_trace if Int(x) > 0]
    tids = sort!(unique(Int(r.thread_id) for r in rows if hasproperty(r, :thread_id)))
    return (attempts=length(rows), cuts=sum(Int(r.nogood_cuts) for r in rows),
        max_rounds=maximum(Int(r.nogood_rounds) for r in rows),
        median_subset=isempty(subsets) ? missing : median(subsets),
        cache_hits=sum(Int(r.nogood_barren_cache_hits) for r in rows),
        thread_ids=join(tids, ';'))
end

let
result = nothing
error_message = ""
started = time()
try
    result = run_opt(problem, formulation, solver)
catch err
    error_message = replace(sprint(showerror, err), '\n' => ' ')
end
wall_sec = time() - started

if result === nothing
    row = DataFrame(job_id=[job_id], phase=[phase], arm=[arm], n_stations=[n],
        n_pairs=[p], n_scenarios=[s], seed=[seed], cluster_count=[k0],
        cluster_max_count=[kmax], status=["error"], wall_sec=[wall_sec],
        error_message=[error_message])
else
    metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)
    cert = benchmark_certification_metrics(result)
    iters = benchmark_iteration_metrics(result)
    trace = trace_summary(result)
    md = result.metadata
    scope = string(get(md, "cg_optimality_scope", "unknown"))
    certified = metrics.cg_converged && scope == "full_route_universe"
    final_counts = get(md, "cg_relaxed_cluster_final_counts", Int[])
    splits = get(md, "cg_relaxed_cluster_splits", Int[])
    pricing_tids = sort!(unique(Int(r.thread_id) for r in get(md, "cg_pricing_stats", Any[])
        if hasproperty(r, :thread_id)))
    row = DataFrame(
        job_id=[job_id], phase=[phase], arm=[arm], n_stations=[n], n_pairs=[p],
        n_scenarios=[s], seed=[seed], n_pairs_actual=[sum(instance_meta.pairs_per_scenario)],
        selection_k=[selection_k], max_stops=[max_stops], julia_threads=[Threads.nthreads()],
        cluster_count=[k0], cluster_max_count=[kmax], guide_routes=[guide_routes],
        barren_cache=[barren_cache], cut_management=[cut_management],
        pricing_limit_sec=[pricing_limit], certifying_limit_sec=[certifying_limit],
        total_limit_sec=[total_limit], status=[certified ? "certified" : "incomplete"],
        termination_status=[string(result.termination_status)], optimality_scope=[scope],
        objective_value=[something(result.objective_value, missing)],
        lp_objective_value=[get(md, "cg_lp_objective_value", missing)], wall_sec=[wall_sec],
        runtime_sec=[metrics.runtime_sec], lp_loop_sec=[iters.lp_loop_sec],
        integer_recovery_sec=[iters.integer_recovery_sec], cg_iterations=[metrics.cg_iterations],
        cg_stop_reason=[string(get(md, "cg_stop_reason", "unknown"))],
        n_columns=[metrics.n_columns], labels_generated=[metrics.labels_generated],
        certification_rounds=[cert.certification_rounds],
        certification_refuted_rounds=[cert.certification_refuted_rounds],
        certification_inconclusive_rounds=[cert.certification_inconclusive_rounds],
        certification_sec=[cert.certification_sec],
        certification_harvested_columns=[cert.certification_harvested_columns],
        certified_by_relaxation=[cert.certified_by_relaxation],
        exact_certifying_rounds=[cert.certifying_rounds], nogood_attempts=[trace.attempts],
        nogood_total_cuts=[trace.cuts], nogood_max_rounds=[trace.max_rounds],
        nogood_median_subset_size=[trace.median_subset], barren_cache_hits=[trace.cache_hits],
        final_cluster_counts=[join(final_counts, ';')], cluster_splits=[join(splits, ';')],
        pricing_thread_ids=[join(pricing_tids, ';')], certification_thread_ids=[trace.thread_ids],
        error_message=[error_message])
    identity = (job_id=job_id, phase=phase, arm=arm, n_stations=n, seed=seed)
    iteration_rows = benchmark_iteration_rows(result, identity)
    idir = joinpath(outdir, "iterations")
    mkpath(idir)
    CSV.write(joinpath(idir, "n$(n)_seed$(seed)_$(arm).csv"), DataFrame(iteration_rows))
end

outfile = joinpath(outdir, "n$(n)_seed$(seed)_$(arm).csv")
CSV.write(outfile, row)
println("Wrote $outfile status=$(row.status[1]) wall=$(round(wall_sec; digits=1))s")
end
