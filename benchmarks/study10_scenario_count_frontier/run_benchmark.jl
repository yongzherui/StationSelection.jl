using StationSelection
using CSV
using DataFrames
using Statistics
using Serialization
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
f = split(strip(ARGS[1]), '\t')
# 17, 18 or 19 fields. The 18th is the two-tier macro count K1, absent from every table
# written before `:relaxed_cluster_two_tier` existed; the 19th is the aligned-subset station
# budget. Accepting all three widths keeps arrays already in flight runnable after a
# preemption requeue, which a hard width would break.
length(f) in (17, 18, 19) || error("expected 17, 18 or 19 fields, got $(length(f))")
job_id = parse(Int, f[1])
phase, arm = f[2], f[3]
n, p, s, seed, max_stops, n_threads, k0, kmax, guide_routes = parse.(Int, f[4:12])
barren_cache, cut_management = parse.(Bool, f[13:14])
pricing_limit, certifying_limit, total_limit = parse.(Float64, f[15:17])
macro_count = length(f) >= 18 ? parse(Int, f[18]) : 0
# The station budget for a macro-ALIGNED support, i.e. the knob that decides whether a
# barrenness proof can be carried up to the macro layer at all. Swept because it is not a
# free parameter but a binding one: it interacts with K1 through
# `|aligned stations| ~ (macro cells the support touches) x (n / K1)`, so a coarse macro
# layer needs a larger budget to align the SAME support. MEASURED at n=40/K2=24/K1=12 with
# the default 15: alignment refused on 105 of 111 station searches and the run produced
# ZERO macro cuts, so nothing could certify however much budget was left (it quit having
# used 6% of it).
aligned_subset_max = length(f) >= 19 ? parse(Int, f[19]) : 15
# `twotier_k60m14` names a two-tier arm with an explicit K1, so two K1 settings at the same
# K2 can run without colliding on the output filename (which is keyed by arm). The optional
# `c<N>`/`g<N>`/`p<N>` suffixes name the aligned-subset cap, the guide-route count and the
# ordinary pricing-round budget respectively, for the same reason: any two arms that differ
# in a swept parameter must differ in their filename.
arm in ("exact", "relaxed_k60", "relaxed_k80", "twotier_k60", "twotier_k80") ||
    occursin(r"^twotier_k\d+m\d+([gcp]\d+)*$", arm) ||
    error("unknown arm $arm")
two_tier = startswith(arm, "twotier")
Threads.nthreads() == n_threads || error(
    "job requests $n_threads Julia threads, process has $(Threads.nthreads())")
(arm == "exact") == (k0 == 0) || error("exact iff cluster_count=0")
# K1 belongs to the two-tier arms and only to them -- a macro count nothing reads would
# quietly build a second partition and change nothing, which is the failure mode that made
# the v2 rows worthless.
if two_tier
    0 < macro_count < k0 || error(
        "two-tier arms need 0 < macro_count ($macro_count) < cluster_count ($k0)")
else
    macro_count == 0 || error("macro_count is only read by the twotier_* arms")
end
kmax == 0 || error("cluster refinement is excluded from Study 9")
# The barren-support cache and active-cut cut management (once called "subsumption
# pruning") are LIVE and UNCONDITIONAL in `:relaxed_cluster` -- see
# `relaxed_cluster/utils/certification/results.jl`. They were briefly absent, dropped as
# unnecessary at the then-measured cut load, and returned in 3767740 once n=40 seed 42
# started exhausting all 64 mask bits. What no longer exists is the SWITCH: there is no
# `barren_cache`/`cut_management` field on `CGPricingConfig`, so neither can be turned on
# or off and these two columns must stay `false`, which keeps one schema across rows
# written before and after the change.
!barren_cache ||
    error("barren cache is no longer configurable (now unconditional); this column must be false")
!cut_management ||
    error("cut management is no longer configurable (now unconditional); this column must be false")

problem, selection_k, instance_meta = benchmark_problem(@__DIR__, "STUDY10", n, p, s, seed)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops)
pricing = if arm == "exact"
    CGPricingConfig(mode=:exact)
elseif two_tier
    CGPricingConfig(
        mode=:relaxed_cluster_two_tier,
        relaxed_cluster_aligned_subset_max=aligned_subset_max,
        relaxed_cluster_count=k0,
        relaxed_cluster_macro_count=macro_count,
        relaxed_cluster_guide_routes=guide_routes,
    )
else
    CGPricingConfig(
        mode=:relaxed_cluster,
        relaxed_cluster_count=k0,
        relaxed_cluster_max_count=(kmax == 0 ? nothing : kmax),
        relaxed_cluster_guide_routes=guide_routes,
    )
end
outdir = benchmark_output_dir(@__DIR__, "STUDY10", "study10_scenario_count_frontier")
stem = "n$(n)_seed$(seed)_$(arm)"
job_identity = (job_id=job_id, phase=phase, arm=arm, n_stations=n, seed=seed)
iter_dir = joinpath(outdir, "iterations")
mkpath(iter_dir)
iter_path = joinpath(iter_dir, "$(stem).csv")
# `.progress.csv` deliberately does NOT match the `n<N>_seed<S>_<arm>.csv` pattern that
# `analyze.jl`, `check_gate.jl` and `compare_two_tier.jl` glob, so an in-flight run is
# visible to a human without being swept into a gate as if it were a result.
progress_path = joinpath(outdir, "$(stem).progress.csv")
started = time()
iterations_seen = Ref(0)

# Streamed after EVERY CG iteration. A six-hour task that gets preempted or hits its wall
# used to leave nothing at all behind -- the run was simply unobservable while it ran and
# unrecoverable if it died. Now the per-iteration history lands as it happens and a
# one-row snapshot says how far the run got.
function on_iteration(row)
    iterations_seen[] += 1
    tidy = (; job_identity..., row...)
    fresh = !isfile(iter_path)
    CSV.write(iter_path, DataFrame([tidy]); append=!fresh, writeheader=fresh)
    CSV.write(progress_path, DataFrame([(; job_identity...,
        status="in_progress", iterations_done=iterations_seen[],
        elapsed_sec=round(time() - started; digits=1),
        cluster_count=k0, macro_count=macro_count,
        master_objective=get(row, :master_objective, missing),
        cumulative_columns_added=get(row, :cumulative_columns_added, missing),
        certification_outcome=string(get(row, :certification_outcome, "")),
        certification_sec=get(row, :certification_sec, missing),
        total_limit_sec=total_limit)]))
    return nothing
end

# Late-stage dual snapshots, opt-in via STUDY10_EXPORT_DUALS=1.
#
# A parameter diagnostic that re-runs CG only ever reaches EARLY iterations, where every
# search exhausts in milliseconds -- MEASURED at n=40, K2=24: a round costs ~1 s at
# iteration 4 and 300 s late in a real solve. The regime that decides certification is a
# near-converged master fighting a ~0 margin, and replaying captured duals is the only way
# to sweep hyperparameters against it without paying for a full solve each time.
export_duals = get(ENV, "STUDY10_EXPORT_DUALS", "0") == "1"
dual_dir = joinpath(outdir, "duals", stem)
export_duals && mkpath(dual_dir)
function on_duals(iteration, duals)
    alpha, gamma_o, gamma_d = duals
    Serialization.serialize(joinpath(dual_dir, "it$(lpad(iteration, 4, '0')).jls"),
        (iteration=iteration, n_stations=n, n_pairs=p, n_scenarios=s, seed=seed,
         max_stops=max_stops, alpha=alpha, gamma_o=gamma_o, gamma_d=gamma_d))
    return nothing
end

solver = benchmark_cg_solver(pricing_limit;
    recover_integer_solution=true, threads=1,
    certifying_pricing_time_limit_sec=certifying_limit,
    total_time_limit_sec=total_limit,
    parallel_scenario_pricing=true, pricing=pricing,
    iteration_callback=on_iteration,
    dual_callback=(export_duals ? on_duals : nothing))

function trace_summary(result)
    rows = [r for r in get(result.metadata, "cg_relaxed_cluster_guide_stats", Any[])
            if hasproperty(r, :nogood_outcome)]
    isempty(rows) && return (attempts=0, cuts=0, max_rounds=0, macro_cuts=0, meso_cuts=0,
        max_cuts=0, max_macro_cuts=0, max_meso_cuts=0, aligned_rounds=0,
        macro_rounds_t=0.0, macro_sec_t=0.0, meso_rounds_t=0.0, meso_sec_t=0.0,
        station_rounds_t=0.0, station_sec_t=0.0, station_unexhausted_t=0.0,
        align_skipped_t=0.0, align_downgraded_t=0.0, exit_reasons="",
        meso_escalations_t=0.0, station_escalations_t=0.0, guides_used_mean=missing,
        median_subset=missing, thread_ids="")
    subsets = [Int(x) for r in rows for x in r.nogood_subset_size_trace if Int(x) > 0]
    tids = sort!(unique(Int(r.thread_id) for r in rows if hasproperty(r, :thread_id)))
    # Two-tier rows carry the cut split and the aligned-round count; one-tier rows do not,
    # so both are read defensively and land as 0 for the single-tier arms.
    tt(r, field) = hasproperty(r, field) ? getproperty(r, field) : 0
    ttsum(field) = sum(Float64(tt(r, field)) for r in rows)
    return (attempts=length(rows), cuts=sum(Int(r.nogood_cuts) for r in rows),
        macro_cuts=sum(Int(tt(r, :two_tier_macro_cuts)) for r in rows),
        meso_cuts=sum(Int(tt(r, :two_tier_meso_cuts)) for r in rows),
        aligned_rounds=sum(
            hasproperty(r, :two_tier_aligned_trace) ?
                count(identity, r.two_tier_aligned_trace) : 0
            for r in rows),
        # Peaks, per scenario attempt. Cut pools are per-attempt state (duals change every
        # iteration, so a barrenness proof does not survive one) and each pool has its own
        # 64-bit mask, so the MAXIMUM is the number that can hit the cap -- a mean of ~1
        # hides an attempt that used 10.
        max_cuts=maximum(Int(r.nogood_cuts) for r in rows),
        max_macro_cuts=maximum(Int(tt(r, :two_tier_macro_cuts)) for r in rows),
        max_meso_cuts=maximum(Int(tt(r, :two_tier_meso_cuts)) for r in rows),
        # Per-tier time and round accounting, summed over this run's scenario attempts:
        # WHICH of the three searches consumes a saturating round.
        macro_rounds_t=ttsum(:two_tier_macro_rounds), macro_sec_t=ttsum(:two_tier_macro_sec),
        meso_rounds_t=ttsum(:two_tier_meso_rounds), meso_sec_t=ttsum(:two_tier_meso_sec),
        station_rounds_t=ttsum(:two_tier_station_rounds),
        station_sec_t=ttsum(:two_tier_station_sec),
        station_unexhausted_t=ttsum(:two_tier_station_unexhausted),
        align_skipped_t=ttsum(:two_tier_align_skipped),
        align_downgraded_t=ttsum(:two_tier_align_downgraded),
        # Sweeps whose SCHEDULED slice truncated them and which the attempt's real remaining
        # budget then finished. A high count says the schedule was the binding constraint,
        # which is a different problem from a hard instance.
        meso_escalations_t=ttsum(:two_tier_meso_escalations),
        station_escalations_t=ttsum(:two_tier_station_escalations),
        # Mean guide-prefix length alignment actually priced. Below `guide_routes` means
        # shrinking is active and is what is buying the macro cuts; `0` entries are searches
        # that still had to be priced unaligned.
        guides_used_mean=(
            let gs = [Float64(x) for r in rows
                      for x in (hasproperty(r, :two_tier_guides_used_trace) ?
                                r.two_tier_guides_used_trace : Int[])]
                isempty(gs) ? missing : mean(gs)
            end),
        # Attempt exits tallied BY REASON -- "3 hit the inner-round cap, 1 ran the station
        # search out of slice" is actionable where a bare `inconclusive` count is not.
        exit_reasons=join(sort!([string(k, '=', v) for (k, v) in
            Dict(r => count(x -> string(get(x, :two_tier_exit_reason, :none)) == r, rows)
                 for r in unique(string(get(x, :two_tier_exit_reason, :none))
                                 for x in rows))]), ';'),
        max_rounds=maximum(Int(r.nogood_rounds) for r in rows),
        median_subset=isempty(subsets) ? missing : median(subsets),
        thread_ids=join(tids, ';'))
end

let
result = nothing
error_message = ""
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
        cluster_count=[k0], cluster_max_count=[kmax], macro_count=[macro_count],
        guide_routes=[guide_routes],
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
        certification_column_found_rounds=[cert.certification_column_found_rounds],
        certification_inconclusive_rounds=[cert.certification_inconclusive_rounds],
        certification_sec=[cert.certification_sec],
        certification_harvested_columns=[cert.certification_harvested_columns],
        certified_by_relaxation=[cert.certified_by_relaxation],
        exact_certifying_rounds=[cert.certifying_rounds], nogood_attempts=[trace.attempts],
        nogood_total_cuts=[trace.cuts], nogood_max_rounds=[trace.max_rounds],
        two_tier_macro_cuts=[trace.macro_cuts], two_tier_meso_cuts=[trace.meso_cuts],
        nogood_max_cuts_in_attempt=[trace.max_cuts],
        two_tier_max_macro_cuts_in_attempt=[trace.max_macro_cuts],
        two_tier_max_meso_cuts_in_attempt=[trace.max_meso_cuts],
        two_tier_aligned_rounds=[trace.aligned_rounds],
        two_tier_macro_rounds=[trace.macro_rounds_t], two_tier_macro_sec=[trace.macro_sec_t],
        two_tier_meso_rounds=[trace.meso_rounds_t], two_tier_meso_sec=[trace.meso_sec_t],
        two_tier_station_rounds=[trace.station_rounds_t],
        two_tier_station_sec=[trace.station_sec_t],
        two_tier_station_unexhausted=[trace.station_unexhausted_t],
        two_tier_align_skipped=[trace.align_skipped_t],
        two_tier_align_downgraded=[trace.align_downgraded_t],
        aligned_subset_max=[aligned_subset_max],
        two_tier_meso_escalations=[trace.meso_escalations_t],
        two_tier_station_escalations=[trace.station_escalations_t],
        two_tier_guides_used_mean=[trace.guides_used_mean],
        two_tier_exit_reasons=[trace.exit_reasons],
        nogood_median_subset_size=[trace.median_subset], barren_cache_hits=[0],
        final_cluster_counts=[join(final_counts, ';')], cluster_splits=[join(splits, ';')],
        pricing_thread_ids=[join(pricing_tids, ';')], certification_thread_ids=[trace.thread_ids],
        error_message=[error_message])
    # Rewrite the streamed file from the returned log, so a completed run's iterations CSV
    # is byte-identical to what it would have been without streaming -- one canonical
    # schema, whatever order the rows arrived in.
    CSV.write(iter_path, DataFrame(benchmark_iteration_rows(result, job_identity)))
end

# Per-attempt and per-round detail, to disk.
#
# `cg_relaxed_cluster_guide_stats` carries one row per scenario per CG iteration, each with
# the full per-round trace, and `trace_summary` above collapses every one of them into run
# level sums and maxima. That is the right summary for a gate but it is the wrong data for
# tuning: it cannot answer "what does a round cost at ITERATION 40" or "which slice caps are
# saturating", because both questions live inside the rows it threw away. Two extra files,
# neither matching the result glob:
#
#   attempts/<stem>.csv -- one row per (iteration, scenario) attempt, scalars only
#   rounds/<stem>.csv   -- one row per round WITHIN an attempt, with sec/slice/exhausted
#
# `rounds` is the long-format table the parameter work needs: `sec` against `slice` says
# whether a round finished inside its budget or was cut off by it.
if !isnothing(result)
    stat_rows = [r for r in get(result.metadata, "cg_relaxed_cluster_guide_stats", Any[])
                 if hasproperty(r, :nogood_outcome)]
    if !isempty(stat_rows)
        g(r, f, default) = hasproperty(r, f) ? getproperty(r, f) : default
        attempts = NamedTuple[]
        rounds_long = NamedTuple[]
        for r in stat_rows
            it = Int(g(r, :iteration, 0))
            sc = Int(g(r, :scenario, 0))
            push!(attempts, (; job_identity..., iteration=it, scenario=sc,
                cluster_count=k0, macro_count=macro_count, guide_routes=guide_routes,
                outcome=string(r.nogood_outcome),
                exit_reason=string(g(r, :two_tier_exit_reason, :none)),
                rounds=Int(r.nogood_rounds), cuts=Int(r.nogood_cuts),
                subset_size=Int(g(r, :subset_size, 0)),
                macro_cuts=Int(g(r, :two_tier_macro_cuts, 0)),
                meso_cuts=Int(g(r, :two_tier_meso_cuts, 0)),
                macro_rounds=Int(g(r, :two_tier_macro_rounds, 0)),
                macro_sec=Float64(g(r, :two_tier_macro_sec, 0.0)),
                meso_rounds=Int(g(r, :two_tier_meso_rounds, 0)),
                meso_sec=Float64(g(r, :two_tier_meso_sec, 0.0)),
                station_rounds=Int(g(r, :two_tier_station_rounds, 0)),
                station_sec=Float64(g(r, :two_tier_station_sec, 0.0)),
                station_unexhausted=Int(g(r, :two_tier_station_unexhausted, 0)),
                align_skipped=Int(g(r, :two_tier_align_skipped, 0)),
                align_downgraded=Int(g(r, :two_tier_align_downgraded, 0)),
                meso_escalations=Int(g(r, :two_tier_meso_escalations, 0)),
                station_escalations=Int(g(r, :two_tier_station_escalations, 0))))
            secs = g(r, :nogood_round_sec_trace, Float64[])
            slices = g(r, :nogood_round_slice_trace, Float64[])
            exh = g(r, :nogood_round_exhausted_trace, Bool[])
            tiers_v = g(r, :two_tier_tier_trace, Symbol[])
            aligned_v = g(r, :two_tier_aligned_trace, Bool[])
            rc_v = r.nogood_rc_trace
            srin_v = r.nogood_subset_rc_trace
            ssz_v = r.nogood_subset_size_trace
            for k in eachindex(rc_v)
                at(v, i) = i <= length(v) ? v[i] : missing
                push!(rounds_long, (; job_identity..., iteration=it, scenario=sc,
                    cluster_count=k0, macro_count=macro_count,
                    round_idx=k,
                    tier=(k <= length(tiers_v) ? string(tiers_v[k]) : ""),
                    sec=at(secs, k), slice=at(slices, k), exhausted=at(exh, k),
                    relaxed_rc=at(rc_v, k), subset_rc=at(srin_v, k),
                    subset_size=at(ssz_v, k), aligned=at(aligned_v, k)))
            end
        end
        for (sub, data) in (("attempts", attempts), ("rounds", rounds_long))
            isempty(data) && continue
            d = joinpath(outdir, sub); mkpath(d)
            CSV.write(joinpath(d, "$(stem).csv"), DataFrame(data))
        end
        println("Wrote $(length(attempts)) attempt rows, $(length(rounds_long)) round rows")
    end
end

outfile = joinpath(outdir, "$(stem).csv")
CSV.write(outfile, row)
# The real row supersedes the snapshot; leaving it would show a finished run as in-flight.
isfile(progress_path) && rm(progress_path; force=true)
println("Wrote $outfile status=$(row.status[1]) wall=$(round(wall_sec; digits=1))s")
end
