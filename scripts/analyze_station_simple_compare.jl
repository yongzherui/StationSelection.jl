"""
    scripts/analyze_station_simple_compare.jl

Head-to-head comparison of the station-simple (elementary-route) pricer against
the normal (revisit-tolerant, station-age) pricer for BendersYZ zero_completion,
using the per-(instance, method) summary CSVs written by run_method_compare_task.jl
into the station-simple-compare experiment directory (see
generate_station_simple_compare_job_list.jl / submit_station_simple_compare.sh).

IMPORTANT correctness caveat this script checks for: station-simple pricing only
ever considers ELEMENTARY routes (no station revisited). If the true optimum for
some instance genuinely needs a revisiting route, the station-simple run isn't
just "faster or slower for the same answer" -- it may converge to a strictly worse
(or otherwise different) objective, because its column universe is a strict
subset of the normal pricer's. So this reports objective agreement PER PAIR
first, and only compares iteration counts within pairs that agree -- an iteration
win on a pair with a worse objective is not a real speed win, it's pricing an
easier (smaller) problem.

Usage:
    julia --project=. scripts/analyze_station_simple_compare.jl [base_outdir]

Default base_outdir:
    experiments/aggregate_od_route_station_simple_compare
"""

using CSV, DataFrames, Statistics, Printf

function main()
    base_outdir = length(ARGS) >= 1 ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "aggregate_od_route_station_simple_compare")
    results_dir = joinpath(base_outdir, "results")
    isdir(results_dir) || error("no results directory at $results_dir -- run some tasks first")

    files = filter(f -> endswith(f, ".csv"), readdir(results_dir; join=true))
    isempty(files) && error("no result CSVs found in $results_dir")

    df = vcat([CSV.read(f, DataFrame; stringtype=String) for f in files]...; cols=:union)
    analysis_dir = joinpath(base_outdir, "analysis")
    mkpath(analysis_dir)
    combined_path = joinpath(analysis_dir, "combined_results.csv")
    CSV.write(combined_path, df)
    println("Combined $(nrow(df)) rows from $(length(files)) files -> $combined_path")
    println()

    df.obj_num = [
        (row.status == "ok" && !ismissing(row.objective_value) && row.objective_value != "") ?
            parse(Float64, string(row.objective_value)) : missing
        for row in eachrow(df)
    ]
    df.n_iter_num = [
        (!ismissing(row.n_iterations) && string(row.n_iterations) != "") ?
            tryparse(Float64, string(row.n_iterations)) : missing
        for row in eachrow(df)
    ]
    df.inner_cg_iter_num = [
        (!ismissing(row.inner_cg_iterations) && string(row.inner_cg_iterations) != "") ?
            tryparse(Float64, string(row.inner_cg_iterations)) : missing
        for row in eachrow(df)
    ]
    df.is_ss = [endswith(string(row.method), "_ss") for row in eachrow(df)]
    df.base_method = [
        row.is_ss ? string(row.method)[1:end-length("_ss")] : string(row.method)
        for row in eachrow(df)
    ]

    n_failed = count(s -> startswith(string(s), "error"), df.status)
    n_failed > 0 && println("WARNING: $n_failed / $(nrow(df)) rows failed (status starts with \"error\") -- excluded below.")
    println()

    pairs = NamedTuple[]
    for g in groupby(filter(row -> row.status == "ok", df), [:instance, :base_method])
        normal_rows = filter(row -> !row.is_ss, g)
        ss_rows = filter(row -> row.is_ss, g)
        (isempty(normal_rows) || isempty(ss_rows)) && continue
        normal = only(eachrow(normal_rows))
        ss = only(eachrow(ss_rows))
        push!(pairs, (
            instance=normal.instance, base_method=normal.base_method,
            obj_normal=normal.obj_num, obj_ss=ss.obj_num,
            iters_normal=normal.n_iter_num, iters_ss=ss.n_iter_num,
            inner_cg_normal=normal.inner_cg_iter_num, inner_cg_ss=ss.inner_cg_iter_num,
            wall_normal=normal.wall_time_sec, wall_ss=ss.wall_time_sec,
        ))
    end
    isempty(pairs) && error("no (normal, _ss) pairs with matching (instance, base_method) and status==ok found")
    pairdf = DataFrame(pairs)

    println("=== Objective agreement (station-simple's elementary-only column universe can miss the true optimum) ===")
    pairdf.obj_gap_pct = [
        (ismissing(r.obj_normal) || ismissing(r.obj_ss)) ? missing :
            100.0 * (r.obj_ss - r.obj_normal) / max(1.0, abs(r.obj_normal))
        for r in eachrow(pairdf)
    ]
    agreeing = filter(r -> !ismissing(r.obj_gap_pct) && abs(r.obj_gap_pct) <= 1e-2, pairdf)
    disagreeing = filter(r -> !ismissing(r.obj_gap_pct) && abs(r.obj_gap_pct) > 1e-2, pairdf)
    println("  $(nrow(agreeing))/$(nrow(pairdf)) pairs agree (|gap| <= 0.01%); $(nrow(disagreeing)) disagree")
    for r in eachrow(sort(disagreeing, :obj_gap_pct; rev=true))
        @printf("  DISAGREE  %-24s %-28s  normal=%12.4f  ss=%12.4f  gap=%+.2f%%\n",
                r.instance, r.base_method, r.obj_normal, r.obj_ss, r.obj_gap_pct)
    end
    println()

    println("=== Outer Benders iteration count: station-simple vs normal (agreeing-objective pairs only) ===")
    iter_pairs = filter(r -> !ismissing(r.iters_normal) && !ismissing(r.iters_ss), agreeing)
    if isempty(iter_pairs)
        println("  no agreeing pairs with both iteration counts present")
    else
        deltas = iter_pairs.iters_ss .- iter_pairs.iters_normal
        n_ss_fewer = count(<(0), deltas)
        n_ss_more = count(>(0), deltas)
        n_tied = count(==(0), deltas)
        @printf("  station-simple fewer outer iters: %d   more: %d   tied: %d   (n=%d)\n",
                n_ss_fewer, n_ss_more, n_tied, length(deltas))
        @printf("  mean(iters_ss - iters_normal) = %+.2f   median = %+.1f\n", mean(deltas), median(deltas))
        for r in eachrow(sort(iter_pairs, :instance))
            @printf("    %-24s %-28s  normal=%5.0f  ss=%5.0f  Δ=%+4.0f\n",
                    r.instance, r.base_method, r.iters_normal, r.iters_ss, r.iters_ss - r.iters_normal)
        end
    end
    println()

    println("=== Cumulative inner CG iteration count: station-simple vs normal (agreeing-objective pairs only) ===")
    inner_pairs = filter(r -> !ismissing(r.inner_cg_normal) && !ismissing(r.inner_cg_ss), agreeing)
    if isempty(inner_pairs)
        println("  no agreeing pairs with both inner_cg_iterations present")
    else
        deltas = inner_pairs.inner_cg_ss .- inner_pairs.inner_cg_normal
        n_ss_fewer = count(<(0), deltas)
        n_ss_more = count(>(0), deltas)
        n_tied = count(==(0), deltas)
        @printf("  station-simple fewer inner CG iters: %d   more: %d   tied: %d   (n=%d)\n",
                n_ss_fewer, n_ss_more, n_tied, length(deltas))
        @printf("  mean(inner_cg_ss - inner_cg_normal) = %+.2f   median = %+.1f\n", mean(deltas), median(deltas))
    end
    println()

    println("=== Wall-clock: station-simple vs normal (agreeing-objective pairs only) ===")
    wall_pairs = filter(r -> !ismissing(r.wall_normal) && !ismissing(r.wall_ss), agreeing)
    if !isempty(wall_pairs)
        deltas = wall_pairs.wall_ss .- wall_pairs.wall_normal
        @printf("  mean(wall_ss - wall_normal) = %+.2fs   median = %+.2fs   (n=%d)\n",
                mean(deltas), median(deltas), length(deltas))
    end
    println()

    pairs_path = joinpath(analysis_dir, "station_simple_vs_normal_pairs.csv")
    CSV.write(pairs_path, pairdf)
    println("Per-pair comparison written to $pairs_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
