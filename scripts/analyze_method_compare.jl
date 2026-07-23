"""
    scripts/analyze_method_compare.jl

Aggregate all per-(instance, method) summary CSVs from the AggregateODRouteModel
method comparison experiment into one table, and report:
  - objective-value agreement across methods, per instance (correctness check --
    Direct solve and every Benders decomposition should agree exactly since
    they are all exact for the same max_stops setting; plain CG is a heuristic
    and may land on a strictly worse objective)
  - wall-clock runtime by method
  - Benders outer-iteration counts (and plain-CG iteration counts) by method

Usage:
    julia --project=. scripts/analyze_method_compare.jl [base_outdir]

Default base_outdir:
    experiments/aggregate_od_route_method_compare
"""

using CSV, DataFrames, Statistics, Printf

function main()
    base_outdir = length(ARGS) >= 1 ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "aggregate_od_route_method_compare")
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

    # Three tiers, not two:
    #   1. "Provably exact"  = Direct, plus every Benders variant WITH repricing
    #      (reprice_subproblem=true). These must always agree exactly -- repricing
    #      is what compare_benders_decompositions.jl's docstring says is required
    #      for BendersY/BendersYZ to be provably optimal, and is also needed for
    #      BendersYZH since its "exact without repricing" argument is unproven
    #      under subproblem dual degeneracy.
    #   2. "Benders w/o repricing" (reprice_subproblem=false: std_noreprice,
    #      zerocomp, mw) -- NOT provably exact. May legitimately land on a worse
    #      objective on instances with subproblem dual degeneracy. A gap here is
    #      an expected research finding (repricing's cost/benefit), not a bug.
    #   3. cg_ms4/cg_uncapped -- a heuristic, expected to be worse than #1.
    # Lumping #2 in with #1 (as an earlier version of this script did) makes a
    # real known-and-documented gap look like a correctness bug.
    provably_exact_df = filter(row -> row.kind == "direct" || (row.kind == "benders" && row.reprice_subproblem == true), df)
    unrepriced_benders_df = filter(row -> row.kind == "benders" && row.reprice_subproblem == false, df)

    println("=== Objective value agreement among PROVABLY EXACT methods (Direct + repriced Benders), per instance x max_stops_mode ===")
    mismatches = 0
    for g in groupby(provably_exact_df, [:instance, :max_stops_mode])
        objs = collect(skipmissing(g.obj_num))
        isempty(objs) && continue
        spread = maximum(objs) - minimum(objs)
        rel = spread / max(1.0, maximum(abs, objs))
        flag = rel > 1e-4 ? "  <-- MISMATCH (real bug candidate)" : ""
        rel > 1e-4 && (mismatches += 1)
        @printf("  %-24s %-10s  n=%-3d min=%12.4f max=%12.4f rel_spread=%.2e%s\n",
                g.instance[1], g.max_stops_mode[1], length(objs), minimum(objs), maximum(objs), rel, flag)
    end
    println()
    println(mismatches == 0 ?
        "No objective mismatches among provably exact methods (Direct + repriced Benders)." :
        "$mismatches instance/max_stops_mode group(s) with mismatched objectives among PROVABLY EXACT methods -- investigate, this should never happen.")
    println()

    println("=== Non-repriced Benders gap vs best provably-exact objective (expected on dual-degenerate instances, not a bug) ===")
    for g in groupby(df, [:instance, :max_stops_mode])
        unrepriced_rows = filter(row -> row.kind == "benders" && row.reprice_subproblem == false, g)
        exact_objs = collect(skipmissing(filter(row -> row.kind == "direct" || (row.kind == "benders" && row.reprice_subproblem == true), g).obj_num))
        isempty(unrepriced_rows) && continue
        isempty(exact_objs) && continue
        best_exact = minimum(exact_objs)
        for row in eachrow(unrepriced_rows)
            ismissing(row.obj_num) && continue
            gap_pct = 100.0 * (row.obj_num - best_exact) / max(1.0, best_exact)
            gap_pct > 1e-2 && @printf("  %-24s %-10s  %-30s obj=%12.4f exact=%12.4f gap=%.2f%%\n",
                    g.instance[1], g.max_stops_mode[1], row.method, row.obj_num, best_exact, gap_pct)
        end
    end
    println()

    println("=== CG heuristic gap vs best provably-exact objective, per instance x max_stops_mode ===")
    for g in groupby(df, [:instance, :max_stops_mode])
        cg_rows = filter(row -> row.kind == "cg", g)
        exact_objs = collect(skipmissing(filter(row -> row.kind == "direct" || (row.kind == "benders" && row.reprice_subproblem == true), g).obj_num))
        isempty(cg_rows) && continue
        isempty(exact_objs) && continue
        best_exact = minimum(exact_objs)
        for cg_row in eachrow(cg_rows)
            ismissing(cg_row.obj_num) && continue
            gap_pct = 100.0 * (cg_row.obj_num - best_exact) / max(1.0, best_exact)
            @printf("  %-24s %-10s  %-14s cg=%12.4f exact=%12.4f gap=%.1f%%\n",
                    g.instance[1], g.max_stops_mode[1], cg_row.method, cg_row.obj_num, best_exact, gap_pct)
        end
    end
    println()

    println("=== Wall-clock runtime by method (seconds) ===")
    for g in sort(collect(groupby(df, :method)), by=g -> g.method[1])
        times = collect(skipmissing(g.wall_time_sec))
        isempty(times) && continue
        @printf("  %-28s  n=%-4d  mean=%9.1f  median=%9.1f  max=%9.1f\n",
                g.method[1], length(times), mean(times), median(times), maximum(times))
    end
    println()

    println("=== Iteration counts (Benders outer / plain CG) by method ===")
    for g in sort(collect(groupby(df, :method)), by=g -> g.method[1])
        vals = [tryparse(Float64, string(v)) for v in g.n_iterations if !ismissing(v) && string(v) != ""]
        iters = collect(skipmissing(vals))
        isempty(iters) && continue
        @printf("  %-28s  n=%-4d  mean=%7.1f  median=%7.1f  max=%7.0f\n",
                g.method[1], length(iters), mean(iters), median(iters), maximum(iters))
    end
    println()

    # Collapse cut_derivation/reprice/max_stops_mode variants into one row per
    # (family, n_stations, decomposition-or-kind) -- scaling trend is the point here,
    # not every individual method's number (see the per-method table above for that).
    println("=== Wall-clock runtime by family x n_stations x decomposition (seconds) ===")
    df.method_group = [
        row.kind == "benders" ? row.decomposition : row.kind
        for row in eachrow(df)
    ]
    for g in sort(collect(groupby(df, [:family, :n_stations, :method_group])),
                  by=g -> (g.family[1], g.n_stations[1], g.method_group[1]))
        times = collect(skipmissing(g.wall_time_sec))
        isempty(times) && continue
        @printf("  %-8s n=%-4d %-12s  n_runs=%-4d  mean=%9.1f  median=%9.1f  max=%9.1f\n",
                g.family[1], g.n_stations[1], g.method_group[1], length(times),
                mean(times), median(times), maximum(times))
    end
    println()

    n_failed = count(s -> startswith(string(s), "error"), df.status)
    n_failed > 0 && println("WARNING: $n_failed / $(nrow(df)) rows failed (status starts with \"error\") -- see iters_log_path / slurm_logs.")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
