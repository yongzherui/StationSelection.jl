"""
    diagnose.jl cg_iteration_profile -- full column-generation run instrumented
    per iteration: how does time-per-iteration (RMP LP solve + pricing) grow as
    the column pool accumulates? Unlike the station-subset diagnostics (a single
    seeded pricing snapshot), this runs the real CG loop and dumps its
    per-iteration telemetry.

Usage:
    julia --project=. scripts/diagnose.jl cg_iteration_profile <n_stations> <output.csv>

Env overrides (all prefixed PFACG_):
    N_PAIRS default 16   N_SCENARIOS default 1   MAX_STOPS default 7 (0 => unbounded)
    MAX_ITERS default 400   TOTAL_TIME default 3000   CERT_TIME default 600
    PRICING_TIME default 60   IP_TIME default 600
"""

function run_cg_iteration_profile(args::Vector{String})
    length(args) == 2 || error("usage: diagnose.jl cg_iteration_profile <n_stations> <output.csv>")
    n = parse(Int, args[1]); output = args[2]

    n_pairs = env_int("PFACG_N_PAIRS", 16)
    n_scen = env_int("PFACG_N_SCENARIOS", 1)
    max_stops = diag_unbounded(env_int("PFACG_MAX_STOPS", 7))
    max_cg_iters = env_int("PFACG_MAX_ITERS", 400)
    total_time = env_float("PFACG_TOTAL_TIME", 3000.0)
    cert_time = env_float("PFACG_CERT_TIME", 600.0)
    pricing_time = env_float("PFACG_PRICING_TIME", 60.0)
    ip_time = env_float("PFACG_IP_TIME", 600.0)

    l = diag_l_for(n)
    data, _ = diag_zz_data(n; n_pairs=n_pairs, n_scenarios=n_scen, seed=42)
    model = diag_zz_model(n; l=l, max_stops=max_stops)
    @printf("CONFIG n_stations=%d L=%d n_pairs=%d n_scenarios=%d max_stops=%s\n",
        n, l, n_pairs, n_scen, max_stops == typemax(Int) ? "inf" : string(max_stops))
    flush(stdout)

    result = run_passenger_free_assignment_column_generation(model, data;
        optimizer_env=diag_grb_env(), max_cg_iters=max_cg_iters,
        total_time_limit_sec=total_time, certification_time_limit_sec=cert_time,
        pricing_time_limit_sec=pricing_time, ip_time_limit_sec=ip_time, verbose=true)

    rows = NamedTuple[]
    for r in result.iteration_rows
        push!(rows, (; n_stations=n, L=l, n_pairs=n_pairs, n_scenarios=n_scen,
            max_stops=(max_stops == typemax(Int) ? -1 : max_stops),
            round=r.round, iteration=r.iteration, phase=r.phase,
            lp_bound=r.lp_bound, lp_seconds=r.lp_seconds,
            pricing_seconds=r.pricing_seconds, iter_seconds=r.lp_seconds + r.pricing_seconds,
            labels_generated=r.labels_generated, columns_priced=r.columns_priced,
            columns_added=r.columns_added, pool_size=r.pool_size))
    end
    diag_safe_csv_write(output, DataFrame(rows)); flush(stdout)

    certified = result.cg_stop_reason == :optimality_proven
    summary = (; n_stations=n, L=l, n_pairs=n_pairs, n_scenarios=n_scen,
        max_stops=(max_stops == typemax(Int) ? -1 : max_stops),
        n_passengers=result.n_passengers, status=result.status,
        cg_stop_reason=result.cg_stop_reason, certified=certified,
        lp_bound=result.lp_bound, lp_bound_certified=result.lp_bound_certified,
        mip_objective=(isnothing(result.mip_objective) ? NaN : result.mip_objective),
        certification_exhausted=result.certification_exhausted,
        n_cg_iters=result.n_cg_iters, n_rounds=result.n_rounds, n_columns=result.n_columns,
        n_master_rows=result.n_master_rows,
        total_lp_seconds=result.total_lp_seconds,
        total_pricing_seconds=result.total_pricing_seconds,
        certification_seconds=result.certification_seconds,
        total_labels_generated=result.total_labels_generated,
        total_seconds=result.total_seconds,
        sec_per_iter=(result.n_cg_iters > 0 ?
            (result.total_lp_seconds + result.total_pricing_seconds) / result.n_cg_iters : NaN))
    diag_safe_csv_write(replace(output, ".csv" => ".summary.csv"), DataFrame([summary])); flush(stdout)
    @printf("DONE n=%d scen=%d certified=%s stop=%s cg_iters=%d columns=%d lp_bound=%.2f mip=%.2f total=%.1fs\n",
        n, n_scen, string(certified), string(result.cg_stop_reason), result.n_cg_iters,
        result.n_columns, result.lp_bound,
        isnothing(result.mip_objective) ? NaN : result.mip_objective, result.total_seconds)
end

register_mode!("cg_iteration_profile", run_cg_iteration_profile)
