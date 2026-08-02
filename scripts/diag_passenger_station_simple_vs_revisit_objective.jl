"""
    scripts/diag_passenger_station_simple_vs_revisit_objective.jl

How far does the elementary (station-simple) pricer take us before we need the
exact (revisit-tolerant) pricer? Runs the full passenger free-assignment column
generation TWICE on the same model+data -- once with `use_station_simple=true`
(prices only elementary routes) and once with `use_station_simple=false` (the
revisit-tolerant pricer) -- and reports the OBJECTIVE difference.

The station-simple pool is a subset of the revisit-tolerant pool (elementary ⊂ all
routes), so for this minimisation its LP and MIP optima are >= (no better than) the
revisit-tolerant ones. The gap `ss - rev` is exactly what station-simple leaves on
the table, i.e. what a subsequent exact-pricing phase would still have to recover.

Only the LP bounds that are CERTIFIED (`cg_stop_reason == :optimality_proven`) are
true optima; an uncertified side is only a restricted-pool bound, flagged as such.

Usage (one station count per invocation, so an array can run them concurrently):
    julia --project=. scripts/diag_passenger_station_simple_vs_revisit_objective.jl <n_stations> [outdir]

Env overrides:
    PFASS_N_PAIRS      default 16
    PFASS_SEED         default 42
    PFASS_N_SCENARIOS  default 3
    PFASS_MAX_STOPS    default 5   (0 => unbounded)
    PFASS_MAX_VISITS   default 3   (revisit-tolerant only; station-simple ignores it)
    PFASS_PRICING_TIME default 120
    PFASS_CERT_TIME    default 600
    PFASS_IP_TIME      default 300
    PFASS_CASE_TIME    default 1800
"""

using CSV, DataFrames, Gurobi, Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFASS_N_PAIRS", "16"))
const SEED = parse(Int, get(ENV, "PFASS_SEED", "42"))
const N_SCENARIOS = parse(Int, get(ENV, "PFASS_N_SCENARIOS", "3"))
const _RAW_MS = parse(Int, get(ENV, "PFASS_MAX_STOPS", "5"))
const MAX_STOPS = _RAW_MS <= 0 ? typemax(Int) : _RAW_MS
const MAX_VISITS = parse(Int, get(ENV, "PFASS_MAX_VISITS", "3"))
const PRICING_TIME = parse(Float64, get(ENV, "PFASS_PRICING_TIME", "120"))
const CERT_TIME = parse(Float64, get(ENV, "PFASS_CERT_TIME", "600"))
const IP_TIME = parse(Float64, get(ENV, "PFASS_IP_TIME", "300"))
const CASE_TIME = parse(Float64, get(ENV, "PFASS_CASE_TIME", "1800"))
const MAX_WALK = 600.0

const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

function build_model_for(n_stations::Int)
    return AggregateODRouteModel(
        _l_for(n_stations);
        route_regularization_weight = 1.0,
        walk_cost_weight            = 0.1,
        repositioning_time          = 20.0,
        max_walking_distance        = MAX_WALK,
        max_wait_time               = 900.0,
        detour_factor               = 2.0,
        max_stops                   = MAX_STOPS,
        max_visits_per_node         = MAX_VISITS,
    )
end

function attempt(label, model, data, use_station_simple; warm_start=false, reward_levels=0)
    t0 = time()
    res = nothing
    err = ""
    try
        res = run_passenger_free_assignment_column_generation(
            model, data;
            optimizer_env=GRB_ENV,
            max_cg_iters=5000,
            n_candidates=20, max_new_columns=20,
            pricing_time_limit_sec=PRICING_TIME,
            certification_time_limit_sec=CERT_TIME,
            ip_time_limit_sec=IP_TIME,
            total_time_limit_sec=CASE_TIME,
            use_station_simple=use_station_simple,
            station_simple_warm_start=warm_start,
            reward_coarsening_levels=reward_levels,
            verify_reduced_costs=true,
            verbose=false,
        )
    catch e
        err = sprint(showerror, e)
        showerror(stderr, e, catch_backtrace()); println(stderr)
    end
    wall = time() - t0
    @printf("  [%-14s] wall=%7.1fs stop=%-22s certified=%-5s lp=%14s mip=%14s cols=%s\n",
        label, wall,
        isnothing(res) ? "error" : string(res.cg_stop_reason),
        isnothing(res) ? "-" : string(res.lp_bound_certified),
        isnothing(res) ? "-" : @sprintf("%.4f", res.lp_bound),
        isnothing(res) || isnothing(res.mip_objective) ? "-" : @sprintf("%.4f", res.mip_objective),
        isnothing(res) ? "-" : string(res.n_columns))
    flush(stdout)
    return res, err, wall
end

_gap(a, b) = (a isa Real && b isa Real) ? a - b : missing
_gap_pct(a, b) = (a isa Real && b isa Real && abs(b) > 1e-9) ? 100 * (a - b) / abs(b) : missing

function run_case(n_stations::Int, results_dir::String)
    l = _l_for(n_stations)
    @printf("=== n=%d p=%d scenarios=%d l=%d ms=%s max_visits=%d ===\n",
        n_stations, N_PAIRS, N_SCENARIOS, l,
        MAX_STOPS == typemax(Int) ? "unb" : string(MAX_STOPS), MAX_VISITS)
    flush(stdout)

    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )

    ss, ss_err, ss_wall = attempt("station_simple", build_model_for(n_stations), data, true)
    coarse, coarse_err, coarse_wall = attempt(
        "reward_L2", build_model_for(n_stations), data, false; reward_levels=2,
    )
    rev, rev_err, rev_wall = attempt("revisit", build_model_for(n_stations), data, false)
    ws, ws_err, ws_wall = attempt("warm_start", build_model_for(n_stations), data, false; warm_start=true)

    ss_lp = isnothing(ss) ? missing : ss.lp_bound
    coarse_lp = isnothing(coarse) ? missing : coarse.lp_bound
    rev_lp = isnothing(rev) ? missing : rev.lp_bound
    ws_lp = isnothing(ws) ? missing : ws.lp_bound
    ss_mip = isnothing(ss) || isnothing(ss.mip_objective) ? missing : ss.mip_objective
    coarse_mip = isnothing(coarse) || isnothing(coarse.mip_objective) ? missing : coarse.mip_objective
    rev_mip = isnothing(rev) || isnothing(rev.mip_objective) ? missing : rev.mip_objective
    ws_mip = isnothing(ws) || isnothing(ws.mip_objective) ? missing : ws.mip_objective
    both_lp_certified = !isnothing(ss) && !isnothing(rev) &&
        ss.lp_bound_certified && rev.lp_bound_certified
    # Warm start should reach the SAME certified optimum as pure revisit -- ideally faster.
    ws_matches_rev = !isnothing(ws) && !isnothing(rev) && ws.lp_bound_certified &&
        rev.lp_bound_certified && isapprox(ws_lp, rev_lp; atol=1e-4)
    ws_speedup = (rev_wall > 0 && ws_wall > 0) ? rev_wall / ws_wall : missing
    coarse_matches_rev = !isnothing(coarse) && !isnothing(rev) && coarse.lp_bound_certified &&
        rev.lp_bound_certified && isapprox(coarse_lp, rev_lp; atol=1e-4)
    coarse_harvest_speedup = !isnothing(coarse) && !isnothing(rev) &&
        coarse.total_pricing_seconds > 0 ?
        rev.total_pricing_seconds / coarse.total_pricing_seconds : missing

    lp_gap = _gap(ss_lp, rev_lp)
    lp_gap_pct = _gap_pct(ss_lp, rev_lp)
    mip_gap = _gap(ss_mip, rev_mip)
    mip_gap_pct = _gap_pct(ss_mip, rev_mip)

    @printf("  OBJGAP\tn=%d\tlp_gap=%s\tlp_gap_pct=%s\tmip_gap=%s\tmip_gap_pct=%s\tboth_lp_certified=%s\n",
        n_stations,
        lp_gap isa Real ? @sprintf("%.4f", lp_gap) : "-",
        lp_gap_pct isa Real ? @sprintf("%.3f%%", lp_gap_pct) : "-",
        mip_gap isa Real ? @sprintf("%.4f", mip_gap) : "-",
        mip_gap_pct isa Real ? @sprintf("%.3f%%", mip_gap_pct) : "-",
        string(both_lp_certified))
    @printf("  WARMSTART\tn=%d\tws_wall=%.1f\trev_wall=%.1f\tws_speedup=%s\tws_matches_rev=%s\tws_lp=%s\trev_lp=%s\n",
        n_stations, ws_wall, rev_wall,
        ws_speedup isa Real ? @sprintf("%.2f", ws_speedup) : "-",
        string(ws_matches_rev),
        ws_lp isa Real ? @sprintf("%.4f", ws_lp) : "-",
        rev_lp isa Real ? @sprintf("%.4f", rev_lp) : "-")
    @printf("  REWARDL2\tn=%d\tcoarse_wall=%.1f\trev_wall=%.1f\tpricing_speedup=%s\tmatches_rev=%s\tcoarse_lp=%s\trev_lp=%s\n",
        n_stations, coarse_wall, rev_wall,
        coarse_harvest_speedup isa Real ? @sprintf("%.2f", coarse_harvest_speedup) : "-",
        string(coarse_matches_rev),
        coarse_lp isa Real ? @sprintf("%.4f", coarse_lp) : "-",
        rev_lp isa Real ? @sprintf("%.4f", rev_lp) : "-")
    flush(stdout)

    summary = (
        n_stations=n_stations, n_pairs=N_PAIRS, n_scenarios=N_SCENARIOS, l=l, seed=SEED,
        max_stops=(MAX_STOPS == typemax(Int) ? -1 : MAX_STOPS), max_visits=MAX_VISITS,
        ss_status=isnothing(ss) ? "error" : string(ss.cg_stop_reason),
        ss_certified=isnothing(ss) ? missing : ss.lp_bound_certified,
        ss_lp=ss_lp, ss_mip=ss_mip,
        ss_columns=isnothing(ss) ? missing : ss.n_columns,
        ss_wall=ss_wall, ss_error=ss_err,
        coarse_status=isnothing(coarse) ? "error" : string(coarse.cg_stop_reason),
        coarse_certified=isnothing(coarse) ? missing : coarse.lp_bound_certified,
        coarse_lp=coarse_lp, coarse_mip=coarse_mip,
        coarse_columns=isnothing(coarse) ? missing : coarse.n_columns,
        coarse_pricing_sec=isnothing(coarse) ? missing : coarse.total_pricing_seconds,
        coarse_wall=coarse_wall, coarse_error=coarse_err,
        rev_status=isnothing(rev) ? "error" : string(rev.cg_stop_reason),
        rev_certified=isnothing(rev) ? missing : rev.lp_bound_certified,
        rev_lp=rev_lp, rev_mip=rev_mip,
        rev_columns=isnothing(rev) ? missing : rev.n_columns,
        rev_wall=rev_wall, rev_error=rev_err,
        ws_status=isnothing(ws) ? "error" : string(ws.cg_stop_reason),
        ws_certified=isnothing(ws) ? missing : ws.lp_bound_certified,
        ws_lp=ws_lp, ws_mip=ws_mip,
        ws_columns=isnothing(ws) ? missing : ws.n_columns,
        ws_pricing_sec=isnothing(ws) ? missing : ws.total_pricing_seconds,
        ws_wall=ws_wall, ws_error=ws_err,
        lp_gap=lp_gap, lp_gap_pct=lp_gap_pct,
        mip_gap=mip_gap, mip_gap_pct=mip_gap_pct,
        both_lp_certified=both_lp_certified,
        ws_matches_rev=ws_matches_rev, ws_speedup=ws_speedup,
        coarse_matches_rev=coarse_matches_rev,
        coarse_harvest_speedup=coarse_harvest_speedup,
    )
    try
        CSV.write(joinpath(results_dir, "pfass_n$(n_stations)_p$(N_PAIRS)_s$(SEED).csv"), DataFrame([summary]))
    catch e
        @warn "summary CSV write failed" exception=(e, catch_backtrace())
    end
    println()
    return summary
end

function main()
    length(ARGS) >= 1 || error("usage: diag_passenger_station_simple_vs_revisit_objective.jl <n_stations> [outdir]")
    n = parse(Int, ARGS[1])
    outdir = length(ARGS) >= 2 ? abspath(ARGS[2]) : pwd()
    results_dir = joinpath(outdir, "results")
    mkpath(results_dir)
    run_case(n, results_dir)
end

main()
