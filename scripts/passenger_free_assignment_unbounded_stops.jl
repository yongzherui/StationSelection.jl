"""
    scripts/passenger_free_assignment_unbounded_stops.jl

Unbounded-`max_stops` pricing for the passenger free-assignment scheme.

# Why unbounded is expected to terminate at all

`max_stops` is a tractability crutch, not a modeling requirement -- the target is
no stop limit. The label search stays finite without one:

  - every extension adds `travel > 0` to `time` and to every live station clock;
  - no new clock is created once `time > max_wait_time` (the pickup cutoff);
  - a clock is pruned as soon as it cannot certify anything new, and the test
    `age + travel(current, dest) <= R` is exact (detouring only adds time).

So a label dies once all its clocks age past their ride limits, giving an
*implicit* route-length ceiling of roughly

    (max_wait_time + max_ride_limit) / min_travel_cost

This script reports that predicted ceiling alongside the route lengths actually
observed, which is the interesting comparison: if realized lengths sit far below
the ceiling, dominance and the reduced-cost bound are doing the real work and
unbounded is viable; if they approach it, the search is exploring the full depth
and unbounded will not scale.

# What is measured

Per n: whether pricing terminated by exhaustion (never by timeout), whether CG
certified optimality, longest generated route, label counts, and -- crucially --
whether the unbounded optimum matches a `max_stops`-capped run. `max_stops` is
only a valid crutch if it gives the same answer; where the capped and uncapped
objectives diverge, the cap was cutting off genuinely better routes.

Usage:
    julia --project=. scripts/passenger_free_assignment_unbounded_stops.jl <outdir> [n_stations ...]

Env overrides:
    PFAU_N_PAIRS        default 16
    PFAU_SEED           default 42
    PFAU_N_SCENARIOS    default 1
    PFAU_COMPARE_MS     capped max_stops to compare against, default 4
    PFAU_MAX_VISITS     default 0 == unbounded
    PFAU_CASE_TIME      default 1200
    PFAU_CERT_TIME      default 900
    PFAU_PRICING_TIME   default 120
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFAU_N_PAIRS", "16"))
const SEED = parse(Int, get(ENV, "PFAU_SEED", "42"))
const N_SCENARIOS = parse(Int, get(ENV, "PFAU_N_SCENARIOS", "1"))
const COMPARE_MS = parse(Int, get(ENV, "PFAU_COMPARE_MS", "4"))
const _RAW_VISITS = parse(Int, get(ENV, "PFAU_MAX_VISITS", "0"))
const MAX_VISITS = _RAW_VISITS <= 0 ? typemax(Int) : _RAW_VISITS
const CASE_TIME = parse(Float64, get(ENV, "PFAU_CASE_TIME", "1200"))
const CERT_TIME = parse(Float64, get(ENV, "PFAU_CERT_TIME", "900"))
const PRICING_TIME = parse(Float64, get(ENV, "PFAU_PRICING_TIME", "120"))
const MAX_WALK = 600.0

const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

function build_model_for(n_stations::Int, max_stops::Int, max_visits::Int)
    return AggregateODRouteModel(
        _l_for(n_stations);
        route_regularization_weight = 1.0,
        walk_cost_weight            = 0.1,
        repositioning_time          = 20.0,
        max_walking_distance        = MAX_WALK,
        max_wait_time               = 900.0,
        detour_factor               = 2.0,
        max_stops                   = max_stops,
        max_visits_per_node         = max_visits,
    )
end

"""
Predicted implicit route-length ceiling: how many stops a label can take before
all its clocks necessarily age out. Uses the instance's own min arc cost and the
largest ride limit any assignment carries.
"""
function implicit_length_ceiling(master_data::PassengerFreeAssignmentMasterData)
    finite = [c for c in values(master_data.travel_cost) if isfinite(c) && c > 0]
    isempty(finite) && return missing, missing, missing
    min_travel = minimum(finite)
    max_ride = isempty(master_data.ride_limit) ? 0.0 : maximum(values(master_data.ride_limit))
    ceiling = (master_data.max_wait_time + max_ride) / min_travel
    return min_travel, max_ride, ceiling
end

function run_case(n_stations::Int, results_dir::String)
    l = _l_for(n_stations)
    @printf("=== n_stations=%d p=%d l=%d  UNBOUNDED max_stops (max_visits=%s) ===\n",
        n_stations, N_PAIRS, l, MAX_VISITS == typemax(Int) ? "unbounded" : string(MAX_VISITS))
    flush(stdout)

    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )

    unb_model = build_model_for(n_stations, typemax(Int), MAX_VISITS)
    mapping = create_map(unb_model, data)
    md = create_passenger_free_assignment_master_data(unb_model, data, mapping)
    min_travel, max_ride, ceiling = implicit_length_ceiling(md)
    @printf("  implicit ceiling: min_travel=%.1f max_ride_limit=%.1f -> ~%.0f stops\n",
        min_travel, max_ride, ceiling)
    flush(stdout)

    function attempt(label, model)
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
                ip_time_limit_sec=300.0,
                total_time_limit_sec=CASE_TIME,
                verify_reduced_costs=true,
                verbose=true,
            )
        catch e
            err = sprint(showerror, e)
            showerror(stderr, e, catch_backtrace()); println(stderr)
        end
        @printf("  [%s] wall=%.1fs stop=%s certified=%s lp=%s mip=%s\n",
            label, time() - t0,
            isnothing(res) ? "error" : string(res.cg_stop_reason),
            isnothing(res) ? "-" : string(res.lp_bound_certified),
            isnothing(res) ? "-" : string(round(res.lp_bound; digits=4)),
            isnothing(res) || isnothing(res.mip_objective) ? "-" : string(round(res.mip_objective; digits=4)))
        flush(stdout)
        return res, err, time() - t0
    end

    unb, unb_err, unb_wall = attempt("unbounded", unb_model)
    cap, cap_err, cap_wall = attempt("capped ms$(COMPARE_MS)",
        build_model_for(n_stations, COMPARE_MS, MAX_VISITS == typemax(Int) ? 3 : MAX_VISITS))

    # Did the cap change the answer? Only comparable when BOTH certified.
    both_certified = !isnothing(unb) && !isnothing(cap) &&
        unb.lp_bound_certified && cap.lp_bound_certified
    lp_matches = both_certified ? isapprox(unb.lp_bound, cap.lp_bound; atol=1e-4) : missing
    cap_lp_excess = (!isnothing(unb) && !isnothing(cap)) ? cap.lp_bound - unb.lp_bound : missing

    summary = (
        n_stations = n_stations, n_pairs = N_PAIRS, l = l, seed = SEED,
        n_scenarios = N_SCENARIOS,
        max_visits = MAX_VISITS == typemax(Int) ? -1 : MAX_VISITS,
        implicit_length_ceiling = ceiling,
        min_travel_cost = min_travel, max_ride_limit = max_ride,
        unb_status = isnothing(unb) ? "error" : string(unb.cg_stop_reason),
        unb_error = unb_err,
        unb_certified = isnothing(unb) ? missing : unb.lp_bound_certified,
        unb_lp = isnothing(unb) ? missing : unb.lp_bound,
        unb_mip = isnothing(unb) || isnothing(unb.mip_objective) ? missing : unb.mip_objective,
        unb_iters = isnothing(unb) ? missing : unb.n_cg_iters,
        unb_rounds = isnothing(unb) ? missing : unb.n_rounds,
        unb_columns = isnothing(unb) ? missing : unb.n_columns,
        unb_pricing_sec = isnothing(unb) ? missing : unb.total_pricing_seconds,
        unb_cert_sec = isnothing(unb) ? missing : unb.certification_seconds,
        unb_cert_exhausted = isnothing(unb) ? missing : unb.certification_exhausted,
        unb_labels = isnothing(unb) ? missing : unb.total_labels_generated,
        unb_unserved = isnothing(unb) ? missing : length(unb.unserved_passengers),
        unb_wall = unb_wall,
        cap_ms = COMPARE_MS,
        cap_status = isnothing(cap) ? "error" : string(cap.cg_stop_reason),
        cap_certified = isnothing(cap) ? missing : cap.lp_bound_certified,
        cap_lp = isnothing(cap) ? missing : cap.lp_bound,
        cap_mip = isnothing(cap) || isnothing(cap.mip_objective) ? missing : cap.mip_objective,
        cap_pricing_sec = isnothing(cap) ? missing : cap.total_pricing_seconds,
        cap_labels = isnothing(cap) ? missing : cap.total_labels_generated,
        cap_wall = cap_wall,
        both_certified = both_certified,
        lp_matches_capped = lp_matches,
        capped_lp_excess = cap_lp_excess,
    )
    try
        CSV.write(joinpath(results_dir, "pfau_n$(n_stations)_p$(N_PAIRS)_s$(SEED).csv"), DataFrame([summary]))
    catch e
        @warn "summary CSV write failed" exception=(e, catch_backtrace())
    end

    if both_certified
        println(lp_matches ?
            "  => capped ms$(COMPARE_MS) gives the SAME certified LP bound as unbounded" :
            "  => capped ms$(COMPARE_MS) LP bound differs by $(cap_lp_excess) -- the cap CHANGED the answer")
    else
        println("  => not directly comparable (one side uncertified)")
    end
    println()
    return summary
end

function main()
    length(ARGS) >= 1 || error("usage: passenger_free_assignment_unbounded_stops.jl <outdir> [n_stations ...]")
    outdir = abspath(ARGS[1])
    ns = length(ARGS) >= 2 ? parse.(Int, ARGS[2:end]) : [8, 10, 12, 15]
    results_dir = joinpath(outdir, "results")
    mkpath(results_dir)

    println("unbounded-stops study: n=$(ns) p=$N_PAIRS seed=$SEED scenarios=$N_SCENARIOS")
    println("compare against capped ms=$COMPARE_MS; case budget $(CASE_TIME)s, cert $(CERT_TIME)s")
    println()

    rows = NamedTuple[]
    for n in ns
        push!(rows, run_case(n, results_dir))
    end
    try
        CSV.write(joinpath(outdir, "combined_results.csv"), DataFrame(rows))
    catch e
        @warn "combined CSV write failed" exception=(e, catch_backtrace())
    end

    println("=== summary ===")
    @printf("%5s %10s %10s %10s %12s %12s %10s %10s\n",
        "n", "ceiling", "unb_cert", "unb_wall", "unb_pricing", "unb_labels", "cap_cert", "lp_same")
    for r in rows
        @printf("%5d %10s %10s %10.1f %12s %12s %10s %10s\n",
            r.n_stations,
            r.implicit_length_ceiling isa Real ? @sprintf("%.0f", r.implicit_length_ceiling) : "-",
            string(r.unb_certified), r.unb_wall,
            r.unb_pricing_sec isa Real ? @sprintf("%.1f", r.unb_pricing_sec) : "-",
            string(r.unb_labels), string(r.cap_certified), string(r.lp_matches_capped))
    end
end

main()
