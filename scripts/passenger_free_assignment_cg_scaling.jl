"""
    scripts/passenger_free_assignment_cg_scaling.jl

Scaling study for the passenger free-assignment column-generation scheme.

Runs the full CG loop (early-return pricing rounds + exhaustive certification)
across a grid of `n_stations`, recording where time goes and where the scheme
stops being able to certify. One CSV row per (n_stations, n_pairs, seed) case,
plus a per-iteration CSV so the pricing-cost growth is inspectable.

Columns of interest for "how does this pricer scale":
  - `total_pricing_seconds`   -- all label-search time (early-return + certification)
  - `certification_seconds`   -- the exhaustive pass alone, which is the part that
                                 grows worst since it cannot stop early
  - `total_labels_generated`  -- label-search work, independent of machine speed
  - `n_master_rows`           -- disaggregated (p,j)/(p,k) linking rows, the master's
                                 own size driver
  - `cg_stop_reason` / `lp_bound_certified` -- whether optimality was actually proven,
                                 which is the thing that fails first at scale

Usage:
    julia --project=. scripts/passenger_free_assignment_cg_scaling.jl <outdir> [n_stations ...]

Env overrides:
    PFA_N_PAIRS         default 16
    PFA_SEEDS           comma-separated, default "42"
    PFA_N_SCENARIOS     default 1
    PFA_MAX_STOPS       default 4
    PFA_N_CANDIDATES    default 20 (early-return phase only)
    PFA_PRICING_TIME    default 60 (per early-return pricing call, seconds)
    PFA_CERT_TIME       default 900 (exhaustive certification budget, seconds)
    PFA_IP_TIME         default 600
    PFA_MAX_CG_ITERS    default 2000
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFA_N_PAIRS", "16"))
const SEEDS = parse.(Int, split(get(ENV, "PFA_SEEDS", "42"), ','))
const N_SCENARIOS = parse(Int, get(ENV, "PFA_N_SCENARIOS", "1"))
# `PFA_MAX_STOPS=0` means TRUE unbounded: `typemax(Int)`, the model's own no-limit
# sentinel. That is NOT the same as a large finite cap -- a finite value (even
# 10^6) sets `bounded_max_stops=true`, which makes label dominance additionally
# compare `route_length` and is materially SLOWER than the genuinely uncapped
# path. Same for `PFA_MAX_VISITS=0`.
const _RAW_MAX_STOPS = parse(Int, get(ENV, "PFA_MAX_STOPS", "4"))
const MAX_STOPS = _RAW_MAX_STOPS <= 0 ? typemax(Int) : _RAW_MAX_STOPS
const _RAW_MAX_VISITS = parse(Int, get(ENV, "PFA_MAX_VISITS", "3"))
const MAX_VISITS = _RAW_MAX_VISITS <= 0 ? typemax(Int) : _RAW_MAX_VISITS
const N_CANDIDATES = parse(Int, get(ENV, "PFA_N_CANDIDATES", "20"))
const PRICING_TIME = parse(Float64, get(ENV, "PFA_PRICING_TIME", "60"))
const CERT_TIME = parse(Float64, get(ENV, "PFA_CERT_TIME", "900"))
const IP_TIME = parse(Float64, get(ENV, "PFA_IP_TIME", "600"))
const MAX_CG_ITERS = parse(Int, get(ENV, "PFA_MAX_CG_ITERS", "2000"))
# Per-case budget for the CG phases, so one slow size cannot starve the rest of
# the grid of the job's wall clock.
const CASE_TIME = parse(Float64, get(ENV, "PFA_CASE_TIME", "1200"))
const MAX_WALK = 600.0
# Route-vs-walk weight ratio. The repo's established convention for this zhuzhou
# instance family is "route100x" (beta=10.0, walk=0.1 -- see
# scripts/zhuzhou_p16_scaling_route100x.jl), i.e. a 100x ratio, so that is the
# default here. Earlier runs of this script used beta=1.0 (a 10x ratio), which did
# NOT match the convention; set PFA_ROUTE_WEIGHT=1.0 to reproduce those.
const ROUTE_WEIGHT = parse(Float64, get(ENV, "PFA_ROUTE_WEIGHT", "10.0"))
const WALK_WEIGHT = parse(Float64, get(ENV, "PFA_WALK_WEIGHT", "0.1"))
# Pricing-aware dual selection (dual_selection.jl). Off by default, matching the
# library default -- every scaling number reported before this was added used
# ordinary CG with the solver's raw RMP duals.
const USE_DUAL_SELECTOR = get(ENV, "PFA_DUAL_SELECTOR", "0") in ("1", "true", "yes")
# Concurrent per-scenario pricing. Exact and deterministic either way; set to 0 to
# force the sequential path for an A/B timing comparison.
const PARALLEL_SCENARIOS = get(ENV, "PFA_PARALLEL_SCENARIOS", "1") in ("1", "true", "yes")
# Restrict pricing to columns with at most `l` distinct stations. Slower to price,
# but excludes the broad multi-station "hub" columns that earn LP dual credit
# while being unusable in any integer solution -- so the metric to watch here is
# `lp_mip_gap_pct`, not wall time.
const STATION_BUDGET_CAP = get(ENV, "PFA_STATION_BUDGET_CAP", "0") in ("1", "true", "yes")
# Compensated layer dominance. On by default; set 0 to fall back to the plain
# `A_a subseteq A_b` rule. Faster pricing but fewer columns per search, so this
# exists to settle the tradeoff end to end rather than on pricing speed alone.
const COMPENSATED_DOMINANCE = get(ENV, "PFA_COMPENSATED_DOMINANCE", "1") in ("1", "true", "yes")
# Seed the pool with every two-stop route before the first LP. On by default (the
# library default too); set 0 to reproduce the empty-pool runs, whose opening
# iterations price against `unserved_penalty` duals rather than real costs.
const SEED_TWO_STOP = get(ENV, "PFA_SEED_TWO_STOP", "1") in ("1", "true", "yes")
# Selector objective weights. Defaults follow the original design (stabilization
# dominant). Measured result: stabilization makes the duals DENSER, raising the
# positive-rho count and hence pricing cost -- so inverting these is the direct
# test of "sparser duals price faster".
const SEL_STAB_W = parse(Float64, get(ENV, "PFA_SELECTOR_STAB_WEIGHT", "1.0"))
const SEL_POSREW_W = parse(Float64, get(ENV, "PFA_SELECTOR_POSREWARD_WEIGHT", "1e-4"))
# :l1_stabilized (LP) or :l0_count (MIP minimizing the NUMBER of attractive
# assignments -- the quantity that actually drives pricing cost).
const SEL_OBJ = Symbol(get(ENV, "PFA_SELECTOR_OBJECTIVE", "l1_stabilized"))
const SEL_MIP_TIME = parse(Float64, get(ENV, "PFA_SELECTOR_MIP_TIME", "10.0"))
const SEL_MIP_GAP = parse(Float64, get(ENV, "PFA_SELECTOR_MIP_GAP", "0.05"))

# One Gurobi.Env reused for every solve in this process -- constructing several
# has previously caused a silent multi-minute stall on this cluster.
const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

function build_model_for(n_stations::Int)
    return AggregateODRouteModel(
        _l_for(n_stations);
        route_regularization_weight = ROUTE_WEIGHT,
        walk_cost_weight            = WALK_WEIGHT,
        repositioning_time          = 20.0,
        max_walking_distance        = MAX_WALK,
        max_wait_time               = 900.0,
        detour_factor               = 2.0,
        max_stops                   = MAX_STOPS,
        max_visits_per_node         = MAX_VISITS,
    )
end

function run_one(n_stations::Int, seed::Int, results_dir::String, iters_dir::String)
    # scenario count is part of the identity: passengers = n_pairs * n_scenarios, so a
    # 1-scenario and 3-scenario run of the same (n, p) are different problems.
    case = "pfa_n$(n_stations)_p$(N_PAIRS)_sc$(N_SCENARIOS)_s$(seed)_ms$(MAX_STOPS == typemax(Int) ? "inf" : string(MAX_STOPS))"
    @printf("=== %s ===\n", case)
    flush(stdout)

    t0 = time()
    status = "ok"
    failure = ""
    result = nothing
    data = nothing
    try
        data, _meta = generate_zhuzhou_data(
            DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=seed,
        )
        model = build_model_for(n_stations)
        result = run_passenger_free_assignment_column_generation(
            model, data;
            optimizer_env=GRB_ENV,
            max_cg_iters=MAX_CG_ITERS,
            n_candidates=N_CANDIDATES,
            max_new_columns=N_CANDIDATES,
            pricing_time_limit_sec=PRICING_TIME,
            certification_time_limit_sec=CERT_TIME,
            ip_time_limit_sec=IP_TIME,
            total_time_limit_sec=CASE_TIME,
            parallel_scenarios=PARALLEL_SCENARIOS,
            station_budget_cap=STATION_BUDGET_CAP,
            compensated_dominance=COMPENSATED_DOMINANCE,
            seed_two_stop_routes=SEED_TWO_STOP,
            dual_selector=PassengerDualSelectorConfig(
                use_pricing_aware_dual_selection=USE_DUAL_SELECTOR,
                dual_selector_stabilization_weight=SEL_STAB_W,
                dual_selector_positive_reward_weight=SEL_POSREW_W,
                dual_selector_objective=SEL_OBJ,
                dual_selector_mip_time_limit_sec=SEL_MIP_TIME,
                dual_selector_mip_gap=SEL_MIP_GAP,
            ),
            verify_reduced_costs=true,
            verbose=true,
        )
    catch err
        status = "error"
        failure = sprint(showerror, err)
        showerror(stderr, err, catch_backtrace())
        println(stderr)
    end
    wall = time() - t0

    # `best_reduced_cost` is `nothing` on iterations that priced no columns. CSV.jl
    # cannot serialize `nothing` (only `missing`), so convert before writing --
    # and never let a reporting failure abort the rest of the grid.
    if !isnothing(result) && !isempty(result.iteration_rows)
        try
            iters_df = DataFrame(result.iteration_rows)
            for col in names(iters_df)
                if any(x -> x === nothing, iters_df[!, col])
                    iters_df[!, col] = [x === nothing ? missing : x for x in iters_df[!, col]]
                end
            end
            CSV.write(joinpath(iters_dir, "$(case)_iterations.csv"), iters_df)
        catch err
            @warn "failed to write per-iteration CSV for $case" exception=(err, catch_backtrace())
        end
    end

    summary = (
        case = case,
        n_stations = n_stations,
        n_pairs = N_PAIRS,
        n_scenarios = N_SCENARIOS,
        seed = seed,
        l = _l_for(n_stations),
        max_stops = MAX_STOPS == typemax(Int) ? -1 : MAX_STOPS,
        max_visits = MAX_VISITS == typemax(Int) ? -1 : MAX_VISITS,
        truly_unbounded_stops = MAX_STOPS == typemax(Int),
        n_candidates = N_CANDIDATES,
        status = status,
        error = failure,
        cg_stop_reason = isnothing(result) ? "" : string(result.cg_stop_reason),
        lp_bound = isnothing(result) ? missing : result.lp_bound,
        lp_bound_certified = isnothing(result) ? missing : result.lp_bound_certified,
        mip_objective = isnothing(result) || isnothing(result.mip_objective) ? missing : result.mip_objective,
        mip_termination = isnothing(result) ? "" : string(result.mip_termination),
        lp_mip_gap_pct = (isnothing(result) || isnothing(result.mip_objective) ||
                          !isfinite(result.lp_bound) || abs(result.mip_objective) < 1e-9) ? missing :
            100.0 * (result.mip_objective - result.lp_bound) / abs(result.mip_objective),
        n_cg_iters = isnothing(result) ? missing : result.n_cg_iters,
        n_rounds = isnothing(result) ? missing : result.n_rounds,
        n_columns = isnothing(result) ? missing : result.n_columns,
        n_passengers = isnothing(result) ? missing : result.n_passengers,
        n_master_rows = isnothing(result) ? missing : result.n_master_rows,
        total_pricing_seconds = isnothing(result) ? missing : result.total_pricing_seconds,
        total_lp_seconds = isnothing(result) ? missing : result.total_lp_seconds,
        certification_seconds = isnothing(result) ? missing : result.certification_seconds,
        certification_exhausted = isnothing(result) ? missing : result.certification_exhausted,
        total_labels_generated = isnothing(result) ? missing : result.total_labels_generated,
        n_unserved = isnothing(result) ? missing : length(result.unserved_passengers),
        # Dual-selector accounting. The selector is only worth it if the pricing
        # effort it saves exceeds `selector_seconds` (its own auxiliary LP solves).
        compensated_dominance = COMPENSATED_DOMINANCE,
        seed_two_stop = SEED_TWO_STOP,
        n_seed_columns = isnothing(result) ? missing :
            get(result.final_result.metadata, "seed_two_stop_columns", missing),
        use_dual_selector = USE_DUAL_SELECTOR,
        parallel_scenarios = PARALLEL_SCENARIOS,
        n_threads = Threads.nthreads(),
        selector_seconds = isnothing(result) ? missing : result.total_selector_seconds,
        selector_iterations_used = isnothing(result) ? missing : result.selector_iterations_used,
        selector_fallbacks = isnothing(result) ? missing :
            count(l -> !l.used_selected_duals, result.selector_logs),
        # mean count of attractive (p,j,k) under the SELECTED duals -- the quantity the
        # positive-reward term is meant to shrink, and what drives pricing cost.
        selector_objective = string(SEL_OBJ),
        selector_stab_weight = SEL_STAB_W,
        selector_posreward_weight = SEL_POSREW_W,
        mean_positive_rho_used = isnothing(result) ? missing : result.mean_positive_rho_used,
        mean_raw_positive_rho = isnothing(result) ? missing : result.mean_raw_positive_rho,
        open_stations = isnothing(result) ? "" : string(result.open_stations),
        wall_time_sec = wall,
    )
    try
        CSV.write(joinpath(results_dir, "$(case).csv"), DataFrame([summary]))
    catch err
        @warn "failed to write summary CSV for $case" exception=(err, catch_backtrace())
    end

    @printf("  -> status=%s stop=%s lp=%s certified=%s mip=%s pricing=%.1fs cert=%.1fs labels=%s wall=%.1fs\n\n",
        status, summary.cg_stop_reason, string(summary.lp_bound), string(summary.lp_bound_certified),
        string(summary.mip_objective),
        isnothing(result) ? 0.0 : result.total_pricing_seconds,
        isnothing(result) ? 0.0 : result.certification_seconds,
        string(summary.total_labels_generated), wall)
    flush(stdout)
    return summary
end

function main()
    length(ARGS) >= 1 || error("usage: passenger_free_assignment_cg_scaling.jl <outdir> [n_stations ...]")
    outdir = abspath(ARGS[1])
    n_stations_list = length(ARGS) >= 2 ? parse.(Int, ARGS[2:end]) : [10, 15, 20, 30]
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath(results_dir)
    mkpath(iters_dir)

    println("passenger free-assignment CG scaling: n_stations=$(n_stations_list) " *
            "n_pairs=$N_PAIRS seeds=$(SEEDS) n_scenarios=$N_SCENARIOS max_stops=$MAX_STOPS")
    println("n_candidates=$N_CANDIDATES pricing_time=$(PRICING_TIME)s cert_time=$(CERT_TIME)s case_time=$(CASE_TIME)s")
    println("max_stops=$(MAX_STOPS == typemax(Int) ? "typemax(Int) [TRUE UNBOUNDED]" : string(MAX_STOPS))  " *
            "max_visits=$(MAX_VISITS == typemax(Int) ? "typemax(Int)" : string(MAX_VISITS))  " *
            "threads=$(Threads.nthreads())")
    println()

    rows = NamedTuple[]
    for n_stations in n_stations_list, seed in SEEDS
        push!(rows, run_one(n_stations, seed, results_dir, iters_dir))
    end

    combined = joinpath(outdir, "combined_results.csv")
    CSV.write(combined, DataFrame(rows))
    println("Wrote $combined")

    println()
    println("=== summary ===")
    @printf("%6s %6s %8s %10s %12s %12s %10s %12s %10s\n",
        "n", "iters", "rounds", "columns", "pricing_s", "cert_s", "labels", "certified", "wall_s")
    for r in rows
        @printf("%6d %6s %8s %10s %12s %12s %10s %12s %10.1f\n",
            r.n_stations, string(r.n_cg_iters), string(r.n_rounds), string(r.n_columns),
            r.total_pricing_seconds isa Real ? @sprintf("%.1f", r.total_pricing_seconds) : "-",
            r.certification_seconds isa Real ? @sprintf("%.1f", r.certification_seconds) : "-",
            string(r.total_labels_generated), string(r.lp_bound_certified), r.wall_time_sec)
    end
end

main()
