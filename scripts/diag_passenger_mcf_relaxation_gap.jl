"""
    scripts/diag_passenger_mcf_relaxation_gap.jl

How tight is the time-expanded multi-commodity-flow relaxation
(`pricing/passenger/mcf_relaxation.jl`) against the exact label search, and can
it ever certify?

The MCF LP is a valid lower bound on the pricing optimum, so it can replace the
exhaustive certification pass whenever `L_s >= -tol`. Whether that ever happens
is an empirical question about tightness, and this script answers it *before*
anything is wired into the CG loop: it runs an ordinary CG loop with **exhaustive
exact pricing** and, at every iteration and every scenario, computes BOTH the MCF
bound and the exact minimum reduced cost over the same `pricing_data`.

Emits one `MCFGAP` line per (iteration, scenario):

    MCFGAP n= s= iter= lp_bound= exact_rc= abs_gap= rel_gap= would_certify=
           exact_says_certified= arcs= commodities= mcf_sec= label_sec= reason=

Decision rules, pre-registered:

  1. `would_certify=true` while `exact_rc < -tol` is a VALIDITY BUG, not a
     tuning problem. `n_false_certificates` in the summary must be 0.
  2. The feature is worth wiring in if, on the iterations where the exact pricer
     finds nothing (`exact_says_certified=true`), the MCF certifies a useful
     fraction of scenarios at a cost well under the label search's. Those late
     iterations are the only ones that matter -- early-iteration gaps are
     irrelevant, since CG is still adding columns there anyway.
  3. If `rel_gap` stays large at convergence, retry with more boarding buckets
     and a finer grid before concluding the relaxation is too loose.

Usage (one station count per invocation, so an array can run them concurrently):
    julia --project=. scripts/diag_passenger_mcf_relaxation_gap.jl <n_stations> [outdir]

Env overrides:
    PFAMCF_N_PAIRS      default 16
    PFAMCF_SEED         default 42
    PFAMCF_N_SCENARIOS  default 3
    PFAMCF_MAX_STOPS    default 5    (0 => unbounded)
    PFAMCF_MAX_VISITS   default 3
    PFAMCF_BUCKETS      default "1,4"  (comma-separated boarding-bucket counts)
    PFAMCF_STEP_DIVS    default "1,2"  (grid = min_travel / div, comma-separated)
    PFAMCF_MAX_ARCS     default 4000000
    PFAMCF_LP_TIME      default 120
    PFAMCF_LABEL_TIME   default 300
    PFAMCF_MAX_ITERS    default 200
    PFAMCF_CASE_TIME    default 3000
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFAMCF_N_PAIRS", "16"))
const SEED = parse(Int, get(ENV, "PFAMCF_SEED", "42"))
const N_SCENARIOS = parse(Int, get(ENV, "PFAMCF_N_SCENARIOS", "3"))
const _RAW_MS = parse(Int, get(ENV, "PFAMCF_MAX_STOPS", "5"))
const MAX_STOPS = _RAW_MS <= 0 ? typemax(Int) : _RAW_MS
const MAX_VISITS = parse(Int, get(ENV, "PFAMCF_MAX_VISITS", "3"))
const BUCKETS = parse.(Int, split(get(ENV, "PFAMCF_BUCKETS", "1,4"), ","))
const STEP_DIVS = parse.(Int, split(get(ENV, "PFAMCF_STEP_DIVS", "1,2"), ","))
const MAX_ARCS = parse(Int, get(ENV, "PFAMCF_MAX_ARCS", "4000000"))
const LP_TIME = parse(Float64, get(ENV, "PFAMCF_LP_TIME", "120"))
const LABEL_TIME = parse(Float64, get(ENV, "PFAMCF_LABEL_TIME", "300"))
const MAX_ITERS = parse(Int, get(ENV, "PFAMCF_MAX_ITERS", "200"))
const CASE_TIME = parse(Float64, get(ENV, "PFAMCF_CASE_TIME", "3000"))
const MAX_WALK = 600.0
const RC_TOL = 1e-6

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

"""
The smallest positive travel time among the opportunity endpoints -- the coarsest
grid the DAG argument allows, and the base the `STEP_DIVS` sweep divides.
"""
function min_positive_travel(pd)
    endpoints = Set{Int}()
    for opp in pd.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end
    best = Inf
    for u in endpoints, v in endpoints
        u == v && continue
        c = get(pd.travel_cost, (u, v), Inf)
        (isfinite(c) && c > 0) && (best = min(best, c))
    end
    return best
end

"""
One CG iteration's worth of measurement for a single scenario: the exact pricer
run to exhaustion, plus the MCF bound at every (grid, bucket) setting in the
sweep. Returns the exact columns (so the caller can grow the pool) and the rows.
"""
function measure_scenario(master, alpha, gamma_o, gamma_d, s, iter, n_stations, next_column_id)
    md = master.master_data
    candidates = passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, s)
    isempty(candidates) && return PassengerFreeAssignmentRouteColumn[], NamedTuple[]

    pd = StationSelection.create_passenger_free_assignment_pricing_data(
        s, md.nodes, md.travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time,
        repositioning_time=md.repositioning_time,
        max_stops=md.max_stops,
        max_visits_per_node=md.max_visits_per_node,
    )
    isempty(pd.opportunities) && return PassengerFreeAssignmentRouteColumn[], NamedTuple[]

    t0 = time()
    cols, exhausted, _stats = passenger_free_assignment_pricing_by_label_setting(
        pd, PassengerFreeAssignmentRouteColumn[];
        next_column_id=next_column_id,
        reduced_cost_tol=RC_TOL,
        max_new_columns=typemax(Int) ÷ 2,
        n_candidates=typemax(Int) ÷ 2,
        time_limit=LABEL_TIME,
    )
    label_sec = time() - t0
    exact_rc = isempty(cols) ? Inf :
        minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in cols)
    # Only a genuinely exhausted search proves "no improving column"; a timeout
    # says nothing, so those rows cannot be used to score the certificate.
    exact_says_certified = exhausted && isempty(cols)

    base_step = min_positive_travel(pd)
    rows = NamedTuple[]
    for div in STEP_DIVS, nb in BUCKETS
        step = isfinite(base_step) ? base_step / div : 0.0
        config = PassengerMCFRelaxationConfig(;
            enabled=true, time_step=step, n_boarding_buckets=nb,
            lp_time_limit_sec=LP_TIME, max_arcs=MAX_ARCS,
        )
        bound, certified, mstats = passenger_free_assignment_mcf_lower_bound(
            pd, GRB_ENV; config=config, reduced_cost_tol=RC_TOL,
        )
        abs_gap = (isfinite(bound) && isfinite(exact_rc)) ? exact_rc - bound : missing
        rel_gap = (abs_gap isa Real && abs(exact_rc) > 1e-9) ?
            100 * abs_gap / abs(exact_rc) : missing
        # The bug we are hunting: certifying while an improving column exists.
        false_certificate = certified && isfinite(exact_rc) && exact_rc < -RC_TOL
        # The bound must never exceed the exact optimum, certificate or not.
        bound_violation = isfinite(bound) && isfinite(exact_rc) && bound > exact_rc + 1e-6

        @printf("  MCFGAP\tn=%d\ts=%d\titer=%d\tdiv=%d\tnb=%d\tlp_bound=%s\texact_rc=%s\tabs_gap=%s\trel_gap=%s\twould_certify=%s\texact_says_certified=%s\tstates=%d\tarcs=%d\tcommodities=%d\tmcf_sec=%.2f\tlabel_sec=%.2f\treason=%s\n",
            n_stations, s, iter, div, nb,
            isfinite(bound) ? @sprintf("%.4f", bound) : "-inf",
            isfinite(exact_rc) ? @sprintf("%.4f", exact_rc) : "none",
            abs_gap isa Real ? @sprintf("%.4f", abs_gap) : "-",
            rel_gap isa Real ? @sprintf("%.2f%%", rel_gap) : "-",
            string(certified), string(exact_says_certified),
            mstats.n_states, mstats.n_arcs, mstats.n_commodities,
            mstats.build_sec + mstats.solve_sec, label_sec, string(mstats.reason))

        push!(rows, (
            n_stations=n_stations, scenario=s, iteration=iter, step_div=div,
            n_boarding_buckets=nb, lp_bound=bound, exact_rc=exact_rc,
            abs_gap=abs_gap, rel_gap=rel_gap, would_certify=certified,
            exact_says_certified=exact_says_certified,
            exact_exhausted=exhausted,
            false_certificate=false_certificate, bound_violation=bound_violation,
            effective_time_step=mstats.effective_time_step,
            n_time_layers=mstats.n_time_layers, n_states=mstats.n_states,
            n_arcs=mstats.n_arcs, n_commodities=mstats.n_commodities,
            n_opportunities=mstats.n_opportunities,
            mcf_build_sec=mstats.build_sec, mcf_solve_sec=mstats.solve_sec,
            label_sec=label_sec, reason=string(mstats.reason),
        ))
    end
    flush(stdout)
    return cols, rows
end

function run_case(n_stations::Int, results_dir::String)
    l = _l_for(n_stations)
    @printf("=== n=%d p=%d scenarios=%d l=%d ms=%s max_visits=%d buckets=%s step_divs=%s ===\n",
        n_stations, N_PAIRS, N_SCENARIOS, l,
        MAX_STOPS == typemax(Int) ? "unb" : string(MAX_STOPS), MAX_VISITS,
        string(BUCKETS), string(STEP_DIVS))
    flush(stdout)

    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    model = build_model_for(n_stations)
    mapping = create_map(model, data)
    md = StationSelection.create_passenger_free_assignment_master_data(model, data, mapping)
    master = build_passenger_free_assignment_master(md, GRB_ENV; relax_integrality=true)
    set_silent(master.model)

    next_column_id = 1
    for column in passenger_free_assignment_two_stop_seed_columns(md; next_column_id=next_column_id)
        StationSelection.add_passenger_free_assignment_column!(master, column)
        next_column_id += 1
    end

    scenarios = sort!(collect(keys(md.passengers_by_scenario)))
    all_rows = NamedTuple[]
    t_start = time()
    iter = 0
    stop_reason = "max_iters"

    while iter < MAX_ITERS
        if time() - t_start > CASE_TIME
            stop_reason = "case_time"
            break
        end
        iter += 1
        optimize!(master.model)
        if primal_status(master.model) != MOI.FEASIBLE_POINT
            stop_reason = "no_primal"
            break
        end
        lp = objective_value(master.model)
        alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)

        added = 0
        for s in scenarios
            cols, rows = measure_scenario(
                master, alpha, gamma_o, gamma_d, s, iter, n_stations, next_column_id,
            )
            append!(all_rows, rows)
            for c in cols
                # Ids collide across scenarios (each prices from the same base), so
                # renumber before inserting, exactly as the real loop does.
                renumbered = PassengerFreeAssignmentRouteColumn(
                    next_column_id, c.route, c.assignments, c.tau; metadata=c.metadata,
                )
                next_column_id += 1
                _theta, action = StationSelection.add_passenger_free_assignment_column!(master, renumbered)
                action == :added && (added += 1)
            end
        end
        @printf("  [iter %3d] lp=%.4f added=%d pool=%d elapsed=%.1fs\n",
            iter, lp, added, length(master.theta), time() - t_start)
        flush(stdout)
        if added == 0
            stop_reason = "converged"
            break
        end
    end

    df = DataFrame(all_rows)
    n_false = isempty(df) ? 0 : count(skipmissing(df.false_certificate))
    n_violations = isempty(df) ? 0 : count(skipmissing(df.bound_violation))

    # Score the certificate only where the exact pricer PROVED there was nothing
    # to find -- those are the iterations the certification pass would have to
    # burn its budget on.
    scored = isempty(df) ? df : filter(r -> r.exact_says_certified === true, df)
    @printf("MCFSUMMARY\tn=%d\tstop=%s\titers=%d\trows=%d\tfalse_certificates=%d\tbound_violations=%d\n",
        n_stations, stop_reason, iter, nrow(df), n_false, n_violations)
    if !isempty(scored)
        for div in STEP_DIVS, nb in BUCKETS
            sub = filter(r -> r.step_div == div && r.n_boarding_buckets == nb, scored)
            isempty(sub) && continue
            hit = count(sub.would_certify)
            mcf_sec = sum(sub.mcf_build_sec .+ sub.mcf_solve_sec) / nrow(sub)
            lab_sec = sum(sub.label_sec) / nrow(sub)
            @printf("MCFSCORE\tn=%d\tdiv=%d\tnb=%d\tcertifiable_rows=%d\tcertified=%d\trate=%.1f%%\tmean_mcf_sec=%.2f\tmean_label_sec=%.2f\tspeedup=%.2f\n",
                n_stations, div, nb, nrow(sub), hit, 100 * hit / nrow(sub),
                mcf_sec, lab_sec, mcf_sec > 0 ? lab_sec / mcf_sec : NaN)
        end
    else
        println("MCFSCORE\tn=$(n_stations)\tno certifiable rows (CG never reached a proven-empty pricing iteration)")
    end
    flush(stdout)

    try
        CSV.write(joinpath(results_dir, "pfamcf_n$(n_stations)_p$(N_PAIRS)_s$(SEED).csv"), df)
    catch e
        @warn "results CSV write failed" exception=(e, catch_backtrace())
    end
    return df
end

function main()
    length(ARGS) >= 1 || error("usage: diag_passenger_mcf_relaxation_gap.jl <n_stations> [outdir]")
    n = parse(Int, ARGS[1])
    outdir = length(ARGS) >= 2 ? abspath(ARGS[2]) : pwd()
    results_dir = joinpath(outdir, "results")
    mkpath(results_dir)
    run_case(n, results_dir)
end

main()
