"""
    scripts/diag_passenger_reward_ladder_census.jl

Step 0 + Step 1 of `notes/2026-07-31_pfa_state_space_relaxation_design.md`, in one
run and with **no source changes**.

# The shortcut

The proposed state-space relaxation retains only a subset `V_p` of each
passenger's distinct reward values and maps every candidate to the prefix ending
at the smallest retained value `>= its reward`. That is exactly what
`_build_passenger_reward_layers` builds if you hand it candidates whose rewards
have already been **rounded up to the retained rungs** -- it groups distinct
reward values itself. So the whole relaxation family is a pre-transform on the
candidate vector, and can be measured before implementing anything.

# What this answers

1. **The census (the gate).** `sum_p m_p` versus `|P|`, where `m_p` is passenger
   `p`'s number of distinct positive reward values. If `m_p ~ 1` the ladder cannot
   merge anything and the whole reward axis is dead -- go build the clock axis
   instead.

2. **Does the collapse actually shrink the search?** Runs the exhaustive label
   search on the exact ladder and on `|V_p| in {1, 2, 3}`, and compares labels
   generated, `max_live` and wall.

3. **Is it valid?** Every route the relaxed search returns is replayed against the
   EXACT pricing data. `relaxed_rc <= true_rc + 1e-6` must hold on every route --
   that is the relaxation property itself, and a violation is a hard bug.

4. **Would the fast path fire?** Whether the relaxed optimum, priced honestly,
   is still a genuinely improving column (`true_rc < -tol`) -- i.e. whether an
   ordinary CG iteration would be served without any DSSR promotion round.

5. **Would it certify?** On iterations where the exact pass PROVES nothing
   remains, whether the relaxed search also comes back empty and exhausted.

Emits `LADDER` (census), `RELAX` (per L) and `SUMMARY` lines plus a CSV.

Usage:
    julia --project=. scripts/diag_passenger_reward_ladder_census.jl <n_stations> [outdir]

Env overrides:
    PFALAD_N_PAIRS      default 16
    PFALAD_SEED         default 42
    PFALAD_N_SCENARIOS  default 3
    PFALAD_MAX_STOPS    default 5    (0 => unbounded)
    PFALAD_MAX_VISITS   default 3
    PFALAD_LEVELS       default "1,2,3"  (comma-separated |V_p| to test)
    PFALAD_LABEL_TIME   default 300
    PFALAD_MAX_ITERS    default 200
    PFALAD_CASE_TIME    default 3000
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, Statistics, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFALAD_N_PAIRS", "16"))
const SEED = parse(Int, get(ENV, "PFALAD_SEED", "42"))
const N_SCENARIOS = parse(Int, get(ENV, "PFALAD_N_SCENARIOS", "3"))
const _RAW_MS = parse(Int, get(ENV, "PFALAD_MAX_STOPS", "5"))
const MAX_STOPS = _RAW_MS <= 0 ? typemax(Int) : _RAW_MS
const MAX_VISITS = parse(Int, get(ENV, "PFALAD_MAX_VISITS", "3"))
const LEVELS = parse.(Int, split(get(ENV, "PFALAD_LEVELS", "1,2,3"), ","))
const LABEL_TIME = parse(Float64, get(ENV, "PFALAD_LABEL_TIME", "300"))
const MAX_ITERS = parse(Int, get(ENV, "PFALAD_MAX_ITERS", "200"))
const CASE_TIME = parse(Float64, get(ENV, "PFALAD_CASE_TIME", "3000"))
const MAX_WALK = 600.0
const RC_TOL = 1e-6
const TOL = 1e-9

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
Passenger `p`'s distinct positive reward values, grouped by the SAME tolerance
rule `_build_passenger_reward_layers` uses, so `m_p` here is exactly the number
of layers that constructor would emit for `p`.
"""
function distinct_reward_values(rewards::Vector{Float64})
    values = Float64[]
    for v in sort(rewards)
        (!isempty(values) && v - values[end] <= TOL) && continue
        push!(values, v)
    end
    return values
end

function reward_values_by_passenger(candidates)
    by_p = Dict{Int, Vector{Float64}}()
    for c in candidates
        c.reward > TOL || continue
        push!(get!(() -> Float64[], by_p, c.passenger), c.reward)
    end
    return Dict(p => distinct_reward_values(vs) for (p, vs) in by_p)
end

"""
Retain `L` of a passenger's distinct values, evenly spaced **in value** and always
including the maximum, so the worst-case round-up is `(v_max - v_min) / L`.
"""
function retained_values(values::Vector{Float64}, L::Int)
    length(values) <= L && return values
    v_min, v_max = first(values), last(values)
    kept = Float64[]
    for i in 1:L
        target = v_min + (v_max - v_min) * i / L
        idx = findfirst(v -> v >= target - TOL, values)
        idx === nothing && continue
        v = values[idx]
        (isempty(kept) || v > kept[end] + TOL) && push!(kept, v)
    end
    (isempty(kept) || kept[end] < v_max - TOL) && push!(kept, v_max)
    return kept
end

"Round every candidate's reward UP to its passenger's next retained rung."
function coarsen_candidates(candidates, values_by_p, L::Int)
    kept_by_p = Dict(p => retained_values(vs, L) for (p, vs) in values_by_p)
    out = PassengerAssignmentCandidate[]
    for c in candidates
        if c.reward <= TOL
            push!(out, c)
            continue
        end
        kept = kept_by_p[c.passenger]
        idx = findfirst(v -> v >= c.reward - TOL, kept)
        rounded = idx === nothing ? kept[end] : kept[idx]
        push!(out, PassengerAssignmentCandidate(
            c.passenger, c.origin, c.destination, c.ride_limit, rounded,
        ))
    end
    return out
end

function make_pricing_data(md, s, candidates)
    return StationSelection.create_passenger_free_assignment_pricing_data(
        s, md.nodes, md.travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time,
        repositioning_time=md.repositioning_time,
        max_stops=md.max_stops,
        max_visits_per_node=md.max_visits_per_node,
    )
end

function exhaustive_search(pd, next_column_id)
    t0 = time()
    cols, exhausted, stats = passenger_free_assignment_pricing_by_label_setting(
        pd, PassengerFreeAssignmentRouteColumn[];
        next_column_id=next_column_id, reduced_cost_tol=RC_TOL,
        max_new_columns=typemax(Int) ÷ 2, n_candidates=typemax(Int) ÷ 2,
        time_limit=LABEL_TIME,
    )
    return cols, exhausted, stats, time() - t0
end

function measure_scenario(master, alpha, gamma_o, gamma_d, s, iter, n_stations, next_column_id)
    md = master.master_data
    candidates = passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, s)
    isempty(candidates) && return PassengerFreeAssignmentRouteColumn[], NamedTuple[]

    exact_pd = make_pricing_data(md, s, candidates)
    isempty(exact_pd.opportunities) && return PassengerFreeAssignmentRouteColumn[], NamedTuple[]

    # ── the census ────────────────────────────────────────────────────────────
    values_by_p = reward_values_by_passenger(candidates)
    m_values = collect(length(v) for v in values(values_by_p))
    n_passengers = length(values_by_p)
    sum_m = sum(m_values; init=0)
    @printf("  LADDER\tn=%d\ts=%d\titer=%d\tpassengers=%d\tsum_m_p=%d\tn_layers=%d\tm_min=%d\tm_median=%.1f\tm_mean=%.1f\tm_max=%d\topportunities=%d\n",
        n_stations, s, iter, n_passengers, sum_m, exact_pd.n_layers,
        isempty(m_values) ? 0 : minimum(m_values),
        isempty(m_values) ? 0.0 : median(m_values),
        isempty(m_values) ? 0.0 : mean(m_values),
        isempty(m_values) ? 0 : maximum(m_values),
        length(exact_pd.opportunities))

    # ── the exact reference ───────────────────────────────────────────────────
    exact_cols, exact_exhausted, exact_stats, exact_wall = exhaustive_search(exact_pd, next_column_id)
    exact_rc = isempty(exact_cols) ? Inf :
        minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in exact_cols)
    exact_says_certified = exact_exhausted && isempty(exact_cols)

    rows = NamedTuple[]
    for L in LEVELS
        coarse = coarsen_candidates(candidates, values_by_p, L)
        rel_pd = make_pricing_data(md, s, coarse)
        rel_cols, rel_exhausted, rel_stats, rel_wall = exhaustive_search(rel_pd, next_column_id)
        rel_rc = isempty(rel_cols) ? Inf :
            minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in rel_cols)

        # Replay every relaxed route against the EXACT data: this is both the
        # validity check and the fast-path test.
        worst_violation = -Inf
        best_true_rc = Inf
        for c in rel_cols
            _a, _t, true_rc = StationSelection._passenger_free_assignment_column_from_route(
                collect(Int, c.route), exact_pd,
            )
            relaxed_rc = Float64(get(c.metadata, "reduced_cost", Inf))
            worst_violation = max(worst_violation, relaxed_rc - true_rc)
            best_true_rc = min(best_true_rc, true_rc)
        end
        # The relaxation property: relaxed_rc <= true_rc on every route.
        validity_ok = !isfinite(worst_violation) || worst_violation <= 1e-6
        fast_path = isfinite(best_true_rc) && best_true_rc < -RC_TOL
        relaxed_certifies = rel_exhausted && isempty(rel_cols)
        # A wrong certificate: relaxed says "nothing" while the exact pricer found one.
        false_certificate = relaxed_certifies && isfinite(exact_rc) && exact_rc < -RC_TOL

        @printf("  RELAX\tn=%d\ts=%d\titer=%d\tL=%d\tlayers=%d/%d\trel_rc=%s\texact_rc=%s\tbest_true_rc=%s\tlabels=%d/%d\tlabel_ratio=%.3f\twall=%.2f/%.2f\tspeedup=%.2f\tfast_path=%s\trelaxed_certifies=%s\texact_certified=%s\tvalid=%s\n",
            n_stations, s, iter, L, rel_pd.n_layers, exact_pd.n_layers,
            isfinite(rel_rc) ? @sprintf("%.2f", rel_rc) : "none",
            isfinite(exact_rc) ? @sprintf("%.2f", exact_rc) : "none",
            isfinite(best_true_rc) ? @sprintf("%.2f", best_true_rc) : "none",
            rel_stats.labels_generated, exact_stats.labels_generated,
            exact_stats.labels_generated > 0 ?
                rel_stats.labels_generated / exact_stats.labels_generated : NaN,
            rel_wall, exact_wall, rel_wall > 0 ? exact_wall / rel_wall : NaN,
            string(fast_path), string(relaxed_certifies), string(exact_says_certified),
            string(validity_ok))

        push!(rows, (
            n_stations=n_stations, scenario=s, iteration=iter, levels=L,
            n_passengers=n_passengers, sum_m_p=sum_m,
            exact_n_layers=exact_pd.n_layers, relaxed_n_layers=rel_pd.n_layers,
            m_min=isempty(m_values) ? 0 : minimum(m_values),
            m_median=isempty(m_values) ? 0.0 : median(m_values),
            m_max=isempty(m_values) ? 0 : maximum(m_values),
            n_opportunities=length(exact_pd.opportunities),
            exact_rc=exact_rc, relaxed_rc=rel_rc, best_true_rc=best_true_rc,
            exact_labels=exact_stats.labels_generated,
            relaxed_labels=rel_stats.labels_generated,
            exact_max_live=exact_stats.max_live_labels,
            relaxed_max_live=rel_stats.max_live_labels,
            exact_wall=exact_wall, relaxed_wall=rel_wall,
            exact_exhausted=exact_exhausted, relaxed_exhausted=rel_exhausted,
            exact_says_certified=exact_says_certified,
            relaxed_certifies=relaxed_certifies, fast_path=fast_path,
            worst_violation=isfinite(worst_violation) ? worst_violation : missing,
            validity_ok=validity_ok, false_certificate=false_certificate,
            n_relaxed_routes=length(rel_cols),
        ))
    end
    flush(stdout)
    return exact_cols, rows
end

function run_case(n_stations::Int, results_dir::String)
    l = _l_for(n_stations)
    @printf("=== n=%d p=%d scenarios=%d l=%d ms=%s max_visits=%d levels=%s ===\n",
        n_stations, N_PAIRS, N_SCENARIOS, l,
        MAX_STOPS == typemax(Int) ? "unb" : string(MAX_STOPS), MAX_VISITS, string(LEVELS))
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
        added == 0 && (stop_reason = "converged"; break)
    end

    df = DataFrame(all_rows)
    n_invalid = isempty(df) ? 0 : count(.!df.validity_ok)
    n_false = isempty(df) ? 0 : count(df.false_certificate)
    @printf("SUMMARY\tn=%d\tstop=%s\titers=%d\trows=%d\tvalidity_violations=%d\tfalse_certificates=%d\n",
        n_stations, stop_reason, iter, nrow(df), n_invalid, n_false)

    if !isempty(df)
        cens = unique(select(df, [:iteration, :scenario, :n_passengers, :sum_m_p, :exact_n_layers]))
        @printf("CENSUS\tn=%d\tmean_passengers=%.1f\tmean_sum_m_p=%.1f\tmean_ratio=%.2f\n",
            n_stations, mean(cens.n_passengers), mean(cens.sum_m_p),
            mean(cens.sum_m_p ./ max.(cens.n_passengers, 1)))
        for L in LEVELS
            sub = filter(r -> r.levels == L, df)
            isempty(sub) && continue
            certifiable = filter(r -> r.exact_says_certified === true, sub)
            ordinary = filter(r -> r.exact_says_certified !== true, sub)
            @printf("SCORE\tn=%d\tL=%d\tmean_label_ratio=%.3f\tmean_speedup=%.2f\tfast_path_rate=%.1f%%\tcertifiable_rows=%d\tcertified=%d\tcert_rate=%s\n",
                n_stations, L,
                mean(sub.relaxed_labels ./ max.(sub.exact_labels, 1)),
                mean(sub.exact_wall ./ max.(sub.relaxed_wall, 1e-9)),
                isempty(ordinary) ? NaN : 100 * count(ordinary.fast_path) / nrow(ordinary),
                nrow(certifiable), isempty(certifiable) ? 0 : count(certifiable.relaxed_certifies),
                isempty(certifiable) ? "-" :
                    @sprintf("%.1f%%", 100 * count(certifiable.relaxed_certifies) / nrow(certifiable)))
        end
    end
    flush(stdout)

    try
        CSV.write(joinpath(results_dir, "pfaladder_n$(n_stations)_p$(N_PAIRS)_s$(SEED).csv"), df)
    catch e
        @warn "results CSV write failed" exception=(e, catch_backtrace())
    end
    return df
end

function main()
    length(ARGS) >= 1 || error("usage: diag_passenger_reward_ladder_census.jl <n_stations> [outdir]")
    n = parse(Int, ARGS[1])
    outdir = length(ARGS) >= 2 ? abspath(ARGS[2]) : pwd()
    results_dir = joinpath(outdir, "results")
    mkpath(results_dir)
    run_case(n, results_dir)
end

main()
