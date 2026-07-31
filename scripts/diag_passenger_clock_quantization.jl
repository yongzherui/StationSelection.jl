"""
    scripts/diag_passenger_clock_quantization.jl

The second relaxation axis: **quantize the clocks**, not the reward ladder.

`notes/2026-07-31_pfa_state_space_relaxation_design.md` measured the reward axis
and found the predicted mechanism wrong: collapsing the layer universe 17x moved
label count by only 23%, while wall still dropped 2-4.5x. The win was a cheaper
dominance scan per label, not fewer labels -- which says `station_age`, not
`activated_reward_layers`, is what the PFA state space is made of.

# The relaxation

Floor the travel-time matrix to a grid `q`: `d'(i,j) = q * floor(d(i,j) / q)`.
The label search then advances `time`, `station_age` and `tau` on that lattice.

**Valid, on both terms at once.** `d' <= d`, so along any route
`tau' <= tau` (cost down) and every clock is younger, so every ride-limit and
pickup-window test is easier and the certified reward can only go up. Hence

    rc'(r) = beta*(tau'_r + repo) - B'(r) <= beta*(tau_r + repo) - B(r) = rc(r)

for every route. Flooring accumulates no drift on ride limits, because a ride's
two endpoints are cumulative sums over the *same* arcs (the argument from the MCF
note, which is the one thing worth keeping from it).

**Why it should shrink the state.** `time` and the ages become multiples of `q`,
so labels that differed by a hair now collide exactly, and dominance conditions 4
(`a.time <= b.time`) and 6 (the per-origin age walk) start hitting. Coarser `q`
means more collisions.

# Two variants

  - `floor` -- the honest relaxation. Arcs shorter than `q` floor to **zero**, so
    the vehicle can teleport between close stations at no time and no cost. Valid,
    but potentially a label explosion, since a zero-cost move cannot be pruned by
    reduced cost.
  - `floor_min1` -- `max(q, q*floor(d/q))`, i.e. no zero-length arcs. Rounds sub-`q`
    arcs *up*, which breaks the bound for those arcs, so this is a pure heuristic:
    it may over-estimate `rc` and must never be used to certify. Included because
    the columns it returns are still priced honestly by replay.

Also tests the reward ladder (`L`) alone and combined with quantization, to see
whether the two axes compose.

Usage:
    julia --project=. scripts/diag_passenger_clock_quantization.jl <n_stations> [outdir]

Env overrides:
    PFAQ_N_PAIRS      default 16
    PFAQ_SEED         default 42
    PFAQ_N_SCENARIOS  default 3
    PFAQ_MAX_STOPS    default 5    (0 => unbounded)
    PFAQ_MAX_VISITS   default 3
    PFAQ_QUANTA       default "1,2,4"  (multiples of the min positive travel time)
    PFAQ_LADDER       default 2        (reward-ladder L for the combined variant; 0 disables)
    PFAQ_LABEL_TIME   default 300
    PFAQ_MAX_ITERS    default 200
    PFAQ_CASE_TIME    default 3000
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, Statistics, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFAQ_N_PAIRS", "16"))
const SEED = parse(Int, get(ENV, "PFAQ_SEED", "42"))
const N_SCENARIOS = parse(Int, get(ENV, "PFAQ_N_SCENARIOS", "3"))
const _RAW_MS = parse(Int, get(ENV, "PFAQ_MAX_STOPS", "5"))
const MAX_STOPS = _RAW_MS <= 0 ? typemax(Int) : _RAW_MS
const MAX_VISITS = parse(Int, get(ENV, "PFAQ_MAX_VISITS", "3"))
const QUANTA = parse.(Float64, split(get(ENV, "PFAQ_QUANTA", "1,2,4"), ","))
const LADDER = parse(Int, get(ENV, "PFAQ_LADDER", "2"))
const LABEL_TIME = parse(Float64, get(ENV, "PFAQ_LABEL_TIME", "300"))
const MAX_ITERS = parse(Int, get(ENV, "PFAQ_MAX_ITERS", "200"))
const CASE_TIME = parse(Float64, get(ENV, "PFAQ_CASE_TIME", "3000"))
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

# ── the reward-ladder transform, carried over from the census script ──────────
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

function coarsen_candidates(candidates, values_by_p, L::Int)
    L <= 0 && return candidates
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

# ── the new axis: quantize the travel-time matrix ────────────────────────────
"""
Floor every arc to the grid `q`, then take the **metric closure**.

The closure is not cosmetic. `_passenger_free_assignment_age_is_useful` discards a
live clock once its opportunities are unreachable in time *from the current node*,
which is only sound when no detour can beat the direct arc -- i.e. when the travel
matrix obeys the triangle inequality. Flooring each arc independently loses up to
`q` per hop, so a two-hop path can undercut the direct arc; the pruning then drops
a clock that later certifies, and the label's incremental reduced cost stops
matching replay (this fires the assertion in
`_passenger_free_assignment_column_from_route`).

Re-closing restores the precondition and keeps the relaxation valid, since
`closure(floor(d)) <= floor(d) <= d` pointwise. Only pairs already present in
`travel_cost` are emitted: inventing an arc would let the relaxed search return a
route that the exact data cannot even price.

`floor_min1` (forbid zero-length arcs by rounding sub-`q` arcs up) was dropped --
the closure re-introduces short arcs anyway, so it bought nothing but an invalid
bound.
"""
function quantize_travel(travel_cost::Dict{Tuple{Int, Int}, Float64}, nodes::Vector{Int}, q::Float64)
    q > 0 || return travel_cost
    n = length(nodes)
    idx = Dict(node => i for (i, node) in enumerate(nodes))
    d = fill(Inf, n, n)
    for i in 1:n
        d[i, i] = 0.0
    end
    for ((u, v), c) in travel_cost
        (haskey(idx, u) && haskey(idx, v) && isfinite(c)) || continue
        u == v && continue
        d[idx[u], idx[v]] = min(d[idx[u], idx[v]], floor(c / q) * q)
    end
    for k in 1:n, i in 1:n
        isfinite(d[i, k]) || continue
        @inbounds for j in 1:n
            alt = d[i, k] + d[k, j]
            alt < d[i, j] && (d[i, j] = alt)
        end
    end
    out = Dict{Tuple{Int, Int}, Float64}()
    for ((u, v), c) in travel_cost
        if !haskey(idx, u) || !haskey(idx, v) || !isfinite(c)
            out[(u, v)] = c
            continue
        end
        out[(u, v)] = u == v ? c : min(c, d[idx[u], idx[v]])
    end
    return out
end

function min_positive_travel(travel_cost)
    best = Inf
    for (_k, v) in travel_cost
        (isfinite(v) && v > TOL) && (best = min(best, v))
    end
    return best
end

function make_pricing_data(md, s, candidates, travel_cost)
    return StationSelection.create_passenger_free_assignment_pricing_data(
        s, md.nodes, travel_cost, candidates;
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

    exact_pd = make_pricing_data(md, s, candidates, md.travel_cost)
    isempty(exact_pd.opportunities) && return PassengerFreeAssignmentRouteColumn[], NamedTuple[]

    exact_cols, exact_exhausted, exact_stats, exact_wall = exhaustive_search(exact_pd, next_column_id)
    exact_rc = isempty(exact_cols) ? Inf :
        minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in exact_cols)
    exact_says_certified = exact_exhausted && isempty(exact_cols)

    base_q = min_positive_travel(md.travel_cost)
    values_by_p = reward_values_by_passenger(candidates)

    # variant := (label, ladder L, quantum multiplier, quantization mode)
    variants = Tuple{String, Int, Float64, Symbol}[]
    for mult in QUANTA
        push!(variants, ("q$(mult)", 0, mult, :floor))
    end
    LADDER > 0 && push!(variants, ("L$(LADDER)", LADDER, 0.0, :none))
    LADDER > 0 && !isempty(QUANTA) &&
        push!(variants, ("L$(LADDER)+q$(first(QUANTA))", LADDER, first(QUANTA), :floor))

    rows = NamedTuple[]
    for (label, L, mult, mode) in variants
        cand = coarsen_candidates(candidates, values_by_p, L)
        tc = mode === :none ? md.travel_cost :
            quantize_travel(md.travel_cost, md.nodes, mult * base_q)
        rel_pd = make_pricing_data(md, s, cand, tc)
        rel_cols, rel_exhausted, rel_stats, rel_wall = exhaustive_search(rel_pd, next_column_id)
        rel_rc = isempty(rel_cols) ? Inf :
            minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in rel_cols)

        worst_violation = -Inf
        best_true_rc = Inf
        for c in rel_cols
            _a, _t, true_rc = StationSelection._passenger_free_assignment_column_from_route(
                collect(Int, c.route), exact_pd,
            )
            worst_violation = max(worst_violation, Float64(get(c.metadata, "reduced_cost", Inf)) - true_rc)
            best_true_rc = min(best_true_rc, true_rc)
        end
        validity_ok = !isfinite(worst_violation) || worst_violation <= 1e-6
        fast_path = isfinite(best_true_rc) && best_true_rc < -RC_TOL
        relaxed_certifies = rel_exhausted && isempty(rel_cols)
        false_certificate = relaxed_certifies && isfinite(exact_rc) && exact_rc < -RC_TOL
        shortfall = (isfinite(best_true_rc) && isfinite(exact_rc) && exact_rc < -TOL) ?
            (best_true_rc - exact_rc) / (-exact_rc) : missing

        @printf("  QUANT\tn=%d\ts=%d\titer=%d\tvar=%-22s\trel_rc=%s\texact_rc=%s\tbest_true_rc=%s\tshortfall=%s\tlabels=%d/%d\tratio=%.3f\twall=%.2f/%.2f\tspeedup=%.2f\texhausted=%s\tfast=%s\tcert=%s\tvalid=%s\n",
            n_stations, s, iter, label,
            isfinite(rel_rc) ? @sprintf("%.2f", rel_rc) : "none",
            isfinite(exact_rc) ? @sprintf("%.2f", exact_rc) : "none",
            isfinite(best_true_rc) ? @sprintf("%.2f", best_true_rc) : "none",
            shortfall isa Real ? @sprintf("%.2f%%", 100 * shortfall) : "-",
            rel_stats.labels_generated, exact_stats.labels_generated,
            exact_stats.labels_generated > 0 ? rel_stats.labels_generated / exact_stats.labels_generated : NaN,
            rel_wall, exact_wall, rel_wall > 0 ? exact_wall / rel_wall : NaN,
            string(rel_exhausted), string(fast_path), string(relaxed_certifies), string(validity_ok))

        push!(rows, (
            n_stations=n_stations, scenario=s, iteration=iter, variant=label,
            ladder_L=L, quantum_mult=mult, quantize_mode=string(mode),
            base_quantum=base_q,
            exact_rc=exact_rc, relaxed_rc=rel_rc, best_true_rc=best_true_rc,
            shortfall=shortfall,
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
    @printf("=== n=%d p=%d scenarios=%d l=%d ms=%s max_visits=%d quanta=%s ladder=%d ===\n",
        n_stations, N_PAIRS, N_SCENARIOS, _l_for(n_stations),
        MAX_STOPS == typemax(Int) ? "unb" : string(MAX_STOPS), MAX_VISITS,
        string(QUANTA), LADDER)
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

    # Absorb JIT before any timing: the exact search runs first in each scenario
    # and would otherwise charge compilation to the baseline it is compared against.
    try
        optimize!(master.model)
        a0, o0, d0 = extract_passenger_free_assignment_duals(master)
        s0 = first(sort!(collect(keys(md.passengers_by_scenario))))
        c0 = passenger_free_assignment_pricing_candidates(md, a0, o0, d0, s0)
        if !isempty(c0)
            wpd = make_pricing_data(md, s0, c0, md.travel_cost)
            isempty(wpd.opportunities) || exhaustive_search(wpd, 1)
        end
    catch e
        @warn "warmup failed" exception=(e,)
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
    @printf("SUMMARY\tn=%d\tstop=%s\titers=%d\trows=%d\n", n_stations, stop_reason, iter, nrow(df))
    if !isempty(df)
        for v in unique(df.variant)
            sub = filter(r -> r.variant == v, df)
            ordinary = filter(r -> r.exact_says_certified !== true, sub)
            certifiable = filter(r -> r.exact_says_certified === true, sub)
            sf = collect(skipmissing(ordinary.shortfall))
            @printf("SCORE\tn=%d\tvar=%-22s\twall %8.2f->%8.2f (%.2fx)\tlabels %9d->%9d (%.3f)\tfast=%.1f%%\tmean_shortfall=%s\tvalidity_viol=%d\tfalse_cert=%d\tcertified=%d/%d\ttimeouts=%d\n",
                n_stations, v,
                sum(sub.exact_wall), sum(sub.relaxed_wall),
                sum(sub.relaxed_wall) > 0 ? sum(sub.exact_wall) / sum(sub.relaxed_wall) : NaN,
                sum(sub.exact_labels), sum(sub.relaxed_labels),
                sum(sub.exact_labels) > 0 ? sum(sub.relaxed_labels) / sum(sub.exact_labels) : NaN,
                isempty(ordinary) ? NaN : 100 * count(ordinary.fast_path) / nrow(ordinary),
                isempty(sf) ? "-" : @sprintf("%.2f%%", 100 * mean(sf)),
                count(.!sub.validity_ok), count(sub.false_certificate),
                isempty(certifiable) ? 0 : count(certifiable.relaxed_certifies), nrow(certifiable),
                count(.!sub.relaxed_exhausted))
        end
    end
    flush(stdout)

    try
        CSV.write(joinpath(results_dir, "pfaquant_n$(n_stations)_p$(N_PAIRS)_s$(SEED).csv"), df)
    catch e
        @warn "results CSV write failed" exception=(e, catch_backtrace())
    end
    return df
end

function main()
    length(ARGS) >= 1 || error("usage: diag_passenger_clock_quantization.jl <n_stations> [outdir]")
    n = parse(Int, ARGS[1])
    outdir = length(ARGS) >= 2 ? abspath(ARGS[2]) : pwd()
    results_dir = joinpath(outdir, "results")
    mkpath(results_dir)
    run_case(n, results_dir)
end

main()
