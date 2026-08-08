"""
    diagnose.jl clock_quantization -- the second relaxation axis: **quantize the
    clocks**, not the reward ladder.

`notes/2026-07-31_pfa_state_space_relaxation_design.md` measured the reward
axis (see the `reward_ladder_census` mode) and found the predicted mechanism
wrong: collapsing the layer universe 17x moved label count by only 23%, while
wall still dropped 2-4.5x. The win was a cheaper dominance scan per label, not
fewer labels -- which says `station_age`, not `activated_reward_layers`, is
what the PFA state space is made of.

# The relaxation

Floor the travel-time matrix to a grid `q`: `d'(i,j) = q * floor(d(i,j) / q)`.
The label search then advances `time`, `station_age` and `tau` on that
lattice.

**Valid, on both terms at once.** `d' <= d`, so along any route `tau' <= tau`
(cost down) and every clock is younger, so every ride-limit and pickup-window
test is easier and the certified reward can only go up. Hence
`rc'(r) <= rc(r)` for every route. Flooring accumulates no drift on ride
limits, because a ride's two endpoints are cumulative sums over the *same*
arcs.

**Why it should shrink the state.** `time` and the ages become multiples of
`q`, so labels that differed by a hair now collide exactly, and dominance
conditions 4 (`a.time <= b.time`) and 6 (the per-origin age walk) start
hitting. Coarser `q` means more collisions.

The travel matrix is re-closed (metric closure) after flooring: flooring each
arc independently can otherwise let a two-hop path undercut the direct arc,
breaking the triangle-inequality precondition `_passenger_free_assignment_age_is_useful`
relies on. `floor_min1` (forbid zero-length arcs) was dropped -- the closure
re-introduces short arcs anyway.

Also tests the reward ladder (`L`) alone and combined with quantization, to
see whether the two axes compose.

Usage:
    julia --project=. scripts/diagnose.jl clock_quantization <n_stations> [outdir]

Env overrides (all prefixed PFAQ_):
    N_PAIRS default 16   SEED default 42   N_SCENARIOS default 3
    MAX_STOPS default 5 (0 => unbounded)   MAX_VISITS default 3
    QUANTA default "1,2,4" (multiples of the min positive travel time)
    LADDER default 2 (reward-ladder L for the combined variant; 0 disables)
    LABEL_TIME default 300   MAX_ITERS default 200   CASE_TIME default 3000
"""

using Statistics

"""
Floor every arc to the grid `q`, then take the **metric closure** (see mode
docstring for why the closure is required, not cosmetic). Only pairs already
present in `travel_cost` are emitted: inventing an arc would let the relaxed
search return a route the exact data cannot even price.
"""
function _quantize_travel(travel_cost::Dict{Tuple{Int, Int}, Float64}, nodes::Vector{Int}, q::Float64)
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

function _min_positive_travel(travel_cost)
    best = Inf
    for (_k, v) in travel_cost
        (isfinite(v) && v > DIAG_LADDER_TOL) && (best = min(best, v))
    end
    return best
end

function run_clock_quantization(args::Vector{String})
    length(args) >= 1 || error("usage: diagnose.jl clock_quantization <n_stations> [outdir]")
    n = parse(Int, args[1])
    outdir = length(args) >= 2 ? abspath(args[2]) : pwd()
    results_dir = joinpath(outdir, "results")
    mkpath(results_dir)

    n_pairs = env_int("PFAQ_N_PAIRS", 16)
    seed = env_int("PFAQ_SEED", 42)
    n_scenarios = env_int("PFAQ_N_SCENARIOS", 3)
    max_stops = diag_unbounded(env_int("PFAQ_MAX_STOPS", 5))
    max_visits = env_int("PFAQ_MAX_VISITS", 3)
    quanta = env_floats("PFAQ_QUANTA", "1,2,4")
    ladder = env_int("PFAQ_LADDER", 2)
    label_time = env_float("PFAQ_LABEL_TIME", 300.0)
    max_iters = env_int("PFAQ_MAX_ITERS", 200)
    case_time = env_float("PFAQ_CASE_TIME", 3000.0)
    rc_tol = 1e-6

    exhaustive_search(pd, next_column_id) = begin
        t0 = time()
        cols, exhausted, stats = passenger_free_assignment_pricing_by_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=next_column_id, reduced_cost_tol=rc_tol,
            max_new_columns=typemax(Int) ÷ 2, n_candidates=typemax(Int) ÷ 2, time_limit=label_time,
        )
        (cols, exhausted, stats, time() - t0)
    end

    make_pricing_data(md, s, candidates, travel_cost) = StationSelection.create_passenger_free_assignment_pricing_data(
        s, md.nodes, travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time, repositioning_time=md.repositioning_time,
        max_stops=md.max_stops, max_visits_per_node=md.max_visits_per_node,
    )

    function measure_scenario(md, alpha, gamma_o, gamma_d, s, iter, n_stations, next_column_id)
        candidates = passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, s)
        isempty(candidates) && return PassengerFreeAssignmentRouteColumn[], NamedTuple[]
        exact_pd = make_pricing_data(md, s, candidates, md.travel_cost)
        isempty(exact_pd.opportunities) && return PassengerFreeAssignmentRouteColumn[], NamedTuple[]

        exact_cols, exact_exhausted, exact_stats, exact_wall = exhaustive_search(exact_pd, next_column_id)
        exact_rc = isempty(exact_cols) ? Inf : minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in exact_cols)
        exact_says_certified = exact_exhausted && isempty(exact_cols)

        base_q = _min_positive_travel(md.travel_cost)
        values_by_p = diag_reward_values_by_passenger(candidates)

        variants = Tuple{String, Int, Float64, Symbol}[]
        for mult in quanta
            push!(variants, ("q$(mult)", 0, mult, :floor))
        end
        ladder > 0 && push!(variants, ("L$(ladder)", ladder, 0.0, :none))
        ladder > 0 && !isempty(quanta) &&
            push!(variants, ("L$(ladder)+q$(first(quanta))", ladder, first(quanta), :floor))

        rows = NamedTuple[]
        for (label, L, mult, mode) in variants
            cand = diag_coarsen_candidates(candidates, values_by_p, L)
            tc = mode === :none ? md.travel_cost : _quantize_travel(md.travel_cost, md.nodes, mult * base_q)
            rel_pd = make_pricing_data(md, s, cand, tc)
            rel_cols, rel_exhausted, rel_stats, rel_wall = exhaustive_search(rel_pd, next_column_id)
            rel_rc = isempty(rel_cols) ? Inf : minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in rel_cols)

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
            fast_path = isfinite(best_true_rc) && best_true_rc < -rc_tol
            relaxed_certifies = rel_exhausted && isempty(rel_cols)
            false_certificate = relaxed_certifies && isfinite(exact_rc) && exact_rc < -rc_tol
            shortfall = (isfinite(best_true_rc) && isfinite(exact_rc) && exact_rc < -DIAG_LADDER_TOL) ?
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

            push!(rows, (n_stations=n_stations, scenario=s, iteration=iter, variant=label,
                ladder_L=L, quantum_mult=mult, quantize_mode=string(mode), base_quantum=base_q,
                exact_rc=exact_rc, relaxed_rc=rel_rc, best_true_rc=best_true_rc, shortfall=shortfall,
                exact_labels=exact_stats.labels_generated, relaxed_labels=rel_stats.labels_generated,
                exact_max_live=exact_stats.max_live_labels, relaxed_max_live=rel_stats.max_live_labels,
                exact_wall=exact_wall, relaxed_wall=rel_wall,
                exact_exhausted=exact_exhausted, relaxed_exhausted=rel_exhausted,
                exact_says_certified=exact_says_certified, relaxed_certifies=relaxed_certifies, fast_path=fast_path,
                worst_violation=isfinite(worst_violation) ? worst_violation : missing,
                validity_ok=validity_ok, false_certificate=false_certificate, n_relaxed_routes=length(rel_cols)))
        end
        flush(stdout)
        return exact_cols, rows
    end

    @printf("=== n=%d p=%d scenarios=%d l=%d ms=%s max_visits=%d quanta=%s ladder=%d ===\n",
        n, n_pairs, n_scenarios, diag_l_for(n), max_stops == typemax(Int) ? "unb" : string(max_stops),
        max_visits, string(quanta), ladder)
    flush(stdout)

    data, _meta = diag_zz_data(n; n_pairs=n_pairs, n_scenarios=n_scenarios, seed=seed)
    model = diag_zz_model(n; route_regularization_weight=1.0, max_stops=max_stops, max_visits_per_node=max_visits)
    mapping = create_map(model, data)
    md = StationSelection.create_passenger_free_assignment_master_data(model, data, mapping)
    master = build_passenger_free_assignment_master(md, diag_grb_env(); relax_integrality=true)
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
    MOI = JuMP.MOI

    while iter < max_iters
        if time() - t_start > case_time
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
            cols, rows = measure_scenario(md, alpha, gamma_o, gamma_d, s, iter, n, next_column_id)
            append!(all_rows, rows)
            for c in cols
                renumbered = PassengerFreeAssignmentRouteColumn(next_column_id, c.route, c.assignments, c.tau; metadata=c.metadata)
                next_column_id += 1
                _theta, action = StationSelection.add_passenger_free_assignment_column!(master, renumbered)
                action == :added && (added += 1)
            end
        end
        @printf("  [iter %3d] lp=%.4f added=%d pool=%d elapsed=%.1fs\n", iter, lp, added, length(master.theta), time() - t_start)
        flush(stdout)
        added == 0 && (stop_reason = "converged"; break)
    end

    df = DataFrame(all_rows)
    @printf("SUMMARY\tn=%d\tstop=%s\titers=%d\trows=%d\n", n, stop_reason, iter, nrow(df))
    if !isempty(df)
        for v in unique(df.variant)
            sub = filter(r -> r.variant == v, df)
            ordinary = filter(r -> r.exact_says_certified !== true, sub)
            certifiable = filter(r -> r.exact_says_certified === true, sub)
            sf = collect(skipmissing(ordinary.shortfall))
            @printf("SCORE\tn=%d\tvar=%-22s\twall %8.2f->%8.2f (%.2fx)\tlabels %9d->%9d (%.3f)\tfast=%.1f%%\tmean_shortfall=%s\tvalidity_viol=%d\tfalse_cert=%d\tcertified=%d/%d\ttimeouts=%d\n",
                n, v, sum(sub.exact_wall), sum(sub.relaxed_wall),
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
    diag_safe_csv_write(joinpath(results_dir, "pfaquant_n$(n)_p$(n_pairs)_s$(seed).csv"), df)
end

register_mode!("clock_quantization", run_clock_quantization)
