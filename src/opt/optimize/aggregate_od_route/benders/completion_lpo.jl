"""
Locally Pareto-optimal dual completion for the ACTIVATED Benders subproblem
(`oracle = :column_generation_activated_lpo`).

The activated oracle prices only over the built stations `S` and repairs the duals on the
unbuilt ones with a closed-form bound. That bound is deliberately conservative -- it forces
every out-of-`S` triple's net reward non-positive one triple at a time -- and it is why
activated cuts are weak: measured at n=10, 77 cuts / 30 iterations versus 5 / 4 for
enumeration and plain CG, for the same optimum.

This file replaces the *cut's* completion with the strongest one that is still valid, chosen
in the Magnanti-Wong sense against an interior point.

# What is fixed, what is free

Fixed from the activated solve at `yhat`: `alpha_p` for every group, and `gamma` on the
linking rows of BUILT stations. Free: `g >= 0`, one variable per linking row of an unbuilt
station. Write `gamma~` for the combination.

# The constraint family

Dual feasibility needs, for every column `c` in the universe,
`sum_{A(c)} [alpha_p - gamma~O - gamma~D] <= f_c`. Splitting `A(c)` by whether both stations
are built:

- `A_out(c) = {}` -- every term is fixed and the constraint ALREADY holds, because the
  activated pricing exhausted over exactly those columns. These impose nothing on `g`, which
  is what makes fixing the activated duals legitimate rather than an approximation.
- `A_out(c) != {}` -- with `L_in(c) = sum_{A_in} [alpha_p - gammaO - gammaD]` known,

      sum_{free g in A_out(c)}  g  >=  b(c)
      b(c) = sum_{A_out} alpha_p - f_c + L_in(c) - (fixed gamma terms in A_out)

So the completion's feasible set is a **covering polyhedron**, one row per column that touches
an unbuilt station.

# The objective

The cut is `Theta_s >= sum alpha - sum_j Gamma~_j y_j`, so a stronger cut means smaller `g`.
Evaluating at an interior point `y0` and dropping the fixed part:

    (LPC)  min  sum_{(p,j): j not in S} y0_j * g_pj
           s.t. sum_{free g in A_out(c)} g >= b(c)   for all c touching an unbuilt station
                g >= 0

**Why this is Pareto-optimal.** If some feasible `g'` dominated the optimum `g*` -- cut value
at least as high everywhere, strictly higher somewhere -- then because `y0` is in the
RELATIVE INTERIOR of the master's feasible hull, that strict gain shows up strictly at `y0`,
contradicting optimality of `g*`. Magnanti-Wong's argument, verbatim.

**Why "locally".** Non-dominated only within the family reachable by completing THIS
`(alpha, gamma_S)`; we do not re-optimize the activated duals jointly. And note MW's usual
normalisation constraint (`keep the dual optimal at yhat`) is **vacuous here**: every free
variable sits on a coordinate with `yhat_j = 0`, so every feasible completion is already
tight at `yhat`. That removes the perturbation step whose primal/dual mismatch caused the
historical MW completion-infeasibility bug.

# Two constraint families for `(LPC)`, and the one that works

`(LPC)` has one row per column, i.e. exponentially many. There are two ways to handle that,
both implemented, selected by `BendersSubproblemConfig.lpo_completion`.

## `:separation` (default) -- generate the real rows

The rows above ARE the constraint set; they are just not all written down:

1. **Seed** from columns already in the pool that touch an unbuilt station. The pool
   accumulates across Benders iterations, so it already holds columns through stations that
   are unbuilt at THIS incumbent -- real rows at zero search cost, and likely the binding
   ones.
2. Solve `(LPC)` over the known rows.
3. **Separate** by pricing with `gamma~` set to the candidate `g`. Any column returned with
   negative reduced cost is a violated row. Note a column entirely inside `S` cannot be
   returned, since `gamma~` agrees with `gamma_S` there and the activated pricing already
   exhausted those -- so every violation genuinely touches an unbuilt station.
4. No violation AND the round exhausted ⇒ the candidate is feasible for ALL rows and optimal
   over a relaxation of the true problem, hence optimal. Otherwise add the row(s) and repeat.

**Validity.** A cut is valid iff the dual point it is read off is dual-FEASIBLE -- optimality
at `yhat` only makes it tight. `rc(c) >= 0` for every `c` IS dual feasibility, so an
exhausted round returning nothing below `-reduced_cost_tol` certifies exactly that. An
intermediate `(LPC)` solution is optimal only over the *generated* rows, so it may violate an
ungenerated one and is **not** automatically valid; the loop therefore tracks the last
separation-verified completion, initialised to the closed-form bound, and returns that.
Truncating on budget yields a cut that is valid and never worse than the closed form --
soundness never depends on the row generation converging.

**Cost.** Each round is a full label-setting search per scenario, and lowering `gamma` on the
unbuilt stations re-admits them to the pricer's filter, so that search is over the whole
universe. This is the expense the activated oracle exists to avoid, which is why the
route-free family below was tried.

## `:route_free` -- a sufficient family the certificate licenses outright, NO pricing

The activated loop installs the closed-form completion **before** pricing, deliberately, so
the exhaustion it certifies is over the FULL universe at those duals. Editing `g` only touches
linking rows of unbuilt stations, so a column with `A_out(c) = {}` has its reduced cost
unchanged by ANY completion and stays feasible for free -- those rows are discharged
permanently.

The columns that DO touch an unbuilt station are discharged by *shortcutting*. Given `c`, let
`c'` be `c` with the unbuilt stations dropped from its visit sequence, serving `A_in(c)` only:

    tau_{c'} <= tau_c                                  (triangle inequality)
    f_c = f_{c'} + w * sum_{A_out} demand_p * walk_p    (the dropped triples' walking)

`c'` visits only built stations, so `sum_{A_in} [alpha - gammaO - gammaD] <= f_{c'}` is the
carried-over certificate, and the row for `c` then follows from

    sum_{A_out} [alpha_p - g^O_pj - g^D_pk]  <=  w * sum_{A_out} demand_p * walk_p

which is implied one term at a time. So the family collapses to a condition on individual
triples, with no reference to any route:

    (T)  g^O_pj + g^D_pk  >=  alpha_p - w * demand_p * walk(o_p, d_p, (j,k))
         for every valid triple whose pickup or dropoff station is unbuilt

`tau_{c'} <= tau_c` is where the triangle inequality enters. The travel matrix is required to
be metric package-wide already (the pricer's age pruning assumes it and asserts on violation),
so this is an existing precondition. Shortcutting also cannot break feasibility of the
retained triples: it removes stops, preserves their order, and only shortens ride and wait
times.

**`(T)` is SUFFICIENT, not equivalent -- it is a STRONGER requirement than `(LPC)`'s rows, so
its feasible set is a SUBSET.** Never mix `(T)` rows into the separation LP: they would
over-restrict it and throw away exactly the strength separation exists to find.

**MEASURED, and it does not work** (n=10 s=3 seed 42, same instance, all four oracles agreeing
on 25771.187908):

    closed-form (plain activated)   30 iters   77 cuts    gain/completion   --
    (T), route-free                 30 iters   77 cuts    gain/completion 1181
    separation                       3 iters    6 cuts    gain/completion 5985
    plain :column_generation          4 iters    5 cuts

`(T)` is valid (weaker cuts are the safe direction, and an unsound completion would prune the
optimum and show up as a higher objective) but recovers NONE of the strength. In hindsight the
reason is structural: `(T)` forces each out-of-`S` triple non-positive *in isolation*, which is
the same termwise shape as the closed-form bound it replaces -- all it can add is the
`demand_p` factor and letting the two endpoints split the requirement. The strength lives in
`sum_{A_in} net_in <= f_{c'}` being slack, i.e. in most columns being nowhere near binding,
and only separation against the real pricer can see that.

# `(T)` and the closed form

The closed-form completion sets `gamma^O_pj = max(0, alpha_p - w * min_k walk)` -- it satisfies
`(T)` by loading the entire requirement onto ONE side, at the cheapest partner. So the
closed-form point is feasible for `(T)` (and for `(LPC)`, which `(T)` implies), which makes it
a sound fallback for either family and means the route-free LP can only match or beat it;
`(T)`'s branch asserts that rather than assuming it. It also drops the `demand_p` factor the
column cost actually charges, which for `demand_p >= 1` is conservative -- valid, just slack.
"""

using JuMP

"""
    _benders_core_point(data, mapping, k) -> Vector{Float64}

An interior point of the master's feasible region, by maximising the minimum slack:

    max  t
    s.t. sum_j y_j == k
         sum_{j near pt} y_j >= 1 + t     for every required location
         t <= y_j <= 1 - t
         t >= 0

The `t <= y_j <= 1 - t` rows are what push the point off the 0/1 bounds; without them the LP
would happily return a vertex, and the Pareto argument needs the RELATIVE INTERIOR -- at a
boundary point a dominating completion need not be strictly better there, and the argument
collapses.

Uniform `y_j = k/n` is not good enough: it satisfies the budget but can sit exactly on an
endpoint-feasibility row when a required location has fewer than `n/k` candidate stations,
which is the boundary case the argument excludes.

`t* == 0` means some face is structurally tight (a required location with a single candidate
forces that `y_j = 1`). That coordinate is in the affine hull of every feasible point, so the
returned point is still relatively interior in the directions that matter; the value is
reported so a caller can see it happened.
"""
function _benders_core_point(
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        k::Int,
    )::Tuple{Vector{Float64}, Float64}
    n = data.n_stations
    required = Set{Int}()
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
            push!(required, o)
            push!(required, d)
        end
    end

    m = Model(() -> Gurobi.Optimizer())
    set_silent(m)
    @variable(m, 0.0 <= y[1:n] <= 1.0)
    @variable(m, t >= 0.0)
    @constraint(m, sum(y) == k)
    for pt in sort!(collect(required))
        candidates = [j for j in 1:n
                      if get_walking_cost(data, pt, j) <= mapping.max_walking_distance]
        isempty(candidates) && continue
        @constraint(m, sum(y[j] for j in candidates) >= 1 + t)
    end
    for j in 1:n
        @constraint(m, y[j] >= t)
        @constraint(m, y[j] <= 1 - t)
    end
    @objective(m, Max, t)
    optimize!(m)
    if termination_status(m) != MOI.OPTIMAL
        # Fall back to the uniform point rather than failing the solve. It may sit on a
        # face, which weakens the Pareto claim to "non-dominated among cuts that agree on
        # that face" -- reported via the returned slack of 0.
        return fill(k / n, n), 0.0
    end
    return Float64.(value.(y)), Float64(value(t))
end
"""
    _benders_lpo_completion!(alpha, gamma_o, gamma_d, incumbent, data, mapping, scenario,
                             build, config, core_point) -> NamedTuple

Overwrite the unbuilt-station entries of `gamma_o`/`gamma_d` with a locally Pareto-optimal
completion, leaving built-station entries and `alpha` untouched.

`config.lpo_completion` picks the constraint family (`:separation`, the default, or
`:route_free`) -- see this file's header for the two and for the measurement that separates
them. Both leave the cut's value at `yhat` untouched, so both stay tight there.

Callers must have applied the closed-form completion first: it is this function's feasible
fallback for either family, and the reference the reported gain is measured against.
"""
function _benders_lpo_completion!(
        alpha::Dict{Tuple{Int, Int}, Float64},
        gamma_o::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
        gamma_d::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
        incumbent::Vector{Float64},
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        scenario::Int,
        build::BuildResult,
        config::BendersSubproblemConfig,
        core_point::Vector{Float64},
    )
    sm = build.model
    built(j) = incumbent[j] > 0.5

    # ---- the free variables: one per unbuilt station's linking row ----
    free_o = Tuple{Tuple{Int, Int}, Int}[]
    free_d = Tuple{Tuple{Int, Int}, Int}[]
    for (p, (o, d)) in enumerate(mapping.Omega_s[scenario])
        mapping.Q_s[scenario][p] > 0 || continue
        key2 = (scenario, p)
        seen_o, seen_d = Set{Int}(), Set{Int}()
        for pair in get_valid_jk_pairs(mapping, o, d)
            is_walk_only_pair(pair) && continue
            j, kk = pair
            if !built(j) && !(j in seen_o)
                push!(seen_o, j); push!(free_o, (key2, j))
            end
            if !built(kk) && !(kk in seen_d)
                push!(seen_d, kk); push!(free_d, (key2, kk))
            end
        end
    end
    isempty(free_o) && isempty(free_d) && return (rounds=0, rows=0, improved=0.0,
                                                  status="nothing_free", pricing_sec=0.0)

    # The closed-form values already in the dicts: our feasible fallback and the reference
    # for how much the LPO version improved.
    fallback_o = Dict(key => get(gamma_o, key, 0.0) for key in free_o)
    fallback_d = Dict(key => get(gamma_d, key, 0.0) for key in free_d)
    core_value(vo, vd) = sum(core_point[key[2]] * v for (key, v) in vo; init=0.0) +
                         sum(core_point[key[2]] * v for (key, v) in vd; init=0.0)
    baseline = core_value(fallback_o, fallback_d)

    lp = Model(() -> Gurobi.Optimizer())
    set_silent(lp)
    @variable(lp, go[key in free_o] >= 0.0)
    @variable(lp, gd[key in free_d] >= 0.0)
    @objective(lp, Min,
        sum(core_point[key[2]] * go[key] for key in free_o; init=0.0) +
        sum(core_point[key[2]] * gd[key] for key in free_d; init=0.0))
    free_o_set, free_d_set = Set(free_o), Set(free_d)

    """Add `sum(free g in A_out(c)) >= b(c)` for one column -- literally `(R_c)` with the
    free coordinates moved to the left. Returns false if the column touches no unbuilt
    station, in which case it constrains nothing (the activated certificate already holds
    it, for ANY completion)."""
    function add_row!(column)
        s = Int(column.metadata["scenario"])
        s == scenario || return false
        terms = AffExpr(0.0)
        rhs = -joint_routing_assignment_column_cost(sm, data, mapping, column)
        any_free = false
        for (p, j, kk) in column.assignments
            key2 = (s, p)
            a = get(alpha, key2, 0.0)
            if built(j) && built(kk)
                # Fully-built triple: contributes its fixed net to L_in(c).
                rhs += a - get(gamma_o, (key2, j), 0.0) - get(gamma_d, (key2, kk), 0.0)
                continue
            end
            rhs += a
            if built(j)
                rhs -= get(gamma_o, (key2, j), 0.0)
            elseif (key2, j) in free_o_set
                add_to_expression!(terms, go[(key2, j)]); any_free = true
            end
            if built(kk)
                rhs -= get(gamma_d, (key2, kk), 0.0)
            elseif (key2, kk) in free_d_set
                add_to_expression!(terms, gd[(key2, kk)]); any_free = true
            end
        end
        any_free || return false
        @constraint(lp, terms >= rhs)
        return true
    end

    if config.lpo_completion === :route_free
        # ---- (T), written out in full: one row per (group, pair) with an unbuilt end ----
        # A SUFFICIENT family, not a subset of (LPC)'s rows -- so these must never be mixed
        # with `add_row!`'s rows, which are the real ones. See the header.
        walk_cost_weight = Float64(sm[:joint_routing_assignment_walk_cost_weight])
        n_rows = 0
        for (p, (o, d)) in enumerate(mapping.Omega_s[scenario])
            demand = mapping.Q_s[scenario][p]
            demand > 0 || continue
            key2 = (scenario, p)
            a = get(alpha, key2, 0.0)
            a > 0.0 || continue          # a <= 0 is satisfied by g >= 0 alone
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, kk = pair
                (built(j) && built(kk)) && continue   # discharged by the certificate

                rhs = a - walk_cost_weight * demand * od_pair_walking_cost(data, o, d, pair)
                rhs > 0.0 || continue

                terms = AffExpr(0.0)
                if built(j)
                    rhs -= get(gamma_o, (key2, j), 0.0)
                elseif (key2, j) in free_o_set
                    add_to_expression!(terms, go[(key2, j)])
                end
                if built(kk)
                    rhs -= get(gamma_d, (key2, kk), 0.0)
                elseif (key2, kk) in free_d_set
                    add_to_expression!(terms, gd[(key2, kk)])
                end
                # A built partner's fixed gamma can already cover the requirement alone.
                rhs > 0.0 || continue
                @constraint(lp, terms >= rhs)
                n_rows += 1
            end
        end

        optimize!(lp)
        if termination_status(lp) != MOI.OPTIMAL
            # The closed-form point is feasible for (T), so this should not happen; keep the
            # fallback rather than installing an unverified candidate.
            return (rounds=1, rows=n_rows, improved=0.0,
                    status="lp_$(termination_status(lp))", pricing_sec=0.0)
        end
        got_o = Dict(key => max(0.0, value(go[key])) for key in free_o)
        got_d = Dict(key => max(0.0, value(gd[key])) for key in free_d)
        achieved = core_value(got_o, got_d)
        # (T) admits the closed-form point, so the LP can only match or beat it.
        achieved <= baseline + 1e-6 || error(
            "the route-free LPO completion came out worse at the core point than the " *
            "closed-form bound it relaxes ($(achieved) vs $(baseline)); (T) admits the " *
            "closed-form point, so this means the row set or objective is wrong",
        )
        for (key, v) in got_o; gamma_o[key] = v; end
        for (key, v) in got_d; gamma_d[key] = v; end
        return (rounds=1, rows=n_rows, improved=baseline - achieved, status="optimal",
                pricing_sec=0.0)
    end

    # ---- :separation -- the real rows of (LPC), generated on demand ----
    n_rows = 0
    for (_id, column) in sm[:joint_routing_assignment_columns]
        add_row!(column) && (n_rows += 1)
    end

    settings = _benders_subproblem_cg_settings(config)
    pricing_formulation = sm[:joint_routing_assignment_pricing_formulation]
    verified_o, verified_d = fallback_o, fallback_d
    status = "row_limit"
    rounds = 0
    # Separation prices, and that pricing is NOT part of the CG loop's own budget or its
    # `pricing_sec`. It has to be reported separately or the LPO oracle looks free when it
    # is not -- the entire question about this oracle is whether its pricing is cheaper than
    # the plain one's, and an uncounted round would answer it wrongly.
    sep_pricing_sec = 0.0

    for round in 1:config.lpo_max_rounds
        rounds = round
        optimize!(lp)
        if termination_status(lp) != MOI.OPTIMAL
            status = "lp_$(termination_status(lp))"
            break
        end
        cand_o = Dict(key => max(0.0, value(go[key])) for key in free_o)
        cand_d = Dict(key => max(0.0, value(gd[key])) for key in free_d)

        # Install the candidate and separate by pricing against it. Lowering gamma on the
        # unbuilt stations un-restricts the pricer's own filter, so this round searches the
        # FULL universe -- which is what makes its exhaustion a full-universe certificate,
        # and also what it costs.
        for (key, v) in cand_o; gamma_o[key] = v; end
        for (key, v) in cand_d; gamma_d[key] = v; end
        t_sep = time()
        columns = _run_pricing_round(
            pricing_formulation, mapping, sm, (alpha, gamma_o, gamma_d), settings;
            only_scenarios = [scenario],
            time_limit = config.lpo_pricing_time_limit_sec,
        )
        sep_pricing_sec += time() - t_sep
        if isempty(columns) && _cg_pricing_exhausted(sm)
            # Feasible for every row, and optimal over a relaxation of the true constraint
            # set -- hence optimal. This is the Pareto-optimal completion.
            verified_o, verified_d = cand_o, cand_d
            status = "optimal"
            break
        end
        added = 0
        for column in columns
            add_row!(column) && (added += 1)
        end
        n_rows += added
        if added == 0
            # Violations exist (or the search was truncated) but none produced a row we can
            # act on. Keep the last verified completion rather than the unvalidated
            # candidate.
            status = isempty(columns) ? "separation_inconclusive" : "no_actionable_row"
            break
        end
    end

    # Whatever we return must be separation-verified; the closed-form fallback is.
    for (key, v) in verified_o; gamma_o[key] = v; end
    for (key, v) in verified_d; gamma_d[key] = v; end
    achieved = core_value(verified_o, verified_d)
    return (rounds=rounds, rows=n_rows, improved=baseline - achieved, status=status,
            pricing_sec=sep_pricing_sec)
end
