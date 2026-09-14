"""What do the real CG iterations DO to the dual, and how close does that get to the best dual
that exists?

Two measurements have already localised the activated oracle's weakness: the gold standard
says 90-99% of the missing cut strength is per-column slack no completion can see, and the
detour credit says the largest cost term a completion could still charge is worth 5.6-9.0%.
Both compared a completion against `gold`. Neither looked at what PLAIN CG -- full-station
pricing, raw LP duals, already implemented and already working -- actually produces.

That is the comparison this script makes, and it reframes the problem: `gold` is not a distant
target, it is roughly where plain CG already lands. The question is therefore not "how do we
reach gold" but "what do those full-station iterations produce, and can it be had cheaper".

# Three things, at the same anchors, on the same core point

1. **Four duals, one table.** closed-form completion / activated CG / plain CG / gold, scored
   on the same anchors against the same exact value function: `g_j` on unbuilt stations, the
   one-swap cut gap `Q(y') - C(y')`, the fraction of neighbours where the cut says nothing, and
   `min rc` over the ENUMERATED universe (which at `max_stops = ANA_MAX_STOPS` IS the universe,
   so that column is a proof of dual feasibility, not a sample).

2. **The dual's trajectory across inner CG iterations.** After every inner iteration of one
   subproblem, the dual point is scored the same way. This says whether the dual becomes
   gold-like early and then sits there, or only at convergence -- i.e. whether the expensive
   late iterations buy VALUE or only DUALS, which is the premise the whole activated family
   rests on.

   The loop here is a local copy of `_solve_joint_routing_assignment_subproblem_by_cg!`'s, not
   a callback into it. Deliberate: a diagnostic should not put a hook into the production loop
   it is measuring, and the copy is 20 lines calling the same `_run_pricing_round` /
   `extract_joint_routing_assignment_duals` / `add_joint_routing_assignment_column!` the real
   loop calls. If the two ever disagree, the `min rc` column will show it.

3. **Binding-row anatomy of the gold dual.** The gold LP's route rows are indexed by columns,
   and its ROW DUALS are exactly the `theta` weights -- so a row with a non-zero dual names a
   column that actually holds the gold dual in place. Those are the columns you would have to
   price to get a gold-quality dual. Reported: how many there are, their shape (stops, unbuilt
   stations touched, reduced cost at the closed-form duals), and -- the actionable part --
   whether they are already in the ACTIVATED pool or only in the plain-CG pool.

   That last question subsumes "could the accumulated pool's rows pin a near-gold dual", and
   answers it more usefully: if gold's binding set is small and has a recognisable shape, the
   activated oracle could price just those instead of the full station set.

Usage: sbatch --array=1-3 benchmarks/diagnostics/run_benders_dual_anatomy.sh
Env: ANA_N ANA_P ANA_S ANA_SEED ANA_MAX_STOPS ANA_ANCHORS ANA_TRACE_ANCHORS ANA_MAX_CG
"""

using StationSelection
using JuMP
using Gurobi
using Printf
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "ANA_N", "10"))
const P = parse(Int, get(ENV, "ANA_P", "8"))
const S = parse(Int, get(ENV, "ANA_S", "1"))
const SEED = parse(Int, get(ENV, "ANA_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "ANA_MAX_STOPS", "4"))
const N_ANCHORS = parse(Int, get(ENV, "ANA_ANCHORS", "8"))
const N_TRACE = parse(Int, get(ENV, "ANA_TRACE_ANCHORS", "3"))
const MAX_CG = parse(Int, get(ENV, "ANA_MAX_CG", "60"))
const TOL = 1e-6

problem, k, _meta = benchmark_problem(@__DIR__, "ANA", N, P, S, SEED)
data = problem.data
# `max_stops` EXPLICIT on the formulation as well as the oracle: that is what makes the
# enumerated pool the COMPLETE universe, and every `min rc` below a proof rather than a sample.
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops = MAX_STOPS)

make_solver(oracle) = BendersSolver(
    config = SolverOptions(silent = true), max_iterations = 500,
    subproblem = BendersSubproblemConfig(
        oracle = oracle, max_stops = MAX_STOPS, max_routes = 500_000,
        enumeration_time_limit_sec = 600.0, max_cg_iterations = MAX_CG,
        cg_pricing_time_limit_sec = 300.0))

enum_solver = make_solver(:direct_enumeration)
enum_build = build_model(problem, formulation, enum_solver)
enum_master = enum_build.model
mapping = enum_build.mapping
enum_subs = enum_master[:benders_subproblem_builds]
universe = [collect(values(b.model[:joint_routing_assignment_columns])) for b in enum_subs]
w = Float64(enum_subs[1].model[:joint_routing_assignment_walk_cost_weight])
beta = Float64(enum_subs[1].model[:joint_routing_assignment_route_regularization_weight])
@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d | universe %s columns\n",
        N, P, S, SEED, k, MAX_STOPS, join(length.(universe), "+"))
core_point, core_slack = StationSelection._benders_core_point(data, mapping, k)
@printf("core point: min slack %.4f | y^c in [%.3f, %.3f]\n",
        core_slack, minimum(core_point), maximum(core_point))
flush(stdout)

# ---------------------------------------------- master-feasible sets, exact value function
required = Set{Int}()
for s in 1:n_scenarios(data)
    for (p, (o, d)) in enumerate(mapping.Omega_s[s])
        mapping.Q_s[s][p] > 0 || continue
        any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
        push!(required, o); push!(required, d)
    end
end
cands = Dict(pt => Set(j for j in 1:N
                       if get_walking_cost(data, pt, j) <= mapping.max_walking_distance)
             for pt in required)
combos = filter(collect(combinations(1:N, k))) do c
    all(!isempty(intersect(cands[pt], c)) for pt in required)
end
as_y(c) = (y = zeros(Float64, N); y[c] .= 1.0; y)
combo_index = Dict(Tuple(sort(c)) => i for (i, c) in enumerate(combos))
Q = Matrix{Float64}(undef, length(combos), n_scenarios(data))
totals = Float64[]
for (i, c) in enumerate(combos)
    sub = StationSelection._solve_joint_routing_assignment_benders_subproblems(
        enum_master, as_y(c), enum_solver)
    for r in sub.scenarios; Q[i, r.scenario] = r.objective; end
    push!(totals, sub.total_objective)
end
order = sortperm(totals)
picks = unique(vcat(1, [1 + round(Int, (length(order) - 1) * i / max(1, N_ANCHORS - 1))
                        for i in 0:(N_ANCHORS - 1)]))
anchors = order[picks[1:min(end, N_ANCHORS)]]
trace_anchors = anchors[unique([1, (length(anchors) + 1) ÷ 2, length(anchors)])][1:min(end, N_TRACE)]
@printf("%d endpoint-feasible sets | %d anchors | tracing %d of them\n",
        length(combos), length(anchors), length(trace_anchors))
flush(stdout)

# ---------------------------------------------------------------- scoring one dual point
gamma_by_station(go, gd) = begin
    g = Dict{Int, Float64}()
    for gs in (go, gd), (key, v) in gs
        v == 0.0 && continue
        g[key[2]] = get(g, key[2], 0.0) + v
    end
    g
end
mean_of(v) = isempty(v) ? NaN : sum(v) / length(v)

"""Everything we compare a dual point on, at one anchor and one scenario. `min_rc` is over the
ENUMERATED universe, so it is dual feasibility itself rather than evidence for it."""
function score(s::Int, anchor_i::Int, alpha, go, gd)
    coef = gamma_by_station(go, gd)
    constant = sum(values(alpha); init = 0.0)
    built = combos[anchor_i]
    min_rc = Inf
    for column in universe[s]
        rc = joint_routing_assignment_column_cost(enum_subs[s].model, data, mapping, column)
        for (p, j, kk) in column.assignments
            key2 = (s, p)
            rc -= get(alpha, key2, 0.0)
            rc += get(go, (key2, j), 0.0) + get(gd, (key2, kk), 0.0)
        end
        min_rc = min(min_rc, rc)
    end
    gaps, dominated, n = Float64[], 0, 0
    for a in built, b in 1:N
        b in built && continue
        idx = get(combo_index, Tuple(sort(vcat(setdiff(built, [a]), b))), nothing)
        isnothing(idx) && continue
        pred = constant - sum(get(coef, j, 0.0) for j in combos[idx]; init = 0.0)
        push!(gaps, Q[idx, s] - pred)
        pred <= 0.0 && (dominated += 1)
        n += 1
    end
    worst_violation = -Inf
    for (i, c) in enumerate(combos)
        pred = constant - sum(get(coef, j, 0.0) for j in c; init = 0.0)
        worst_violation = max(worst_violation, pred - Q[i, s])
    end
    inactive = [get(coef, j, 0.0) for j in 1:N if !(j in built)]
    return (g_inactive = mean_of(inactive), constant = constant, min_rc = min_rc,
            swap_mean = mean_of(gaps), swap_max = isempty(gaps) ? NaN : maximum(gaps),
            dominated = n == 0 ? NaN : dominated / n, violation = worst_violation,
            core = constant - sum(core_point[j] * get(coef, j, 0.0) for j in 1:N; init = 0.0))
end

"""The gold dual over the ENUMERATED universe, with its route-row ConstraintRefs kept so the
BINDING rows can be read off afterwards -- the gold LP's row duals are the `theta` weights, so
a non-zero one names a column that actually holds the gold dual in place."""
function gold_dual(s::Int, incumbent::Vector{Float64}, z_star::Float64)
    sm = enum_subs[s].model
    gk = Tuple{Int,Int}[]; pr = Dict{Tuple{Int,Int}, Vector{Tuple{Int,Int}}}()
    wb = Dict{Tuple{Int,Int}, Float64}()
    pk = Tuple{Tuple{Int,Int},Int}[]; dk = Tuple{Tuple{Int,Int},Int}[]
    for (p, (o, d)) in enumerate(mapping.Omega_s[s])
        demand = mapping.Q_s[s][p]; demand > 0 || continue
        key2 = (s, p); push!(gk, key2)
        ps, so, sd = Tuple{Int,Int}[], Set{Int}(), Set{Int}()
        for pair in get_valid_jk_pairs(mapping, o, d)
            if is_walk_only_pair(pair)
                wb[key2] = w * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)
                continue
            end
            j, kk = pair; push!(ps, (j, kk))
            j in so || (push!(so, j); push!(pk, (key2, j)))
            kk in sd || (push!(sd, kk); push!(dk, (key2, kk)))
        end
        pr[key2] = ps
    end
    lp = Model(() -> Gurobi.Optimizer()); set_silent(lp)
    @variable(lp, a[key in gk] >= 0)
    @variable(lp, go[key in pk] >= 0)
    @variable(lp, gd[key in dk] >= 0)
    @objective(lp, Max, sum(a[key] for key in gk; init = 0.0) -
        sum(core_point[key[2]] * go[key] for key in pk; init = 0.0) -
        sum(core_point[key[2]] * gd[key] for key in dk; init = 0.0))
    @constraint(lp, sum(a[key] for key in gk; init = 0.0) -
        sum(go[key] for key in pk if incumbent[key[2]] > 0.5; init = 0.0) -
        sum(gd[key] for key in dk if incumbent[key[2]] > 0.5; init = 0.0) == z_star)
    for (key, b) in wb; @constraint(lp, a[key] <= b); end
    rows = Tuple{Any, Any}[]
    for column in universe[s]
        row = AffExpr(0.0)
        for (p, j, kk) in column.assignments
            key2 = (s, p)
            add_to_expression!(row, 1.0, a[key2])
            add_to_expression!(row, -1.0, go[(key2, j)])
            add_to_expression!(row, -1.0, gd[(key2, kk)])
        end
        push!(rows, (column,
            @constraint(lp, row <= joint_routing_assignment_column_cost(sm, data, mapping, column))))
    end
    optimize!(lp)
    termination_status(lp) == MOI.OPTIMAL || return nothing
    binding = [(col, abs(dual(con))) for (col, con) in rows if abs(dual(con)) > 1e-9]
    return (Dict(key => max(0.0, value(a[key])) for key in gk),
            Dict(key => max(0.0, value(go[key])) for key in pk),
            Dict(key => max(0.0, value(gd[key])) for key in dk),
            binding)
end

"""Run one subproblem's inner CG loop from a COLD pool, scoring the dual after every
iteration. A local copy of `_solve_joint_routing_assignment_subproblem_by_cg!`'s loop -- see
this file's header for why it is a copy and not a callback. Returns the per-iteration scores
and the final pool."""
function cg_trace(s::Int, anchor_i::Int, incumbent::Vector{Float64}, activated::Bool)
    solver = make_solver(activated ? :column_generation_activated : :column_generation)
    build = build_model(problem, formulation, solver)
    sub = build.model[:benders_subproblem_builds][s]
    sm = sub.model
    for j in eachindex(sm[:y])
        JuMP.fix(sm[:y][j], incumbent[j]; force = true)
    end
    settings = StationSelection._benders_subproblem_cg_settings(solver.subproblem)
    pf = sm[:joint_routing_assignment_pricing_formulation]
    trace = NamedTuple[]
    for it in 1:MAX_CG
        t_lp = time(); optimize!(sm); lp_sec = time() - t_lp
        termination_status(sm) == MOI.OPTIMAL || break
        obj = objective_value(sm)
        duals = extract_joint_routing_assignment_duals(sm)
        # The completion runs BEFORE pricing under the activated oracle -- that is what makes
        # the search built-only -- so the point scored here is the one a cut would be built
        # from at this iteration, not a raw intermediate.
        activated && StationSelection._benders_activated_complete_duals!(
            duals..., incumbent, data, mapping, s, w)
        sc = score(s, anchor_i, duals...)
        t_p = time()
        cols = StationSelection._run_pricing_round(
            pf, mapping, sm, duals, settings; only_scenarios = [s], time_limit = 120.0)
        price_sec = time() - t_p
        push!(trace, merge(sc, (iter = it, objective = obj, pool = length(sm[:joint_routing_assignment_columns]),
                                priced = length(cols), lp_sec = lp_sec, price_sec = price_sec)))
        isempty(cols) && break
        added = 0
        for c in cols
            _t, act = add_joint_routing_assignment_column!(sm, data, mapping, c)
            act === :added && (added += 1)
        end
        added == 0 && break
    end
    return trace, collect(values(sm[:joint_routing_assignment_columns]))
end

signature_of(c) = (Int(c.metadata["scenario"]),
                   StationSelection._joint_routing_assignment_column_signature(c))

# ================================================================ 1. four duals, one table
println("\n", "="^118)
println("1. FOUR DUALS AT THE SAME ANCHORS  (g_J0 = mean cut coefficient on unbuilt stations;")
println("   swap gap = mean Q(y') - C(y') over one-station swaps; dominated = swaps where the cut says nothing)")
println("="^118)
@printf("%-20s %2s %-12s %10s %12s %12s %10s %12s %12s\n",
        "y", "s", "dual", "g_J0", "swap mean", "swap max", "dominated", "min rc", "C(y^c)")

closed_solver = make_solver(:column_generation_activated)
closed_build = build_model(problem, formulation, closed_solver)
closed_subs = closed_build.model[:benders_subproblem_builds]
plain_solver = make_solver(:column_generation)
plain_build = build_model(problem, formulation, plain_solver)
plain_subs = plain_build.model[:benders_subproblem_builds]

summary = Dict{String, Vector{NamedTuple}}()
binding_report = NamedTuple[]
for anchor_i in anchors
    y = as_y(combos[anchor_i])
    sub_closed = StationSelection._solve_joint_routing_assignment_benders_subproblems(
        closed_build.model, y, closed_solver)
    sub_plain = StationSelection._solve_joint_routing_assignment_benders_subproblems(
        plain_build.model, y, plain_solver)
    by_closed = Dict(r.scenario => r for r in sub_closed.scenarios)
    for s in 1:n_scenarios(data)
        rows = Tuple{String, NamedTuple}[]

        # closed-form completion, off the ACTIVATED solve
        a1, o1, d1 = extract_joint_routing_assignment_duals(closed_subs[s].model)
        StationSelection._benders_activated_complete_duals!(a1, o1, d1, y, data, mapping, s, w)
        push!(rows, ("closed", score(s, anchor_i, a1, o1, d1)))

        # plain CG, RAW duals from full-station pricing -- no completion anywhere
        a2, o2, d2 = extract_joint_routing_assignment_duals(plain_subs[s].model)
        push!(rows, ("plain CG", score(s, anchor_i, a2, o2, d2)))

        g = gold_dual(s, y, by_closed[s].objective)
        if !isnothing(g)
            push!(rows, ("gold", score(s, anchor_i, g[1], g[2], g[3])))
            push!(binding_report, (anchor = anchor_i, scenario = s, binding = g[4]))
        end
        for (name, sc) in rows
            @printf("%-20s %2d %-12s %10.1f %12.2f %12.2f %9.1f%% %+12.2e %12.1f\n",
                    name == "closed" ? string(combos[anchor_i]) : "", s, name,
                    sc.g_inactive, sc.swap_mean, sc.swap_max, 100 * sc.dominated,
                    sc.min_rc, sc.core)
            push!(get!(summary, name, NamedTuple[]), sc)
        end
        flush(stdout)
    end
end

println("\n", "-"^118)
@printf("%-34s %10s %12s %12s %10s %12s\n",
        "MEAN over anchors x scenarios", "g_J0", "swap mean", "swap max", "dominated", "min rc")
for name in ("closed", "plain CG", "gold")
    haskey(summary, name) || continue
    v = summary[name]
    @printf("%-34s %10.1f %12.2f %12.2f %9.1f%% %+12.2e\n", name,
            mean_of([x.g_inactive for x in v]), mean_of([x.swap_mean for x in v]),
            mean_of([x.swap_max for x in v]), 100 * mean_of([x.dominated for x in v]),
            minimum(x.min_rc for x in v))
end
if haskey(summary, "gold") && haskey(summary, "closed") && haskey(summary, "plain CG")
    c = mean_of([x.swap_mean for x in summary["closed"]])
    p = mean_of([x.swap_mean for x in summary["plain CG"]])
    gg = mean_of([x.swap_mean for x in summary["gold"]])
    @printf("\nplain CG closes %.1f%% of the closed->gold distance on the one-swap gap (%.1f -> %.1f, gold %.1f)\n",
            100 * (c - p) / max(1e-9, c - gg), c, p, gg)
end
flush(stdout)

# ================================================ 2. the dual's trajectory across CG iterations
println("\n", "="^118)
println("2. DUAL TRAJECTORY ACROSS INNER CG ITERATIONS  (cold pool, one subproblem, scored every iteration)")
println("   The question: do the late, expensive iterations buy VALUE, or only DUALS?")
println("="^118)
traces = Dict{Tuple{Int,Int,Bool}, Vector{NamedTuple}}()
pools = Dict{Tuple{Int,Int,Bool}, Any}()
for anchor_i in trace_anchors
    y = as_y(combos[anchor_i])
    for s in 1:n_scenarios(data), activated in (false, true)
        tr, pool = cg_trace(s, anchor_i, y, activated)
        traces[(anchor_i, s, activated)] = tr
        pools[(anchor_i, s, activated)] = pool
        # The gold row on the same axes, for scale. `z*` is the converged LP objective, which
        # the audit verified equals the exact `Q_s(yhat)`; computed once per (anchor, scenario)
        # on the PLAIN pass so the two arms are compared against the same reference.
        gsc = nothing
        if !activated && !isempty(tr)
            gg = gold_dual(s, y, tr[end].objective)
            isnothing(gg) || (gsc = score(s, anchor_i, gg[1], gg[2], gg[3]))
        end
        @printf("\ny=%s s=%d  %s\n", string(combos[anchor_i]), s,
                activated ? "ACTIVATED (built-only pricing, closed-form completion)" :
                            "PLAIN CG (full-station pricing, raw duals)")
        @printf("  %4s %8s %8s %14s %10s %12s %12s %10s %12s %8s\n",
                "iter", "pool", "priced", "Q_s (LP obj)", "g_J0", "swap mean", "min rc",
                "dominated", "C(y^c)", "price s")
        for t in tr
            @printf("  %4d %8d %8d %14.4f %10.1f %12.2f %+12.2e %9.1f%% %12.1f %8.2f\n",
                    t.iter, t.pool, t.priced, t.objective, t.g_inactive, t.swap_mean,
                    t.min_rc, 100 * t.dominated, t.core, t.price_sec)
        end
        isnothing(gsc) || @printf("  %4s %8s %8s %14s %10.1f %12.2f %+12.2e %9.1f%% %12.1f\n",
                                  "GOLD", "-", "-", "-", gsc.g_inactive, gsc.swap_mean,
                                  gsc.min_rc, 100 * gsc.dominated, gsc.core)
        # The premise the activated family rests on: the VALUE is attained long before the
        # duals are. If `Q_s` stops moving early while `min rc` stays negative, every later
        # iteration is buying duals only -- which is exactly the claim, made visible.
        if length(tr) >= 2
            v_final = tr[end].objective
            v_hit = findfirst(t -> abs(t.objective - v_final) <= 1e-6 * max(1.0, abs(v_final)), tr)
            d_hit = findfirst(t -> t.min_rc >= -1e-6, tr)
            @printf("  -> value reached at iteration %s of %d; dual feasibility at %s\n",
                    string(v_hit), length(tr), isnothing(d_hit) ? "never" : string(d_hit))
        end
        flush(stdout)
    end
end

# ================================================ 3. binding-row anatomy of the gold dual
println("\n", "="^118)
println("3. BINDING-ROW ANATOMY OF THE GOLD DUAL")
println("   The gold LP's route-row duals ARE the theta weights, so a non-zero one names a column")
println("   that holds the gold dual in place. These are the columns you would have to price.")
println("="^118)
@printf("%-20s %2s %8s %10s %10s %10s %10s %12s %12s\n",
        "y", "s", "binding", "of pool", "med stops", "med unblt", "max unblt",
        "in activated", "in plain CG")
anat = NamedTuple[]
for br in binding_report
    anchor_i, s, binding = br.anchor, br.scenario, br.binding
    isempty(binding) && continue
    built = Set(combos[anchor_i])
    stops = [length(c.route) for (c, _w) in binding]
    unblt = [count(x -> !(x in built), unique(c.route)) for (c, _w) in binding]
    key = (anchor_i, s, true)
    in_act = haskey(pools, key) ?
        (sigs = Set(signature_of(c) for c in pools[key]);
         count(((c, _w),) -> signature_of(c) in sigs, binding)) : -1
    key2 = (anchor_i, s, false)
    in_plain = haskey(pools, key2) ?
        (sigs = Set(signature_of(c) for c in pools[key2]);
         count(((c, _w),) -> signature_of(c) in sigs, binding)) : -1
    med(v) = isempty(v) ? NaN : sort(v)[max(1, (length(v) + 1) ÷ 2)]
    @printf("%-20s %2d %8d %10d %10.1f %10.1f %10d %12s %12s\n",
            string(combos[anchor_i]), s, length(binding), length(universe[s]),
            med(stops), med(unblt), maximum(unblt),
            in_act < 0 ? "(not traced)" : "$in_act/$(length(binding))",
            in_plain < 0 ? "(not traced)" : "$in_plain/$(length(binding))")
    push!(anat, (n = length(binding), stops = med(stops), unblt = med(unblt),
                 max_unblt = maximum(unblt), in_act = in_act, in_plain = in_plain,
                 total = length(binding)))
    flush(stdout)
end

println("\n", "-"^118)
if !isempty(anat)
    @printf("gold's dual is held in place by %.1f columns on average (%.4f%% of the %d-column universe)\n",
            mean_of([Float64(x.n) for x in anat]),
            100 * mean_of([Float64(x.n) for x in anat]) / mean_of([Float64(length(u)) for u in universe]),
            length(universe[1]))
    @printf("their shape: median %.1f stops, median %.1f unbuilt stations touched (max %d)\n",
            mean_of([x.stops for x in anat]), mean_of([x.unblt for x in anat]),
            maximum(x.max_unblt for x in anat))
    traced = filter(x -> x.in_act >= 0, anat)
    if !isempty(traced)
        @printf("coverage of that binding set: ACTIVATED pool %.1f%%, plain-CG pool %.1f%%\n",
                100 * sum(x.in_act for x in traced) / max(1, sum(x.total for x in traced)),
                100 * sum(x.in_plain for x in traced) / max(1, sum(x.total for x in traced)))
        println("\nREAD THIS AS: if the activated pool already contains gold's binding columns, the")
        println("gap is in which DUAL the LP picks, not in which columns were priced -- and an")
        println("optimal-face reoptimisation over the pool's own rows would close it. If it does")
        println("not contain them, the gap is a PRICING gap and no reweighting of the pool can fix it.")
    end
end
println("\nDONE")
