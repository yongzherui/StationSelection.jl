"""Export the full column, route, dual and cut anatomy for the activated vs full-pricing
comparison, so the difference can be explored rather than summarised.

The activated oracle (closed-form completion, NO row generation) and plain full-station CG
solve the identical model, reach the identical `Q_s(yhat)`, and produce cuts differing by 8x
in coefficient mass (`notes/2026-09-10_activated_dual_completion_verified_and_why_weak.md`).
The difference must be visible in the COLUMNS. This script writes every column with its
route, assignments, cost, pool membership and reduced cost under both arms' duals, plus the
duals themselves, so the question "what do the extra columns buy" can be asked directly of
the data.

# Design: the enumerated pool is the spine -- but it is NOT the whole universe

MEASURED, and it corrects an earlier claim in this repo: `enumerate_joint_routing_assignment_columns`
emits, per physical route, the assignment sets that route can CERTIFY -- and the pricer emits
the sub-assignment that is optimal at the current duals, keeping only triples with positive
reward. Those subsets are separate columns with separate dual constraints, and 329 of them
turned up in the arms' pools while being absent from the enumeration (job 22493545), plus ~57
rotations of revisiting cycles (`9>3>9` where the enumeration holds `3>9>3`).

So "the enumerated pool IS the universe" is false, and any test that reads a minimum over it
as a full-universe statement is an UPPER bound on the true minimum, not the minimum.
`route_min_rc.csv` fixes that analytically: for each route the reduced-cost-minimising column
keeps exactly the triples whose net contribution is negative, one triple per group, so

    min over the TRUE universe  =  min over routes R of
        [ beta*(tau_R + rho) + sum_p min(0, min over p's triples on R of net) ]
    net(p,j,k) = w*demand_p*walk(o,d,(j,k)) - (alpha_p - gammaO_pj - gammaD_pk)

which needs no enumeration of subsets at all -- the triple sets come from the union of the
enumerated columns on that route, and the minimisation is closed form. That is the number to
read for dual feasibility.

The export still uses the enumerated pool as its spine, marking which arm's pool contains
each column (`in_plain`, `in_activated`), because set membership answers the structural
questions by filtering rather than joining:

    in_plain & !in_activated     what full-station pricing found and built-only did not
    n_unbuilt_assign > 0         columns pinned to theta = 0 at this incumbent -- they
                                 exist only for their dual constraint
    visits_unbuilt & n_unbuilt_assign == 0   columns the restricted search cannot reach even
                                 though nobody is served at the unbuilt stop (candidate
                                 generation is reward-driven, so hiding a station's
                                 candidates hides the station from the route search -- this
                                 is why the soundness proof compares against `c''`, the route
                                 with the stop DELETED, and not `c'`, the route with only the
                                 assignment dropped)

`pool_not_in_universe.csv` lists every pooled column absent from the enumeration, with its
reduced cost under both arms and its relation to the enumerated columns on the same route
(`subset` / `route_absent` / `incomparable`). It is EXPECTED to be non-empty -- see above.
What would be alarming is an `incomparable` or `superset` row, which would mean the two
disagree about what a route can certify rather than merely about which subsets to list.

# The ingredient, not just the derived number

`gamma_pj` for an unbuilt `j` is absent from the dual objective (its coefficient is
`yhat_j = 0`), so the LP is indifferent to it and only the dual CONSTRAINTS pin it down. For
a column whose unbuilt assignment stations are exactly `{j}`, its constraint says

    sum_{assignments at j} gamma_pj  >=  excess_over_built(c)
    excess_over_built(c) = sum_p alpha_p - sum_{built-station assignments} gamma - f_c

so grouping the export by `unbuilt_assign_stations` and taking the max of
`excess_over_built` reproduces the per-station requirement. That column is exported per
column rather than pre-aggregated, so the grouping can be done any way -- by station, by
station SET (columns touching two unbuilt stations constrain a sum, not one coordinate), by
passenger count, by route length.

`station_need.csv` has the single-station aggregate as a convenience, next to what the closed
form charges and what plain CG's duals actually carry.

Usage: sbatch benchmarks/diagnostics/run_benders_pool_and_cut_anatomy.sh
Env: PA_N PA_P PA_S PA_SEED PA_MAX_STOPS PA_ANCHORS PA_OUT
"""

using StationSelection
using JuMP
using Printf
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "PA_N", "10"))
const P = parse(Int, get(ENV, "PA_P", "8"))
const S = parse(Int, get(ENV, "PA_S", "1"))
const SEED = parse(Int, get(ENV, "PA_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "PA_MAX_STOPS", "4"))
const N_ANCHORS = parse(Int, get(ENV, "PA_ANCHORS", "4"))
const TAG = "n$(N)_p$(P)_s$(S)_seed$(SEED)_ms$(MAX_STOPS)"
const OUT = get(ENV, "PA_OUT", joinpath(@__DIR__, "results", "pool_anatomy", TAG))
mkpath(OUT)

problem, k, _meta = benchmark_problem(@__DIR__, "PA", N, P, S, SEED)
data = problem.data
monolith = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops = MAX_STOPS)

make_solver(oracle) = BendersSolver(
    config = SolverOptions(silent = true),
    max_iterations = 500,
    subproblem = BendersSubproblemConfig(
        oracle = oracle, max_stops = MAX_STOPS, max_routes = 500_000,
        enumeration_time_limit_sec = 600.0, max_cg_iterations = 500,
        cg_pricing_time_limit_sec = 300.0,
    ),
)

@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d\nout: %s\n\n",
        N, P, S, SEED, k, MAX_STOPS, OUT)
flush(stdout)

# ---------------------------------------------------------------- the universe
enum_solver = make_solver(:direct_enumeration)
enum_build = build_model(problem, monolith, enum_solver)
enum_master = enum_build.model
mapping = enum_build.mapping
enum_subs = enum_master[:benders_subproblem_builds]
universe = [collect(values(b.model[:joint_routing_assignment_columns])) for b in enum_subs]
walk_w = Float64(enum_subs[1].model[:joint_routing_assignment_walk_cost_weight])
beta = Float64(enum_subs[1].model[:joint_routing_assignment_route_regularization_weight])
rho_repo = Float64(enum_subs[1].model[:joint_routing_assignment_repositioning_time])
@printf("enumerated universe: %s columns | walk weight %.4g | drive weight %.4g\n",
        join(length.(universe), "+"), walk_w, beta)

# Per physical route, every (group -> station pair) the enumeration says it can certify, as
# the UNION over the enumerated columns on that route. The enumeration lists sub-assignments
# selectively, but the union of what it lists is the route's full certifiable set -- which is
# all the closed-form minimiser below needs.
route_triples = [Dict{Tuple, Dict{Int, Set{Tuple{Int, Int}}}}() for _ in 1:n_scenarios(data)]
route_tau = [Dict{Tuple, Float64}() for _ in 1:n_scenarios(data)]
for s in 1:n_scenarios(data), c in universe[s]
    key = Tuple(c.route)
    d = get!(route_triples[s], key) do; Dict{Int, Set{Tuple{Int, Int}}}(); end
    for (p, j, kk) in c.assignments
        push!(get!(d, p) do; Set{Tuple{Int, Int}}(); end, (j, kk))
    end
    route_tau[s][key] = c.tau
end
@printf("distinct physical routes: %s\n", join((length(route_tau[s]) for s in 1:n_scenarios(data)), "+"))

# ------------------------------------------- the master's feasible set, and anchors
required = Set{Int}()
for s in 1:n_scenarios(data), (p, (o, d)) in enumerate(mapping.Omega_s[s])
    mapping.Q_s[s][p] > 0 || continue
    any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
    push!(required, o); push!(required, d)
end
cands = Dict(pt => Set(j for j in 1:N
                       if get_walking_cost(data, pt, j) <= mapping.max_walking_distance)
             for pt in required)
combos = filter(c -> all(!isempty(intersect(cands[pt], c)) for pt in required),
                collect(combinations(1:N, k)))
as_y(combo) = (y = zeros(Float64, N); y[combo] .= 1.0; y)

per_scen = Vector{Vector{Float64}}(undef, length(combos))
totals = Vector{Float64}(undef, length(combos))
for (i, c) in enumerate(combos)
    r = StationSelection._solve_joint_routing_assignment_benders_subproblems(
        enum_master, as_y(c), enum_solver)
    per_scen[i] = Float64[x.objective for x in r.scenarios]
    totals[i] = r.total_objective
end
order = sortperm(totals)
picks = unique([1 + round(Int, (length(order) - 1) * t / max(1, N_ANCHORS - 1))
                for t in 0:(N_ANCHORS - 1)])
anchors = order[picks]
@printf("%d endpoint-feasible station sets | anchors by Q rank: %s\n\n",
        length(combos), join((string(combos[i]) for i in anchors), ", "))
flush(stdout)

# ---------------------------------------------------------------- helpers
signature(c) = (Tuple(c.route), Tuple(sort(c.assignments)))
join_ids(xs) = join(sort(collect(xs)), "|")
route_str(c) = join(c.route, ">")
assign_str(c) = join(("$(p):$(j):$(kk)" for (p, j, kk) in sort(c.assignments)), "|")
assign_stations(c) = Set(Iterators.flatten(((j, kk) for (_p, j, kk) in c.assignments)))
cost(s, c) = joint_routing_assignment_column_cost(enum_subs[s].model, data, mapping, c)

"""Reduced cost of `c` at duals `(a, go, gd)` -- the dual constraint is `rc >= 0`."""
function rc_at(s, c, a, go, gd)
    v = cost(s, c)
    for (p, j, kk) in c.assignments
        v -= get(a, (s, p), 0.0)
        v += get(go, ((s, p), j), 0.0) + get(gd, ((s, p), kk), 0.0)
    end
    return v
end

"""The reduced cost of the BEST column on route `key`, over every sub-assignment -- which is
the minimiser, since a column may serve each group at most once and including a triple whose
net contribution is positive can only raise the reduced cost. No subset enumeration needed.

    rc_min(R) = beta*(tau_R + rho) + sum_p min(0, min over p's triples on R of net)
    net(p,j,k) = w*demand_p*walk(o,d,(j,k)) - (alpha_p - gammaO_pj - gammaD_pk)

Returns `(rc_min, n_groups_served)`."""
function route_min_rc(s, key, a, go, gd)
    v = beta * (route_tau[s][key] + rho_repo)
    served = 0
    for (p, triples) in route_triples[s][key]
        (o, d) = mapping.Omega_s[s][p]
        demand = mapping.Q_s[s][p]
        best = 0.0
        for (j, kk) in triples
            net = walk_w * demand * od_pair_walking_cost(data, o, d, (j, kk)) -
                  (get(a, (s, p), 0.0) - get(go, ((s, p), j), 0.0) - get(gd, ((s, p), kk), 0.0))
            net < best && (best = net)
        end
        best < 0.0 && (served += 1)
        v += best
    end
    return v, served
end

"""`sum alpha - sum(gamma at BUILT stations) - f_c` -- what the unbuilt stations' worths
must cover between them for this column. Grouping by `unbuilt_assign_stations` and taking
the max reproduces the per-station requirement."""
function excess_over_built(s, c, a, go, gd, built)
    v = -cost(s, c)
    for (p, j, kk) in c.assignments
        v += get(a, (s, p), 0.0)
        j in built && (v -= get(go, ((s, p), j), 0.0))
        kk in built && (v -= get(gd, ((s, p), kk), 0.0))
    end
    return v
end

# ---------------------------------------------------------------- outputs
col_io = open(joinpath(OUT, "columns.csv"), "w")
println(col_io, "anchor_id,anchor_stations,scenario,route,n_stops,n_pax,assignments,tau,f_c," *
    "assign_stations,unbuilt_assign_stations,n_unbuilt_assign,n_unbuilt_visits,visits_unbuilt," *
    "in_plain,in_activated,rc_plain,rc_activated,excess_over_built")

alpha_io = open(joinpath(OUT, "duals_alpha.csv"), "w")
println(alpha_io, "arm,anchor_id,scenario,p,origin,dest,demand,alpha,c_walk_bound")

gamma_io = open(joinpath(OUT, "duals_gamma.csv"), "w")
println(gamma_io, "arm,anchor_id,scenario,p,station,side,gamma,station_built")

need_io = open(joinpath(OUT, "station_need.csv"), "w")
println(need_io, "anchor_id,scenario,station,built,need_j,closed_form_j,plain_cg_j," *
    "n_cols_only_j,n_cols_multi_pax_at_j,best_route,best_assignments,best_tau,best_f_c")

cut_io = open(joinpath(OUT, "cuts.csv"), "w")
println(cut_io, "arm,anchor_id,scenario,cut_constant,station,gamma_total,station_built")

anchor_io = open(joinpath(OUT, "anchors.csv"), "w")
println(anchor_io, "anchor_id,anchor_stations,q_rank,q_total," *
    join(("q_scenario_$s" for s in 1:n_scenarios(data)), ","))

miss_io = open(joinpath(OUT, "pool_not_in_universe.csv"), "w")
println(miss_io, "arm,anchor_id,scenario,route,assignments,tau,f_c,relation," *
    "rc_plain,rc_activated")

rmin_io = open(joinpath(OUT, "route_min_rc.csv"), "w")
println(rmin_io, "arm,anchor_id,scenario,route,tau,n_groups_certifiable,n_groups_served,rc_min")
# `Ref`, not a bare global: a top-level `for` loop that ASSIGNS to a global turns it into a
# loop-local in a script (soft scope is interactive-only), and the first read then throws
# `UndefVarError`. Mutating through a Ref is a read, so the rule does not apply.
const n_missing = Ref(0)

# ---------------------------------------------------------------- per anchor
for anchor_i in anchors
    combo = combos[anchor_i]
    built = Set(combo)
    y = as_y(combo)
    aid = join(combo, "|")
    println(anchor_io, "$aid,$aid,$(findfirst(==(anchor_i), order)),$(totals[anchor_i])," *
        join(per_scen[anchor_i], ","))

    println("="^130)
    @printf("anchor y = %s   Q = %.4f\n", string(combo), totals[anchor_i])
    println("="^130)

    pools = Dict{Symbol, Any}()
    duals = Dict{Symbol, Any}()
    for arm in (:column_generation, :column_generation_activated)
        solver = make_solver(arm)
        build = build_model(problem, monolith, solver)
        master = build.model
        StationSelection._solve_joint_routing_assignment_benders_subproblems(master, y, solver)
        subs = master[:benders_subproblem_builds]
        pools[arm] = [collect(values(b.model[:joint_routing_assignment_columns])) for b in subs]
        d = []
        for s in 1:n_scenarios(data)
            sm = subs[s].model
            a, go, gd = extract_joint_routing_assignment_duals(sm)
            if arm === :column_generation_activated
                StationSelection._benders_activated_complete_duals!(
                    a, go, gd, y, data, mapping, s, walk_w)
            end
            push!(d, (a, go, gd))
        end
        duals[arm] = d
    end

    for s in 1:n_scenarios(data)
        a_cg, go_cg, gd_cg = duals[:column_generation][s]
        a_ac, go_ac, gd_ac = duals[:column_generation_activated][s]

        # ---- duals
        for (arm, (a, go, gd)) in (("plain" => duals[:column_generation][s]),
                                   ("activated" => duals[:column_generation_activated][s]))
            for (p, (o, d)) in enumerate(mapping.Omega_s[s])
                demand = mapping.Q_s[s][p]
                demand > 0 || continue
                pairs = get_valid_jk_pairs(mapping, o, d)
                cw = any(is_walk_only_pair, pairs) ?
                    walk_w * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR) : NaN
                println(alpha_io, "$arm,$aid,$s,$p,$o,$d,$demand,$(get(a, (s, p), 0.0)),$cw")
            end
            for (key, v) in go
                (k2, j) = key
                k2[1] == s || continue
                println(gamma_io, "$arm,$aid,$s,$(k2[2]),$j,O,$v,$(j in built)")
            end
            for (key, v) in gd
                (k2, j) = key
                k2[1] == s || continue
                println(gamma_io, "$arm,$aid,$s,$(k2[2]),$j,D,$v,$(j in built)")
            end
        end

        # ---- cuts
        for (arm, (a, go, gd)) in (("plain" => duals[:column_generation][s]),
                                   ("activated" => duals[:column_generation_activated][s]))
            constant = sum(v for ((sc, _p), v) in a if sc == s; init = 0.0)
            gam = Dict{Int, Float64}()
            for gg in (go, gd), (key, v) in gg
                (k2, j) = key
                k2[1] == s || continue
                gam[j] = get(gam, j, 0.0) + v
            end
            for j in 1:N
                println(cut_io, "$arm,$aid,$s,$constant,$j,$(get(gam, j, 0.0)),$(j in built))")
            end
        end

        # ---- columns: the universe, marked with pool membership
        in_plain = Set(signature(c) for c in pools[:column_generation][s])
        in_act = Set(signature(c) for c in pools[:column_generation_activated][s])
        uni_sigs = Set(signature(c) for c in universe[s])
        for (arm, pool) in (("plain" => pools[:column_generation][s]),
                            ("activated" => pools[:column_generation_activated][s]))
            for c in pool
                signature(c) in uni_sigs && continue
                n_missing[] += 1
                key = Tuple(c.route)
                mine = Set(c.assignments)
                rel = if !haskey(route_triples[s], key)
                    "route_absent"
                else
                    others = [Set(u.assignments) for u in universe[s] if Tuple(u.route) == key]
                    any(mine ⊊ o for o in others) ? "subset" :
                        any(o ⊊ mine for o in others) ? "superset" : "incomparable"
                end
                println(miss_io, join((arm, aid, s, route_str(c), assign_str(c), c.tau,
                                       cost(s, c), rel,
                                       rc_at(s, c, a_cg, go_cg, gd_cg),
                                       rc_at(s, c, a_ac, go_ac, gd_ac)), ","))
            end
        end

        # ---- the closed-form true minimum over EVERY sub-assignment, per route
        for (arm, (a, go, gd)) in (("plain" => duals[:column_generation][s]),
                                   ("activated" => duals[:column_generation_activated][s]))
            worst = Inf
            for key in keys(route_triples[s])
                rmin, served = route_min_rc(s, key, a, go, gd)
                worst = min(worst, rmin)
                println(rmin_io, join((arm, aid, s, join(key, ">"), route_tau[s][key],
                                       length(route_triples[s][key]), served, rmin), ","))
            end
            @printf("       %-10s true min rc over EVERY sub-assignment of every route: %+.6e%s\n",
                    arm, worst, worst < -1e-6 ? "   <-- DUAL INFEASIBLE" : "")
        end

        for c in universe[s]
            sg = signature(c)
            ast = assign_stations(c)
            un_assign = setdiff(ast, built)
            un_visits = setdiff(Set(c.route), built)
            println(col_io, join((
                aid, aid, s, route_str(c), length(c.route), length(c.assignments),
                assign_str(c), c.tau, cost(s, c),
                join_ids(ast), join_ids(un_assign), length(un_assign),
                length(un_visits), !isempty(un_visits),
                sg in in_plain, sg in in_act,
                rc_at(s, c, a_cg, go_cg, gd_cg),
                rc_at(s, c, a_ac, go_ac, gd_ac),
                excess_over_built(s, c, a_cg, go_cg, gd_cg, built),
            ), ","))
        end

        # ---- per-station requirement, single-unbuilt-station columns only
        for j in sort(collect(setdiff(1:N, built)))
            best_excess = -Inf
            best_col = nothing
            n_only, n_multi = 0, 0
            for c in universe[s]
                setdiff(assign_stations(c), built) == Set([j]) || continue
                n_only += 1
                count(t -> t[2] == j || t[3] == j, c.assignments) > 1 && (n_multi += 1)
                e = excess_over_built(s, c, a_cg, go_cg, gd_cg, built)
                if e > best_excess
                    best_excess = e
                    best_col = c
                end
            end
            need = isfinite(best_excess) ? max(0.0, best_excess) : 0.0
            cf = sum(v for gg in (go_ac, gd_ac) for ((k2, jj), v) in gg
                     if jj == j && k2[1] == s; init = 0.0)
            tru = sum(v for gg in (go_cg, gd_cg) for ((k2, jj), v) in gg
                      if jj == j && k2[1] == s; init = 0.0)
            println(need_io, join((
                aid, s, j, false, need, cf, tru, n_only, n_multi,
                best_col === nothing ? "" : route_str(best_col),
                best_col === nothing ? "" : assign_str(best_col),
                best_col === nothing ? "" : string(best_col.tau),
                best_col === nothing ? "" : string(cost(s, best_col)),
            ), ","))
        end

        # ---- printed summary, so the log is readable on its own
        np, na, nu = length(pools[:column_generation][s]),
                     length(pools[:column_generation_activated][s]), length(universe[s])
        only_p = length(setdiff(in_plain, in_act))
        only_a = length(setdiff(in_act, in_plain))
        pinned = count(c -> !isempty(setdiff(assign_stations(c), built)), universe[s])
        ghost = count(c -> isempty(setdiff(assign_stations(c), built)) &&
                           !isempty(setdiff(Set(c.route), built)), universe[s])
        @printf(" s=%d | pools: plain %d, activated %d, universe %d | plain-only %d, activated-only %d\n",
                s, np, na, nu, only_p, only_a)
        @printf("       universe: %d assign at an unbuilt station (theta pinned to 0), %d visit one while assigning only at built ones\n",
                pinned, ghost)
        needs = Float64[]
        for j in sort(collect(setdiff(1:N, built)))
            e = maximum((excess_over_built(s, c, a_cg, go_cg, gd_cg, built)
                         for c in universe[s]
                         if setdiff(assign_stations(c), built) == Set([j])); init = -Inf)
            push!(needs, isfinite(e) ? max(0.0, e) : 0.0)
        end
        @printf("       need_j over unbuilt stations: %s | closed form charges: %s\n",
                join((@sprintf("%.1f", x) for x in needs), " "),
                join((@sprintf("%.0f", sum(v for gg in (go_ac, gd_ac)
                                           for ((k2, jj), v) in gg
                                           if jj == j && k2[1] == s; init = 0.0))
                      for j in sort(collect(setdiff(1:N, built)))), " "))
        flush(stdout)
    end
end

for io in (col_io, alpha_io, gamma_io, need_io, cut_io, anchor_io, miss_io, rmin_io)
    close(io)
end

open(joinpath(OUT, "README.md"), "w") do io
    println(io, """
# Pool and cut anatomy — $TAG

Zhuzhou n=$N, p=$P, s=$S, seed $SEED, k=$k, max_stops=$MAX_STOPS.
Walking weight $(walk_w), driving weight $(beta).
At this `max_stops` the enumerated pool IS the whole column universe, so `columns.csv`
is exhaustive rather than a sample.

Two arms, both solving the identical model at the same incumbents:
- `plain` — `:column_generation`, prices over every station.
- `activated` — `:column_generation_activated`, prices over the built stations only and
  repairs the duals with the closed-form completion. **No row generation.**

## columns.csv — one row per (anchor, scenario, column), universe-wide

| field | meaning |
| --- | --- |
| `anchor_id`, `anchor_stations` | the incumbent, `\\|`-joined station ids |
| `route` | visit sequence, `>`-joined |
| `n_stops`, `n_pax` | route length, number of assignments |
| `assignments` | `p:j:k` triples, `\\|`-joined — group, pickup station, dropoff station |
| `tau`, `f_c` | driving time, and total column cost `β(τ+ρ) + w·Σ demand·walk` |
| `assign_stations` | every station used by an assignment |
| `unbuilt_assign_stations`, `n_unbuilt_assign` | those outside the incumbent |
| `n_unbuilt_visits`, `visits_unbuilt` | unbuilt stations on the ROUTE, served or not |
| `in_plain`, `in_activated` | whether each arm's pool contains this column |
| `rc_plain`, `rc_activated` | reduced cost at each arm's duals; the dual constraint is `rc >= 0` |
| `excess_over_built` | `Σα − Σ(γ at built stations) − f_c` at the **plain** duals |

`n_unbuilt_assign > 0` means `theta` is pinned to 0 at this incumbent, so the column cannot
improve the objective — it exists only for its dual constraint. `visits_unbuilt` with
`n_unbuilt_assign == 0` is the class the restricted search cannot reach at all, and the
reason the soundness proof compares against the route with the stop DELETED.

Group by `unbuilt_assign_stations` and take `max(excess_over_built)` to get what those
stations' worths must cover between them. For single-station groups that is the per-station
requirement; `station_need.csv` has it pre-aggregated.

## station_need.csv — per unbuilt station

`need_j` (from single-unbuilt-station columns, exhaustive) against `closed_form_j` (what the
activated arm charges) and `plain_cg_j` (what full pricing's duals carry), plus the column
attaining `need_j` in full. `n_cols_multi_pax_at_j` counts columns serving 2+ passengers at
`j` — those constrain a SUM of worths, not one coordinate, so they are excluded from `need_j`
and are the reason a per-station number is a lower bound on the true requirement.

## duals_alpha.csv / duals_gamma.csv / cuts.csv

`alpha` per group with its `c_walk_bound` (`NaN` where the group has no direct-walk option,
which is exactly where `alpha` has no a-priori ceiling). `gamma` per `(group, station, side)`
with `station_built`. `cuts.csv` is the aggregate the master actually sees:
`Θ_s ≥ cut_constant − Σ_j gamma_total·y_j`.

## anchors.csv, pool_not_in_universe.csv

Anchors with their Q rank and per-scenario costs. The second file should be EMPTY — a row in
it means some pooled column is absent from the enumerated universe, i.e. the two are not the
same model and every comparison here is void. Rows written this run: $(n_missing[]).
""")
end

@printf("\nwrote %s\n", OUT)
foreach(f -> @printf("  %-28s %9.1f KB\n", f, filesize(joinpath(OUT, f)) / 1024),
        sort(readdir(OUT)))
n_missing[] == 0 || @printf("\n!! %d pooled columns are NOT in the enumerated universe -- see pool_not_in_universe.csv\n",
                            n_missing[])
