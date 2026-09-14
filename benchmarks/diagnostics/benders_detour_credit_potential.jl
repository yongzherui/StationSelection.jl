"""How much cut strength is the DETOUR term actually worth? Measure before building.

The activated completion must force `rho_pjk <= 0` on every assignment through an unbuilt
station, and the only term it credits is `w * walk` -- 0.7-2.2% of `alpha_p`. A column that
serves a passenger at an unbuilt `j` must also DRIVE there, and that detour is a real term in
`f_r` the completion throws away. The proposal is to credit it through a per-station budget
(`notes/2026-09-10_locally_pareto_optimal_activated_cut.md`).

This script answers the one question that decides whether that is worth implementing:
**how big is the budget against the mass it has to remove?**

# The quantity, and why the obvious version is unsound

Validity needs `sum_{A_out} rho <= c_route(V) - c_route(V'')`, and deleting the unbuilt nodes
one at a time bounds the right side below by `beta * sum_{j in U(r)} delta_j`. `delta_j` must
be the minimum saving over EVERY position `j` could occupy in a route:

    delta_j^all = min{ min_{a,b != j} [t(a,j) + t(j,b) - t(a,b)],
                       min_b t(j,b),                                  (j first on the path)
                       min_a t(a,j) }                                 (j last on the path)

The minimum over BUILT pairs only (`delta_j^built`, what the completion audit prints) is
LARGER, and crediting it would be unsound: when two unbuilt stations are adjacent, deleting
the first has an unbuilt neighbour, and `route` is an open path with no depot leg so the
endpoint cases are real. Both are computed here, so the size of that correction is visible
rather than assumed.

# The comparison that decides it

Per unbuilt station, one budget of `beta * delta_j` is shared by every demand group, so the
credit is a single ADDITIVE reduction of `g_j` -- it cannot scale with the number of groups.
The bar it has to clear is `g_j^closed - g_j^gold`: what the closed-form completion charges,
against what the true Magnanti-Wong dual over the enumerated route family charges. That gold
dual is computed here the same way `benders_lpo_gold_standard.jl` computes it.

Reported per anchor and in aggregate:

    g_closed        mean g_j over unbuilt stations, closed-form completion
    g_gold          the same at the gold dual -- the target
    beta*delta      the budget, all-pairs (sound) and built-pairs (unsound, for scale)
    recovery        beta*delta / (g_closed - g_gold), the share of the gap it could close

`recovery` is an OPTIMISTIC ceiling: it assumes every unit of budget converts into a unit of
`g_j` reduction, which the `rho <= 0` rows need not allow. If the ceiling is small, the idea
is dead without writing it.

Also measured, because it says how much the unsound/sound correction costs in practice: the
fraction of enumerated columns whose route visits two unbuilt stations CONSECUTIVELY, which is
the case that forces the all-pairs minimum.

Usage: sbatch --array=1-4 benchmarks/diagnostics/run_benders_detour_credit_potential.sh
Env: DC_N DC_P DC_S DC_SEED DC_MAX_STOPS DC_ANCHORS
"""

using StationSelection
using JuMP
using Gurobi
using Printf
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "DC_N", "10"))
const P = parse(Int, get(ENV, "DC_P", "8"))
const S = parse(Int, get(ENV, "DC_S", "1"))
const SEED = parse(Int, get(ENV, "DC_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "DC_MAX_STOPS", "4"))
const N_ANCHORS = parse(Int, get(ENV, "DC_ANCHORS", "8"))

problem, k, _meta = benchmark_problem(@__DIR__, "DC", N, P, S, SEED)
data = problem.data
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops = MAX_STOPS)
solver = BendersSolver(
    config = SolverOptions(silent = true), max_iterations = 500,
    subproblem = BendersSubproblemConfig(
        oracle = :direct_enumeration, max_stops = MAX_STOPS, max_routes = 500_000,
        enumeration_time_limit_sec = 600.0))

build = build_model(problem, formulation, solver)
master = build.model
mapping = build.mapping
subs = master[:benders_subproblem_builds]
pool = [collect(values(b.model[:joint_routing_assignment_columns])) for b in subs]
w = Float64(subs[1].model[:joint_routing_assignment_walk_cost_weight])
beta = Float64(subs[1].model[:joint_routing_assignment_route_regularization_weight])
@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d | pool %s | beta=%.4g w=%.4g\n",
        N, P, S, SEED, k, MAX_STOPS, join(length.(pool), "+"), beta, w)
flush(stdout)

core_point, core_slack = StationSelection._benders_core_point(data, mapping, k)

# ---------------------------------------------------------------- delta_j
t(a, b) = get_routing_cost(data, a, b)

"""`delta_j` over the positions in `allowed`: the minimum travel saved by deleting `j` from a
route. `allowed` restricts which neighbours count -- the sound version admits every station,
the `built` version only the ones built at the incumbent (and is therefore an over-estimate,
kept to show the size of the correction)."""
function delta(j::Int, allowed::AbstractVector{Int}; endpoints::Bool = true)
    best = Inf
    for a in allowed, b in allowed
        (a == j || b == j || a == b) && continue
        best = min(best, t(a, j) + t(j, b) - t(a, b))
    end
    if endpoints
        for b in allowed
            b == j && continue
            best = min(best, t(j, b))          # j first on the path
            best = min(best, t(b, j))          # j last on the path
        end
    end
    return isfinite(best) ? max(0.0, best) : 0.0
end

all_stations = collect(1:N)
delta_all = [delta(j, all_stations) for j in 1:N]
delta_all_noend = [delta(j, all_stations; endpoints = false) for j in 1:N]
@printf("\ndelta_j (all-pairs, sound): min %.3f median %.3f max %.3f | beta*delta: min %.1f median %.1f max %.1f\n",
        minimum(delta_all), sort(delta_all)[(N + 1) ÷ 2], maximum(delta_all),
        beta * minimum(delta_all), beta * sort(delta_all)[(N + 1) ÷ 2], beta * maximum(delta_all))
@printf("delta_j ignoring the two endpoint cases: median %.3f  (endpoint cases cost %.1f%% of the credit)\n",
        sort(delta_all_noend)[(N + 1) ÷ 2],
        100 * (1 - sum(delta_all) / max(1e-12, sum(delta_all_noend))))
flush(stdout)

# ------------------------------------------- master-feasible sets and the exact Q
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
totals = Float64[]
for c in combos
    sub = StationSelection._solve_joint_routing_assignment_benders_subproblems(
        master, as_y(c), solver)
    push!(totals, sub.total_objective)
end
order = sortperm(totals)
picks = unique(vcat(1, [1 + round(Int, (length(order) - 1) * i / max(1, N_ANCHORS - 1))
                        for i in 0:(N_ANCHORS - 1)]))
anchors = order[picks[1:min(end, N_ANCHORS)]]
@printf("%d endpoint-feasible sets | %d anchors\n", length(combos), length(anchors))
flush(stdout)

gamma_by_station(go, gd) = begin
    g = Dict{Int, Float64}()
    for gs in (go, gd), (key, v) in gs
        v == 0.0 && continue
        g[key[2]] = get(g, key[2], 0.0) + v
    end
    g
end

"""The true Magnanti-Wong dual over the ENUMERATED route family: optimal face, core-point
objective, real route rows, no `rho <= 0`. Testing only -- it enumerates R."""
function gold_dual(s::Int, incumbent::Vector{Float64}, z_star::Float64)
    sm = subs[s].model
    gkeys = Tuple{Int, Int}[]; pairs_of = Dict{Tuple{Int,Int}, Vector{Tuple{Int,Int}}}()
    wbound = Dict{Tuple{Int,Int}, Float64}()
    pk = Tuple{Tuple{Int,Int}, Int}[]; dk = Tuple{Tuple{Int,Int}, Int}[]
    for (p, (o, d)) in enumerate(mapping.Omega_s[s])
        demand = mapping.Q_s[s][p]; demand > 0 || continue
        key2 = (s, p); push!(gkeys, key2)
        ps, so, sd = Tuple{Int,Int}[], Set{Int}(), Set{Int}()
        for pair in get_valid_jk_pairs(mapping, o, d)
            if is_walk_only_pair(pair)
                wbound[key2] = w * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)
                continue
            end
            j, kk = pair; push!(ps, (j, kk))
            j in so || (push!(so, j); push!(pk, (key2, j)))
            kk in sd || (push!(sd, kk); push!(dk, (key2, kk)))
        end
        pairs_of[key2] = ps
    end
    lp = Model(() -> Gurobi.Optimizer()); set_silent(lp)
    @variable(lp, a[key in gkeys] >= 0)
    @variable(lp, go[key in pk] >= 0)
    @variable(lp, gd[key in dk] >= 0)
    @objective(lp, Max, sum(a[key] for key in gkeys; init = 0.0) -
        sum(core_point[key[2]] * go[key] for key in pk; init = 0.0) -
        sum(core_point[key[2]] * gd[key] for key in dk; init = 0.0))
    @constraint(lp, sum(a[key] for key in gkeys; init = 0.0) -
        sum(go[key] for key in pk if incumbent[key[2]] > 0.5; init = 0.0) -
        sum(gd[key] for key in dk if incumbent[key[2]] > 0.5; init = 0.0) == z_star)
    for (key, b) in wbound; @constraint(lp, a[key] <= b); end
    for column in pool[s]
        row = AffExpr(0.0)
        for (p, j, kk) in column.assignments
            key2 = (s, p)
            add_to_expression!(row, 1.0, a[key2])
            add_to_expression!(row, -1.0, go[(key2, j)])
            add_to_expression!(row, -1.0, gd[(key2, kk)])
        end
        @constraint(lp, row <= joint_routing_assignment_column_cost(sm, data, mapping, column))
    end
    optimize!(lp)
    termination_status(lp) == MOI.OPTIMAL || return nothing
    return (Dict(key => max(0.0, value(a[key])) for key in gkeys),
            Dict(key => max(0.0, value(go[key])) for key in pk),
            Dict(key => max(0.0, value(gd[key])) for key in dk))
end

mean_of(v) = isempty(v) ? NaN : sum(v) / length(v)

println("\n", "="^104)
println("per anchor: what the closed form charges on unbuilt stations, what the detour budget could remove,")
println("            and what the real dual actually charges")
println("="^104)
@printf("%-20s %2s %10s %10s %10s %10s %10s %9s %9s\n",
        "y", "s", "g_closed", "b*d_all", "b*d_built", "g_gold", "gap", "rec_all", "rec_blt")
rows = NamedTuple[]
for anchor_i in anchors
    y = as_y(combos[anchor_i])
    built = combos[anchor_i]
    unbuilt = [j for j in 1:N if !(j in built)]
    dall = mean_of([beta * delta_all[j] for j in unbuilt])
    dblt = mean_of([beta * delta(j, built) for j in unbuilt])
    sub = StationSelection._solve_joint_routing_assignment_benders_subproblems(master, y, solver)
    by_s = Dict(r.scenario => r for r in sub.scenarios)
    for s in 1:n_scenarios(data)
        sm = subs[s].model
        alpha, go, gd = extract_joint_routing_assignment_duals(sm)
        StationSelection._benders_activated_complete_duals!(alpha, go, gd, y, data, mapping, s, w)
        g_closed = gamma_by_station(go, gd)
        gp = gold_dual(s, y, by_s[s].objective)
        isnothing(gp) && continue
        g_gold = gamma_by_station(gp[2], gp[3])
        mc = mean_of([get(g_closed, j, 0.0) for j in unbuilt])
        mg = mean_of([get(g_gold, j, 0.0) for j in unbuilt])
        gap = mc - mg
        @printf("%-20s %2d %10.1f %10.1f %10.1f %10.1f %10.1f %8.1f%% %8.1f%%\n",
                string(built), s, mc, dall, dblt, mg, gap,
                100 * dall / max(1e-9, gap), 100 * dblt / max(1e-9, gap))
        push!(rows, (g_closed = mc, d_all = dall, d_built = dblt, g_gold = mg, gap = gap))
        flush(stdout)
    end
end

println("\n", "="^104)
println("verdict")
println("="^104)
mc = mean_of([r.g_closed for r in rows]); mg = mean_of([r.g_gold for r in rows])
da = mean_of([r.d_all for r in rows]);    db = mean_of([r.d_built for r in rows])
gap = mc - mg
@printf("mean g_j on unbuilt stations: closed form %.1f -> gold %.1f   (gap %.1f)\n", mc, mg, gap)
@printf("detour budget beta*delta_j:   all-pairs (SOUND) %.1f   built-pairs (unsound) %.1f\n", da, db)
@printf("optimistic recovery ceiling:  %.1f%% sound, %.1f%% if the unsound version were usable\n",
        100 * da / max(1e-9, gap), 100 * db / max(1e-9, gap))
@printf("for scale, what the closed form credits today (w*walk, per triple): ~%.1f\n",
        w * mean_of([od_pair_walking_cost(data, o, d, pr)
                     for s in 1:n_scenarios(data)
                     for (p, (o, d)) in enumerate(mapping.Omega_s[s])
                     for pr in get_valid_jk_pairs(mapping, o, d)
                     if !is_walk_only_pair(pr)]))
println()
if 100 * da / max(1e-9, gap) >= 15.0
    println("VIABLE: the sound budget clears 15% of the gap -- worth implementing.")
else
    println("NOT VIABLE: the sound budget is below 15% of the gap. Crediting the detour")
    println("correctly cannot close it; the strength lives in per-column slack instead.")
end
