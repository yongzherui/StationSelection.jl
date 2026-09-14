"""Is the value function flat enough for a LIPSCHITZ cut to beat the dual completion?

Every mechanism measured so far routes through dual feasibility, and so inherits "you cannot
know the floor at station j without searching through j". One family escapes that: a cut only
has to UNDER-ESTIMATE Q, and -- since the master is a MIP -- only at BINARY master-feasible
points. It need not be a supporting hyperplane of the convex relaxation, and it need not come
from a dual point at all.

The candidate is

    Theta  >=  Q_s(yhat)  -  V * sum_{j not in yhat} y_j

whose ingredients are `Q_s(yhat)` (cheap: the ACTIVATED oracle gives it, built-only, no
full-universe search anywhere) and `V`, a structural constant of the instance. If `V` is small
this is a good cut obtained with no pricing at all.

# The quantity V actually has to satisfy

Validity at every master-feasible `y` means `Q(y) >= Q(yhat) - V * |y \\ yhat|`, so

    V_required = max over ORDERED pairs (yhat, y) of  [Q(yhat) - Q(y)] / |y \\ yhat|

That is a LIPSCHITZ constant in the swap metric, not the one-swap maximum. The one-swap
maximum is a lower bound on it and is reported alongside, because the difference between the
two is exactly the amount of trouble multi-station moves cause -- and multi-station moves are
where station COMPLEMENTARITY lives (a group needs a pickup near `o` AND a dropoff near `d`,
so two stations together can be worth more than twice either alone).

# What is reported

`V_required` and `V_1swap` per scenario, against `mean Q_s` and against the reference points
already measured on these instances: the closed-form completion charges `g_j ~ 5350` and the
gold dual `g_j ~ 485`. Then the flatness cut is BUILT at every anchor with `V = V_required`
and scored exactly like every other arm -- one-swap gap, dominated fraction, and worst
violation over the whole feasible set, which must be <= 0 by construction and is checked
rather than assumed.

Usage: sbatch --array=1-3 benchmarks/diagnostics/run_benders_flatness_bound.sh
Env: FL_N FL_P FL_S FL_SEED FL_MAX_STOPS FL_ANCHORS
"""

using StationSelection
using JuMP
using Printf
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "FL_N", "10"))
const P = parse(Int, get(ENV, "FL_P", "8"))
const S = parse(Int, get(ENV, "FL_S", "1"))
const SEED = parse(Int, get(ENV, "FL_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "FL_MAX_STOPS", "4"))
const N_ANCHORS = parse(Int, get(ENV, "FL_ANCHORS", "8"))

problem, k, _meta = benchmark_problem(@__DIR__, "FL", N, P, S, SEED)
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
@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d\n",
        N, P, S, SEED, k, MAX_STOPS)
flush(stdout)

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
Q = Matrix{Float64}(undef, length(combos), n_scenarios(data))
totals = Float64[]
for (i, c) in enumerate(combos)
    sub = StationSelection._solve_joint_routing_assignment_benders_subproblems(
        master, as_y(c), solver)
    for r in sub.scenarios; Q[i, r.scenario] = r.objective; end
    push!(totals, sub.total_objective)
end
sets = [Set(c) for c in combos]
@printf("%d endpoint-feasible sets of %d | total Q spread %.4f (max/min)\n",
        length(combos), binomial(N, k), maximum(totals) / minimum(totals))
flush(stdout)

# ---------------------------------------------------------------- the Lipschitz constants
println("\n", "="^100)
println("V: the Lipschitz constant of the value function in the swap metric")
println("="^100)
@printf("%2s %12s %12s %14s %14s %12s %12s\n",
        "s", "mean Q_s", "V_1swap", "V_required", "V_req/meanQ", "vs g_closed", "vs g_gold")
Vreq = Float64[]
for s in 1:n_scenarios(data)
    v_any, v_one = 0.0, 0.0
    for i in eachindex(combos), j in eachindex(combos)
        i == j && continue
        drop = Q[i, s] - Q[j, s]
        drop > 0 || continue
        d = length(setdiff(sets[j], sets[i]))          # stations y builds that yhat did not
        d == 0 && continue
        v_any = max(v_any, drop / d)
        d == 1 && (v_one = max(v_one, drop))
    end
    push!(Vreq, v_any)
    mq = sum(Q[:, s]) / size(Q, 1)
    @printf("%2d %12.1f %12.1f %14.1f %13.1f%% %12s %12s\n", s, mq, v_one, v_any,
            100 * v_any / mq, "~5350", "~485")
    flush(stdout)
end

# ---------------------------------------------------------------- score the flatness cut
order = sortperm(totals)
picks = unique(vcat(1, [1 + round(Int, (length(order) - 1) * i / max(1, N_ANCHORS - 1))
                        for i in 0:(N_ANCHORS - 1)]))
anchors = order[picks[1:min(end, N_ANCHORS)]]
mean_of(v) = isempty(v) ? NaN : sum(v) / length(v)

println("\n", "="^100)
println("the flatness cut  Theta >= Q_s(yhat) - V * sum_{j not in yhat} y_j,  scored like every other arm")
println("="^100)
@printf("%-20s %2s %14s %14s %12s %14s\n",
        "y", "s", "swap mean", "swap max", "dominated", "worst viol")
allgaps = Float64[]; alldom = Float64[]; allviol = Float64[]
for anchor_i in anchors, s in 1:n_scenarios(data)
    V = Vreq[s]
    zstar = Q[anchor_i, s]
    predict(i) = zstar - V * length(setdiff(sets[i], sets[anchor_i]))
    gaps, dom, n = Float64[], 0, 0
    for i in eachindex(combos)
        length(setdiff(sets[i], sets[anchor_i])) == 1 || continue
        push!(gaps, Q[i, s] - predict(i))
        predict(i) <= 0.0 && (dom += 1)
        n += 1
    end
    wv = maximum(predict(i) - Q[i, s] for i in eachindex(combos))
    push!(allviol, wv)
    append!(allgaps, gaps); push!(alldom, n == 0 ? NaN : dom / n)
    @printf("%-20s %2d %14.2f %14.2f %11.1f%% %+14.2e\n",
            string(combos[anchor_i]), s, mean_of(gaps),
            isempty(gaps) ? NaN : maximum(gaps), 100 * (n == 0 ? NaN : dom / n), wv)
    flush(stdout)
end

println("\n", "="^100)
println("verdict")
println("="^100)
worst = isempty(allviol) ? -Inf : maximum(allviol)
@printf("flatness cut: mean one-swap gap %.2f | dominated %.1f%% | worst violation %+.3e\n",
        mean_of(allgaps), 100 * mean_of(alldom), worst)
println("reference on these instances: closed form ~5300-6100, plain CG ~200-430, gold ~60-220")
worst <= 1e-6 || error("the flatness cut is INVALID -- V_required was computed wrong")
println("\nvalidity: PASS (worst violation <= 0 by construction, checked)")
if mean_of(allgaps) < 1000.0
    println("VERDICT: competitive with plain CG, and it needs NO pricing. Worth pursuing a proof of V.")
elseif mean_of(allgaps) < 3000.0
    println("VERDICT: well better than the completion, well short of plain CG. Only interesting")
    println("         where plain CG cannot run at all -- i.e. n >= 30.")
else
    println("VERDICT: no better than the dual completion. The value function is NOT flat enough")
    println("         in the swap metric, and this direction is dead too.")
end
