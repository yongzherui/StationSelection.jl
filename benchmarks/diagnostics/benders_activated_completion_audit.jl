"""Is the ACTIVATED oracle's dual completion dual-FEASIBLE, and how much cut strength does
it give up? Both questions at the same anchors, against exact references.

The activated oracles (`:column_generation_activated`, `:column_generation_activated_lpo`)
price only over the stations built at `yhat` and then REPAIR the duals on the unbuilt ones.
The repair is what licenses the cut, so it has exactly one requirement: the completed
`(alpha, gamma)` must be feasible for the dual of the FULL-universe second stage. Two
existing scripts touch this indirectly -- `benders_joint_n10.jl` compares objectives,
`benders_brute_force_certificate.jl` audits the resulting cuts pointwise -- but neither tests
the completion itself, and both can pass while the completion is unsound in a direction the
loop happens not to explore.

This script tests the requirement directly, and separately from the loop.

# 1. Dual feasibility, against the exhaustive pool (the correctness check)

At `max_stops = ACA_MAX_STOPS` the enumerated pool IS the column universe, so the dual
constraint set is finite and writable. For every column `c` in it,

    rc(c) = f_c - sum_{(p,j,k) in c} [ alpha_p - gammaO_pj - gammaD_pk ]  >=  0

IS `c`'s dual constraint. `min_c rc(c) >= -tol` at the completed duals therefore certifies
full-universe dual feasibility outright -- no reference to the pricer, the loop, or any
argument about which columns the restricted search covered. A negative minimum names the
exact column the completion under-charges, which is the whole diagnosis.

The second dual family (`alpha_p <= c_walk[s,p]`, for groups that have a direct-walk
option) is checked alongside it: the completion does not touch `alpha`, so a violation
there would mean something else is wrong.

Also checked, since it is free at the same anchors: the activated subproblem's objective
equals the exact `Q_s(yhat)` from the enumerated pool. That is the claim that the
restriction costs nothing in VALUE (a column touching an unbuilt station is pinned to
`theta = 0`), separate from the claim that the completion costs nothing in validity.

# 2. Cut strength, against the true value function (the weakness measurement)

At n=10 with k=5 the master's whole feasible set is enumerable, so `Q_s(y)` is known
exactly everywhere and a cut can be scored rather than described. Per cut:

    valid      max_y [ predicted(y) - Q_s(y) ]     must be <= tol
    tight      Q_s(yhat) - predicted(yhat)          must be ~0 (the anchor)
    dominated  fraction of y where predicted(y) <= 0

`dominated` is the number that says whether a cut does any work. The master already knows
`Theta_s >= 0`, so a cut whose prediction is negative at `y` tells it NOTHING at `y`. A
completion that charges `gamma_pj ~ alpha_p` on every unbuilt `(p,j)` gives
`Gamma_j ~ sum_p alpha_p ~ the cut's own constant`, so the prediction goes non-positive as
soon as `y` builds anything that was unbuilt at the anchor -- i.e. the cut degenerates into
a no-good cut on one station set, which is what "77 cuts / 30 iterations" looks like from
the inside. `gamma_mass = sum_j Gamma_j / constant` measures the same thing without needing
the value function.

The `:column_generation` arm is the control: same model, same anchors, raw duals from
full-station pricing, no completion. Its cuts are the strong ones the activated arms are
being compared against.

Usage: sbatch benchmarks/diagnostics/run_benders_activated_completion_audit.sh
Env: ACA_N ACA_P ACA_S ACA_SEED ACA_MAX_STOPS ACA_ANCHORS ACA_ARMS
"""

using StationSelection
using JuMP
using Printf
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "ACA_N", "10"))
const P = parse(Int, get(ENV, "ACA_P", "8"))
const S = parse(Int, get(ENV, "ACA_S", "1"))
const SEED = parse(Int, get(ENV, "ACA_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "ACA_MAX_STOPS", "4"))
const N_ANCHORS = parse(Int, get(ENV, "ACA_ANCHORS", "8"))
const ARMS = Symbol.(split(get(ENV, "ACA_ARMS",
    "column_generation_activated,column_generation_activated_lpo,column_generation"), ','))
const TOL = 1e-6

problem, k, _meta = benchmark_problem(@__DIR__, "ACA", N, P, S, SEED)
data = problem.data
monolith = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops = MAX_STOPS)

# `max_stops` is passed EXPLICITLY on every arm, enumeration and CG alike. Its default is
# per-oracle (4 / nothing), so leaving it out would have the CG arms search a larger
# universe than the enumerated pool -- and the enumerated pool is what makes check 1 a
# proof rather than a sample.
make_solver(oracle) = BendersSolver(
    config = SolverOptions(silent = true),
    max_iterations = 500,
    subproblem = BendersSubproblemConfig(
        oracle = oracle,
        max_stops = MAX_STOPS,
        max_routes = 500_000,
        enumeration_time_limit_sec = 600.0,
        max_cg_iterations = 500,
        cg_pricing_time_limit_sec = 300.0,
    ),
)

@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d\n",
        N, P, S, SEED, k, MAX_STOPS)
@printf("arms: %s | anchors: %d\n", join(ARMS, ", "), N_ANCHORS)
flush(stdout)

# ---------------------------------------------------------------- the exact reference
enum_solver = make_solver(:direct_enumeration)
t_build = time()
enum_build = build_model(problem, monolith, enum_solver)
enum_master = enum_build.model
mapping = enum_build.mapping
enum_subs = enum_master[:benders_subproblem_builds]
pool = [collect(values(b.model[:joint_routing_assignment_columns])) for b in enum_subs]
walk_cost_weight = Float64(enum_subs[1].model[:joint_routing_assignment_walk_cost_weight])
@printf("enumerated pool: %s columns in %.1fs | walk_cost_weight %.4g\n",
        join(length.(pool), "+"), time() - t_build, walk_cost_weight)
flush(stdout)

# ------------------------------------------- the master's feasible set, and Q on it
# Same rule as `add_aggregate_od_route_endpoint_feasibility_constraints!`: Q is only defined
# where the master can actually propose, and outside it the subproblem is often infeasible.
required = Set{Int}()
for s in 1:n_scenarios(data)
    for (p, (o, d)) in enumerate(mapping.Omega_s[s])
        mapping.Q_s[s][p] > 0 || continue
        any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
        push!(required, o); push!(required, d)
    end
end
candidates = Dict(pt => Set(j for j in 1:N
                            if get_walking_cost(data, pt, j) <= mapping.max_walking_distance)
                  for pt in required)
combos = filter(collect(combinations(1:N, k))) do combo
    all(!isempty(intersect(candidates[pt], combo)) for pt in required)
end
isempty(combos) && error("no endpoint-feasible station set at k=$k")

as_y(combo) = (y = zeros(Float64, N); y[combo] .= 1.0; y)

t0 = time()
Q = Matrix{Float64}(undef, length(combos), n_scenarios(data))
totals = Vector{Float64}(undef, length(combos))
for (i, combo) in enumerate(combos)
    sub = StationSelection._solve_joint_routing_assignment_benders_subproblems(
        enum_master, as_y(combo), enum_solver)
    for r in sub.scenarios
        Q[i, r.scenario] = r.objective
    end
    totals[i] = sub.total_objective
end
@printf("value function: %d endpoint-feasible sets of %d in %.1fs | min %.6f max %.6f (spread %.2f%%)\n",
        length(combos), binomial(N, k), time() - t0, minimum(totals), maximum(totals),
        100 * (maximum(totals) - minimum(totals)) / minimum(totals))
flush(stdout)

# Anchors: evenly spaced over the Q RANKING, so the sample spans good and bad incumbents
# rather than clustering wherever `combinations` happens to order them. The optimum is
# always included -- it is the one anchor a real run is guaranteed to visit.
order = sortperm(totals)
picks = unique(vcat(1, [1 + round(Int, (length(order) - 1) * t / max(1, N_ANCHORS - 1))
                        for t in 0:(N_ANCHORS - 1)]))
anchors = order[picks[1:min(end, N_ANCHORS)]]
@printf("anchors (by Q rank): %s\n",
        join([@sprintf("%s Q=%.1f", string(combos[i]), totals[i]) for i in anchors], " | "))
flush(stdout)

# ---------------------------------------------------------------- checks on one dual point
"""Minimum reduced cost over the EXHAUSTIVE pool -- i.e. worst violation of the
full-universe dual constraints, with the column that attains it."""
function min_reduced_cost(s::Int, alpha, gamma_o, gamma_d, incumbent)
    sm = enum_subs[s].model
    worst = Inf
    worst_col = nothing
    for column in pool[s]
        rc = joint_routing_assignment_column_cost(sm, data, mapping, column)
        for (p, j, kk) in column.assignments
            key2 = (s, p)
            rc -= get(alpha, key2, 0.0)
            rc += get(gamma_o, (key2, j), 0.0) + get(gamma_d, (key2, kk), 0.0)
        end
        if rc < worst
            worst = rc
            worst_col = column
        end
    end
    touches_unbuilt = worst_col === nothing ? false :
        any(incumbent[j] < 0.5 || incumbent[kk] < 0.5 for (_p, j, kk) in worst_col.assignments)
    return worst, touches_unbuilt
end

"""Worst violation of the OTHER dual family, `alpha_p <= c_walk[s,p]`. Only groups with a
direct-walk option have that row (`add_walk_variables!`); the rest bound `alpha` nowhere."""
function worst_alpha_excess(s::Int, alpha)
    worst = -Inf
    for (p, (o, d)) in enumerate(mapping.Omega_s[s])
        demand = mapping.Q_s[s][p]
        demand > 0 || continue
        any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) || continue
        c_walk = walk_cost_weight * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)
        worst = max(worst, get(alpha, (s, p), 0.0) - c_walk)
    end
    return worst
end

"""`Gamma[j]`, aggregated exactly as `_solve_one_joint_routing_assignment_benders_subproblem`
does it, so a disagreement with the solver's own coefficients is detectable."""
function gamma_by_station(gamma_o, gamma_d)
    coefficients = Dict{Int, Float64}()
    for gammas in (gamma_o, gamma_d)
        for (key, gamma) in gammas
            gamma == 0.0 && continue
            j = key[2]
            coefficients[j] = get(coefficients, j, 0.0) + gamma
        end
    end
    return coefficients
end

"""Score one scenario cut `Theta_s >= constant - sum_j Gamma_j y_j` over the whole
first-stage space: validity, tightness at its anchor, and how often it says anything at all."""
function score_cut(s::Int, constant::Float64, coefficients::Dict{Int, Float64}, anchor_i::Int)
    worst_violation = -Inf
    n_dominated = 0
    rel_slack = Float64[]
    tight_at_anchor = NaN
    for (i, combo) in enumerate(combos)
        predicted = constant - sum(get(coefficients, j, 0.0) for j in combo; init = 0.0)
        worst_violation = max(worst_violation, predicted - Q[i, s])
        predicted <= 0.0 && (n_dominated += 1)
        push!(rel_slack, (Q[i, s] - predicted) / Q[i, s])
        i == anchor_i && (tight_at_anchor = Q[i, s] - predicted)
    end
    sort!(rel_slack)
    return (violation = worst_violation, tight = tight_at_anchor,
            dominated = n_dominated / length(combos),
            median_rel_slack = rel_slack[max(1, end ÷ 2)])
end

# ---------------------------------------------------------------- the arms
struct Row
    arm::Symbol
    anchor::Int
    scenario::Int
    q_exact::Float64
    q_arm::Float64
    min_rc::Float64
    rc_unbuilt::Bool
    alpha_excess::Float64
    gamma_mismatch::Float64
    nz::Int
    gamma_mass::Float64
    constant::Float64
    violation::Float64
    tight::Float64
    dominated::Float64
    median_rel_slack::Float64
    coefficients::Dict{Int, Float64}
end
rows = Row[]

for arm in ARMS
    solver = make_solver(arm)
    build = build_model(problem, monolith, solver)
    master = build.model
    subs = master[:benders_subproblem_builds]
    println("\n", "="^100)
    @printf("arm %s\n", arm)
    println("="^100)
    flush(stdout)
    for anchor_i in anchors
        y = as_y(combos[anchor_i])
        t_solve = time()
        sub = StationSelection._solve_joint_routing_assignment_benders_subproblems(
            master, y, solver)
        solve_sec = time() - t_solve
        by_scenario = Dict(r.scenario => r for r in sub.scenarios)
        for s in 1:n_scenarios(data)
            sm = subs[s].model
            # Reproduce the dual point the solver just built its cut from, using the SAME
            # functions in the SAME order (subproblem.jl). The per-`(p,j)` duals are what
            # the reduced-cost check needs and the scenario result only carries their
            # `Gamma_j` aggregate; `gamma_mismatch` below verifies the reproduction against
            # that aggregate rather than assuming it.
            alpha, gamma_o, gamma_d = extract_joint_routing_assignment_duals(sm)
            if arm in (:column_generation_activated, :column_generation_activated_lpo)
                StationSelection._benders_activated_complete_duals!(
                    alpha, gamma_o, gamma_d, y, data, mapping, s, walk_cost_weight)
                if arm === :column_generation_activated_lpo
                    StationSelection._benders_lpo_completion!(
                        alpha, gamma_o, gamma_d, y, data, mapping, s,
                        subs[s], solver.subproblem,
                        StationSelection._benders_lpo_core_point(sm))
                end
            end
            coefficients = gamma_by_station(gamma_o, gamma_d)
            constant = sum(values(alpha); init = 0.0)

            result = by_scenario[s]
            mismatch = maximum(
                abs(get(coefficients, j, 0.0) - get(result.y_coefficients, j, 0.0))
                for j in 1:N; init = 0.0)
            min_rc, rc_unbuilt = min_reduced_cost(s, alpha, gamma_o, gamma_d, y)
            scored = score_cut(s, constant, coefficients, anchor_i)
            push!(rows, Row(arm, anchor_i, s, Q[anchor_i, s], result.objective,
                            min_rc, rc_unbuilt, worst_alpha_excess(s, alpha), mismatch,
                            length(coefficients), sum(values(coefficients); init = 0.0),
                            constant, scored.violation, scored.tight, scored.dominated,
                            scored.median_rel_slack, coefficients))
            r = rows[end]
            # `println` over a concatenated string, NOT `@printf`: a `*`-joined format
            # string is not a literal and `@printf` rejects it at MACRO EXPANSION -- the
            # script fails to load rather than at the call. (Same trap as the note in
            # `subproblem_cg.jl`.)
            println("  y=", string(combos[anchor_i]), " s=", s,
                    " | Q ", @sprintf("%.4f", Q[anchor_i, s]),
                    " (arm ", @sprintf("%.4f", result.objective),
                    ", d ", @sprintf("%+.1e", result.objective - Q[anchor_i, s]), ")",
                    " | min rc ", @sprintf("%+.4e", min_rc),
                    rc_unbuilt ? " (col touches unbuilt)" : "",
                    " | cut: const ", @sprintf("%.1f", constant),
                    " mass/const ", @sprintf("%.2f", r.gamma_mass / max(constant, 1e-12)),
                    " nz ", r.nz,
                    " | worst viol ", @sprintf("%+.2e", scored.violation),
                    " | anchor slack ", @sprintf("%+.1e", scored.tight),
                    " | dominated ", @sprintf("%.1f%%", 100 * scored.dominated),
                    " | med rel slack ", @sprintf("%.1f%%", 100 * scored.median_rel_slack),
                    " | ", @sprintf("%.1fs", solve_sec))
            flush(stdout)
        end
    end
end

# ---------------------------------------------------------------- summary + checks
println("\n", "="^100)
println("per-arm summary (mean over anchors x scenarios)")
println("="^100)
@printf("%-38s %10s %10s %8s %10s %10s %10s\n",
        "arm", "min rc", "mass/const", "nz", "dominated", "med slack", "worst viol")
for arm in ARMS
    a = filter(r -> r.arm === arm, rows)
    isempty(a) && continue
    @printf("%-38s %+10.2e %10.2f %8.1f %9.1f%% %9.1f%% %+10.2e\n",
            arm, minimum(r.min_rc for r in a),
            sum(r.gamma_mass / max(r.constant, 1e-12) for r in a) / length(a),
            sum(r.nz for r in a) / length(a),
            100 * sum(r.dominated for r in a) / length(a),
            100 * sum(r.median_rel_slack for r in a) / length(a),
            maximum(r.violation for r in a))
end

# ------------------------------------------------- where the completion's slack comes from
# The closed-form bound charges `gammaO_pj = max(0, alpha_p - w * min_k walk)` on every
# unbuilt `(p,j)`, so its slack against the true dual is whatever a column's cost could
# have covered but this bound ignores. There are exactly three candidate credits, and the
# question is which one is big:
#
#   w * walk_min            what the closed form DOES credit (demand-free, deliberately:
#                           the pricer's own reward has no demand factor, and matching it
#                           is what makes the completion double as the search restriction)
#   w * demand_p * walk_min what `:route_free` credits instead -- the same term with the
#                           factor the column cost actually charges
#   beta * delta_j          the term BOTH throw away: any column visiting an unbuilt `j`
#                           pays at least the cheapest detour that touches it,
#                           delta_j = min_{a,b built} [travel(a,j) + travel(j,b) - travel(a,b)].
#                           Route-free to compute, but not chargeable per triple without a
#                           share-out rule, since one column can serve several out-triples
#                           at the same station.
#
# All three are printed against `alpha_p` itself, which is what the completion has to
# cancel. A credit that is small next to `alpha_p` cannot make the cut anything but a
# no-good cut on the anchor.
println("\n", "="^100)
println("what a completion could credit, per unbuilt (p,j), against the alpha it must cancel")
println("="^100)
beta = Float64(enum_subs[1].model[:joint_routing_assignment_route_regularization_weight])
mean_or_nan(v) = isempty(v) ? NaN : sum(v) / length(v)
@printf("%-18s %s %9s %9s %9s %9s %9s\n", "y", "s", "alpha", "w*walk", "w*Q*walk",
        "beta*del", "Gam/alpha")
for anchor_i in anchors
    y = as_y(combos[anchor_i])
    built = combos[anchor_i]
    for s in 1:n_scenarios(data)
        credit, credit_q = Float64[], Float64[]
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            walk_min = Dict{Int, Float64}()
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, kk = pair
                cost = od_pair_walking_cost(data, o, d, pair)
                walk_min[j] = min(get(walk_min, j, Inf), cost)
                walk_min[kk] = min(get(walk_min, kk, Inf), cost)
            end
            for (j, wmin) in walk_min
                y[j] < 0.5 || continue
                push!(credit, walk_cost_weight * wmin)
                push!(credit_q, walk_cost_weight * demand * wmin)
            end
        end
        # delta_j: cheapest detour that touches an unbuilt j, over pairs of BUILT stations.
        deltas = Float64[]
        for j in 1:N
            y[j] < 0.5 || continue
            best = Inf
            for a in built, b in built
                a == b && continue
                best = min(best, get_routing_cost(data, a, j) + get_routing_cost(data, j, b) -
                                 get_routing_cost(data, a, b))
            end
            push!(deltas, beta * max(0.0, best))
        end
        # `alpha_p` itself, and the ratio the cut lives or dies by: Gamma_j summed over
        # unbuilt stations against the cut's own constant.
        act = filter(r -> r.arm === :column_generation_activated && r.anchor == anchor_i &&
                          r.scenario == s, rows)
        cg = filter(r -> r.arm === :column_generation && r.anchor == anchor_i &&
                         r.scenario == s, rows)
        alpha_mean = isempty(act) ? NaN : first(act).constant /
            count(>(0), mapping.Q_s[s])
        @printf("%-18s %d %9.2f %9.2f %9.2f %9.2f %9.2f", string(built), s, alpha_mean,
                mean_or_nan(credit), mean_or_nan(credit_q), mean_or_nan(deltas),
                isempty(act) ? NaN : first(act).gamma_mass / max(first(act).constant, 1e-12))
        isempty(cg) || @printf("   (full CG: mass/const %.3f, nz %d)",
                               first(cg).gamma_mass / max(first(cg).constant, 1e-12),
                               first(cg).nz)
        println()
    end
end
flush(stdout)

println("\n", "="^100)
println("Gamma_j at the SAME anchor, per arm (unbuilt stations only -- built ones are never completed)")
println("="^100)
for anchor_i in anchors, s in 1:n_scenarios(data)
    built = combos[anchor_i]
    @printf("y=%-18s s=%d\n", string(built), s)
    for arm in ARMS
        r = filter(x -> x.arm === arm && x.anchor == anchor_i && x.scenario == s, rows)
        isempty(r) && continue
        c = first(r).coefficients
        parts = [@sprintf("%d:%.1f", j, get(c, j, 0.0)) for j in 1:N if !(j in built)]
        @printf("  %-38s const %9.2f | %s\n", arm, first(r).constant, join(parts, " "))
    end
end
flush(stdout)

println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
push_check!(name, ok, detail) = push!(checks, (name, ok, detail))
for arm in ARMS
    a = filter(r -> r.arm === arm, rows)
    isempty(a) && continue
    # THE correctness check: the completed duals must satisfy every column's dual
    # constraint over the exhaustive pool. This is dual feasibility itself, not a
    # consequence of it.
    worst_rc = minimum(r.min_rc for r in a)
    push_check!("$arm: full-universe dual feasible", worst_rc >= -1e-6,
        @sprintf("min rc over %d columns x %d anchors = %+.3e", sum(length.(pool)),
                 length(anchors), worst_rc))
    push_check!("$arm: alpha within its own bound",
        maximum(r.alpha_excess for r in a) <= 1e-6,
        @sprintf("worst alpha - c_walk = %+.3e", maximum(r.alpha_excess for r in a)))
    push_check!("$arm: restricted solve attains exact Q",
        maximum(abs(r.q_arm - r.q_exact) for r in a) <= 1e-6 * max(1.0, maximum(r.q_exact for r in a)),
        @sprintf("worst |Q_arm - Q_exact| = %.3e", maximum(abs(r.q_arm - r.q_exact) for r in a)))
    push_check!("$arm: cut valid at every y", maximum(r.violation for r in a) <= TOL,
        @sprintf("worst violation over %d cuts x %d sets = %+.3e",
                 length(a), length(combos), maximum(r.violation for r in a)))
    push_check!("$arm: cut tight at its anchor",
        maximum(abs(r.tight) for r in a) <= 1e-4,
        @sprintf("worst anchor slack = %+.3e", maximum(abs(r.tight) for r in a)))
    # The LPO's separation LP can be degenerate, so its second run may legitimately land on
    # a different optimal completion; the other arms are deterministic and must agree.
    mm = maximum(r.gamma_mismatch for r in a)
    if arm === :column_generation_activated_lpo
        @printf("note %-33s reproduction differs from the solver's own cut by %.3e (degenerate LP, informational)\n",
                arm, mm)
    else
        push_check!("$arm: reproduction matches solver's cut", mm <= 1e-9,
            @sprintf("worst |Gamma_mine - Gamma_solver| = %.3e", mm))
    end
end
for (name, ok, detail) in checks
    @printf("%-58s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)
n_fail == 0 || error("verification failed")
println("ALL CHECKS PASSED")
