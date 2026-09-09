"""Is the Benders optimum actually the optimum, and is every cut it generated VALID?

The existing check (`benders_joint_n10.jl`) compares the Benders objective against the same
mixed model solved monolithically. That is a real check, but it shares the model DEFINITION
with the thing under test (same `mapping`, same enumerated pool, same cost weights) and it
is end-to-end: it says "the two agree", not "each cut was sound". And Benders converging in
3-5 cuts is exactly the signature an INVALID (too strong) cut would produce, so
fast convergence is the case that most deserves an independent check rather than the one
that least does.

This script provides two checks that share none of the master/cut machinery.

# 1. Brute-force value function

At n=10 with k=5 there are only C(10,5) = 252 station sets. For each one that the MASTER
could actually propose (see step 0 -- endpoint feasibility), evaluate the second stage
EXACTLY: fix `y`, solve each scenario's subproblem LP, sum. The minimum is the true mixed
optimum by construction, computed with no master, no cuts, no loop, and no monolithic MIP.
If the Benders objective differs, Benders is wrong.

Note this reuses `_solve_joint_routing_assignment_benders_subproblems`, so the subproblem
LPs are shared -- deliberately: the subproblem is the *definition* of the second-stage cost
here, and an independent reimplementation of it would be testing a different model. What is
NOT shared is everything that makes Benders an algorithm: the master, the cut derivation,
the loop, the convergence test. Those are what this validates. (It also runs the
strong-duality assertion once per station set per scenario as a side effect.)

# 2. Pointwise cut validity over the ENTIRE first-stage space

A Benders optimality cut `Theta_s >= c - sum_j Gamma_j y_j` is valid only if it
UNDERESTIMATES the true second-stage cost `Q_s(y)` at every feasible `y` -- not merely at
the `yhat` it was derived at -- over the MASTER's feasible set, which is where `Q_s` is
defined. With those `y` enumerated and `Q_s(y)` known exactly from check 1, that is
directly testable:

    for every cut, for every y:   c - sum_j Gamma_j y_j  <=  Q_s(y) + tol

A cut that violates this anywhere is invalid and could exclude the true optimum -- the
failure mode that produces a confident wrong answer and the one the fast convergence
raises. The script also reports each cut's minimum slack, which should be ~0 for at least
one `y` (the point it was derived at): a cut that never touches `Q` is valid but was
derived wrong, and a cut whose minimum slack is far above 0 everywhere is doing no work.

Cuts are read back from `m[:benders_cut_signatures]`, which is what
`_add_joint_routing_assignment_benders_cuts!` records for deduplication -- the group,
the constant, and the nonzero `y` coefficients, i.e. exactly the row it built.

MultiCut only: the check needs each cut mapped to the scenario whose cost it bounds, and
under `SingleCut` a cut bounds the SUM over scenarios (group 0), which is handled but is a
weaker statement.

Usage: sbatch benchmarks/diagnostics/run_benders_brute_force.sh
Env: BF_N BF_P BF_S BF_SEED BF_MAX_STOPS
"""

using StationSelection
using JuMP
using Printf
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "BF_N", "10"))
const P = parse(Int, get(ENV, "BF_P", "8"))
const S = parse(Int, get(ENV, "BF_S", "1"))
const SEED = parse(Int, get(ENV, "BF_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "BF_MAX_STOPS", "4"))
const TOL = 1e-6

problem, k, _meta = benchmark_problem(@__DIR__, "BF", N, P, S, SEED)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)
solver = BendersSolver(
    config=SolverOptions(silent=true, time_limit_sec=300.0),
    max_iterations=500,
    subproblem=BendersSubproblemConfig(
        max_stops=MAX_STOPS, max_routes=200_000, enumeration_time_limit_sec=300.0))

n_sets = binomial(N, k)
@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d | C(%d,%d)=%d station sets\n",
        N, P, S, SEED, k, MAX_STOPS, N, k, n_sets)
flush(stdout)

# ---------------------------------------------------------------- Benders
result = run_opt(problem, formulation, solver)
md = result.metadata
benders_obj = something(result.objective_value, NaN)
@printf("\nBenders: %s obj %.6f | iters %d cuts %d | stations %s\n",
        result.termination_status, benders_obj, md["benders_iterations"],
        md["benders_cuts_added"], string(result.solution.selected_station_indices))
flush(stdout)

master = result.model
cut_signatures = collect(master[:benders_cut_signatures])
@printf("cuts recorded: %d\n", length(cut_signatures))

# --------------------------------------- 0. which station sets the MASTER can propose
# The value function Q(y) is only defined on the master's feasible set, and a Benders cut
# only has to underestimate Q there -- not over all C(n,k) sets. That distinction is not a
# technicality here: a station set violating endpoint feasibility leaves some demand group
# with no direct-walk fallback and no buildable (j,k) pair, so its coverage row `sum(theta)
# >= 1` is unsatisfiable and the SUBPROBLEM IS INFEASIBLE. Feeding those to the audit
# crashes it (measured: runs 22417479/22417480 both died on scenario 1 this way).
#
# Reproduces `add_aggregate_od_route_endpoint_feasibility_constraints!`'s own rule: a
# location is required iff some positive-demand group at it lacks WALK_ONLY_PAIR, and it is
# satisfied iff some station within `max_walking_distance` of it is built.
data = problem.data
mapping = create_aggregate_od_route_map(problem, formulation, data)
required = Set{Int}()
for s in 1:n_scenarios(data)
    for (p, (o, d)) in enumerate(mapping.Omega_s[s])
        mapping.Q_s[s][p] > 0 || continue
        any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
        push!(required, o)
        push!(required, d)
    end
end
candidates = Dict(pt => Set(j for j in 1:N
                            if get_walking_cost(data, pt, j) <= mapping.max_walking_distance)
                  for pt in required)
master_feasible(combo) = all(!isempty(intersect(candidates[pt], combo)) for pt in required)
@printf("required locations: %d | endpoint-feasible station sets: ", length(required))

# ------------------------------------------------- 1. brute-force value function
# Q[i] = per-scenario second-stage costs at station set i; totals[i] their sum.
all_combos = collect(combinations(1:N, k))
length(all_combos) == n_sets || error("combination count mismatch")
combos = filter(master_feasible, all_combos)
@printf("%d of %d\n", length(combos), n_sets)
flush(stdout)
isempty(combos) && error("no endpoint-feasible station set at k=$k -- instance is infeasible")

per_scenario = Vector{Vector{Float64}}(undef, length(combos))
totals = Vector{Float64}(undef, length(combos))
n_unexpected_infeasible = 0

t0 = time()
for (i, combo) in enumerate(combos)
    y = zeros(Float64, N)
    y[combo] .= 1.0
    try
        sub = StationSelection._solve_joint_routing_assignment_benders_subproblems(master, y, solver)
        per_scenario[i] = Float64[r.objective for r in sub.scenarios]
        totals[i] = sub.total_objective
    catch err
        # An endpoint-feasible y whose subproblem is STILL infeasible is a real finding, not
        # a script bug: endpoint feasibility is a NECESSARY condition only (it places a
        # station near each endpoint; it does not promise a physically feasible route column
        # linking them), so the master can legitimately propose a `y` the second stage
        # cannot serve -- and `solve_subproblem` raises rather than adding a feasibility cut.
        # Recorded and excluded from the value function rather than aborting the audit.
        n_unexpected_infeasible += 1
        per_scenario[i] = Float64[]
        totals[i] = Inf
        @printf("  !! endpoint-feasible set %s has an INFEASIBLE subproblem: %s\n",
                string(combo), sprint(showerror, err)[1:min(end, 120)])
    end
end
brute_sec = time() - t0

best_i = argmin(totals)
brute_obj = totals[best_i]
n_optima = count(t -> isapprox(t, brute_obj; rtol=1e-9, atol=1e-9), totals)
@printf("\nbrute force: %d endpoint-feasible sets in %.1fs | min %.6f at %s | %d tie at the optimum\n",
        length(combos), brute_sec, brute_obj, string(combos[best_i]), n_optima)
n_unexpected_infeasible == 0 ||
    @printf("             %d endpoint-feasible set(s) had an INFEASIBLE subproblem (see above)\n",
            n_unexpected_infeasible)
@printf("             worst set %.6f (%.1f%% above optimum) | spread over sets %.6f\n",
        maximum(totals), 100 * (maximum(totals) - brute_obj) / brute_obj,
        maximum(totals) - brute_obj)
flush(stdout)

# --------------------------------------- 2. pointwise cut validity over all 252 y
"""Worst violation and tightest slack of one recorded cut across every station set."""
function audit_cut(group::Int, constant::Float64, coefficients::Vector)
    worst_violation = -Inf   # max over y of (cut prediction - true Q)
    tightest = Inf           # min over y of (true Q - cut prediction)
    at_worst = Int[]
    for (i, combo) in enumerate(combos)
        predicted = constant
        for (j, coefficient) in coefficients
            j in combo && (predicted -= coefficient)
        end
        isempty(per_scenario[i]) && continue   # subproblem-infeasible set, excluded above
        q = group == 0 ? totals[i] : per_scenario[i][group]
        violation = predicted - q
        if violation > worst_violation
            worst_violation = violation
            at_worst = combo
        end
        tightest = min(tightest, q - predicted)
    end
    return worst_violation, tightest, at_worst
end

println("\ncut audit (a VALID cut never exceeds the true second-stage cost at ANY y):")
cut_rows = Tuple{Int, Float64, Float64, Vector{Int}}[]
for (group, constant, coefficients) in cut_signatures
    violation, tightest, at_worst = audit_cut(group, constant, coefficients)
    push!(cut_rows, (group, violation, tightest, at_worst))
    @printf("  group %d | %2d nonzero coefs | worst violation %+.3e | tightest slack %+.3e%s\n",
            group, length(coefficients), violation, tightest,
            violation > TOL ? "  <-- INVALID at $(at_worst)" : "")
end
flush(stdout)

# ---------------------------------------------------------------- checks
println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
push_check!(name, ok, detail) = push!(checks, (name, ok, detail))

push_check!("benders == brute-force optimum",
    isapprox(benders_obj, brute_obj; rtol=1e-6, atol=1e-6),
    @sprintf("%.6f vs %.6f (diff %.3e)", benders_obj, brute_obj, benders_obj - brute_obj))
# Directional, and the direction matters: the UB is an achievable cost, so Benders can
# never be BELOW the true optimum. Only "above" is a reachable failure, and that is what an
# invalid cut would produce -- it would have excluded the true optimum.
push_check!("benders not below the optimum",
    benders_obj >= brute_obj - TOL,
    @sprintf("%.6f >= %.6f", benders_obj, brute_obj))
push_check!("every cut valid at every y",
    all(r -> r[2] <= TOL, cut_rows),
    @sprintf("worst violation over %d cuts x %d sets = %+.3e",
             length(cut_rows), length(combos), maximum(r[2] for r in cut_rows; init=-Inf)))
# A cut that never touches Q anywhere is valid but useless -- it would mean the derivation
# is not tight at the point it came from, which is the other way to get a wrong answer
# (too weak rather than too strong: the loop would stall or converge to a bad bound).
push_check!("every cut tight somewhere",
    all(r -> abs(r[3]) <= 1e-4, cut_rows),
    @sprintf("largest tightest-slack = %.3e", maximum(abs(r[3]) for r in cut_rows; init=0.0)))
# An endpoint-feasible y with an infeasible subproblem means the "no feasibility cuts
# needed" claim does not hold on this instance, and solve_subproblem would RAISE mid-loop
# if the master ever proposed one. Surfaced as a check so it cannot pass unnoticed.
push_check!("no master-feasible y is second-stage infeasible",
    n_unexpected_infeasible == 0,
    "$n_unexpected_infeasible of $(length(combos)) endpoint-feasible sets")
push_check!("converged OPTIMAL",
    md["benders_converged"] === true && string(result.termination_status) == "OPTIMAL",
    "$(result.termination_status) / $(md["benders_stop_reason"])")

for (name, ok, detail) in checks
    @printf("%-34s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)
n_fail == 0 || error("verification failed")
println("ALL CHECKS PASSED")
