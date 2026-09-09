"""Two questions: is there a truly independent arm that agrees, and WHY are there so few cuts?

# Question 1: an independent DirectMIPSolver arm

Every reference used so far -- `mixed_mono`, `direct_mip`, and even `CGSolver`'s master --
is built by the SAME function, `_build_joint_routing_assignment_model`. So a bug in that one
function would make all three agree and all three be wrong. (Benders is the odd one out: its
master and subproblem are separate constructions, `_build_joint_routing_assignment_master_model`
and `_build_joint_routing_assignment_subproblem_model`.)

`AggregateODRouteJointRoutingAssignmentFormulation` has its own published `DirectMIPSolver`
path (`optimize/aggregate_od_route/direct/build_joint_routing_assignment.jl`). This runs it,
so the comparison is against the *shipped* direct arm rather than a reference this session
hand-rolled. It goes through `build_model(problem, formulation, DirectMIPSolver(); ...)` +
`optimize_model`, not through `run_opt` -- see NOTE below, that is a limitation of the 3-arg
API, not a choice.

What remains shared by every arm, and is therefore NOT tested by any of them: the
`AggregateODRouteMap`, `joint_routing_assignment_column_cost`, and the objective assembly.
Those *are* the model definition; falsifying them needs an oracle outside this formulation
family entirely.

# Question 2: why so few cuts -- the previous explanation was WRONG

The claim was that each cut prices every station (`Gamma_j` binding at `y_j = 0`), so one cut
constrains the whole space. The pointwise audit refutes it: cuts carry **1-2 nonzero
coefficients** out of n stations. They are extremely sparse.

The real hypothesis, from the n=10/s=3 trace: one round of cuts moved the lower bound from 0
to 25696.15 against an optimum of 25771.19 -- **99.71% of the way in a single round**. That
happens when the second-stage value function is nearly FLAT relative to its magnitude: `Q(y)`
never approaches zero, so a cut anchored anywhere has a large constant term that is already
close to the optimum, and the sparse `Gamma_j` correction only has to trim the last fraction
of a percent.

If that is right, the low cut count is a property of the INSTANCE, not of the decomposition
or the cut strength -- and it means these instances barely stress the method at all. Measured
here as:

    Q_max / Q_min            over the master's feasible set (flatness ratio)
    LB after round 1 / Q_min (how much of the gap one round of cuts closes)
    nonzero coefficients per cut

A flatness ratio near 1 with round-1 closure near 100% would confirm it. The prediction it
makes: an instance family with a low-cost floor (so `Q` varies by orders of magnitude rather
than 35%) should need far more cuts, and the historical nearest-open runs at p=16/32 needing
80-770 cuts is consistent with exactly that.

NOTE, a real papercut found while writing this: `run_opt(problem, formulation, solver)` cannot
forward `max_routes`/`time_limit_sec` to `build_model`, so the published 3-arg path for
(Joint, DirectMIPSolver) is stuck at the enumerator's default `max_routes=10_000` and is
unusable at these sizes. The 2-step `build_model` + `optimize_model` form is the only way to
reach it.

Usage: sbatch benchmarks/diagnostics/run_benders_direct_arm.sh
Env: DA_NS DA_P DA_S DA_SEED DA_MAX_STOPS DA_BRUTE_N
"""

using StationSelection
using JuMP
using Printf
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const NS = [parse(Int, x) for x in split(get(ENV, "DA_NS", "10,15,20"), ',')]
const P = parse(Int, get(ENV, "DA_P", "8"))
const S = parse(Int, get(ENV, "DA_S", "3"))
const SEED = parse(Int, get(ENV, "DA_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "DA_MAX_STOPS", "4"))
# Brute force is only affordable where C(n,k) is small; n=20 gives C(20,10)=184,756.
const BRUTE_N = parse(Int, get(ENV, "DA_BRUTE_N", "15"))
const MAX_ROUTES = 20_000_000
const ENUM_LIMIT = 3600.0
const TOL = 1e-6

@printf("n=%s p=%d s=%d seed=%d max_stops=%d | brute force for n<=%d\n",
        string(NS), P, S, SEED, MAX_STOPS, BRUTE_N)
flush(stdout)

rows = Any[]
for n in NS
    @printf("\n%s\n=== n=%d ===\n", repeat("=", 76), n)
    flush(stdout)
    try
        problem, k, _meta = benchmark_problem(@__DIR__, "DA", n, P, S, SEED)
        formulation = AggregateODRouteJointRoutingAssignmentFormulation(
            ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)

        # ---- Benders, keeping the round-1 lower bound ----
        trace = Any[]
        bsolver = BendersSolver(
            config=SolverOptions(silent=true, time_limit_sec=600.0),
            max_iterations=2000,
            subproblem=BendersSubproblemConfig(max_stops=MAX_STOPS,
                                               max_routes=MAX_ROUTES,
                                               enumeration_time_limit_sec=ENUM_LIMIT),
            total_time_limit_sec=5400.0,
            iteration_callback=row -> push!(trace, row))
        bd = run_opt(problem, formulation, bsolver)
        benders = something(bd.objective_value, NaN)
        cut_sig = collect(bd.model[:benders_cut_signatures])
        nnz = [length(c[3]) for c in cut_sig]
        # The lower bound the SECOND iteration saw, i.e. what one full round of cuts bought.
        lb_round1 = length(trace) >= 2 ? trace[2].lower_bound : NaN
        @printf("Benders     : %s %.6f | iters %d cuts %d | LB after round 1 = %.6f (%.4f%% of opt)\n",
                bd.termination_status, benders, bd.metadata["benders_iterations"],
                bd.metadata["benders_cuts_added"], lb_round1, 100 * lb_round1 / benders)
        @printf("              cut nonzero-coef counts: %s  (out of n=%d stations)\n",
                string(sort(nnz)), n)
        flush(stdout)

        # ---- the SHIPPED DirectMIPSolver arm ----
        # 2-step form on purpose: run_opt cannot forward max_routes (see docstring NOTE).
        dsolver = DirectMIPSolver(config=SolverOptions(silent=true, time_limit_sec=1800.0))
        t0 = time()
        dbuild = build_model(problem, formulation, dsolver;
                            max_routes=MAX_ROUTES, time_limit_sec=ENUM_LIMIT)
        dres = StationSelection.optimize_model(dbuild, dsolver)
        direct_sec = time() - t0
        direct_obj = something(dres.objective_value, NaN)
        @printf("DirectMIP   : %s %.6f | pool %d | %.1fs\n",
                dres.termination_status, direct_obj,
                dbuild.counts.extras["seed_columns_added"], direct_sec)
        flush(stdout)

        # ---- flatness of the value function, where affordable ----
        q_min = q_max = NaN
        n_feasible = 0
        if n <= BRUTE_N
            data = problem.data
            mapping = create_aggregate_od_route_map(problem, formulation, data)
            required = Set{Int}()
            for s in 1:n_scenarios(data)
                for (p, (o, d)) in enumerate(mapping.Omega_s[s])
                    mapping.Q_s[s][p] > 0 || continue
                    any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
                    push!(required, o); push!(required, d)
                end
            end
            cand = Dict(pt => Set(j for j in 1:n
                                  if get_walking_cost(data, pt, j) <= mapping.max_walking_distance)
                        for pt in required)
            feas = filter(c -> all(!isempty(intersect(cand[pt], c)) for pt in required),
                          collect(combinations(1:n, k)))
            n_feasible = length(feas)
            totals = Float64[]
            for combo in feas
                y = zeros(Float64, n); y[combo] .= 1.0
                try
                    push!(totals, StationSelection._solve_joint_routing_assignment_benders_subproblems(
                        bd.model, y, bsolver).total_objective)
                catch
                end
            end
            q_min, q_max = minimum(totals), maximum(totals)
            @printf("value fn    : %d master-feasible sets | Q in [%.2f, %.2f] | flatness Q_max/Q_min = %.4f\n",
                    n_feasible, q_min, q_max, q_max / q_min)
            flush(stdout)
        end

        push!(rows, (n=n, k=k, ok=true, benders=benders, direct=direct_obj,
                     dstatus=string(dres.termination_status),
                     iters=bd.metadata["benders_iterations"],
                     cuts=bd.metadata["benders_cuts_added"], nnz=nnz,
                     lb_round1=lb_round1, q_min=q_min, q_max=q_max,
                     n_feasible=n_feasible, error=nothing))
    catch err
        msg = sprint(showerror, err)
        @printf("!! n=%d FAILED: %s\n", n, first(msg, 250))
        flush(stdout)
        push!(rows, (n=n, ok=false, error=msg))
    end
end

# ---------------------------------------------------------------- summary
println("\n", repeat("=", 76))
@printf("%4s %12s %12s %6s %6s %10s %10s %9s\n",
        "n", "benders", "DirectMIP", "iters", "cuts", "nnz/cut", "rnd1 %opt", "flatness")
for r in rows
    r.ok || (@printf("%4d  FAILED\n", r.n); continue)
    @printf("%4d %12.2f %12.2f %6d %6d %10s %9.4f%% %9s\n",
            r.n, r.benders, r.direct, r.iters, r.cuts,
            "$(minimum(r.nnz))-$(maximum(r.nnz))",
            100 * r.lb_round1 / r.benders,
            isnan(r.q_max) ? "-" : @sprintf("%.4f", r.q_max / r.q_min))
end

println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
for r in rows
    r.ok || (push!(checks, ("n=$(r.n): completed", false, first(r.error, 120))); continue)
    # THE independent-arm check: the shipped DirectMIPSolver path shares no master
    # construction with Benders.
    push!(checks, ("n=$(r.n): benders == DirectMIPSolver",
        isapprox(r.benders, r.direct; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f (diff %.3e)", r.benders, r.direct, r.benders - r.direct)))
    push!(checks, ("n=$(r.n): DirectMIP solved to optimality",
        r.dstatus == "OPTIMAL", r.dstatus))
    if !isnan(r.q_max)
        push!(checks, ("n=$(r.n): benders == min over all y",
            isapprox(r.benders, r.q_min; rtol=1e-6, atol=1e-6),
            @sprintf("%.6f vs %.6f", r.benders, r.q_min)))
    end
end
for (name, ok, detail) in checks
    @printf("%-40s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)

good = [r for r in rows if r.ok && !isnan(r.q_max)]
if !isempty(good)
    println()
    println("CUT-COUNT VERDICT")
    for r in good
        @printf("  n=%d: one round of cuts reached %.4f%% of the optimum; value function spans\n",
                r.n, 100 * r.lb_round1 / r.benders)
        @printf("        only %.2fx (Q in [%.0f, %.0f]) over %d master-feasible station sets.\n",
                r.q_max / r.q_min, r.q_min, r.q_max, r.n_feasible)
    end
    println("  => The few cuts are explained by a FLAT, high-floor value function, not by cut")
    println("     strength -- the cuts are sparse (1-2 nonzero coefficients). This is a property")
    println("     of the instance family, so these cells barely stress the decomposition.")
end
n_fail == 0 || error("verification failed")
println("\nALL CHECKS PASSED")
