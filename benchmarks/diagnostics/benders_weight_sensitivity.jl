"""What makes this instance family's value function FLAT, and can the weights un-flatten it?

Every Benders cell measured so far used `BENCHMARK_BASELINE` (benchmarks/lib/cg_benchmark.jl)
with `max_stops` overridden to 4:

    route_regularization_weight = 10.0     max_wait_time   = 900.0
    walk_cost_weight            =  0.1     detour_factor   =   2.0
    repositioning_time          = 20.0     max_stops       =   4   (baseline is 10)

plus, from `benchmark_problem`: `k = max(2, ceil(n/2))` and `max_walking_distance = 600.0`,
on Zhuzhou with p=8 OD pairs per scenario.

# The hypothesis

An active route column costs
`route_regularization_weight * (tau + repositioning_time)` = `10*tau + 200`. So every
activated column carries a **fixed charge of 200** regardless of how long the route is,
while the walking term -- the ONLY part of the objective that depends on which stations are
built -- is scaled by `walk_cost_weight = 0.1`. That is a 100x weight asymmetry on top of a
large per-column constant.

If that is what flattens `Q(y)`, then the cost is dominated by "how many vehicle trips must
I activate", which barely depends on the station set, and the station-sensitive part is
damped by 0.1. Measured consequence so far: `Q_max/Q_min` = 1.35 at n=10 and 1.44 at n=15,
i.e. even the worst station set is only 35-44% worse than the best, which is why a handful
of Benders cuts suffices.

# The test

Two parts, both at n=10 / s=3 where the master's feasible set (86 station sets) can be
enumerated exhaustively and so `Q_max/Q_min` is exact rather than estimated.

1. **Objective decomposition at the optimum**, split three ways -- walking, route travel
   (`10*tau`), and route fixed charge (`200` per active column). If the fixed charge
   dominates, the hypothesis has its mechanism.

2. **Weight arms.** `repositioning_time = 0` is the sharp one: it removes the fixed charge
   and changes nothing else, so if flatness collapses there specifically, the fixed charge is
   the cause rather than the weight ratio. The others vary the ratio directly.

   | arm | route_reg | walk_cost | repositioning |
   | --- | --- | --- | --- |
   | baseline | 10.0 | 0.1 | 20.0 |
   | no_fixed_charge | 10.0 | 0.1 | 0.0 |
   | walk_heavy | 10.0 | 1.0 | 20.0 |
   | route_light | 1.0 | 0.1 | 20.0 |
   | balanced | 1.0 | 1.0 | 20.0 |

For each arm: exact flatness over the 86 sets, Benders iterations/cuts, and a correctness
check against the same brute-force minimum. The prediction is that flatness and cut count
move TOGETHER -- a wider-range value function should need more cuts. If cut count stays flat
even as `Q_max/Q_min` grows, then the low cut count is NOT explained by instance flatness
and the explanation is still open.

Note every arm is a different MODEL, so objectives are not comparable across arms; only
flatness and cut counts are. Each arm is checked against its OWN brute-force optimum.

Usage: sbatch benchmarks/diagnostics/run_benders_weights.sh
Env: WS_N WS_P WS_S WS_SEED WS_MAX_STOPS
"""

using StationSelection
using JuMP
using Printf
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "WS_N", "10"))
const P = parse(Int, get(ENV, "WS_P", "8"))
const S = parse(Int, get(ENV, "WS_S", "3"))
const SEED = parse(Int, get(ENV, "WS_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "WS_MAX_STOPS", "4"))
const MAX_ROUTES = 20_000_000
const ENUM_LIMIT = 3600.0

const ARMS = [
    (name="baseline",        route_reg=10.0, walk=0.1, repo=20.0),
    (name="no_fixed_charge", route_reg=10.0, walk=0.1, repo=0.0),
    (name="walk_heavy",      route_reg=10.0, walk=1.0, repo=20.0),
    (name="route_light",     route_reg=1.0,  walk=0.1, repo=20.0),
    (name="balanced",        route_reg=1.0,  walk=1.0, repo=20.0),
]

problem, k, _meta = benchmark_problem(@__DIR__, "WS", N, P, S, SEED)
@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d max_walk=%.0f\n",
        N, P, S, SEED, k, MAX_STOPS, problem.max_walking_distance)
flush(stdout)

_formulation(a) = AggregateODRouteJointRoutingAssignmentFormulation(
    route_regularization_weight=a.route_reg, walk_cost_weight=a.walk,
    repositioning_time=a.repo, max_wait_time=900.0, detour_factor=2.0,
    max_stops=MAX_STOPS)

"""Master-feasible station sets -- the domain on which Q is defined (see the brute-force
certificate script for why the unrestricted set crashes the subproblem)."""
function master_feasible_sets(formulation)
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
    cand = Dict(pt => Set(j for j in 1:N
                          if get_walking_cost(data, pt, j) <= mapping.max_walking_distance)
                for pt in required)
    return filter(c -> all(!isempty(intersect(cand[pt], c)) for pt in required),
                  collect(combinations(1:N, k)))
end

"""Three-way objective split of a SOLVED joint model: walking, route travel, route fixed."""
function decompose(m::JuMP.Model, a)
    obj = JuMP.objective_function(m)
    walk = 0.0
    for (_key, var) in m[:x_walk]
        walk += JuMP.coefficient(obj, var) * JuMP.value(var)
    end
    travel = 0.0
    fixed = 0.0
    columns = m[:joint_routing_assignment_columns]
    for (id, var) in m[:joint_routing_assignment_theta]
        v = JuMP.value(var)
        v > 1e-9 || continue
        # The column cost is route_reg*(tau + repositioning) + walk_cost*assignment walking.
        # Split the route part into its tau-dependent and fixed halves.
        travel += a.route_reg * columns[id].tau * v
        fixed += a.route_reg * a.repo * v
    end
    # Whatever is left is the assignment-walking inside column costs.
    total = JuMP.objective_value(m)
    return (walk_direct=walk, route_travel=travel, route_fixed=fixed,
            assignment_walk=total - walk - travel - fixed, total=total)
end

rows = Any[]
for a in ARMS
    @printf("\n%s\n=== arm %s (route_reg=%.1f walk=%.1f repo=%.1f) ===\n",
            repeat("=", 74), a.name, a.route_reg, a.walk, a.repo)
    flush(stdout)
    try
        formulation = _formulation(a)
        trace = Any[]
        solver = BendersSolver(
            config=SolverOptions(silent=true, time_limit_sec=600.0),
            max_iterations=2000,
            subproblem=BendersSubproblemConfig(max_stops=MAX_STOPS, max_routes=MAX_ROUTES,
                                               enumeration_time_limit_sec=ENUM_LIMIT),
            iteration_callback=r -> push!(trace, r))
        bd = run_opt(problem, formulation, solver)
        benders = something(bd.objective_value, NaN)
        nnz = [length(c[3]) for c in collect(bd.model[:benders_cut_signatures])]

        # exact flatness over the master's feasible set
        sets = master_feasible_sets(formulation)
        totals = Float64[]
        for combo in sets
            y = zeros(Float64, N); y[combo] .= 1.0
            try
                push!(totals, StationSelection._solve_joint_routing_assignment_benders_subproblems(
                    bd.model, y, solver).total_objective)
            catch
            end
        end
        q_min, q_max = minimum(totals), maximum(totals)

        # objective split, from the shipped direct arm (one model holds y, x_walk and theta)
        dsolver = DirectMIPSolver(config=SolverOptions(silent=true, time_limit_sec=1800.0))
        dbuild = build_model(problem, formulation, dsolver;
                             max_routes=MAX_ROUTES, time_limit_sec=ENUM_LIMIT)
        dres = StationSelection.optimize_model(dbuild, dsolver)
        split = decompose(dbuild.model, a)

        @printf("Benders %s %.4f | iters %d cuts %d | nnz/cut %d-%d\n",
                bd.termination_status, benders, bd.metadata["benders_iterations"],
                bd.metadata["benders_cuts_added"], minimum(nnz), maximum(nnz))
        @printf("flatness Q_max/Q_min = %.4f  (Q in [%.2f, %.2f] over %d sets)\n",
                q_max / q_min, q_min, q_max, length(totals))
        @printf("objective split: direct-walk %.1f%% | route travel %.1f%% | ROUTE FIXED %.1f%% | assign-walk %.1f%%\n",
                100*split.walk_direct/split.total, 100*split.route_travel/split.total,
                100*split.route_fixed/split.total, 100*split.assignment_walk/split.total)
        flush(stdout)
        push!(rows, (arm=a.name, ok=true, benders=benders, direct=something(dres.objective_value, NaN),
                     iters=bd.metadata["benders_iterations"], cuts=bd.metadata["benders_cuts_added"],
                     flat=q_max/q_min, q_min=q_min, q_max=q_max, split=split, error=nothing))
    catch err
        msg = sprint(showerror, err)
        @printf("!! arm %s FAILED: %s\n", a.name, first(msg, 250))
        flush(stdout)
        push!(rows, (arm=a.name, ok=false, error=msg))
    end
end

println("\n", repeat("=", 74))
@printf("%-16s %6s %6s %9s %11s %11s\n", "arm", "iters", "cuts", "flatness", "fixed %", "objective")
for r in rows
    r.ok || (@printf("%-16s FAILED\n", r.arm); continue)
    @printf("%-16s %6d %6d %9.4f %10.1f%% %11.2f\n",
            r.arm, r.iters, r.cuts, r.flat, 100*r.split.route_fixed/r.split.total, r.benders)
end

println("\n=== checks (each arm against its OWN brute-force optimum) ===")
checks = Tuple{String, Bool, String}[]
for r in rows
    r.ok || (push!(checks, ("$(r.arm): completed", false, first(r.error, 100))); continue)
    push!(checks, ("$(r.arm): benders == brute min",
        isapprox(r.benders, r.q_min; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f", r.benders, r.q_min)))
    push!(checks, ("$(r.arm): benders == DirectMIPSolver",
        isapprox(r.benders, r.direct; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f", r.benders, r.direct)))
end
for (name, ok, detail) in checks
    @printf("%-40s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)

good = [r for r in rows if r.ok]
if length(good) >= 2
    println("\nVERDICT")
    fl = [(r.arm, r.flat, r.cuts) for r in good]
    sort!(fl; by=x -> x[2])
    @printf("  flattest: %s (%.4f, %d cuts) ... widest: %s (%.4f, %d cuts)\n",
            fl[1][1], fl[1][2], fl[1][3], fl[end][1], fl[end][2], fl[end][3])
    if fl[end][3] > fl[1][3]
        println("  => cut count RISES with the range of the value function, so the low baseline")
        println("     count is explained by instance flatness rather than by cut strength.")
    else
        println("  => cut count did NOT rise with the value-function range. Instance flatness does")
        println("     NOT explain the low cut count, and the explanation is still open.")
    end
end
n_fail == 0 || error("verification failed")
println("\nALL CHECKS PASSED")
