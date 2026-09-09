"""Why are there so few Benders cuts? Compare n=10/15/20 at s=3, MultiCut only.

Every cell so far converged in 4 iterations with 3-5 cuts, at n=10 AND n=15, s=1 AND s=3.
A constant iteration count across two instance sizes is either a real property of this
decomposition or a bug that makes cuts too strong, and the aggregate metadata cannot tell
them apart. This script is built to separate them.

# The discriminator is the size of the first-stage space

The master chooses `y` from C(n,k) station sets:

    n=10, k=5   ->        252
    n=15, k=8   ->      6,435
    n=20, k=10  ->    184,756

That is a 733x growth from n=10 to n=20. If the cut count stays at 3-5 across it, the low
count is a genuine property (the cut carries information about EVERY station, including
unbuilt ones -- `Gamma_j` is the shadow price of the linking row `sum(theta) <= y_j`, which
is binding at `y_j = 0`, so one cut constrains the whole space rather than one vertex). If
it grows roughly with the space, then the low count at n=10 was simply smallness and there
was never anything to explain.

# Per-iteration trace, because the aggregate hides the mechanism

`iteration_callback` records lower/upper bounds per iteration. The expected shape, if the
decomposition is sound, is: iteration 1 lands an arbitrary `y` (the master is degenerate --
`Theta >= 0` is its only bound and the whole objective is second-stage), the incumbent is
found early, and the REMAINING iterations raise the lower bound to meet it. Every run so far
reported `best at 2` with 4 iterations, which is exactly that shape, but it has never been
observed directly.

The failure shape to look for is the opposite: bounds moving together, or a lower bound that
jumps to the final value in one step. That would suggest the master is being *steered* by
over-strong cuts rather than *bounded* by valid ones.

# Correctness at each n, with two independent references

- `mixed_mono` -- the same mixed model solved monolithically over the same enumerated pool.
  Validates the algorithm; shares the model definition.
- `CGSolver` -- prices columns by label-setting, never enumerates, so `cg_lp` lower-bounds
  the true mixed optimum WHATEVER the enumerated pool holds. `cg_ip == direct_mip` is the
  pool-agreement check: two all-binary optima over two independently built pools, where a
  route the enumerator missed shows up and nowhere else. Only meaningful when CG converged
  at `full_route_universe` scope, which is checked.
- Lower-bound invariants per iteration: monotone non-decreasing, and never above the final
  objective. An invalid (too strong) cut would push the lower bound above the true optimum,
  which is a direct test needing no reference at all.

# Deliberately MultiCut only, and deliberately generous limits

`SingleCut` is excluded: at s=3 it aggregates three scenarios into one row and is a strictly
weaker relaxation, which would confound the cut-count question with a cut-mode question.
`max_routes` and `max_iterations` are set far above anything observed so the comparison is
never limit-bound -- if a cell stops early it must be for a real reason, not a cap.

Each `n` is independently guarded: a blow-up at n=20 must not destroy the n=10/15 rows,
since the whole point is the comparison across sizes.

Usage: sbatch benchmarks/diagnostics/run_benders_scaling.sh
Env: SC_NS (comma list, default 10,15,20) SC_P SC_S SC_SEED SC_MAX_STOPS SC_MAX_ROUTES
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const NS = [parse(Int, x) for x in split(get(ENV, "SC_NS", "10,15,20"), ',')]
const P = parse(Int, get(ENV, "SC_P", "8"))
const S = parse(Int, get(ENV, "SC_S", "3"))
const SEED = parse(Int, get(ENV, "SC_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "SC_MAX_STOPS", "4"))
# Generous on purpose: nothing here should ever be limit-bound (n=15/s=3 exceeded the old
# 200_000 cap, which is what motivated raising it rather than a guess).
const MAX_ROUTES = parse(Int, get(ENV, "SC_MAX_ROUTES", "20000000"))
const ENUM_LIMIT = 3600.0
const MAX_ITERS = 2000
const TOL = 1e-6

@printf("n values %s | p=%d s=%d seed=%d max_stops=%d | max_routes=%d max_iters=%d | MultiCut only\n",
        string(NS), P, S, SEED, MAX_STOPS, MAX_ROUTES, MAX_ITERS)
flush(stdout)

results = Any[]

for n in NS
    @printf("\n%s\n=== n=%d ===\n", repeat("=", 78), n)
    flush(stdout)
    try
        problem, k, _meta = benchmark_problem(@__DIR__, "SC", n, P, S, SEED)
        n_sets = binomial(n, k)
        formulation = AggregateODRouteJointRoutingAssignmentFormulation(
            ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)
        @printf("k=%d | C(%d,%d)=%d station sets\n", k, n, k, n_sets)
        flush(stdout)

        # ---- Benders, with the per-iteration trace ----
        trace = Any[]
        solver = BendersSolver(
            config=SolverOptions(silent=true, time_limit_sec=600.0),
            max_iterations=MAX_ITERS,
            subproblem=BendersSubproblemConfig(
                max_stops=MAX_STOPS, max_routes=MAX_ROUTES,
                enumeration_time_limit_sec=ENUM_LIMIT),
            total_time_limit_sec=5400.0,
            iteration_callback=row -> push!(trace, row))
        t0 = time()
        bd = run_opt(problem, formulation, solver)
        benders_sec = time() - t0
        bmd = bd.metadata
        benders = something(bd.objective_value, NaN)
        @printf("Benders: %s obj %.6f | iters %d cuts %d | pool %d (enum %.1fs) | wall %.1fs\n",
                bd.termination_status, benders, bmd["benders_iterations"],
                bmd["benders_cuts_added"], bmd["benders_enumerated_columns"],
                bmd["benders_enumeration_sec"], benders_sec)
        @printf("  stop %s | LB %.6f | gap %.3e | scope %s\n",
                bmd["benders_stop_reason"], bmd["benders_lower_bound"], bmd["benders_gap"],
                bmd["benders_optimality_scope"])
        println("  iter |          LB |          UB | this-iter Q |  cuts | built")
        for r in trace
            @printf("  %4d | %11.2f | %11.2f | %11.2f | %2d/%2d | %d\n",
                    r.iteration, r.lower_bound, r.upper_bound, r.incumbent_objective,
                    r.cuts_added, r.cuts_total, r.n_stations_built)
        end
        flush(stdout)

        # ---- monolithic reference over the SAME enumerated pool ----
        data = problem.data
        mapping = create_aggregate_od_route_map(problem, formulation, data)
        columns = StationSelection.enumerate_joint_routing_assignment_columns(
            problem, formulation, data; max_routes=MAX_ROUTES, time_limit_sec=ENUM_LIMIT)
        function solve_reference(; mixed::Bool)
            build = StationSelection._build_joint_routing_assignment_model(
                data, mapping, problem.k, formulation;
                relax_integrality=mixed, initial_columns=columns)
            mm = build.model
            set_silent(mm)
            set_time_limit_sec(mm, 1800.0)
            mixed && JuMP.set_binary.(mm[:y])
            optimize!(mm)
            return (status=termination_status(mm), objective=objective_value(mm))
        end
        mixed_mono = solve_reference(mixed=true)
        direct_mip = solve_reference(mixed=false)
        @printf("monolithic: mixed %s %.6f | direct %s %.6f\n",
                mixed_mono.status, mixed_mono.objective,
                direct_mip.status, direct_mip.objective)
        flush(stdout)

        # ---- CGSolver: independent COLUMN SOURCE ----
        cg = run_opt(problem, formulation,
                     benchmark_cg_solver(900.0; recover_integer_solution=true,
                                         pricing=CGPricingConfig(mode=:exact)))
        cgmd = cg.metadata
        cg_ip = something(cg.objective_value, NaN)
        cg_lp = get(cgmd, "cg_lp_objective_value", NaN)
        cg_ok = get(cgmd, "cg_converged", false) === true &&
                get(cgmd, "cg_optimality_scope", "") == "full_route_universe"
        @printf("CG: %s LP %.6f IP %.6f | converged %s scope %s iters %s\n",
                cg.termination_status, cg_lp, cg_ip, get(cgmd, "cg_converged", false),
                get(cgmd, "cg_optimality_scope", "?"), string(get(cgmd, "cg_iterations", "?")))
        flush(stdout)

        lbs = Float64[r.lower_bound for r in trace]
        push!(results, (
            n=n, k=k, n_sets=n_sets, ok=true,
            benders=benders, iters=bmd["benders_iterations"], cuts=bmd["benders_cuts_added"],
            pool=bmd["benders_enumerated_columns"], enum_sec=bmd["benders_enumeration_sec"],
            wall=benders_sec, mixed=mixed_mono, direct=direct_mip,
            cg_lp=cg_lp, cg_ip=cg_ip, cg_ok=cg_ok,
            lb_monotone=all(lbs[i] <= lbs[i+1] + 1e-9 for i in 1:length(lbs)-1),
            lb_never_above=all(lb <= benders + 1e-6 for lb in lbs),
            trace=trace, error=nothing))
    catch err
        msg = sprint(showerror, err)
        @printf("!! n=%d FAILED: %s\n", n, first(msg, 300))
        flush(stdout)
        push!(results, (n=n, ok=false, error=msg))
    end
end

# ---------------------------------------------------------------- summary
println("\n", repeat("=", 78))
println("=== summary (s=$S, max_stops=$MAX_STOPS, MultiCut) ===")
@printf("%4s %4s %9s %8s %6s %6s %12s %9s\n",
        "n", "k", "C(n,k)", "pool", "iters", "cuts", "objective", "enum_s")
for r in results
    if r.ok
        @printf("%4d %4d %9d %8d %6d %6d %12.2f %9.1f\n",
                r.n, r.k, r.n_sets, r.pool, r.iters, r.cuts, r.benders, r.enum_sec)
    else
        @printf("%4d %4s %9s %8s %6s %6s %12s %9s   FAILED\n",
                r.n, "-", "-", "-", "-", "-", "-", "-")
    end
end

# ---------------------------------------------------------------- checks
println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
push_check!(name, ok, detail) = push!(checks, (name, ok, detail))
for r in results
    r.ok || (push_check!("n=$(r.n): completed", false, first(r.error, 120)); continue)
    push_check!("n=$(r.n): benders == mixed_mono",
        isapprox(r.benders, r.mixed.objective; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f (diff %.3e)", r.benders, r.mixed.objective,
                 r.benders - r.mixed.objective))
    push_check!("n=$(r.n): references are optima",
        string(r.mixed.status) == "OPTIMAL" && string(r.direct.status) == "OPTIMAL",
        "mixed $(r.mixed.status) / direct $(r.direct.status)")
    # No reference needed for these two: an invalid (too strong) cut raises the master's
    # lower bound above the true optimum, and a valid one can never lower it.
    push_check!("n=$(r.n): LB monotone", r.lb_monotone, "over $(length(r.trace)) iterations")
    push_check!("n=$(r.n): LB never above optimum", r.lb_never_above,
        @sprintf("max LB %.6f vs obj %.6f",
                 maximum(x.lower_bound for x in r.trace; init=-Inf), r.benders))
    if r.cg_ok
        push_check!("n=$(r.n): cg_lp <= benders", r.cg_lp <= r.benders + TOL,
            @sprintf("%.6f <= %.6f (slack %.3e)", r.cg_lp, r.benders, r.benders - r.cg_lp))
        push_check!("n=$(r.n): cg_ip == direct_mip (pools agree)",
            isapprox(r.cg_ip, r.direct.objective; rtol=1e-6, atol=1e-6),
            @sprintf("%.6f vs %.6f", r.cg_ip, r.direct.objective))
    else
        # Not a pass: an unconverged CG bounds nothing, so its comparison is unavailable
        # rather than satisfied. Reported so it cannot look like a silent success.
        push_check!("n=$(r.n): CG converged (else no cross-check)", false,
            "CG did not converge at full-universe scope -- cross-check unavailable")
    end
end
for (name, ok, detail) in checks
    @printf("%-44s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)

# The cut-count verdict, stated rather than left to the reader.
good = [r for r in results if r.ok]
if length(good) >= 2
    println()
    lo, hi = good[1], good[end]
    space_growth = hi.n_sets / lo.n_sets
    cut_growth = hi.cuts / max(1, lo.cuts)
    @printf("CUT COUNT: first-stage space grew %.0fx (n=%d->%d), cuts grew %.1fx (%d->%d).\n",
            space_growth, lo.n, hi.n, cut_growth, lo.cuts, hi.cuts)
    if cut_growth < 0.1 * space_growth
        println("  => cuts are near-flat in the size of the first-stage space. Consistent with")
        println("     the cut constraining EVERY station (Gamma_j is the shadow price of the")
        println("     linking row and is binding at y_j = 0), not one vertex at a time.")
    else
        println("  => cuts scale with the space, so the low count at n=10 was smallness and")
        println("     there was nothing to explain.")
    end
end
n_fail == 0 || error("verification failed")
println("\nALL CHECKS PASSED")
