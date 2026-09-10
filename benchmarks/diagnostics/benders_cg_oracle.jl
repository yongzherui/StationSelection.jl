"""Does the `:column_generation` subproblem oracle give the same answer as `:direct_enumeration`?

The Benders subproblem can now be solved two ways: over a fully enumerated column pool
(`:direct_enumeration`) or by pricing columns on demand (`:column_generation`). They solve
the IDENTICAL model, so the decisive test is that they return the identical objective.

# Why this is not merely a regression check

Cut validity requires the duals to be feasible for the FULL-universe dual. Enumeration gets
that by containing the universe; CG gets it only by proving nothing improving remains. So a
CG run that agrees with enumeration is evidence the exhaustion contract is actually being
honoured -- and a CG run that comes out HIGHER is the signature of the failure mode, since
cuts from a non-exhausted round over-estimate the second-stage cost and prune good `y`.
(`solve_subproblem` is supposed to raise rather than let that happen; this checks it does.)

# Arms

- `parity` (n=10/15/20, max_stops=4): both oracles, plus the shipped `DirectMIPSolver` and
  `CGSolver` references. All four must agree. This is the arm that can fail informatively.
- `baseline_ms` (n=10/15, max_stops=10): `:column_generation` only, because enumeration is not
  tractable there -- that is the entire point of the oracle. Cross-checked against
  `CGSolver`'s own optimum on the same mixed model rather than against enumeration.

  NAMED for what it is. An earlier version called this arm `unbounded`, which asserts
  something false: `max_stops=10` is `BENCHMARK_BASELINE`'s value and still a finite cap.
  What is uncapped is the ORACLE -- `:direct_enumeration` has to force `max_stops=4` because
  its pool is exponential, while `:column_generation` lets the formulation's own value stand.
  Truly unbounded would be `max_stops=nothing`; not tested here.

`max_stops=10` is `BENCHMARK_BASELINE`'s own value, so the `baseline_ms` arm is the first
Benders result on this project's actual baseline formulation rather than a capped stand-in.

# Reported

Objective agreement, plus the CG-specific counters that say whether pool accumulation across
Benders iterations is paying off: total inner CG iterations, rounds (Benders iterations x
scenarios), columns priced, final pool size, and the pricing/LP time split. The bet is that
iteration 1 dominates and later Benders iterations exhaust in one or two rounds; if
`iterations / rounds` stays near the cap instead, the warm start is not working.

Usage: sbatch benchmarks/diagnostics/run_benders_cg_oracle.sh
Env: OR_NS OR_P OR_S OR_SEED OR_CAPPED_MS OR_FULL_MS OR_FULL_NS
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

# Empty is meaningful: it selects NO cells of that arm, which is how one array task runs a
# single (arm, n) cell. See run_benders_cg_oracle_array.sh for why the cell -- rather than
# the individual solve -- is the unit of parallelism here.
_sizes(key, default) = [parse(Int, x) for x in split(get(ENV, key, default), ',') if !isempty(strip(x))]
const NS = _sizes("OR_NS", "10,15,20")
const FULL_NS = _sizes("OR_FULL_NS", "10,15")
const P = parse(Int, get(ENV, "OR_P", "8"))
const S = parse(Int, get(ENV, "OR_S", "3"))
const SEED = parse(Int, get(ENV, "OR_SEED", "42"))
const CAPPED_MS = parse(Int, get(ENV, "OR_CAPPED_MS", "4"))
const FULL_MS = parse(Int, get(ENV, "OR_FULL_MS", "10"))
# Whether an unavailable CGSolver cross-check FAILS the run. Strict by default, because a
# silently-skipped reference is how an unverified number gets mistaken for a verified one.
# Relaxed deliberately at large n: the measured CG frontier is n<=20 all scenarios, n=25 to
# <=5, n=30 only s=1, so at n=30/s=3 the CG master is EXPECTED not to converge and its
# absence says nothing about Benders. There the correctness evidence is the internal
# invariants (LB monotone, LB <= objective, cut audit) plus verification at n<=20.
const REQUIRE_CG_REF = get(ENV, "OR_REQUIRE_CG_REF", "1") == "1"
# Subproblem pricer. `:exact` exhausts by searching the whole route universe; a
# relaxed-cluster mode exhausts by CERTIFYING a relaxation that lower-bounds every real
# route, which is the mode to use past the sizes where the exact search still exhausts
# (measured CG frontier: n<=20 all scenarios, n=25 to <=5, n=30 only s=1). A count is
# required with those modes and inert without them.
const SUB_MODE = Symbol(get(ENV, "OR_SUB_MODE", "exact"))
const SUB_K = parse(Int, get(ENV, "OR_SUB_K", "0"))
const MAX_ROUTES = 20_000_000
const ENUM_LIMIT = 3600.0
const TOL = 1e-6

_formulation(ms) = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=ms)

@printf("subproblem pricer: %s%s\n", SUB_MODE,
        SUB_K > 0 ? " (relaxed_cluster_count=$(SUB_K))" : "")
flush(stdout)

"""Run one Benders solve, streaming progress as it goes.

Long runs MUST report intermediate state. A cell at large `n` can spend a per-round pricing
budget times scenarios times Benders iterations before returning anything, and a silent run
gives no way to distinguish "still pricing productively" from "stuck" -- nor to salvage
partial information if the scheduler's wall arrives first. So this prints a line per Benders
iteration (bounds, gap, cuts, and per-scenario inner-CG counts) and sets `verbose=true` so
the inner CG prints a line per pricing round underneath.
"""
function _benders(problem, formulation, oracle; max_stops=nothing)
    t_start = time()
    function on_iteration(row)
        @printf("    [benders it=%d] LB %.2f UB %.2f gap %.3e | cuts %d/%d | built %d | %.1fs\n",
                row.iteration, row.lower_bound, row.upper_bound, row.gap,
                row.cuts_added, row.cuts_total, row.n_stations_built, time() - t_start)
        sub = row.subproblem
        if !isnothing(sub) && hasproperty(sub, :scenarios)
            for sr in sub.scenarios
                cg = hasproperty(sr, :cg) ? sr.cg : nothing
                if isnothing(cg)
                    @printf("      s=%d Q %.2f\n", sr.scenario, sr.objective)
                else
                    @printf("      s=%d Q %.2f | cg %d it %s | +%d cols | price %.1fs\n",
                            sr.scenario, sr.objective, cg.cg_iterations, cg.stop_reason,
                            cg.columns_added, cg.pricing_sec)
                end
            end
        end
        flush(stdout)
    end
    solver = BendersSolver(
        config=SolverOptions(silent=true, time_limit_sec=600.0),
        max_iterations=2000,
        subproblem=BendersSubproblemConfig(
            oracle=oracle,
            max_stops=oracle === :direct_enumeration ? something(max_stops, CAPPED_MS) : nothing,
            max_routes=MAX_ROUTES, enumeration_time_limit_sec=ENUM_LIMIT,
            pricing=(oracle !== :direct_enumeration ?
                     (SUB_K > 0 ? CGPricingConfig(mode=SUB_MODE, relaxed_cluster_count=SUB_K) :
                                  CGPricingConfig(mode=SUB_MODE)) :
                     CGPricingConfig()),
            cg_pricing_time_limit_sec=600.0, max_cg_iterations=500,
            verbose=(oracle !== :direct_enumeration)),
        total_time_limit_sec=5400.0,
        iteration_callback=on_iteration)
    r = run_opt(problem, formulation, solver)
    return (result=r, wall=time() - t_start)
end

_cgkeys(md) = (
    iters = get(md, "benders_cg_iterations", 0),
    rounds = get(md, "benders_cg_rounds", 0),
    cols = get(md, "benders_cg_columns_added", 0),
    pool = get(md, "benders_cg_pool_final", 0),
    psec = get(md, "benders_cg_pricing_sec", 0.0),
    lsec = get(md, "benders_cg_lp_sec", 0.0),
)

rows = Any[]

# ------------------------------------------------------------------ parity arm
for n in NS
    @printf("\n%s\n=== parity n=%d max_stops=%d ===\n", repeat("=", 74), n, CAPPED_MS)
    flush(stdout)
    try
        problem, k, _ = benchmark_problem(@__DIR__, "OR", n, P, S, SEED)
        formulation = _formulation(CAPPED_MS)

        enum = _benders(problem, formulation, :direct_enumeration)
        @printf("enumeration : %s %.6f | iters %d cuts %d | %.1fs\n",
                enum.result.termination_status, something(enum.result.objective_value, NaN),
                enum.result.metadata["benders_iterations"],
                enum.result.metadata["benders_cuts_added"], enum.wall)
        flush(stdout)

        cg = _benders(problem, formulation, :column_generation)
        c = _cgkeys(cg.result.metadata)
        @printf("column gen  : %s %.6f | iters %d cuts %d | %.1fs\n",
                cg.result.termination_status, something(cg.result.objective_value, NaN),
                cg.result.metadata["benders_iterations"],
                cg.result.metadata["benders_cuts_added"], cg.wall)
        @printf("              CG: %d iterations over %d rounds (%.1f/round) | %d cols priced | pool %d | price %.1fs lp %.1fs\n",
                c.iters, c.rounds, c.rounds == 0 ? NaN : c.iters / c.rounds,
                c.cols, c.pool, c.psec, c.lsec)
        flush(stdout)

        act = _benders(problem, formulation, :column_generation_activated)
        a = _cgkeys(act.result.metadata)
        @printf("activated   : %s %.6f | iters %d cuts %d | %.1fs\n",
                act.result.termination_status, something(act.result.objective_value, NaN),
                act.result.metadata["benders_iterations"],
                act.result.metadata["benders_cuts_added"], act.wall)
        @printf("              CG: %d iterations over %d rounds (%.1f/round) | %d cols priced | pool %d | price %.1fs\n",
                a.iters, a.rounds, a.rounds == 0 ? NaN : a.iters / a.rounds,
                a.cols, a.pool, a.psec)
        flush(stdout)

        # shipped references over the same enumerated universe
        dsolver = DirectMIPSolver(config=SolverOptions(silent=true, time_limit_sec=1800.0))
        dbuild = build_model(problem, formulation, dsolver;
                             max_routes=MAX_ROUTES, time_limit_sec=ENUM_LIMIT)
        dres = StationSelection.optimize_model(dbuild, dsolver)
        cgm = run_opt(problem, formulation,
                      benchmark_cg_solver(900.0; recover_integer_solution=true,
                                          pricing=CGPricingConfig(mode=:exact)))
        @printf("references  : DirectMIP %.6f | CGSolver lp %.6f ip %.6f\n",
                something(dres.objective_value, NaN),
                get(cgm.metadata, "cg_lp_objective_value", NaN),
                something(cgm.objective_value, NaN))
        flush(stdout)

        push!(rows, (arm="parity", n=n, ok=true,
                     enum=something(enum.result.objective_value, NaN),
                     cg=something(cg.result.objective_value, NaN),
                     act=something(act.result.objective_value, NaN),
                     act_cuts=act.result.metadata["benders_cuts_added"],
                     cg_cuts=cg.result.metadata["benders_cuts_added"],
                     act_wall=act.wall, act_stats=a,
                     direct=something(dres.objective_value, NaN),
                     cg_lp=get(cgm.metadata, "cg_lp_objective_value", NaN),
                     enum_wall=enum.wall, cg_wall=cg.wall,
                     enum_cuts=enum.result.metadata["benders_cuts_added"],
                     enum_scope=enum.result.metadata["benders_optimality_scope"],
                     cg_scope=cg.result.metadata["benders_optimality_scope"],
                     stats=c, error=nothing))
    catch err
        msg = sprint(showerror, err)
        @printf("!! parity n=%d FAILED: %s\n", n, first(msg, 400))
        flush(stdout)
        push!(rows, (arm="parity", n=n, ok=false, error=msg))
    end
end

# --------------------------------------------------- baseline_ms max_stops arm
for n in FULL_NS
    @printf("\n%s\n=== baseline_ms n=%d max_stops=%d (CG oracle only) ===\n",
            repeat("=", 74), n, FULL_MS)
    flush(stdout)
    try
        problem, k, _ = benchmark_problem(@__DIR__, "OR", n, P, S, SEED)
        formulation = _formulation(FULL_MS)
        cg = _benders(problem, formulation, :column_generation)
        c = _cgkeys(cg.result.metadata)
        cgm = run_opt(problem, formulation,
                      benchmark_cg_solver(900.0; recover_integer_solution=true,
                                          pricing=CGPricingConfig(mode=:exact)))
        cg_lp = get(cgm.metadata, "cg_lp_objective_value", NaN)
        cg_ok = get(cgm.metadata, "cg_converged", false) === true &&
                get(cgm.metadata, "cg_optimality_scope", "") == "full_route_universe"
        @printf("column gen  : %s %.6f | iters %d cuts %d | %.1fs | scope %s\n",
                cg.result.termination_status, something(cg.result.objective_value, NaN),
                cg.result.metadata["benders_iterations"],
                cg.result.metadata["benders_cuts_added"], cg.wall,
                cg.result.metadata["benders_optimality_scope"])
        @printf("              CG: %d iterations over %d rounds | %d cols | pool %d | price %.1fs\n",
                c.iters, c.rounds, c.cols, c.pool, c.psec)
        @printf("CGSolver    : lp %.6f (converged %s, full universe %s)\n",
                cg_lp, get(cgm.metadata, "cg_converged", false), cg_ok)
        flush(stdout)
        push!(rows, (arm="baseline_ms", n=n, ok=true,
                     enum=NaN, cg=something(cg.result.objective_value, NaN),
                     direct=NaN, cg_lp=cg_ok ? cg_lp : NaN,
                     enum_wall=NaN, cg_wall=cg.wall,
                     enum_scope="-", cg_scope=cg.result.metadata["benders_optimality_scope"],
                     stats=c, error=nothing))
    catch err
        msg = sprint(showerror, err)
        @printf("!! baseline_ms n=%d FAILED: %s\n", n, first(msg, 400))
        flush(stdout)
        push!(rows, (arm="baseline_ms", n=n, ok=false, error=msg))
    end
end

# ---------------------------------------------------------------- summary
println("\n", repeat("=", 74))
@printf("%-10s %4s %14s %14s %14s\n",
        "arm", "n", "enumeration", "column gen", "activated")
for r in rows
    r.ok || (@printf("%-10s %4d  FAILED\n", r.arm, r.n); continue)
    if r.arm == "parity"
        @printf("%-10s %4d %14.4f %14.4f %14.4f | cuts %d/%d/%d | wall %.1f/%.1f/%.1f\n",
                r.arm, r.n, r.enum, r.cg, r.act,
                r.enum_cuts, r.cg_cuts, r.act_cuts,
                r.enum_wall, r.cg_wall, r.act_wall)
    else
        @printf("%-10s %4d %14.4f %14.4f %9.1f %9.1f %8d\n",
                r.arm, r.n, r.enum, r.cg, r.enum_wall, r.cg_wall, r.stats.pool)
    end
end

println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
for r in rows
    r.ok || (push!(checks, ("$(r.arm) n=$(r.n): completed", false, first(r.error, 140))); continue)
    if r.arm == "parity"
        # THE check: two oracles, one model.
        push!(checks, ("parity n=$(r.n): CG == enumeration",
            isapprox(r.cg, r.enum; rtol=1e-6, atol=1e-6),
            @sprintf("%.6f vs %.6f (diff %.3e)", r.cg, r.enum, r.cg - r.enum)))
        # Directional: a non-exhausted CG round yields cuts that over-estimate Q and prune
        # good y, so the CG arm coming out HIGHER is the specific signature of that bug.
        push!(checks, ("parity n=$(r.n): CG not above enumeration",
            r.cg <= r.enum + TOL, @sprintf("%.6f <= %.6f", r.cg, r.enum)))
        push!(checks, ("parity n=$(r.n): CG == DirectMIPSolver",
            isapprox(r.cg, r.direct; rtol=1e-6, atol=1e-6),
            @sprintf("%.6f vs %.6f", r.cg, r.direct)))
        # The activated oracle prices over the built stations only and repairs the duals
        # with a closed-form completion. If that completion is wrong the cut can prune the
        # optimum, which shows up HERE as a higher objective -- so this equality is the
        # completion's correctness test, and the directional check names the failure mode.
        push!(checks, ("parity n=$(r.n): activated == enumeration",
            isapprox(r.act, r.enum; rtol=1e-6, atol=1e-6),
            @sprintf("%.6f vs %.6f (diff %.3e)", r.act, r.enum, r.act - r.enum)))
        push!(checks, ("parity n=$(r.n): activated not above enumeration",
            r.act <= r.enum + TOL, @sprintf("%.6f <= %.6f", r.act, r.enum)))
        push!(checks, ("parity n=$(r.n): both scopes full universe",
            r.enum_scope == "full_route_universe" && r.cg_scope == "full_route_universe",
            "enum $(r.enum_scope) / cg $(r.cg_scope)"))
    else
        push!(checks, ("baseline_ms n=$(r.n): scope full universe",
            r.cg_scope == "full_route_universe", r.cg_scope))
        if !isnan(r.cg_lp)
            push!(checks, ("baseline_ms n=$(r.n): cg_lp <= benders",
                r.cg_lp <= r.cg + TOL,
                @sprintf("%.6f <= %.6f (slack %.3e)", r.cg_lp, r.cg, r.cg - r.cg_lp)))
        elseif REQUIRE_CG_REF
            push!(checks, ("baseline_ms n=$(r.n): CGSolver reference available", false,
                "CGSolver did not converge at full-universe scope"))
        else
            println("NOTE n=$(r.n): CGSolver cross-check UNAVAILABLE (did not converge " *
                    "at full-universe scope). Expected past the CG frontier; correctness " *
                    "here rests on the internal invariants and on verification at n<=20.")
        end
    end
end
for (name, ok, detail) in checks
    @printf("%-44s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)
n_fail == 0 || error("verification failed")
println("ALL CHECKS PASSED")
