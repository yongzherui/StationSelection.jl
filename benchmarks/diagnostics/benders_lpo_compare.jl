"""Does the locally Pareto-optimal completion fix the activated oracle's cut blowup?

The activated oracle prices only over built stations and repairs the duals on the unbuilt
ones with a closed-form bound. That makes pricing nearly free but the cuts very weak, and
the weakness compounds:

    n=10   enumeration 4 it /  5 cuts | plain CG 4 it /  5 cuts | activated  30 it /   77 cuts (0.3 s)
    n=20   enumeration 7 it /  9 cuts | plain CG 9 it / 13 cuts | activated 588+ it / 1764+ cuts,
                                                                  25% gap still open after 1112 s

So the activated oracle as-is does not scale. `:column_generation_activated_lpo` keeps the
cheap pricing and replaces the CUT's completion with the strongest one that still passes
separation, chosen against an interior point in the Magnanti-Wong sense
(`benders/completion_lpo.jl`).

# What this measures

All requested oracles on the SAME instance, so the comparison is paired. Reported per oracle:
objective (a correctness gate -- every oracle must return the identical optimum), Benders
iterations, cuts, wall, and for the LPO oracle its own accounting: how many completion LPs
ran, how many rows they carried, how many solved to optimality, and the core-point-weighted
gain over the closed-form bound.

The completion does NO pricing: the activated solve's exhaustion certificate discharges every
route-tied row, leaving a route-free family written out up front. So `wall` for the LPO arm
should stay close to the plain activated arm's -- a large gap would mean the completion LP
itself became the bottleneck.

The headline number is `iters` and `cuts` for `activated_lpo` against `activated`. The gain
has to be large -- roughly the 15x seen at n=10 -- for the activated family to be viable at
all, because its whole appeal was trading cut quality for free pricing and at n=20 that trade
was catastrophically bad.

`core_slack` is worth watching too: it is the max-min slack of the interior point, and a
value of 0 means some face of the master's feasible region is structurally tight, which
weakens the Pareto claim on that face.

Usage: sbatch benchmarks/diagnostics/run_benders_lpo.sh
Env: LP_N LP_S LP_P LP_SEED LP_MAX_STOPS LP_ORACLES LP_SUB_MODE LP_SUB_K
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "LP_N", "10"))
const S = parse(Int, get(ENV, "LP_S", "3"))
const P = parse(Int, get(ENV, "LP_P", "8"))
const SEED = parse(Int, get(ENV, "LP_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "LP_MAX_STOPS", "4"))
const ORACLES = [Symbol(strip(x)) for x in split(get(ENV, "LP_ORACLES",
    "direct_enumeration,column_generation,column_generation_activated," *
    "column_generation_activated_lpo,column_generation_activated_warm_start"),
    ',') if !isempty(strip(x))]
const SUB_MODE = Symbol(get(ENV, "LP_SUB_MODE", "exact"))
const SUB_K = parse(Int, get(ENV, "LP_SUB_K", "0"))
const LPO_COMPLETION = Symbol(get(ENV, "LP_LPO_COMPLETION", "separation"))
const MAX_ROUTES = 20_000_000
const ENUM_LIMIT = 3600.0
# Generous but finite: the point is to SEE a blowup, not to sit through one. An oracle that
# hits this cap is reported as non-converged rather than silently truncated.
const MAX_ITERS = parse(Int, get(ENV, "LP_MAX_ITERS", "3000"))
const TOTAL_LIMIT = parse(Float64, get(ENV, "LP_TOTAL_LIMIT", "1800.0"))
const TOL = 1e-6

@printf("n=%d s=%d p=%d seed=%d max_stops=%d | pricer %s%s\n",
        N, S, P, SEED, MAX_STOPS, SUB_MODE, SUB_K > 0 ? " (K=$SUB_K)" : "")
@printf("lpo_completion=%s\n", LPO_COMPLETION)
@printf("oracles: %s\n", join(ORACLES, ", "))
flush(stdout)

problem, k, _ = benchmark_problem(@__DIR__, "LP", N, P, S, SEED)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)
@printf("k=%d\n", k)

function run_oracle(oracle)
    is_enum = oracle === :direct_enumeration
    pricing = is_enum ? CGPricingConfig() :
        (SUB_K > 0 ? CGPricingConfig(mode=SUB_MODE, relaxed_cluster_count=SUB_K) :
                     CGPricingConfig(mode=SUB_MODE))
    solver = BendersSolver(
        config=SolverOptions(silent=true, time_limit_sec=600.0),
        max_iterations=MAX_ITERS,
        subproblem=BendersSubproblemConfig(
            oracle=oracle,
            max_stops=is_enum ? MAX_STOPS : nothing,
            max_routes=MAX_ROUTES, enumeration_time_limit_sec=ENUM_LIMIT,
            pricing=pricing,
            lpo_completion=LPO_COMPLETION,
            cg_pricing_time_limit_sec=600.0, max_cg_iterations=500),
        total_time_limit_sec=TOTAL_LIMIT)
    t0 = time()
    r = run_opt(problem, formulation, solver)
    md = r.metadata
    return (oracle=oracle, status=string(r.termination_status),
            obj=something(r.objective_value, NaN),
            iters=md["benders_iterations"], cuts=md["benders_cuts_added"],
            lb=md["benders_lower_bound"], gap=md["benders_gap"],
            stop=md["benders_stop_reason"], wall=time() - t0,
            price_sec=get(md, "benders_cg_pricing_sec", 0.0),
            pool=get(md, "benders_cg_pool_final", 0),
            lpo_calls=get(md, "benders_cg_lpo_calls", 0),
            lpo_rows=get(md, "benders_cg_lpo_rows", 0),
            lpo_optimal=get(md, "benders_cg_lpo_optimal", 0),
            lpo_gain=get(md, "benders_cg_lpo_improved", 0.0),
            lpo_rounds=get(md, "benders_cg_lpo_rounds", 0),
            lpo_price=get(md, "benders_cg_lpo_pricing_sec", 0.0),
            core_slack=get(md, "benders_core_slack", NaN))
end

rows = Any[]
for oracle in ORACLES
    @printf("\n%s\n=== %s ===\n", repeat("=", 76), oracle)
    flush(stdout)
    try
        r = run_oracle(oracle)
        @printf("%s obj %.6f | iters %d cuts %d | LB %.2f gap %.3e | stop %s | wall %.1fs\n",
                r.status, r.obj, r.iters, r.cuts, r.lb, r.gap, r.stop, r.wall)
        r.pool > 0 && @printf("  pricing %.1fs | pool %d\n", r.price_sec, r.pool)
        if r.lpo_calls > 0
            @printf("  LPO: %d completions, %d LP rounds, %d rows, %d optimal (%.0f%%) | separation pricing %.1fs | gain %.3f | core slack %.4f\n",
                    r.lpo_calls, r.lpo_rounds, r.lpo_rows, r.lpo_optimal,
                    100 * r.lpo_optimal / max(1, r.lpo_calls), r.lpo_price, r.lpo_gain,
                    r.core_slack)
        end
        flush(stdout)
        push!(rows, r)
    catch err
        msg = sprint(showerror, err)
        @printf("!! %s FAILED: %s\n", oracle, first(msg, 400))
        flush(stdout)
        push!(rows, (oracle=oracle, status="ERROR", obj=NaN, iters=-1, cuts=-1,
                     lb=NaN, gap=NaN, stop=first(msg, 80), wall=NaN, price_sec=0.0,
                     pool=0, lpo_calls=0, lpo_rows=0, lpo_optimal=0,
                     lpo_rounds=0, lpo_price=0.0,
                     lpo_gain=0.0, core_slack=NaN))
    end
end

println("\n", repeat("=", 76))
@printf("%-34s %9s %6s %7s %9s %10s\n", "oracle", "status", "iters", "cuts", "wall", "objective")
for r in rows
    @printf("%-34s %9s %6d %7d %9.1f %10.2f\n",
            r.oracle, r.status, r.iters, r.cuts, r.wall, r.obj)
end

println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
ok_rows = [r for r in rows if r.status == "OPTIMAL"]
ref = isempty(ok_rows) ? nothing : ok_rows[1]
for r in rows
    push!(checks, ("$(r.oracle): converged OPTIMAL", r.status == "OPTIMAL",
                   "$(r.status) / $(r.stop)"))
    if !isnothing(ref) && r.status == "OPTIMAL"
        # Every oracle solves the identical model, so any disagreement means a wrong cut --
        # and for the two activated variants specifically it means the dual completion is
        # unsound, which is the failure this must catch before any cut-count claim is made.
        push!(checks, ("$(r.oracle): objective == $(ref.oracle)",
            isapprox(r.obj, ref.obj; rtol=1e-6, atol=1e-6),
            @sprintf("%.6f vs %.6f (diff %.3e)", r.obj, ref.obj, r.obj - ref.obj)))
    end
end
for (name, ok, detail) in checks
    @printf("%-52s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)

# The verdict the run exists for.
byname = Dict(r.oracle => r for r in rows)
if haskey(byname, :column_generation_activated) &&
   haskey(byname, :column_generation_activated_lpo)
    a, l = byname[:column_generation_activated], byname[:column_generation_activated_lpo]
    println("\nLPO VERDICT")
    @printf("  activated      %6d iters %7d cuts %9.1fs\n", a.iters, a.cuts, a.wall)
    @printf("  activated_lpo  %6d iters %7d cuts %9.1fs\n", l.iters, l.cuts, l.wall)
    if a.cuts > 0 && l.cuts > 0
        @printf("  => cuts %.2fx, iterations %.2fx, wall %.2fx\n",
                l.cuts / a.cuts, l.iters / a.iters, l.wall / a.wall)
    end
    if haskey(byname, :column_generation)
        c = byname[:column_generation]
        @printf("  reference: plain CG %d iters / %d cuts; LPO is %.1fx its cut count\n",
                c.iters, c.cuts, c.cuts == 0 ? NaN : l.cuts / c.cuts)
    end
end

# Phase 1 of the activated family already attains the EXACT Q_s(yhat) -- a column touching an
# unbuilt station is pinned to theta=0 by its own linking row. So every second of full-station
# pricing under plain CG buys DUALS, not value. This is the null hypothesis: does warm-starting
# the pool make that dual-only grind cheaper, with no completion theory at all?
if haskey(byname, :column_generation) &&
   haskey(byname, :column_generation_activated_warm_start)
    c, w = byname[:column_generation], byname[:column_generation_activated_warm_start]
    println("\nWARM-START VERDICT (does a built-only phase 1 cheapen full-universe certification?)")
    @printf("  plain CG    %6d iters %7d cuts %9.1fs wall %9.1fs pricing, pool %d\n",
            c.iters, c.cuts, c.wall, c.price_sec, c.pool)
    @printf("  warm start  %6d iters %7d cuts %9.1fs wall %9.1fs pricing, pool %d\n",
            w.iters, w.cuts, w.wall, w.price_sec, w.pool)
    if c.price_sec > 0
        @printf("  => pricing %.2fx, wall %.2fx, cuts %+d\n",
                w.price_sec / c.price_sec, w.wall / max(1e-9, c.wall), w.cuts - c.cuts)
    end
end
n_fail == 0 || error("verification failed")
println("\nALL CHECKS PASSED")
