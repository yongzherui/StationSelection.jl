"""Does the CG subproblem oracle need MORE Benders cuts than enumeration? Paired, per seed.

This is the question that decides whether `:column_generation` is worth having. Both oracles
solve the identical model and, at any given `yhat`, return the identical `Q_s(yhat)` -- CG's
exhaustion certificate guarantees the restricted optimum equals the full one. So both cuts
are TIGHT at their anchor. But an LP's optimal dual is generally not unique, and the cut is
the supporting hyperplane of whichever optimal dual vertex the solver lands on: enumeration
picks a vertex of the FULL dual, CG picks a vertex of the RESTRICTED dual that happens to lie
in the full one. Different subgradients, identical at `yhat`, different elsewhere. Neither
dominates theoretically, hence this measurement.

The directional suspicion worth testing: in the restricted dual, `gamma_j` for a station
barely represented in the pool faces less upward pressure and can sit at zero, giving a
SPARSER cut. Sparser cuts constrain stations they were not derived at less, so the plausible
failure shape is "CG cuts are weaker away from the anchor, so CG needs more of them".

# Design: paired, and deterministic

**Paired.** Both oracles run on the SAME instance for each seed, so instance-to-instance
variation is differenced out. The statistic is the per-seed difference
`cuts_cg - cuts_enum`, not two independent means.

**Deterministic master.** Cut counts were previously measured at 12 vs 9 on IDENTICAL n=20
configuration across two runs, because the master is a MIP with many tied optima (21 of 86
station sets tie at n=10) and which one Gurobi returns shifts with threading. That noise band
is as large as any oracle effect we would care about. So the master here runs with
`threads=1`, which makes Gurobi's MIP deterministic and the cut count reproducible; the
`reproducibility` check below re-solves one seed and asserts the count is identical, so the
premise is verified rather than assumed.

Without that, this comparison could not distinguish an oracle effect from tie-break noise --
and reporting a 2-cut difference as meaningful would have been wrong.

# What is reported per seed

Cuts and iterations under each oracle, the paired difference, the objective agreement (a
correctness gate -- a difference here invalidates the cut comparison entirely), and the CG
counters. `max_stops=4` throughout, because enumeration has to exist to be compared against.

# The effectiveness metric is NOT cuts alone

Each Benders iteration under CG costs a full CG solve per scenario instead of one LP, and the
enumeration cost moves from up-front to per-iteration. So the run also reports wall time
split, and the honest read is `cuts x cost-per-iteration`. At n=20/s=3 enumeration is ~12 s
once with a ~2 s loop, so CG has no room to win at `max_stops=4` even at equal cut count --
the oracle earns its keep only where enumeration does not exist (`max_stops=10`, measured
separately by `benders_cg_oracle.jl`'s `unbounded` arm). What THIS script decides is whether
CG's cuts are as GOOD, which is the part that would compound against it at any size.

Usage: sbatch --array=1-10 benchmarks/diagnostics/run_benders_oracle_cut_parity.sh
Three oracles per seed: `:direct_enumeration`, `:column_generation`, and
`:column_generation_activated`. The activated one prices only over the built stations and
repairs its duals with a closed-form completion, so it is expected to need MORE cuts (its
coefficients on unbuilt stations are a bound, not the true shadow value) in exchange for
cheaper pricing. This measures both halves of that trade.

Env: CP_N CP_SEEDS CP_P CP_S CP_MAX_STOPS CP_REPRO
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "CP_N", "20"))
const SEEDS = [parse(Int, x) for x in split(get(ENV, "CP_SEEDS", "42"), ',') if !isempty(strip(x))]
const P = parse(Int, get(ENV, "CP_P", "8"))
const S = parse(Int, get(ENV, "CP_S", "3"))
const MAX_STOPS = parse(Int, get(ENV, "CP_MAX_STOPS", "4"))
# Re-solve the enumeration arm a second time and assert an identical cut count, verifying the
# determinism this comparison rests on rather than assuming it.
const REPRO = get(ENV, "CP_REPRO", "1") == "1"
const MAX_ROUTES = 20_000_000
const ENUM_LIMIT = 3600.0
const TOL = 1e-6

@printf("n=%d p=%d s=%d max_stops=%d | seeds %s | master threads=1 (deterministic)\n",
        N, P, S, MAX_STOPS, string(SEEDS))
flush(stdout)

function run_oracle(problem, formulation, oracle)
    solver = BendersSolver(
        # threads=1 is the experimental control, not a performance choice: it makes the
        # master MIP deterministic so the cut count is reproducible and the paired
        # difference is attributable to the oracle rather than to tie-breaking.
        config=SolverOptions(silent=true, time_limit_sec=600.0, threads=1),
        max_iterations=2000,
        subproblem=BendersSubproblemConfig(
            oracle=oracle,
            max_stops=oracle === :direct_enumeration ? MAX_STOPS : nothing,
            max_routes=MAX_ROUTES, enumeration_time_limit_sec=ENUM_LIMIT,
            cg_pricing_time_limit_sec=600.0, max_cg_iterations=500),
        total_time_limit_sec=5400.0)
    t0 = time()
    r = run_opt(problem, formulation, solver)
    md = r.metadata
    return (objective=something(r.objective_value, NaN),
            iters=md["benders_iterations"], cuts=md["benders_cuts_added"],
            enum_sec=md["benders_enumeration_sec"],
            master_sec=md["benders_master_sec"], sub_sec=md["benders_subproblem_sec"],
            cg_iters=get(md, "benders_cg_iterations", 0),
            cg_rounds=get(md, "benders_cg_rounds", 0),
            cg_pool=get(md, "benders_cg_pool_final", 0),
            cg_price_sec=get(md, "benders_cg_pricing_sec", 0.0),
            wall=time() - t0, status=string(r.termination_status))
end

rows = Any[]
for seed in SEEDS
    @printf("\n%s\n=== n=%d seed=%d ===\n", repeat("=", 74), N, seed)
    flush(stdout)
    try
        problem, k, _ = benchmark_problem(@__DIR__, "CP", N, P, S, seed)
        formulation = AggregateODRouteJointRoutingAssignmentFormulation(
            ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)

        e = run_oracle(problem, formulation, :direct_enumeration)
        @printf("enumeration : %s %.6f | iters %2d cuts %2d | enum %.1fs loop %.1fs | wall %.1fs\n",
                e.status, e.objective, e.iters, e.cuts, e.enum_sec,
                e.master_sec + e.sub_sec, e.wall)
        flush(stdout)

        c = run_oracle(problem, formulation, :column_generation)
        @printf("column gen  : %s %.6f | iters %2d cuts %2d | seed %.1fs loop %.1fs | wall %.1fs\n",
                c.status, c.objective, c.iters, c.cuts, c.enum_sec,
                c.master_sec + c.sub_sec, c.wall)
        @printf("              CG %d iters / %d rounds (%.2f per round) | pool %d | price %.1fs\n",
                c.cg_iters, c.cg_rounds,
                c.cg_rounds == 0 ? NaN : c.cg_iters / c.cg_rounds, c.cg_pool, c.cg_price_sec)
        flush(stdout)

        a = run_oracle(problem, formulation, :column_generation_activated)
        @printf("activated   : %s %.6f | iters %2d cuts %2d | loop %.1fs | wall %.1fs\n",
                a.status, a.objective, a.iters, a.cuts,
                a.master_sec + a.sub_sec, a.wall)
        @printf("              CG %d iters / %d rounds (%.2f per round) | pool %d | price %.1fs\n",
                a.cg_iters, a.cg_rounds,
                a.cg_rounds == 0 ? NaN : a.cg_iters / a.cg_rounds, a.cg_pool, a.cg_price_sec)
        @printf("PAIRED      : cuts cg %+d, activated %+d (enum %d) | iters cg %+d, activated %+d\n",
                c.cuts - e.cuts, a.cuts - e.cuts, e.cuts,
                c.iters - e.iters, a.iters - e.iters)
        flush(stdout)

        repro_cuts = -1
        if REPRO
            e2 = run_oracle(problem, formulation, :direct_enumeration)
            repro_cuts = e2.cuts
            @printf("repro check : enumeration re-solve cuts %d (first run %d)%s\n",
                    e2.cuts, e.cuts, e2.cuts == e.cuts ? "" : "  <-- NON-DETERMINISTIC")
            flush(stdout)
        end

        push!(rows, (seed=seed, ok=true, e=e, c=c, a=a, repro_cuts=repro_cuts, error=nothing))
    catch err
        msg = sprint(showerror, err)
        @printf("!! seed %d FAILED: %s\n", seed, first(msg, 400))
        flush(stdout)
        push!(rows, (seed=seed, ok=false, error=msg))
    end
end

println("\n", repeat("=", 74))
@printf("%6s %6s %6s %6s %8s %8s %6s %6s %6s %8s %8s %8s\n",
        "seed", "cutE", "cutC", "cutA", "dC", "dA", "itE", "itC", "itA",
        "wallE", "wallC", "wallA")
for r in rows
    r.ok || (@printf("%6d  FAILED\n", r.seed); continue)
    @printf("%6d %6d %6d %6d %+8d %+8d %6d %6d %6d %8.1f %8.1f %8.1f\n",
            r.seed, r.e.cuts, r.c.cuts, r.a.cuts,
            r.c.cuts - r.e.cuts, r.a.cuts - r.e.cuts,
            r.e.iters, r.c.iters, r.a.iters,
            r.e.wall, r.c.wall, r.a.wall)
end

println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
for r in rows
    r.ok || (push!(checks, ("seed $(r.seed): completed", false, first(r.error, 120))); continue)
    # Correctness gate FIRST: if the objectives differ, the cut comparison is meaningless
    # (we would be comparing paths to different answers).
    push!(checks, ("seed $(r.seed): objectives agree",
        isapprox(r.c.objective, r.e.objective; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f (diff %.3e)", r.c.objective, r.e.objective,
                 r.c.objective - r.e.objective)))
    # The activated oracle prices only over built stations and repairs the duals with a
    # closed-form completion. A wrong completion prunes the optimum, which surfaces as a
    # HIGHER objective -- so this equality is the completion's correctness gate and must be
    # checked before its cut count means anything.
    push!(checks, ("seed $(r.seed): activated objective agrees",
        isapprox(r.a.objective, r.e.objective; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f (diff %.3e)", r.a.objective, r.e.objective,
                 r.a.objective - r.e.objective)))
    push!(checks, ("seed $(r.seed): all three OPTIMAL",
        r.e.status == "OPTIMAL" && r.c.status == "OPTIMAL" && r.a.status == "OPTIMAL",
        "enum $(r.e.status) / cg $(r.c.status) / act $(r.a.status)"))
    if r.repro_cuts >= 0
        push!(checks, ("seed $(r.seed): master deterministic",
            r.repro_cuts == r.e.cuts,
            "re-solve $(r.repro_cuts) vs $(r.e.cuts) cuts"))
    end
end
for (name, ok, detail) in checks
    @printf("%-40s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)

good = [r for r in rows if r.ok]
if !isempty(good)
    for (label, d) in (("cg - enum", [r.c.cuts - r.e.cuts for r in good]),
                       ("activated - enum", [r.a.cuts - r.e.cuts for r in good]),
                       ("activated - cg", [r.a.cuts - r.c.cuts for r in good]))
        @printf("\nPAIRED CUT DIFFERENCE (%s)\n", label)
        @printf("  per seed: %s\n", string(d))
        @printf("  mean %+.2f | worse on %d seed(s) | equal on %d | better on %d\n",
                sum(d) / length(d), count(>(0), d), count(==(0), d), count(<(0), d))
    end
    # The activated oracle's whole bet: weaker cuts (it credits only a bound on the unbuilt
    # stations' duals) bought with cheaper pricing (it never searches them). Cut count alone
    # does not settle it -- pair it with the wall columns.
    da = [r.a.cuts - r.e.cuts for r in good]
    price_ratio = sum(r.a.cg_price_sec for r in good) /
                  max(1e-9, sum(r.c.cg_price_sec for r in good))
    @printf("\nACTIVATED TRADE: %+.2f cuts vs enumeration on average, pricing time %.2fx of plain CG\n",
            sum(da) / length(da), price_ratio)
end
n_fail == 0 || error("verification failed")
println("\nALL CHECKS PASSED")
