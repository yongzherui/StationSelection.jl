"""ONE Benders subproblem oracle on ONE instance. One arm, one job.

The arms used to run sequentially inside a single task, which had three problems: a
non-converging arm burned the wall the later arms needed, one expected non-convergence made
the whole task exit non-zero (hiding real failures), and co-tenancy on a shared node
contaminated the wall times the comparison is built on. Array tasks 1 and 2 of job 22435620
both landed on node4309.

So each `(instance, oracle)` is its own job on its own node, and each writes one TSV row.
`benders_lpo_report.jl` reads the rows, applies the correctness gate across arms, and prints
the paired verdicts.

# The arms

| oracle | prices over | cut's duals | n=10 s=3 |
| --- | --- | --- | --- |
| `direct_enumeration` | (enumerated pool) | LP duals | 4 it / 5 cuts |
| `column_generation` | all n stations | LP duals | 4 it / 5 cuts |
| `column_generation_activated` | built only | closed-form completion | 30 it / 77 cuts |
| `column_generation_activated_lpo` | built only | Pareto completion | 3 it / 6 cuts |
| `column_generation_activated_warm_start` | built only, THEN all | phase-2 LP duals | 4 it / 7 cuts |

`:column_generation_activated` is a settled negative -- non-convergent at n=15 (657 it /
1971 cuts, gap 1404 at 900 s) and n=20 -- so it is not in the default grid. Name it
explicitly to re-measure.

# Deterministic master

The master is a MIP with many tied optima and Gurobi's pick shifts with threading: 9 vs 12
cuts was measured on IDENTICAL n=20 configuration across two runs. That band is as wide as
the oracle effects here, so `LP_THREADS` defaults to 1. Wall times are then single-threaded;
`LP_THREADS=0` lets Gurobi choose and makes the cut columns indicative only.

Usage: sbatch --array=1-12 benchmarks/diagnostics/run_benders_lpo.sh
Env: LP_N LP_S LP_P LP_SEED LP_MAX_STOPS LP_ORACLE LP_SUB_MODE LP_SUB_K LP_THREADS
     LP_LPO_COMPLETION LP_MAX_ITERS LP_TOTAL_LIMIT LP_OUT
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
const ORACLE = Symbol(get(ENV, "LP_ORACLE", "column_generation"))
const SUB_MODE = Symbol(get(ENV, "LP_SUB_MODE", "exact"))
const SUB_K = parse(Int, get(ENV, "LP_SUB_K", "0"))
const LPO_COMPLETION = Symbol(get(ENV, "LP_LPO_COMPLETION", "separation"))
const THREADS = parse(Int, get(ENV, "LP_THREADS", "1"))
# Solve the per-scenario subproblems concurrently. Shortens the wall only -- it does not
# change any budget, so it cannot make a stuck certification succeed.
const PARALLEL = get(ENV, "LP_PARALLEL_SCENARIOS", "0") == "1"
const MAX_ITERS = parse(Int, get(ENV, "LP_MAX_ITERS", "3000"))
const TOTAL_LIMIT = parse(Float64, get(ENV, "LP_TOTAL_LIMIT", "1800.0"))
const OUT_DIR = get(ENV, "LP_OUT",
                    joinpath(@__DIR__, "results", "benders_lpo"))
const MAX_ROUTES = 20_000_000
const ENUM_LIMIT = 3600.0
# Per-round pricing budget inside one subproblem. The n=30 failures were ALL
# `certification_inconclusive` at 600 s -- the relaxed-cluster attempt ran out of budget and
# proved nothing, and a cut may only come from an exhausted round. So this is the parameter
# to raise at large n, not the iteration cap.
const CG_PRICE_LIMIT = parse(Float64, get(ENV, "LP_CG_PRICE_LIMIT", "600.0"))

@printf("n=%d s=%d p=%d seed=%d max_stops=%d | oracle %s\n",
        N, S, P, SEED, MAX_STOPS, ORACLE)
@printf("pricer %s%s | lpo_completion %s | master threads %s\n",
        SUB_MODE, SUB_K > 0 ? " (K=$SUB_K)" : "", LPO_COMPLETION,
        THREADS > 0 ? string(THREADS) : "auto")
@printf("budgets: %.0fs per pricing round, %.0fs total Benders loop | parallel scenarios %s (%d julia threads)\n",
        CG_PRICE_LIMIT, TOTAL_LIMIT, PARALLEL ? "on" : "off", Threads.nthreads())
flush(stdout)

problem, k, _ = benchmark_problem(@__DIR__, "LP", N, P, S, SEED)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)
@printf("k=%d\n", k)
flush(stdout)

is_enum = ORACLE === :direct_enumeration
pricing = is_enum ? CGPricingConfig() :
    (SUB_K > 0 ? CGPricingConfig(mode=SUB_MODE, relaxed_cluster_count=SUB_K) :
                 CGPricingConfig(mode=SUB_MODE))
config = THREADS > 0 ?
    SolverOptions(silent=true, time_limit_sec=600.0, threads=THREADS) :
    SolverOptions(silent=true, time_limit_sec=600.0)
solver = BendersSolver(
    config=config,
    max_iterations=MAX_ITERS,
    subproblem=BendersSubproblemConfig(
        oracle=ORACLE,
        max_stops=is_enum ? MAX_STOPS : nothing,
        max_routes=MAX_ROUTES, enumeration_time_limit_sec=ENUM_LIMIT,
        pricing=pricing,
        lpo_completion=LPO_COMPLETION,
        cg_pricing_time_limit_sec=CG_PRICE_LIMIT, max_cg_iterations=500,
        verbose=true),
    total_time_limit_sec=TOTAL_LIMIT,
    parallel_scenarios=PARALLEL)

t0 = time()
result = run_opt(problem, formulation, solver)
wall = time() - t0
md = result.metadata

@printf("\n%s %.6f | iters %d cuts %d | LB %.6f gap %.3e | stop %s | wall %.1fs\n",
        string(result.termination_status), something(result.objective_value, NaN),
        md["benders_iterations"], md["benders_cuts_added"],
        md["benders_lower_bound"], md["benders_gap"],
        md["benders_stop_reason"], wall)
@printf("  master %.1fs | subproblems %.1fs | enumeration %.1fs\n",
        md["benders_master_sec"], md["benders_subproblem_sec"],
        md["benders_enumeration_sec"])
if get(md, "benders_cg_pool_final", 0) > 0
    @printf("  CG: %d iters over %d rounds | pool %d | pricing %.1fs (restricted %.1fs + full %.1fs)\n",
            get(md, "benders_cg_iterations", 0), get(md, "benders_cg_rounds", 0),
            md["benders_cg_pool_final"], get(md, "benders_cg_pricing_sec", 0.0),
            get(md, "benders_cg_restricted_pricing_sec", 0.0),
            get(md, "benders_cg_full_pricing_sec", 0.0))
end
if get(md, "benders_cg_lpo_calls", 0) > 0
    @printf("  LPO: %d completions, %d LP rounds, %d rows, %d optimal | separation pricing %.1fs | gain %.3f | core slack %.4f\n",
            md["benders_cg_lpo_calls"], get(md, "benders_cg_lpo_rounds", 0),
            get(md, "benders_cg_lpo_rows", 0), get(md, "benders_cg_lpo_optimal", 0),
            get(md, "benders_cg_lpo_pricing_sec", 0.0),
            get(md, "benders_cg_lpo_improved", 0.0), get(md, "benders_core_slack", NaN))
end
flush(stdout)

# Total pricing is phase-1 plus separation. Reporting only the first made the LPO arm read
# "pricing 0.0s" while half its wall was separation.
price_total = get(md, "benders_cg_pricing_sec", 0.0) +
              get(md, "benders_cg_lpo_pricing_sec", 0.0)

mkpath(OUT_DIR)
row = joinpath(OUT_DIR, "n$(N)_s$(S)_p$(P)_seed$(SEED)_ms$(MAX_STOPS)_$(ORACLE).tsv")
open(row, "w") do io
    println(io, join(["n", "s", "p", "seed", "max_stops", "oracle", "status", "objective",
                      "iters", "cuts", "lower_bound", "gap", "stop_reason", "wall",
                      "master_sec", "sub_sec", "enum_sec", "price_total", "price_cg",
                      "price_restricted", "price_full",
                      "price_separation", "pool", "lpo_calls", "lpo_rounds", "lpo_rows",
                      "lpo_gain", "core_slack", "scope", "node"], '\t'))
    println(io, join(string.([
        N, S, P, SEED, MAX_STOPS, ORACLE, string(result.termination_status),
        something(result.objective_value, NaN), md["benders_iterations"],
        md["benders_cuts_added"], md["benders_lower_bound"], md["benders_gap"],
        md["benders_stop_reason"], round(wall; digits=2),
        round(md["benders_master_sec"]; digits=2),
        round(md["benders_subproblem_sec"]; digits=2),
        round(md["benders_enumeration_sec"]; digits=2),
        round(price_total; digits=2),
        round(get(md, "benders_cg_pricing_sec", 0.0); digits=2),
        round(get(md, "benders_cg_restricted_pricing_sec", 0.0); digits=2),
        round(get(md, "benders_cg_full_pricing_sec", 0.0); digits=2),
        round(get(md, "benders_cg_lpo_pricing_sec", 0.0); digits=2),
        get(md, "benders_cg_pool_final", 0), get(md, "benders_cg_lpo_calls", 0),
        get(md, "benders_cg_lpo_rounds", 0), get(md, "benders_cg_lpo_rows", 0),
        round(get(md, "benders_cg_lpo_improved", 0.0); digits=3),
        get(md, "benders_core_slack", NaN),
        get(md, "benders_optimality_scope", "n/a"), gethostname(),
    ]), '\t'))
end
println("\nwrote $row")

# This job's OWN gate only. Cross-arm agreement is the aggregator's job -- it needs every
# arm, and no single job can see them.
if result.termination_status != StationSelection.SOLVE_OPTIMAL
    @printf("\n!! %s did NOT converge: %s / %s\n", ORACLE,
            string(result.termination_status), md["benders_stop_reason"])
    exit(1)
end
println("\nARM CONVERGED")
