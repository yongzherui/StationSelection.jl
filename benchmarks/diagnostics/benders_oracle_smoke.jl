"""Load-and-solve smoke check for `BendersSolver`'s oracles, including the locally
Pareto-optimal one.

Cheap guard for the kind of edit that adds or removes a code path: it verifies the package
still loads, every live oracle constructs, unsupported oracles and completion modes are
REJECTED rather than silently accepted, and every oracle solves the same tiny instance to the
SAME objective end to end.

That last check is the one with teeth for `:column_generation_activated_lpo`. Its cuts come
from a dual point no other arm produces, so if the Pareto re-optimisation ever emitted an
INVALID cut it would prune part of the first-stage space and converge to a different (higher,
since the cut would have over-estimated `Q`) objective. Agreement with `:direct_enumeration`
and `:column_generation` on the same model is therefore a real, if small, validity test, and
it runs in seconds. The exhaustive version -- every column's dual constraint, every
master-feasible `y`, both completion modes -- is
`benders_activated_completion_audit.jl`.

Usage: sbatch --array=1-4 benchmarks/diagnostics/run_benders_oracle_smoke.sh
Env: SM_N SM_S SM_P SM_SEED
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const LIVE = (:direct_enumeration, :column_generation, :column_generation_activated,
              :column_generation_activated_lpo, :column_generation_warm_start)
const REJECTED_ORACLES = (:column_generation_activated_lpo_separation, :bogus)
const REJECTED_COMPLETIONS = (:separation, :route_free)
# Live again as of the Pareto work: these were removed on 2026-09-10 with the row-generation
# completion and are back with a different constraint family. Named here so the smoke test
# fails loudly if a future cleanup takes them out without taking out the oracle too.
const REQUIRED = (:_benders_lpo_completion!, :_benders_core_point, :_benders_lpo_core_point,
                  :_accumulate_benders_lpo_stats!)

println("package loaded")
for oracle in LIVE
    c = BendersSubproblemConfig(oracle = oracle)
    @printf("  %-34s constructs (max_stops %s)\n", oracle, string(c.max_stops))
end
for oracle in REJECTED_ORACLES
    try
        BendersSubproblemConfig(oracle = oracle)
        error("FAIL: unsupported oracle $oracle is accepted")
    catch e
        e isa ArgumentError || rethrow()
        @printf("  %-34s correctly rejected\n", oracle)
    end
end
for completion in REJECTED_COMPLETIONS
    try
        BendersSubproblemConfig(oracle = :column_generation_activated_lpo,
                                lpo_completion = completion)
        error("FAIL: unsupported lpo_completion $completion is accepted")
    catch e
        e isa ArgumentError || rethrow()
        @printf("  lpo_completion=%-22s correctly rejected\n", completion)
    end
end
for nm in REQUIRED
    isdefined(StationSelection, nm) || error("FAIL: required symbol $nm is not defined")
end
@printf("  %d required symbols present\n", length(REQUIRED))

const N = parse(Int, get(ENV, "SM_N", "10"))
const S = parse(Int, get(ENV, "SM_S", "1"))
const P = parse(Int, get(ENV, "SM_P", "8"))
const SEED = parse(Int, get(ENV, "SM_SEED", "42"))
@printf("\ninstance: zhuzhou n=%d p=%d s=%d seed=%d\n", N, P, S, SEED)
problem, k, _meta = benchmark_problem(@__DIR__, "SMOKE", N, P, S, SEED)
const MAX_STOPS = 4
# `max_stops` EXPLICIT on every arm. Its default is per-oracle (4 for enumeration, unbounded
# for CG), so leaving it out would have the CG arms search a larger universe than the
# enumerated one and the objectives would legitimately disagree -- turning the validity check
# below into a scope mismatch.
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops = MAX_STOPS)

arms = [(:direct_enumeration, :pareto), (:column_generation, :pareto),
        (:column_generation_activated, :pareto),
        (:column_generation_activated_lpo, :baseline),
        (:column_generation_activated_lpo, :pareto)]

println()
@printf("%-36s %-10s %-10s %14s %7s %6s %9s\n",
        "oracle", "completion", "status", "objective", "iters", "cuts", "sec")
objectives = Tuple{String, Float64, String}[]
for (oracle, completion) in arms
    solver = BendersSolver(
        config = SolverOptions(silent = true), max_iterations = 200,
        subproblem = BendersSubproblemConfig(
            oracle = oracle, max_stops = MAX_STOPS, lpo_completion = completion))
    t = time()
    result = run_opt(problem, formulation, solver)
    md = result.metadata
    label = oracle === :column_generation_activated_lpo ? "$oracle/$completion" : string(oracle)
    @printf("%-36s %-10s %-10s %14.6f %7s %6s %9.1f\n",
            oracle, oracle === :column_generation_activated_lpo ? completion : "-",
            result.termination_status, something(result.objective_value, NaN),
            md["benders_iterations"], md["benders_cuts_added"], time() - t)
    if haskey(md, "benders_lpo_calls")
        @printf("    lpo: %d calls, %d certified | rounds %d | rows %d seed + %d generated | core gain %+.4f | worst rho %+.2e | face %.2e | statuses %s\n",
                md["benders_lpo_calls"], md["benders_lpo_certified"], md["benders_lpo_rounds"],
                md["benders_lpo_seed_rows"], md["benders_lpo_generated_rows"],
                md["benders_lpo_core_gain"], md["benders_lpo_worst_rho"],
                md["benders_lpo_worst_face_residual"], md["benders_lpo_statuses"])
        md["benders_lpo_certified"] == md["benders_lpo_calls"] || @printf(
            "    NOTE: %d of %d completions fell back to the closed form -- this arm is not purely Pareto\n",
            md["benders_lpo_calls"] - md["benders_lpo_certified"], md["benders_lpo_calls"])
    end
    flush(stdout)
    push!(objectives, (label, result.objective_value, string(result.termination_status)))
end

# `:direct_enumeration` is the reference: its pool IS the universe, so nothing about its
# optimum depends on a pricer or a completion.
reference = objectives[1][2]
objectives[1][3] == "OPTIMAL" ||
    error("FAIL: the enumeration reference itself did not converge ($(objectives[1][3]))")

# Two separate checks, because the two failure modes are not the same thing.
#
#   An arm reporting OPTIMAL must MATCH. Invalid cuts prune first-stage points, including
#   possibly the optimum, and the loop then converges to a HIGHER objective while still
#   reporting OPTIMAL. That is the exact failure this script exists to catch.
#
#   An arm reporting FEASIBLE stopped on its iteration cap. Its objective is the best
#   incumbent's exact second-stage cost, hence a valid UPPER bound -- so it may exceed the
#   reference, but it must never fall BELOW it. `:column_generation_activated` is expected to
#   land here from about n=12 up (documented: 30 iterations at n=10, non-convergent at n=15),
#   so non-convergence is reported, not failed.
bad = [(l, v, st) for (l, v, st) in objectives
       if (st == "OPTIMAL" && abs(v - reference) > 1e-6 * max(1.0, abs(reference))) ||
          v < reference - 1e-6 * max(1.0, abs(reference))]
stalled = [(l, st) for (l, _v, st) in objectives if st != "OPTIMAL"]
@printf("\nreference %s = %.6f | %d arms | %d did not converge%s\n",
        objectives[1][1], reference, length(objectives), length(stalled),
        isempty(stalled) ? "" : " (" * join(("$l:$st" for (l, st) in stalled), ", ") * ")")
isempty(bad) || error(
    "FAIL: " * join(("$l reported $st at $v against reference $reference"
                     for (l, v, st) in bad), "; ") *
    " -- these arms solve the same model over the same route universe, so a converged " *
    "disagreement means one arm's cuts are not valid underestimators")
println("ALL CHECKS PASSED")
