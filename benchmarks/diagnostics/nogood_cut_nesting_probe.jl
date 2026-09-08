"""Do the no-good cut supports NEST, and would cut management therefore pay?

# The question

`_relaxed_cluster_certify_scenario` accumulates one cut per barren cluster support
`T` and never prunes the set. Two structural facts follow from the cut's own definition
(`relaxed_cluster/cuts.jl`), and both are proved, not assumed:

  Cut(T) == "the route visits some cluster OUTSIDE T"

1. **No duplicates are possible.** A route that survives every existing cut visits a
   cluster outside each `T_i`, so its support `T_new` satisfies `T_new ⊄ T_i` for every
   `i`. It can never equal, or be contained in, a cut already placed. (This also means the
   new cut is never itself redundant, so pruning is only ever needed in one direction.)

2. **Domination IS possible, in exactly one direction.** `T_new` may be a strict SUPERSET
   of an existing `T_i`. Then `T_i ⊆ T_new` gives `∉T_new ⟹ ∉T_i`, hence
   `Cut(T_new) ⟹ Cut(T_i)`: the older cut is implied and excludes nothing further.

A dominated cut is not free. It holds one bit of the `UInt64` satisfied-mask (64 max) and
one round of `RELAXED_CLUSTER_MAX_CUT_ROUNDS` (measured binding at n=30), and -- the real cost --
the search state is `(current, satisfied)`, so `c` cuts admit up to `2^c` mask values per
node. A cut that excludes nothing still doubles the state space, and two labels differing
only in a dead bit can never dominate one another. That is the same mechanism measured for
`bounded_max_stops`, where an extra dominance condition weakened dominance and inflated the
live-label population.

So the logic says the effect CAN occur. This probe asks whether it DOES, and how often,
before any cut-management code is written -- an optimization for a case that never arises
is worse than no optimization, because it still has to be understood by the next reader.

# What is measured

For every no-good attempt, the actual cluster supports cut on, in order (recorded on the
result's `trace` as `support`). Per attempt this reports:

- `nested`: pairs `(i, j)`, `i < j`, with `T_i ⊆ T_j` -- the older cut is dominated and
  `filter!(t -> !issubset(t, T_new), cluster_sets)` would have removed it;
- `duplicate`: pairs with `T_i == T_j`. **Must be zero.** A non-zero count refutes fact 1
  above and means something is wrong with the cut machinery, not with this analysis;
- `disjoint-ish`: pairs where neither contains the other -- cut management would not help.

The headline number is the share of placed cuts that a prune would have removed, and the
peak simultaneous cut count with and without pruning: that difference is the state-space
saving, and it is what decides whether this is worth implementing.

Usage:  julia --project=. benchmarks/diagnostics/nogood_cut_nesting_probe.jl [n_stations]
"""

using StationSelection
using Statistics
# `benchmark_problem` builds the same Zhuzhou instance the Study 10 runs use, so this probe
# measures the cut behaviour on exactly the cells that produced the certification results.
# (`generate_zhuzhou_data` lives in scripts/ and is not a package export -- the lib pulls it
# in, so going through the lib is both correct and the convention here.)
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N_STATIONS = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 15
const N_PAIRS = 16
const N_SCENARIOS = 3
const SEEDS = 42:44
const MAX_STOPS = 10
# Per-run wall cap. The census does not need a converged solve -- it needs no-good rounds,
# which happen throughout -- so a short budget buys more cells rather than fewer.
# MUST be large enough for the run to CONVERGE. Deep no-good loops -- the only ones with
# more than one cut, and therefore the only ones a containment census can read -- happen
# near convergence. A budget that truncates the solve produces one cut per attempt, zero
# pairs, and a census of trivially zero. MEASURED: at n=20 with a 600 s cap every config
# reported 3 cuts over 3 attempts and no pairs at all.
const RUN_BUDGET_SEC = length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 1200.0

"""Pairwise containment census over one attempt's ordered supports."""
function _census(supports::Vector{Set{Int}})
    nested = 0; duplicate = 0; incomparable = 0
    pruned_away = Set{Int}()
    for j in eachindex(supports), i in 1:(j - 1)
        a, b = supports[i], supports[j]
        if a == b
            duplicate += 1
            push!(pruned_away, i)
        elseif issubset(a, b)
            nested += 1
            push!(pruned_away, i)      # Cut(b) implies Cut(a): a is dead weight
        elseif issubset(b, a)
            # Structurally impossible (fact 1). Counted so a violation is loud.
            duplicate += 1
        else
            incomparable += 1
        end
    end
    return (nested=nested, duplicate=duplicate, incomparable=incomparable,
            n_pruneable=length(pruned_away))
end

println("no-good cut nesting probe: n=$N_STATIONS, p=$N_PAIRS, s=$N_SCENARIOS, " *
        "seeds=$(collect(SEEDS)), ms=$MAX_STOPS\n")

total = (attempts=0, cuts=0, nested=0, duplicate=0, incomparable=0, pruneable=0)
peak_with = 0; peak_without = 0
support_counts = Int[]

for seed in SEEDS
    problem, k, _meta = benchmark_problem(
        @__DIR__, "PROBE", N_STATIONS, N_PAIRS, N_SCENARIOS, seed,
    )
    for n_clusters in (round(Int, 0.6 * N_STATIONS), round(Int, 0.8 * N_STATIONS))
        result = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops=MAX_STOPS),
            # Every budget is bounded. An unbounded probe that prints only at the end can
            # burn its whole walltime and yield NOTHING, which is exactly what the first
            # attempt did -- a diagnostic must degrade to partial data, not to silence.
            CGSolver(recover_integer_solution=false, max_iterations=200,
                     pricing=CGPricingConfig(mode=:relaxed_cluster,
                                             relaxed_cluster_count=n_clusters),
                     pricing_time_limit_sec=120.0,
                     certifying_pricing_time_limit_sec=300.0,
                     total_time_limit_sec=RUN_BUDGET_SEC,
                     parallel_scenario_pricing=true),
        )
        stats = get(result.metadata, "cg_relaxed_cluster_guide_stats", Any[])
        for row in stats
            hasproperty(row, :nogood_supports) || continue
            supports = row.nogood_supports
            isempty(supports) && continue
            c = _census(supports)
            global total = (attempts=total.attempts + 1,
                            cuts=total.cuts + length(supports),
                            nested=total.nested + c.nested,
                            duplicate=total.duplicate + c.duplicate,
                            incomparable=total.incomparable + c.incomparable,
                            pruneable=total.pruneable + c.n_pruneable)
            global peak_with = max(peak_with, length(supports))
            global peak_without = max(peak_without, length(supports) - c.n_pruneable)
            push!(support_counts, length(supports))
        end
        println("  seed $seed K=$n_clusters: $(result.termination_status), " *
                "attempts so far $(total.attempts), cuts $(total.cuts), " *
                "nested $(total.nested), dup $(total.duplicate), " *
                "incomparable $(total.incomparable), pruneable $(total.pruneable)")
        flush(stdout)
    end
end

println("\n=== census over $(total.attempts) attempts, $(total.cuts) cuts placed ===")
if total.cuts == 0
    println("no cuts were placed at all -- nothing to manage at this size/config")
else
    pairs = total.nested + total.duplicate + total.incomparable
    println("  pairwise relations ($pairs pairs):")
    println("    nested (T_i ⊆ T_j, older cut dominated) : $(total.nested)")
    println("    duplicate / reverse-nested (MUST be 0)  : $(total.duplicate)")
    println("    incomparable (management would not help): $(total.incomparable)")
    println("\n  cuts a prune would have removed: $(total.pruneable) of $(total.cuts) " *
            "($(round(100 * total.pruneable / total.cuts; digits=1))%)")
    println("  peak simultaneous cuts: $peak_with  ->  $peak_without with pruning")
    println("  cuts per attempt: median $(median(support_counts)), max $(maximum(support_counts))")
    if total.duplicate > 0
        println("\n  !! duplicate/reverse-nested cuts found -- this contradicts the cut's own")
        println("     definition and points at a bug in the cut machinery, not at this analysis.")
    end
    if total.pruneable == 0
        println("\n  VERDICT: supports never nest here. Cut management would be dead code.")
    else
        println("\n  VERDICT: nesting is real; pruning would cut the peak mask/state width as above.")
    end
end
