"""Same-process A/B: does the station_simple dominance *mechanics* rework pay?

The three changes under test are pure access-pattern changes -- they alter where the
predicate reads its operands from, never what it decides:

  NEW (current source)                    LEGACY (reconstructed below)
  age_mask from the inline isbits filters age_mask from the heap Ages struct
  visited/layers from the Ages mirror     visited/layers dereferenced off the label
  Val(Compensated) specialization         4-arg call with a runtime `compensated::Bool`

`LEGACY` is reconstructed against the *current* structs (which still carry every field
both variants need), so this measures exactly those three access patterns and nothing
else -- not a different build, not a different Julia, not a different node.

Why same-process A/B rather than two SLURM runs: the earlier before/after comparison was
one wall-clock sample per side, 1.4% apart, on different jobs. That cannot separate "no
effect" from "small effect" from "noise". Here both variants run interleaved in one
process, against the identical search, alternating to cancel drift, and report a
distribution rather than a point.

Both contexts are built by hand from the same `pricing_data`/`search_index`, differing
ONLY in the `dominates` closure, so the label population and the search order are
identical by construction. The assertion on identical column counts is what proves that.

Usage: julia --project=. benchmarks/diagnostics/ab_station_simple_dominance.jl [n_pairs] [seed] [reps] [warmup_iters]
"""

using StationSelection
using JuMP
using Printf
using Statistics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const S = StationSelection
const N_PAIRS = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 16
const SEED = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 51
const REPS = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 7
const WARMUP_ITERS = length(ARGS) >= 4 ? parse(Int, ARGS[4]) : 6
const N_STATIONS, N_SCENARIOS, MAX_STOPS = 20, 3, 10

# ── LEGACY predicate: the pre-rework access pattern, semantics unchanged ─────
@inline function legacy_dominates_at_state(
    af::S.JointRoutingAssignmentStationSimpleDominanceFilters,
    a::S.JointRoutingAssignmentStationSimpleLabel,
    a_ages::S.JointRoutingAssignmentStationSimpleAges,
    bf::S.JointRoutingAssignmentStationSimpleDominanceFilters,
    b::S.JointRoutingAssignmentStationSimpleLabel,
    b_ages::S.JointRoutingAssignmentStationSimpleAges,
    layer_weight::Vector{Float64},
)::Bool
    af.time <= bf.time + 1e-9 || return false
    # LEGACY: masks come off the heap Ages structs, not the inline filters.
    S._sparse_station_age_support_rejection(
        a_ages.age_idx, a_ages.age_mask, b_ages.age_idx, b_ages.age_mask,
    ) == 0 || return false
    budget = bf.reduced_cost - af.reduced_cost + 1e-9
    budget >= 0.0 || return false
    # LEGACY: dereference the labels for both BitSet resources.
    issubset(a.visited, b.visited) || return false
    S._sparse_station_age_values_dominate(
        a_ages.age_idx, a_ages.age_val, b_ages.age_idx, b_ages.age_val,
    ) || return false
    # LEGACY: runtime-Bool compensation, no Val specialization.
    S._joint_routing_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, layer_weight, budget,
    ) <= budget || return false
    return true
end

problem, k, _ = benchmark_problem(@__DIR__, "AB", N_STATIONS, N_PAIRS, N_SCENARIOS, SEED)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS, pricing_mode=:exact,
)
solver = S.CGSolver(config=S.SolverOptions(silent=true),
                    pricing_time_limit_sec=900.0, parallel_scenario_pricing=false)
build_result = S.build_model(problem, formulation, solver)
m = build_result.model
mapping = build_result.mapping
S._apply_solver_config!(m, solver.config)
@printf("instance n=%d p=%d s=%d seed=%d  reps=%d\n", N_STATIONS, N_PAIRS, N_SCENARIOS, SEED, REPS)
flush(stdout)

for it in 1:WARMUP_ITERS
    optimize!(m)
    JuMP.termination_status(m) == MOI.OPTIMAL || break
    d = S.extract_duals(build_result, mapping, m)
    cols = S.price_columns(build_result, mapping, m, d, solver; time_limit_sec=900.0)
    (isnothing(cols) || isempty(cols)) && break
    S.add_columns!(build_result, mapping, m, cols)
end
optimize!(m)
duals = S.extract_duals(build_result, mapping, m)
@printf("warmed: pool=%d  obj=%.4f\n\n", length(m[:joint_routing_assignment_columns]), JuMP.objective_value(m))
flush(stdout)

# Build one scenario's pricing data once; both variants search it identically.
m[:joint_routing_assignment_pricing_mode] = :station_simple
scenarios = S._pricing_scenarios(formulation, mapping, m)
built = S._pricing_build_scenario_context(formulation, mapping, first(scenarios), m, duals)
built === nothing && error("no pricing context for scenario $(first(scenarios))")
base_ctx, _existing = built
pd = base_ctx.pricing_data

# Both closures MUST be built inside a function. A closure defined at top level captures
# non-const globals, whose types the compiler cannot see, so every call boxes and dispatches
# dynamically -- which measures the capture, not the predicate. The production closure is
# built inside `JointRoutingAssignmentStationSimpleSearchContext`, i.e. over typed locals;
# building the legacy one at top level made it type-unstable and reported a spurious 6.2x.
function build_contexts(pricing_data)
    new_ctx = S.JointRoutingAssignmentStationSimpleSearchContext(pricing_data)
    layer_weight = pricing_data.layer_weight   # typed local, captured by value
    legacy_closure(x::S.PricingLabelEntry, y::S.PricingLabelEntry) = legacy_dominates_at_state(
        x.filters, x.label, x.bitsets, y.filters, y.label, y.bitsets, layer_weight,
    )
    legacy_ctx = S.JointRoutingAssignmentStationSimpleSearchContext(
        pricing_data, legacy_closure, new_ctx.search_index, new_ctx.bound_workspace,
        new_ctx.node_index,
    )
    return new_ctx, legacy_ctx
end
new_ctx, legacy_ctx = build_contexts(pd)

# Both closures must be equally specialized or the comparison is meaningless. A concretely
# inferred closure call returns a concrete Bool; an unstable one infers Any.
let e = S.PricingLabelEntry
    for (name, f) in (("new", new_ctx.dominates), ("legacy", legacy_ctx.dominates))
        rt = Base.return_types(f, (e, e))
        @printf("closure %-7s inferred return type: %s\n", name, isempty(rt) ? "?" : rt[1])
    end
end
flush(stdout)

function run_search(ctx)
    t = time()
    labels, exhausted, stats = S._run_label_setting(
        ctx; time_limit=900.0, reduced_cost_tol=1e-6,
    )
    return time() - t, length(labels), exhausted, stats
end

# JIT both before timing anything.
run_search(new_ctx); run_search(legacy_ctx)

new_times, legacy_times = Float64[], Float64[]
# Ref, not a bare Int: a `for` loop at top level is soft scope, so `new_n = nn` inside it
# binds a fresh local and the outer value stays at its initializer -- which is exactly how
# the first run of this script "verified" semantics by comparing -1 == -1.
new_n, legacy_n = Ref(-1), Ref(-1)
# ORDER-SWAPPED (ABBA). Running one variant first in every rep confounds it with a
# position effect: the first search of a pair absorbs GC and allocator warm-up the second
# then benefits from, which alone produces a perfect 7/7 "the first one is slower". Odd
# reps run NEW first, even reps run LEGACY first, so position is balanced across the
# sample and can be measured rather than assumed away.
first_pos, second_pos = Float64[], Float64[]
for r in 1:REPS
    if isodd(r)
        tn, nn, _, _ = run_search(new_ctx);    tl, nl, _, _ = run_search(legacy_ctx)
        push!(first_pos, tn); push!(second_pos, tl)
    else
        tl, nl, _, _ = run_search(legacy_ctx); tn, nn, _, _ = run_search(new_ctx)
        push!(first_pos, tl); push!(second_pos, tn)
    end
    push!(new_times, tn); push!(legacy_times, tl)
    new_n[] = nn; legacy_n[] = nl
    @printf("  rep %d/%d [%s first]: new %.3fs (%d labels)  legacy %.3fs (%d labels)\n",
            r, REPS, isodd(r) ? "new" : "legacy", tn, nn, tl, nl)
    flush(stdout)
end

println()
(new_n[] > 0 && legacy_n[] > 0) || error(
    "semantics guard never ran (new=$(new_n[]), legacy=$(legacy_n[])) -- refusing to " *
    "report a timing comparison that was not validated"
)
new_n[] == legacy_n[] || error(
    "variants disagree on the search result ($(new_n[]) vs $(legacy_n[]) finished labels) " *
    "-- the LEGACY reconstruction changed semantics, so the timing comparison is void"
)
@printf("both variants returned %d finished labels (semantics identical)\n\n", new_n[])

med_n, med_l = median(new_times), median(legacy_times)
@printf("%-10s median %.3fs   min %.3fs   max %.3fs   spread %.1f%%\n",
        "NEW", med_n, minimum(new_times), maximum(new_times),
        100 * (maximum(new_times) - minimum(new_times)) / med_n)
@printf("%-10s median %.3fs   min %.3fs   max %.3fs   spread %.1f%%\n",
        "LEGACY", med_l, minimum(legacy_times), maximum(legacy_times),
        100 * (maximum(legacy_times) - minimum(legacy_times)) / med_l)
println()
@printf("speedup (legacy/new): %.4fx   -- change of %+.2f%%\n", med_l / med_n, 100 * (med_l / med_n - 1))
println()

# How big is the position effect on its own? If this rivals the variant gap, the variant
# comparison is not interpretable no matter what the paired test says.
@printf("POSITION effect: 1st-in-pair median %.3fs vs 2nd-in-pair median %.3fs (%+.1f%%)\n",
        median(first_pos), median(second_pos),
        100 * (median(first_pos) / median(second_pos) - 1))

# Paired test on the order-balanced sample. Unpaired range comparison under-reports a
# small consistent effect, which is why this replaced it.
diffs = new_times .- legacy_times
n_pos = count(>(0), diffs)
p_sign = 2.0^(1 - REPS) * sum(binomial(REPS, k) for k in max(n_pos, REPS - n_pos):REPS)
@printf("PAIRED diffs (new-legacy): %s\n", join((@sprintf("%+.3f", d) for d in diffs), " "))
@printf("  new slower in %d/%d reps   two-sided sign test p = %.4f   median diff %+.3fs (%+.1f%%)\n",
        n_pos, REPS, min(p_sign, 1.0), median(diffs), 100 * median(diffs) / med_l)
verdict = if min(p_sign, 1.0) > 0.05
    "NOT SIGNIFICANT -- no detectable difference between the variants"
elseif abs(median(diffs)) < abs(median(first_pos) - median(second_pos))
    "AMBIGUOUS -- variant gap is smaller than the position effect it sits inside"
else
    "SIGNIFICANT -- variant gap survives the position effect"
end
println("  verdict: ", verdict)
