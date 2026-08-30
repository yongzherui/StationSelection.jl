"""
The single entrypoint every label-setting-priced formulation's `CGSolver`
`price_columns` hook reduces to calling: `_run_pricing_round`. It fans out
over scenarios (a pricing "unit" is always one scenario -- both live
formulations price `1:n_scenarios`, and nothing has ever needed a coarser or
finer grain) in two phases -- phase 1 prepares each scenario's search context
(`_prepare_pricing_scenario`), phase 2 calls `_run_label_setting`
(`engine.jl`) *directly* against each prepared scenario, one hop, no
formulation-specific wrapper functions in between -- and harvests, verifies,
and materializes the surviving labels into columns.

The two phases are split (rather than one combined per-scenario loop) because
phase 1 isn't free for every formulation: `AggregateODRouteJointRoutingAssignmentFormulation`'s
`_pricing_build_scenario_context` (`joint_routing_assignment/pricing_round.jl`)
turns duals into candidates by scanning every OD pair against every valid
station pair, real per-scenario work, not setup overhead. Keeping it a
distinct, separately-threaded phase means that scan gets the same
`Threads.@threads` parallelism as the search itself, rather than being forced
sequential just because it happens to run "before" the search.

Two dispatch layers, at two different granularities:
- the hooks below dispatch on `formulation::AbstractFormulation` for anything
  that varies per *formulation* (which scenarios to price, how to build a
  scenario's context, how to merge results across scenarios);
- the four remaining hooks dispatch on `ctx::AbstractPricingSearchContext`
  (implemented alongside each concrete context in `route_covering/exact/hooks.jl` /
  `joint_routing_assignment/exact/hooks.jl`) for anything that varies per *label
  type* (candidate extraction, pool signature, column materialization,
  master-reduced-cost verification) -- the same granularity `types.jl`'s
  inner search hooks already dispatch at.

Six hooks total; `_pricing_scenarios` defaults to every scenario in
`mapping.scenarios` (right for both live formulations, including the
single-scenario case -- `1:length(mapping.scenarios)` is just `1:1` and
everything below degenerates for free), so neither overrides it today. A
future Benders subproblem formulation (`RouteCoveringProblem`, fixed
`y`/assignment, only `theta` free) is expected to solve one scenario per
subproblem -- not jointly across scenarios -- so it should get the same
default for free by simply building its `mapping` scoped to that one
scenario, the same way today's two formulations build theirs scoped to all
of them; `_pricing_scenarios` stays a real hook rather than being inlined at
the call site only because every other formulation-varying axis in this file
already is one, not because a concrete override is anticipated. Three of the
other five have defaults matching `AggregateODRouteBaseFormulation`'s
behavior, so only `AggregateODRouteJointRoutingAssignmentFormulation`
(threaded, cross-scenario column budget, real column ids) needs to override
them.
"""

# ── the hub ──────────────────────────────────────────────────────────────────

"""
    _run_pricing_round(formulation, mapping, m, duals, solver::CGSolver;
        n_candidates=typemax(Int) ÷ 2, max_new_columns=typemax(Int) ÷ 2,
        time_limit=30.0, profile=false) -> Vector{<:Any}

`time_limit` is the **wall-clock budget for one whole round**, not a per-scenario budget.
Serial execution divides it *equally* across scenarios (each gets `time_limit /
n_scenarios`) because their searches sum; parallel execution gives each scenario the full
`time_limit` because their searches overlap. Either way a round costs about `time_limit`
of wall time.

Equal division rather than a shared running deadline is deliberate for the serial path. A
shared deadline consumed in scenario order lets the first scenario spend the whole round,
starving the rest -- and because the scenario order is stable across iterations, the same
scenarios would be starved every round, so pricing would improve one scenario's coverage
and never the others'. An equal split guarantees every scenario advances each round.

The **serial** path additionally re-divides the *remaining* budget before each scenario,
so slack left by a scenario that exhausted early is handed to those after it rather than
wasted. When every scenario spends its full slice this is identical to the equal split.

The **parallel** path gives every scenario the *whole* `time_limit`, because concurrent
searches make the round's wall their max rather than their sum. Both paths therefore obey
the same round wall budget, but the parallel one performs up to `n_scenarios` x more label
search within it -- which is exactly how parallelism buys certification rather than merely
finishing sooner. It does not reallocate slack (all scenarios start together), and it has
no need to: no scenario is competing for another's time.

`price_columns`'s real body for every label-setting-priced formulation.
Phase 1: build each scenario's search context and accept/dedupe state
(`_prepare_pricing_scenario`). Phase 2: run `_run_label_setting`
directly against each prepared scenario, accepting/deduping surviving labels
against that scenario's existing pool as they're found (a pool-novelty +
reduced-cost test identical across every pricer, so it lives here once rather
than once per pricer as it used to). Then one cross-scenario merge, and
materialize+verify the final candidates into columns.

The accept/dedupe closure built in phase 1 is handed to
`_run_label_setting` as `stop_if` in phase 2, and every candidate that
becomes final in that scenario's `best_by_signature` was, by construction of
that search loop, already offered to `stop_if` at exactly that final state --
so there is no second pass re-scanning the returned labels afterward (the old
four driver functions each did this; it re-verified nothing that `stop_if`
hadn't already recorded).
"""
function _run_pricing_round(
    formulation::AbstractFormulation,
    mapping,
    m::JuMP.Model,
    duals,
    solver::CGSolver;
    n_candidates::Int=typemax(Int) ÷ 2,
    max_new_columns::Int=typemax(Int) ÷ 2,
    time_limit::Float64=30.0,
    profile::Bool=false,
)
    # Each scenario is priced independently below, then results are merged
    # across scenarios after the loop. Scenarios are independent, so both
    # phases below parallelize across them when the formulation opts in and
    # there's more than one thread to use.
    scenarios = _pricing_scenarios(formulation, mapping, m)
    parallel = (solver.parallel_scenario_pricing || _pricing_parallel_scenarios(formulation)) &&
        length(scenarios) > 1 && Threads.nthreads() > 1

    # `time_limit` budgets the ROUND'S WALL CLOCK. How that converts to a per-scenario
    # search budget depends on how the scenarios are executed:
    #
    #   serial   -- searches run back to back, so the round's wall is their SUM: each
    #               scenario gets time_limit / n_scenarios (re-divided below as slack frees
    #               up), and no scenario can starve another.
    #   parallel -- searches run concurrently, so the round's wall is their MAX: each
    #               scenario can have the FULL time_limit and the round still finishes
    #               within its wall budget.
    #
    # So both arms honour the same round wall budget; the parallel arm simply converts the
    # extra cores into n_scenarios x more search inside it. That is the whole point of the
    # comparison, and it is why this is not `time_limit / n_scenarios` here.
    #
    # This holds only while every scenario gets its own thread. With nthreads < n_scenarios
    # the searches run in waves and the round's wall can reach
    # ceil(n_scenarios / nthreads) x time_limit -- callers that must respect the wall bound
    # have to allocate accordingly (Study 5's runner hard-fails otherwise).
    scenario_time_limit = time_limit

    # Phase 1: prepare -- context, pool-novelty state, accept/dedupe closure
    # -- for every scenario, so phase 2 below has nothing left to do but call
    # `_run_label_setting`. `nothing` for a scenario with nothing to
    # price.
    prepared = Vector{Any}(undef, length(scenarios))
    if parallel
        Threads.@threads for i in eachindex(scenarios)
            prepared[i] = _prepare_pricing_scenario(formulation, mapping, scenarios[i], m, duals, solver, n_candidates)
        end
    else
        for i in eachindex(scenarios)
            prepared[i] = _prepare_pricing_scenario(formulation, mapping, scenarios[i], m, duals, solver, n_candidates)
        end
    end

    # Phase 2: the label search itself, per scenario.
    candidates_by_scenario = Vector{Vector{Any}}(undef, length(scenarios))
    exhausted_by_scenario = trues(length(scenarios))
    stats_by_scenario = Vector{Any}(undef, length(scenarios))
    fill!(stats_by_scenario, nothing)
    if parallel
        Threads.@threads for i in eachindex(scenarios)
            p = prepared[i]
            if isnothing(p)
                candidates_by_scenario[i] = Any[]
            else
                t_scenario = time()
                _labels, exhausted, stats = _run_label_setting(
                    p.ctx; time_limit=scenario_time_limit, reduced_cost_tol=solver.reduced_cost_tol,
                    profile=profile, stop_if=p.accept,
                )
                search_sec = time() - t_scenario
                exhausted_by_scenario[i] = exhausted
                # `search_sec`/`slice_sec` are recorded unconditionally (not gated on
                # `profile`): comparing the SUM of per-scenario searches against their MAX
                # is what distinguishes a serial round from a parallel one, and it must be
                # available on ordinary benchmark runs.
                stats_by_scenario[i] = (; scenario=scenarios[i], search_sec=search_sec,
                                        slice_sec=scenario_time_limit, stats...)
                candidates_by_scenario[i] = collect(values(p.scored))
            end
        end
    else
        # Serial: re-divide the *remaining* round budget before each scenario, over the
        # scenarios still to run. A scenario that exhausts early (or has nothing to price)
        # hands its unused time to the ones after it, instead of that slack being lost.
        # When every scenario uses its whole slice this reduces exactly to the equal split:
        # after k of s scenarios have each spent T/s, the next slice is
        # (T - k*T/s) / (s - k) = T/s. So reallocation can only ever help, never let one
        # scenario claim more than its fair share of what is left.
        round_deadline = time() + time_limit
        for (position, i) in enumerate(eachindex(scenarios))
            p = prepared[i]
            if isnothing(p)
                candidates_by_scenario[i] = Any[]
            else
                remaining_scenarios = length(scenarios) - position + 1
                slice = max(0.0, (round_deadline - time()) / remaining_scenarios)
                t_scenario = time()
                _labels, exhausted, stats = _run_label_setting(
                    p.ctx; time_limit=slice, reduced_cost_tol=solver.reduced_cost_tol,
                    profile=profile, stop_if=p.accept,
                )
                search_sec = time() - t_scenario
                exhausted_by_scenario[i] = exhausted
                stats_by_scenario[i] = (; scenario=scenarios[i], search_sec=search_sec,
                                        slice_sec=slice, stats...)
                candidates_by_scenario[i] = collect(values(p.scored))
            end
        end
    end

    # Read by CGSolver when this round returns no columns. `false` means the
    # empty result came from a timeout or intentional early stop, not a proof
    # that the pricing problem contains no improving column.
    m[:label_setting_pricing_exhausted] = all(exhausted_by_scenario)
    history = get!(() -> Any[], JuMP.object_dictionary(m), :label_setting_pricing_stats)
    append!(history, filter(x -> !isnothing(x), stats_by_scenario))

    # Cross-scenario step: e.g. apply a shared column budget across
    # scenarios. Default (`_pricing_merge_scenarios` fallback below) is a
    # no-op concat.
    merged = _pricing_merge_scenarios(
        formulation, mapping, reduce(vcat, candidates_by_scenario; init=Any[]), max_new_columns,
    )

    # Turn each surviving candidate into a real column, then re-derive its
    # reduced cost from the master's own duals as a cross-check that the
    # pricer's model of the master hasn't drifted from the master itself.
    next_id = _pricing_next_column_id(formulation, mapping, m)
    columns = Any[]
    for (offset, candidate) in enumerate(merged)
        column = _pricing_make_column(candidate.ctx, next_id + offset - 1, candidate)
        ok, pricer_rc, master_rc = _pricing_verify_column(candidate.ctx, column, m, mapping, duals)
        ok || error(
            "label-setting pricing reduced cost $(pricer_rc) disagrees with the master's " *
            "dual-implied $(master_rc) for a column in scenario $(candidate.scenario) -- the " *
            "pricer and master formulations have drifted apart",
        )
        push!(columns, column)
    end
    return columns
end

"""
    _prepare_pricing_scenario(formulation, mapping, scenario, m, duals, solver,
        n_candidates) -> Union{Nothing, NamedTuple}

Phase 1 of `_run_pricing_round` for a single scenario: build its search
context and existing-column pool, the pool-novelty state derived from that
pool, and the accept/dedupe closure (`_pricing_accept_closure`, below) that
closes over both. Returns `(ctx=..., accept=..., scored=...)` -- everything
phase 2 needs to call `_run_label_setting(ctx; stop_if=accept, ...)`
directly with no further setup -- or `nothing` when the scenario has nothing
to price. Kept a separate phase (rather than folded into the search loop)
because building the context isn't free for every formulation --
`AggregateODRouteJointRoutingAssignmentFormulation`'s scans every OD pair
against every valid station pair -- so it needs the same
`Threads.@threads` parallelism as the search itself, not to be forced
sequential just because it happens to run first."""
function _prepare_pricing_scenario(
    formulation::AbstractFormulation, mapping, scenario, m::JuMP.Model, duals,
    solver::CGSolver, n_candidates::Int,
)
    built = _pricing_build_scenario_context(formulation, mapping, scenario, m, duals)
    isnothing(built) && return nothing  # nothing to price for this scenario
    ctx, existing_columns = built

    # For every column already in this scenario's pool, remember the best
    # (lowest) tau seen per signature. A freshly searched candidate only
    # clears the "novel" bar (in `_pricing_accept_closure`) if it beats this.
    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        signature = _pricing_pool_signature(ctx, column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    # Best candidate seen so far per signature, keyed the same way as
    # best_pool_tau; mutated in place by the closure once phase 2 runs the
    # search against `ctx`.
    scored = Dict{Any, Any}()
    accept! = _pricing_accept_closure(ctx, scenario, best_pool_tau, scored, solver, n_candidates)
    return (ctx=ctx, accept=accept!, scored=scored)
end

"""
    _pricing_accept_closure(ctx, scenario, best_pool_tau, scored, solver, n_candidates)
        -> (label -> Bool)

Build the accept/dedupe callback `_prepare_pricing_scenario` packages up for
phase 2 to hand `_run_label_setting` as `stop_if`.
`_run_label_setting` calls the returned closure on every label it
visits; this is the *only* place a label is judged -- there is deliberately
no second filtering pass over the returned candidates afterward (see the
`_run_pricing_round` docstring). It must be returned as a closure (rather
than e.g. a callable struct) because `_run_label_setting`'s `stop_if`
contract is a plain single-argument `label -> Bool` function."""
function _pricing_accept_closure(ctx, scenario, best_pool_tau::Dict, scored::Dict, solver::CGSolver, n_candidates::Int)
    function accept!(label)
        candidate = _pricing_candidate_from_label(ctx, label)
        isnothing(candidate) && return false          # label certifies nothing
        candidate.reduced_cost < -solver.reduced_cost_tol || return false  # not improving
        candidate.tau < get(best_pool_tau, candidate.signature, Inf) - 1e-9 || return false  # not novel vs. pool
        current = get(scored, candidate.signature, nothing)
        if isnothing(current) ||
                candidate.reduced_cost < current.reduced_cost - 1e-9 ||
                (abs(candidate.reduced_cost - current.reduced_cost) <= 1e-9 && candidate.tau < current.tau - 1e-9)
            # first time this signature is seen, or it strictly improves on
            # the best candidate kept for it so far -> keep it
            scored[candidate.signature] = (scenario=scenario, ctx=ctx, candidate...)
        end
        return length(scored) >= n_candidates  # true = tell the search to stop early
    end
    return accept!
end

# ── formulation-level hooks ─────────────────────────────────────────────────

"""
Scenario indices to price for this formulation, out of `mapping`. Default:
every scenario in `mapping.scenarios` -- right for both live formulations
(`AggregateODRouteBaseFormulation`/`AggregateODRouteJointRoutingAssignmentFormulation`),
including the single-scenario case (`1:1`), so neither overrides this. A
formulation only needs to override this if its `mapping` can legitimately
carry scenarios that a given solve shouldn't price -- not expected to arise
from a single-scenario `mapping` (that already falls out of the default),
only from one that's deliberately scoped wider than what's being priced."""
_pricing_scenarios(formulation::AbstractFormulation, mapping, m::JuMP.Model) =
    1:length(mapping.scenarios)

"""
    _pricing_build_scenario_context(formulation, mapping, scenario, m, duals)
        -> Union{Nothing, Tuple{AbstractPricingSearchContext, AbstractVector}}

Build the search context and existing-column pool for one scenario, or
`nothing` to skip a scenario with nothing to price (e.g. no active
pairs/candidates)."""
_pricing_build_scenario_context(formulation::AbstractFormulation, mapping, scenario, m::JuMP.Model, duals) =
    error("_pricing_build_scenario_context not implemented for $(typeof(formulation))")

"""Whether scenarios may be priced concurrently (`Threads.@threads`). Default: sequential."""
_pricing_parallel_scenarios(::AbstractFormulation)::Bool = false

"""Starting id for freshly materialized columns. Default: `1` (a throwaway
placeholder -- correct for formulations whose incremental column-adder mints
its own id by content signature, e.g. `AggregateODRouteBaseFormulation`)."""
_pricing_next_column_id(::AbstractFormulation, mapping, m::JuMP.Model)::Int = 1

"""Merge/order/budget candidates across all scenarios before materialization.
Default: identity (no cross-scenario budget, order doesn't matter)."""
_pricing_merge_scenarios(::AbstractFormulation, mapping, candidates::AbstractVector, max_new_columns::Int) = candidates

# ── context-level hooks (dispatch on the concrete AbstractPricingSearchContext) ──

"""
Normalize a label that survived the search into `(signature, tau, reduced_cost,
payload)`, or `nothing` if it certifies nothing. `payload` is whatever
`_pricing_make_column` needs to build the real column -- the label itself for
formulations where the label already carries the final answer, or e.g. a
route-replay result for formulations (`joint_routing_assignment`) where the
label only tracks a cheap proxy signature during search."""
_pricing_candidate_from_label(ctx::AbstractPricingSearchContext, label) =
    error("_pricing_candidate_from_label not implemented for $(typeof(ctx))")

"""Key an *existing pool* column the same way `_pricing_candidate_from_label`
keys a freshly searched one, so pool-novelty comparisons line up."""
_pricing_pool_signature(ctx::AbstractPricingSearchContext, existing_column) =
    error("_pricing_pool_signature not implemented for $(typeof(ctx))")

"""Build the concrete column type from an accepted, id-assigned candidate."""
_pricing_make_column(ctx::AbstractPricingSearchContext, column_id::Int, candidate) =
    error("_pricing_make_column not implemented for $(typeof(ctx))")

"""Cross-check a materialized column's reduced cost against the master's own
duals: `(ok::Bool, pricer_rc, master_rc)`."""
_pricing_verify_column(ctx::AbstractPricingSearchContext, column, m::JuMP.Model, mapping, duals) =
    error("_pricing_verify_column not implemented for $(typeof(ctx))")
