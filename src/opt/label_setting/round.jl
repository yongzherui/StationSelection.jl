"""
The single entrypoint every label-setting-priced formulation's `CGSolver`
`price_columns` hook reduces to calling: `_run_pricing_round`. It fans out
over units (typically scenarios), builds a search context per unit, calls
`_run_pricing_label_search` (`engine.jl`) *directly* -- one hop, no
formulation-specific wrapper functions in between -- and harvests, verifies,
and materializes the surviving labels into columns.

Two dispatch layers, at two different granularities:
- the hooks below dispatch on `formulation::AbstractFormulation` for anything
  that varies per *formulation* (how many units, how to build a unit's
  context, how to merge results across units);
- the four remaining hooks dispatch on `ctx::AbstractPricingSearchContext`
  (defined alongside each concrete context in `aggregate_od_route/base/exact.jl` /
  `joint_routing_assignment/exact.jl`) for anything that varies per *label
  type* (candidate extraction, pool signature, column materialization,
  master-reduced-cost verification) -- the same granularity `types.jl`'s
  inner search hooks already dispatch at.

Only six hooks are required; three have defaults matching
`AggregateODRouteBaseFormulation`'s behavior, so only
`AggregateODRouteJointRoutingAssignmentFormulation` (threaded, cross-scenario
column budget, real column ids) needs to override them.
"""

# ── the hub ──────────────────────────────────────────────────────────────────

"""
    _run_pricing_round(formulation, mapping, m, duals, solver::CGSolver;
        n_candidates=typemax(Int) ÷ 2, max_new_columns=typemax(Int) ÷ 2,
        time_limit=30.0, profile=false) -> Vector{<:Any}

`price_columns`'s real body for every label-setting-priced formulation. Per
unit: build a context, run `_run_pricing_label_search` directly against it,
and accept/dedupe surviving labels against that unit's existing pool (a
pool-novelty + reduced-cost test identical across every pricer, so it lives
here once rather than once per pricer as it used to). Then one cross-unit
merge, and materialize+verify the final candidates into columns.

The accept/dedupe closure below is handed to `_run_pricing_label_search` as
`stop_if`, and every candidate that becomes final in that unit's
`best_by_signature` was, by construction of that search loop, already offered
to `stop_if` at exactly that final state -- so there is no second pass
re-scanning the returned labels afterward (the old four driver functions each
did this; it re-verified nothing that `stop_if` hadn't already recorded).
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
    units = _pricing_units(formulation, mapping, m)
    candidates_by_unit = Vector{Vector{Any}}(undef, length(units))

    function price_unit(unit)
        built = _pricing_build_unit_context(formulation, mapping, unit, m, duals)
        isnothing(built) && return Any[]
        ctx, existing_columns = built

        best_pool_tau = Dict{Any, Float64}()
        for column in existing_columns
            signature = _pricing_pool_signature(ctx, column)
            best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
        end

        scored = Dict{Any, Any}()
        function accept!(label)
            candidate = _pricing_candidate_from_label(ctx, label)
            isnothing(candidate) && return false
            candidate.reduced_cost < -solver.reduced_cost_tol || return false
            candidate.tau < get(best_pool_tau, candidate.signature, Inf) - 1e-9 || return false
            current = get(scored, candidate.signature, nothing)
            if isnothing(current) ||
                    candidate.reduced_cost < current.reduced_cost - 1e-9 ||
                    (abs(candidate.reduced_cost - current.reduced_cost) <= 1e-9 && candidate.tau < current.tau - 1e-9)
                scored[candidate.signature] = (unit=unit, ctx=ctx, candidate...)
            end
            return length(scored) >= n_candidates
        end

        _run_pricing_label_search(
            ctx; time_limit=time_limit, reduced_cost_tol=solver.reduced_cost_tol,
            profile=profile, stop_if=accept!,
        )
        return collect(values(scored))
    end

    if _pricing_parallel_units(formulation) && length(units) > 1 && Threads.nthreads() > 1
        Threads.@threads for i in eachindex(units)
            candidates_by_unit[i] = price_unit(units[i])
        end
    else
        for i in eachindex(units)
            candidates_by_unit[i] = price_unit(units[i])
        end
    end

    merged = _pricing_merge_units(
        formulation, mapping, reduce(vcat, candidates_by_unit; init=Any[]), max_new_columns,
    )

    next_id = _pricing_next_column_id(formulation, mapping, m)
    columns = Any[]
    for (offset, candidate) in enumerate(merged)
        column = _pricing_make_column(candidate.ctx, next_id + offset - 1, candidate)
        ok, pricer_rc, master_rc = _pricing_verify_column(candidate.ctx, column, m, mapping, duals)
        ok || error(
            "label-setting pricing reduced cost $(pricer_rc) disagrees with the master's " *
            "dual-implied $(master_rc) for a column in unit $(candidate.unit) -- the pricer " *
            "and master formulations have drifted apart",
        )
        push!(columns, column)
    end
    return columns
end

# ── formulation-level hooks ─────────────────────────────────────────────────

"""Independent pricing units for this formulation (typically `1:n_scenarios(data)`)."""
_pricing_units(formulation::AbstractFormulation, mapping, m::JuMP.Model) =
    error("_pricing_units not implemented for $(typeof(formulation))")

"""
    _pricing_build_unit_context(formulation, mapping, unit, m, duals)
        -> Union{Nothing, Tuple{AbstractPricingSearchContext, AbstractVector}}

Build the search context and existing-column pool for one unit, or `nothing`
to skip a unit with nothing to price (e.g. no active pairs/candidates)."""
_pricing_build_unit_context(formulation::AbstractFormulation, mapping, unit, m::JuMP.Model, duals) =
    error("_pricing_build_unit_context not implemented for $(typeof(formulation))")

"""Whether units may be priced concurrently (`Threads.@threads`). Default: sequential."""
_pricing_parallel_units(::AbstractFormulation)::Bool = false

"""Starting id for freshly materialized columns. Default: `1` (a throwaway
placeholder -- correct for formulations whose incremental column-adder mints
its own id by content signature, e.g. `AggregateODRouteBaseFormulation`)."""
_pricing_next_column_id(::AbstractFormulation, mapping, m::JuMP.Model)::Int = 1

"""Merge/order/budget candidates across all units before materialization.
Default: identity (no cross-unit budget, order doesn't matter)."""
_pricing_merge_units(::AbstractFormulation, mapping, candidates::AbstractVector, max_new_columns::Int) = candidates

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
