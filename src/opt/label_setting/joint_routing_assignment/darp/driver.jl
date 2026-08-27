"""
Standalone driver: exercises `hooks.jl`'s hooks directly against
`_run_label_setting` (`engine.jl`), bypassing `round.jl`'s CG hub entirely --
`joint_routing_assignment_pricing_by_darp_label_setting` below is the same
driver shape `darp_modified/driver.jl`'s twin uses. Exists so `darp/` can be
benchmarked/compared against `exact/`/`darp_modified/` directly (no live
master model/duals needed for a bare comparison run), not as a production
entrypoint -- `AggregateODRouteJointRoutingAssignmentFormulation`'s `CGSolver`
build always goes through `round.jl`.
"""

export joint_routing_assignment_pricing_by_darp_label_setting

"""
    joint_routing_assignment_pricing_by_darp_label_setting(pricing_data, existing_columns;
        next_column_id=1, max_new_columns=typemax(Int)÷2, n_candidates=typemax(Int)÷2,
        time_limit=30.0, reduced_cost_tol=1e-6, profile=false) -> (columns, exhausted, stats)

Run this pricer's label search to completion against one scenario's existing
column pool and return improving columns -- same accept/dedupe + harvest
shape as `darp_modified/driver.jl`'s twin, standalone (no master
model/duals cross-check).
"""
function joint_routing_assignment_pricing_by_darp_label_setting(
    pricing_data::JointRoutingAssignmentDarpPricingData,
    existing_columns::AbstractVector{JointRoutingAssignmentRouteColumn};
    next_column_id::Int=1,
    max_new_columns::Int=typemax(Int) ÷ 2,
    n_candidates::Int=typemax(Int) ÷ 2,
    time_limit::Float64=30.0,
    reduced_cost_tol::Float64=1e-6,
    profile::Bool=false,
)
    ctx = JointRoutingAssignmentDarpSearchContext(pricing_data)

    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        signature = _pricing_pool_signature(ctx, column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    scored = Dict{Any, Any}()
    function accept!(label)
        candidate = _pricing_candidate_from_label(ctx, label)
        isnothing(candidate) && return false
        candidate.reduced_cost < -reduced_cost_tol || return false
        candidate.tau < get(best_pool_tau, candidate.signature, Inf) - 1e-9 || return false
        current = get(scored, candidate.signature, nothing)
        if isnothing(current) ||
                candidate.reduced_cost < current.reduced_cost - 1e-9 ||
                (abs(candidate.reduced_cost - current.reduced_cost) <= 1e-9 && candidate.tau < current.tau - 1e-9)
            scored[candidate.signature] = candidate
        end
        return length(scored) >= n_candidates
    end

    _labels, exhausted, stats = _run_label_setting(
        ctx; time_limit=time_limit, reduced_cost_tol=reduced_cost_tol, profile=profile, stop_if=accept!,
    )

    sorted = sort!(collect(values(scored)); by=c -> (c.reduced_cost, c.tau))
    truncated = sorted[1:min(length(sorted), max_new_columns)]
    columns = JointRoutingAssignmentRouteColumn[
        _pricing_make_column(ctx, next_column_id + offset - 1, candidate)
        for (offset, candidate) in enumerate(truncated)
    ]
    return columns, exhausted, stats
end
