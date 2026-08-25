"""
`JointRoutingAssignmentStationSimpleSearchContext`: the elementary-route
passenger free-assignment pricer's plug into the shared search loop
(`_run_label_setting`, `engine.jl`), mirroring `JointRoutingAssignmentSearchContext`'s
hook set (`../exact/exact.jl`) one for one. See `labels.jl` for the label-DP
primitives (candidate generation, extension, dominance) and `types.jl` for the
underlying label/bitsets/dominance types.
"""

"""
Context for the elementary-route passenger free-assignment search: bundles
`pricing_data`, `dominance_mode` (`:exact` states on `(current, visited)`;
`:subset` states on `current` alone, pairing it with a shared `empty_visited`
so both modes share one concrete state type), the once-built `dominates`
closure, and the `search_index`/`bound_workspace` the shared remaining-reward
bound needs. Plugs into `_run_label_setting` (`engine.jl`) the same way
`JointRoutingAssignmentSearchContext` does (`../exact/exact.jl`). Not currently
reachable from `joint_routing_assignment/exact/pricing_round.jl`'s
`_pricing_build_scenario_context` (always builds the revisit-tolerant context
today) -- kept as a real, independently usable capability.
"""
struct JointRoutingAssignmentStationSimpleSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentStationSimpleDominanceFilters, JointRoutingAssignmentStationSimpleLabel, JointRoutingAssignmentStationSimpleAges,
    Tuple{Int, BitSet}, RewardLayerBitset,
}
    pricing_data::JointRoutingAssignmentPricingData
    dominance_mode::Symbol
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    node_index::Dict{Int, Int}
    empty_visited::BitSet
end

function JointRoutingAssignmentStationSimpleSearchContext(
    pricing_data::JointRoutingAssignmentPricingData; dominance_mode::Symbol=:exact,
)
    dominance_mode in (:subset, :exact) ||
        throw(ArgumentError("dominance_mode must be :subset or :exact, got $(dominance_mode)"))
    rules = JointRoutingAssignmentStationSimpleDominanceRules()
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.label, x.bitsets, y.filters, y.label, y.bitsets, pricing_data.layer_weight, rules,
    )
    search_index = _build_joint_routing_assignment_search_index(pricing_data)
    bound_workspace = _create_joint_routing_assignment_bound_workspace()
    return JointRoutingAssignmentStationSimpleSearchContext(
        pricing_data, dominance_mode, dominates, search_index, bound_workspace, search_index.node_index, BitSet(),
    )
end

_pricing_initial_labels(ctx::JointRoutingAssignmentStationSimpleSearchContext) =
    _initial_joint_routing_assignment_station_simple_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) =
    _make_joint_routing_assignment_station_simple_ages(label, ctx.node_index)

# `:subset` states on `current` alone; pairing it with the shared `empty_visited`
# keeps a single concrete key type for both modes.
_pricing_state(
    ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel, ::JointRoutingAssignmentStationSimpleAges,
) = ctx.dominance_mode === :exact ? (label.current, label.visited) : (label.current, ctx.empty_visited)

# The reward bound reads only `current`/`time`/`activated_reward_layers` from the
# label and `age_idx`/`age_val` from the ages mirror, so the revisit-tolerant
# pricer's bound (`../exact/exact.jl`) applies unchanged here.
_pricing_label_priority(
    ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel, label_ages::JointRoutingAssignmentStationSimpleAges,
) = label.reduced_cost -
    _joint_routing_assignment_remaining_reward_bound(label, label_ages, ctx.pricing_data, ctx.search_index, ctx.bound_workspace)

_pricing_best_signature(::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) =
    isempty(label.activated_reward_layers) ? nothing : label.activated_reward_layers

_pricing_route_length(::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) = label.route_length

_pricing_max_route_length(ctx::JointRoutingAssignmentStationSimpleSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) =
    _joint_routing_assignment_station_simple_candidate_next_nodes(label, ctx.pricing_data)

_pricing_extend_label(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel, next_node::Int) =
    _extend_joint_routing_assignment_station_simple_label(label, next_node, ctx.pricing_data)

_pricing_dominates_fn(ctx::JointRoutingAssignmentStationSimpleSearchContext) = ctx.dominates

"""
Elementary-route counterpart of the revisit-tolerant pricer's
`_pricing_candidate_from_label` (`../exact/exact.jl`): identical route-replay, since
replay is agnostic to how the physical route was found."""
function _pricing_candidate_from_label(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel)
    assignments, tau, reduced_cost = _joint_routing_assignment_column_from_route(
        label.route, ctx.pricing_data; label_reduced_cost=label.reduced_cost,
    )
    isempty(assignments) && return nothing
    return (
        signature=_joint_routing_assignment_column_signature(assignments),
        tau=tau, reduced_cost=reduced_cost, payload=(route=label.route, assignments=assignments),
    )
end

_pricing_pool_signature(::JointRoutingAssignmentStationSimpleSearchContext, existing_column::JointRoutingAssignmentRouteColumn) =
    _joint_routing_assignment_column_signature(existing_column)

_pricing_make_column(ctx::JointRoutingAssignmentStationSimpleSearchContext, column_id::Int, candidate) =
    JointRoutingAssignmentRouteColumn(
        column_id, candidate.payload.route, candidate.payload.assignments, candidate.tau;
        metadata=Dict{String, Any}(
            "scenario" => ctx.pricing_data.scenario,
            "route" => Tuple(candidate.payload.route),
            "reduced_cost" => candidate.reduced_cost,
        ),
    )

"""Same master reduced-cost cross-check as the revisit-tolerant context's twin
(`../exact/exact.jl`) -- the master doesn't know which route universe a column came
from, only the assignments it carries."""
function _pricing_verify_column(::JointRoutingAssignmentStationSimpleSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end
