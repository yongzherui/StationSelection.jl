"""
All wiring, no logic: every method that plugs
`JointRoutingAssignmentStationSimpleSearchContext` (`context.jl`) into the
two generic hook contracts this pricer implements, mirroring
`../exact/hooks.jl`'s hook set one for one. Two contracts, both forwarded
from here:

  - the twelve `AbstractPricingSearchContext` hooks (`../../types.jl`) that
    `_run_label_setting` (`engine.jl`) calls during the search itself --
    forwarded to `seed.jl` / `extend.jl` / `dominate.jl`, with the
    remaining-reward bound (`_pricing_label_priority`) and the round-level
    replay (below) reused directly from `../exact/prune.jl` /
    `../exact/accept.jl` -- this pricer has no `prune.jl`/`accept.jl` of its
    own since replay is agnostic to how the physical route was found;
  - the four context-level hooks `round.jl` calls once per surviving label to
    harvest it into a column.
"""

# ── AbstractPricingSearchContext hooks (engine.jl / label_setting/types.jl) ──
_pricing_initial_labels(ctx::JointRoutingAssignmentStationSimpleSearchContext) =
    _initial_joint_routing_assignment_station_simple_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) =
    _make_joint_routing_assignment_station_simple_ages(label, ctx.node_index)

# States on `current` alone; the elementarity resource `U_a ⊆ U_b` is enforced
# inside the dominance predicate itself (`dominate.jl`), not the state key.
_pricing_state(
    ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel, ::JointRoutingAssignmentStationSimpleAges,
) = label.current

# The reward bound reads only `current`/`time`/`activated_reward_layers` from the
# label and `age_idx`/`age_val` from the ages mirror, so the revisit-tolerant
# pricer's bound (`../exact/prune.jl`) applies unchanged here.
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

# ── round.jl context-level hooks (candidate → column → master verification) ──
"""
Elementary-route counterpart of the revisit-tolerant pricer's
`_pricing_candidate_from_label` (`../exact/hooks.jl`): identical route-replay, since
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
(`../exact/hooks.jl`) -- the master doesn't know which route universe a column came
from, only the assignments it carries."""
function _pricing_verify_column(::JointRoutingAssignmentStationSimpleSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end
