"""
All wiring, no logic: every method that plugs `JointRoutingAssignmentSearchContext`
(`context.jl`) into the two generic hook contracts this pricer implements, and
nothing else. Two contracts, both forwarded from here:

  - the twelve `AbstractPricingSearchContext` hooks (`../../types.jl`) that
    `_run_label_setting` (`engine.jl`) calls during the search itself --
    forwarded to `seed.jl` / `extend.jl` / `prune.jl` / `dominate.jl`;
  - the four context-level hooks `round.jl` calls once per surviving label to
    harvest it into a column -- forwarded to `accept.jl`'s route replay.

Every method below is either a one-line forward or a thin adapter shaping a
sibling file's return value into what the hook contract expects
(`_pricing_candidate_from_label`, `_pricing_make_column`) -- chase the hook
you care about into the sibling file it forwards to; this file answers "what
hook does X" and "who calls Y", not "how does Y work". Needs
`JointRoutingAssignmentSearchContext` (`context.jl`) for every method's `ctx`
argument, so loads after it.
"""

# ── AbstractPricingSearchContext hooks (engine.jl / label_setting/types.jl) ──
_pricing_initial_labels(ctx::JointRoutingAssignmentSearchContext) =
    initial_joint_routing_assignment_pricing_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel) =
    _make_joint_routing_assignment_label_bitsets(label, ctx.search_index.node_index, ctx.n_nodes)

_pricing_state(::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel, ::JointRoutingAssignmentLabelBitsets) =
    _joint_routing_assignment_state(label)

function _pricing_label_priority(
    ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel, label_bs::JointRoutingAssignmentLabelBitsets,
)::Float64
    return label.reduced_cost -
        _joint_routing_assignment_remaining_reward_bound(label, label_bs, ctx.pricing_data, ctx.search_index, ctx.bound_workspace)
end

_pricing_best_signature(::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel) =
    isempty(label.activated_reward_layers) ? nothing : _joint_routing_assignment_layer_signature(label)

_pricing_route_length(::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel) = label.route_length

_pricing_max_route_length(ctx::JointRoutingAssignmentSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel) =
    _joint_routing_assignment_candidate_next_nodes(label, ctx.pricing_data)

# One child per stop, returned directly: see `_extend_joint_routing_assignment_pricing_label`.
_pricing_extend_label(ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel, next_node::Int) =
    _extend_joint_routing_assignment_pricing_label(label, next_node, ctx.pricing_data)

_pricing_dominates_fn(ctx::JointRoutingAssignmentSearchContext) = ctx.dominates

function _pricing_search_started!(::JointRoutingAssignmentSearchContext, ::Float64, ::Float64)
    return nothing
end

function _pricing_on_label_inserted(ctx::JointRoutingAssignmentSearchContext, label)
    isnothing(ctx.label_observer) || ctx.label_observer(label)
    return nothing
end

# ── round.jl context-level hooks (candidate → column → master verification) ──
"""
Route-replay (`accept.jl`) is what makes this pricer's `_pricing_candidate_from_label`
different from the trivial projection every other pricer uses: the label only
tracks a cheap reward-layer proxy during search (section 16), so the real,
concrete `(p, j, k)` assignments -- and the real dedup signature over them,
not the search's proxy signature (`_joint_routing_assignment_layer_signature`,
`dominate.jl`) -- only exist after replaying the finished route.
`signature`/`tau`/`reduced_cost` below are the *replayed*, exact values;
`payload` carries what `_pricing_make_column` needs to build the column.
"""
function _pricing_candidate_from_label(ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel)
    assignments, tau, reduced_cost = _joint_routing_assignment_column_from_route(
        label.route, ctx.pricing_data; label_reduced_cost=label.reduced_cost,
    )
    isempty(assignments) && return nothing
    return (
        signature=_joint_routing_assignment_column_signature(assignments),
        tau=tau, reduced_cost=reduced_cost, payload=(route=label.route, assignments=assignments),
    )
end

_pricing_pool_signature(::JointRoutingAssignmentSearchContext, existing_column::JointRoutingAssignmentRouteColumn) =
    _joint_routing_assignment_column_signature(existing_column)

_pricing_make_column(ctx::JointRoutingAssignmentSearchContext, column_id::Int, candidate) =
    JointRoutingAssignmentRouteColumn(
        column_id, candidate.payload.route, candidate.payload.assignments, candidate.tau;
        metadata=Dict{String, Any}(
            "scenario" => ctx.pricing_data.scenario,
            "route" => Tuple(candidate.payload.route),
            "reduced_cost" => candidate.reduced_cost,
        ),
    )

"""
Unlike Base's single-constraint-family check (`route_covering/exact/hooks.jl`),
Joint's master reduced cost sums the coverage row plus both linking-row
families per assignment (`_verify_joint_routing_assignment_master_reduced_cost`,
`../duals.jl`) -- so this hook, unlike the aggregate pricers' twins, genuinely
needs `m`/`mapping`/`duals`, not just `ctx`."""
function _pricing_verify_column(::JointRoutingAssignmentSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end
