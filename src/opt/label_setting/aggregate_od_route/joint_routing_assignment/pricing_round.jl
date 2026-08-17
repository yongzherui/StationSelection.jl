"""
`AggregateODRouteJointRoutingAssignmentFormulation`'s formulation-level hooks
into `_run_pricing_round` (`round.jl`): one unit per scenario, threaded
(scenarios are independent and pricing touches no Gurobi state, so concurrent
search is safe), a real starting column id (unlike Base, the pricer-assigned
id *is* the master's real key here), and a cross-scenario merge that sorts
and imposes one global `max_new_columns` budget so thread completion order
can't decide which columns enter the RMP.
"""

"""
    joint_routing_assignment_pricing_candidates(data, mapping, alpha, gamma_o, gamma_d, walk_cost_weight, detour_factor, scenario)
        -> Vector{PassengerAssignmentCandidate}

Turn the current RMP duals into `PassengerAssignmentCandidate`s for one scenario,
directly off `AggregateODRouteMap` -- no `MasterData`. The search's `p::Int` field is
the demand group's index, the position of `(o,d)` within `mapping.Omega_s[scenario]`
(matching the aggregate model's own convention); it's only ever compared for equality
within one scenario's pricing call, so reusing the same small dense range each scenario
is fine. `WALK_ONLY_PAIR` is excluded: a route can't certify it (see
`add_walk_variables!`'s docstring). Only `rho > 0` survives
(the pricer's reward-layer preprocessing drops the rest anyway).
"""
# ── candidate extraction from RMP duals ─────────────────────────────────────
function joint_routing_assignment_pricing_candidates(
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    alpha::Dict{Tuple{Int, Int}, Float64},
    gamma_o::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
    gamma_d::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
    walk_cost_weight::Float64,
    detour_factor::Float64,
    scenario::Int,
)::Vector{PassengerAssignmentCandidate}
    candidates = PassengerAssignmentCandidate[]
    for (p, (o, d)) in enumerate(mapping.Omega_s[scenario])
        mapping.Q_s[scenario][p] > 0 || continue
        key2 = (scenario, p)
        a = get(alpha, key2, 0.0)
        a > 1e-9 || continue
        for pair in get_valid_jk_pairs(mapping, o, d)
            is_walk_only_pair(pair) && continue
            j, k = pair
            rho = a - get(gamma_o, (key2, j), 0.0) - get(gamma_d, (key2, k), 0.0) -
                walk_cost_weight * od_pair_walking_cost(data, o, d, pair)
            rho > 1e-9 || continue
            ride_limit = detour_factor * get_routing_cost(data, j, k)
            push!(candidates, PassengerAssignmentCandidate(p, j, k, ride_limit, rho))
        end
    end
    return candidates
end

# ── formulation-level hooks (units / threading / column ids / unit context / merge) ──
_pricing_units(::AggregateODRouteJointRoutingAssignmentFormulation, mapping::AggregateODRouteMap, m::JuMP.Model) =
    1:n_scenarios(m[:joint_routing_assignment_data])

_pricing_parallel_units(::AggregateODRouteJointRoutingAssignmentFormulation) = true

_pricing_next_column_id(::AggregateODRouteJointRoutingAssignmentFormulation, mapping::AggregateODRouteMap, m::JuMP.Model) =
    maximum(keys(m[:joint_routing_assignment_columns]); init=0) + 1

"""
Build one scenario's pricing context: duals -> candidates -> reward-layer
pricing data -> search context, plus the existing column pool restricted to
this scenario. `nothing` when a scenario has no positive-reward candidates
(and therefore no opportunities) to price at all.
"""
function _pricing_build_unit_context(
    ::AggregateODRouteJointRoutingAssignmentFormulation, mapping::AggregateODRouteMap, s::Int,
    m::JuMP.Model, duals,
)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    walk_cost_weight = Float64(m[:joint_routing_assignment_walk_cost_weight])
    detour_factor = Float64(m[:joint_routing_assignment_detour_factor])
    candidates = joint_routing_assignment_pricing_candidates(
        data, mapping, alpha, gamma_o, gamma_d, walk_cost_weight, detour_factor, s,
    )
    isempty(candidates) && return nothing

    pricing_data = create_joint_routing_assignment_pricing_data(
        s, m[:joint_routing_assignment_nodes], m[:joint_routing_assignment_travel_cost], candidates;
        route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
        max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
        repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
        max_stops=Int(m[:joint_routing_assignment_max_stops]),
    )
    isempty(pricing_data.opportunities) && return nothing

    existing = JointRoutingAssignmentRouteColumn[
        c for c in values(m[:joint_routing_assignment_columns]) if Int(get(c.metadata, "scenario", 0)) == s
    ]

    return JointRoutingAssignmentSearchContext(pricing_data), existing
end

"""
Sort by `(reduced_cost, tau, scenario, route)` and truncate to one global
`max_new_columns` across every scenario -- the one real behavioral difference
from `AggregateODRouteBaseFormulation`'s `_pricing_merge_units` (`round.jl`'s
default, unbounded identity): scenarios are threaded here, so without a
deterministic global order thread completion order would decide which
columns enter the RMP.
"""
function _pricing_merge_units(
    ::AggregateODRouteJointRoutingAssignmentFormulation, mapping::AggregateODRouteMap,
    candidates::AbstractVector, max_new_columns::Int,
)
    sorted = _sort_pricing_results_by_route(
        candidates, entry -> (entry.reduced_cost, entry.tau, entry.unit, string(entry.payload.route)),
    )
    return sorted[1:min(length(sorted), max_new_columns)]
end
