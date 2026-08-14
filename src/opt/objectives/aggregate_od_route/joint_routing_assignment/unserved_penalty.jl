"""
Unserved-demand-group penalty sizing for the joint routing+assignment CG master.
"""

export default_joint_routing_assignment_unserved_penalty

"""
    default_joint_routing_assignment_unserved_penalty(problem, data, mapping) -> Float64

A penalty strictly worse than serving any single demand group by its most expensive
possible route, so `v[(s,p)] > 0` at the optimum means "genuinely unservable", never
"cheaper to abandon". Serving one demand group never requires more than a direct
two-stop route `[j,k]` (the cheapest route certifying `(j,k)`), so
`route_regularization_weight*(max_travel + repositioning_time)` plus the worst walking
cost already exceeds any single demand group's true service cost; 10x that is a safe
big-M. Deliberately independent of `max_stops`, so penalty magnitude doesn't shift
between an uncapped and a capped run (which would make their objectives incomparable
whenever slack is active in only one of them).
"""
function _default_joint_routing_assignment_unserved_penalty_core(
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        route_regularization_weight::Float64,
        repositioning_time::Float64,
        walk_cost_weight::Float64,
    )::Float64
    n = data.n_stations
    max_travel = 0.0
    for i in 1:n, j in 1:n
        i == j && continue
        cost = get_routing_cost(data, i, j)
        isfinite(cost) && (max_travel = max(max_travel, cost))
    end
    max_walk = 0.0
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                max_walk = max(max_walk, od_pair_walking_cost(data, o, d, pair))
            end
        end
    end
    worst_route = route_regularization_weight * (max_travel + repositioning_time)
    worst_walk = walk_cost_weight * max_walk
    return 10.0 * (worst_route + worst_walk) + 1.0
end

"""
    default_joint_routing_assignment_unserved_penalty(problem::StationSelectionProblem,
        formulation, data, mapping) -> Float64
"""
function default_joint_routing_assignment_unserved_penalty(
    problem::StationSelectionProblem,
    formulation::AnyAggregateODRouteFormulation,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Float64
    return _default_joint_routing_assignment_unserved_penalty_core(
        data, mapping,
        formulation.route_regularization_weight, formulation.repositioning_time,
        formulation.walk_cost_weight,
    )
end
