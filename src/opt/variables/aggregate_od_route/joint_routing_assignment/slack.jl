"""
Unserved-demand-group slack variable for the joint routing+assignment CG master, built
directly off `AggregateODRouteMap` (`data/maps/aggregate_od_route_map.jl`).

Demand groups are keyed by `(s, o, d)::NTuple{3,Int}` throughout (one per scenario-demand
group with positive `Q_s[s][(o,d)]`), not by a synthetic passenger id.
"""

export add_joint_routing_assignment_slack_variables!

"""
    add_joint_routing_assignment_slack_variables!(m, data, mapping) -> Dict{NTuple{3,Int}, VariableRef}

Per-demand-group unserved slack `v[(s,o,d)] >= 0` -- see
`set_joint_routing_assignment_objective!` for why this exists (RMP feasibility from an
empty column pool).
"""
function add_joint_routing_assignment_slack_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Dict{NTuple{3, Int}, VariableRef}
    v = Dict{NTuple{3, Int}, VariableRef}()
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            v[(s, o, d)] = @variable(m, lower_bound = 0.0, base_name = "v[$s,$o,$d]")
        end
    end
    m[:joint_routing_assignment_v] = v
    return v
end
