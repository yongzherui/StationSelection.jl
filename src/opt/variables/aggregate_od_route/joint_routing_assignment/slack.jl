"""
Unserved-demand-group slack variable for the joint routing+assignment CG master, built
directly off `AggregateODRouteMap` (`data/maps/aggregate_od_route_map.jl`).

Demand groups are keyed by `(s, p)::Tuple{Int,Int}` throughout (one per scenario-demand
group with positive `Q_s[s][p]`, `p` being the position of `(o,d)` within
`mapping.Omega_s[s]`), not by a synthetic passenger id.
"""

export add_joint_routing_assignment_slack_variables!

"""
    add_joint_routing_assignment_slack_variables!(m, data, mapping) -> Dict{Tuple{Int,Int}, VariableRef}

Per-demand-group unserved slack `v[(s,p)] >= 0` -- see
`set_joint_routing_assignment_objective!` for why this exists (RMP feasibility from an
empty column pool).
"""
function add_joint_routing_assignment_slack_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Dict{Tuple{Int, Int}, VariableRef}
    v = Dict{Tuple{Int, Int}, VariableRef}()
    for s in 1:n_scenarios(data)
        for p in eachindex(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            v[(s, p)] = @variable(m, lower_bound = 0.0, base_name = "v[$s,$p]")
        end
    end
    m[:joint_routing_assignment_v] = v
    return v
end
