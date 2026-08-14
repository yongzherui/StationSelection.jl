"""
Coverage constraints for the joint routing+assignment CG master, built directly off
`AggregateODRouteMap`.
"""

export add_joint_routing_assignment_coverage_constraints!

"""
    add_joint_routing_assignment_coverage_constraints!(m, data, mapping, v) -> Dict{Tuple{Int,Int}, ConstraintRef}

One row per demand group `(s,p)`, `v[(s,p)] >= 1` -- route-column and `x_same`
coefficients are added later, by `add_joint_routing_assignment_same_station_variables!`
and `add_joint_routing_assignment_column!` respectively.
"""
function add_joint_routing_assignment_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    v::Dict{Tuple{Int, Int}, VariableRef},
)::Dict{Tuple{Int, Int}, ConstraintRef}
    coverage = Dict{Tuple{Int, Int}, ConstraintRef}()
    for s in 1:n_scenarios(data)
        for p in eachindex(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            key2 = (s, p)
            coverage[key2] = @constraint(m, v[key2] >= 1)
        end
    end
    return coverage
end
