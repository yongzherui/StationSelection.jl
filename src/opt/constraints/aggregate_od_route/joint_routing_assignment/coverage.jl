"""
Coverage constraints for the joint routing+assignment CG master, built directly off
`AggregateODRouteMap`.
"""

export add_joint_routing_assignment_coverage_constraints!

"""
    add_joint_routing_assignment_coverage_constraints!(m, data, mapping, v) -> Dict{NTuple{3,Int}, ConstraintRef}

One row per demand-positive `(s,o,d)`, `v[(s,o,d)] >= 1` -- route-column and `x_same`
coefficients are added later, by `add_joint_routing_assignment_same_station_variables!`
and `add_joint_routing_assignment_column!` respectively.
"""
function add_joint_routing_assignment_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    v::Dict{NTuple{3, Int}, VariableRef},
)::Dict{NTuple{3, Int}, ConstraintRef}
    coverage = Dict{NTuple{3, Int}, ConstraintRef}()
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            key3 = (s, o, d)
            coverage[key3] = @constraint(m, v[key3] >= 1)
        end
    end
    return coverage
end
