"""
Coverage constraints for `AggregateODRouteBaseFormulation`'s `y`/`x`/`θ` master.
"""

export add_aggregate_od_route_base_coverage_constraints!

"""
    add_aggregate_od_route_base_coverage_constraints!(m, data, mapping, x; scenarios=1:n_scenarios(data)) -> Dict{NTuple{3,Int}, ConstraintRef}

One row per demand-positive `(s,o,d)` with `s in scenarios`: exactly one feasible
`(j,k)` assignment, `sum_{(j,k)} x[s,o,d,j,k] == 1`. `scenarios` mirrors
`add_aggregate_od_route_base_assignment_variables!`'s own kwarg -- used by
`AggregateODRouteBendersYXFormulation`'s per-scenario subproblem.
"""
function add_aggregate_od_route_base_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    x::Dict{NTuple{5, Int}, VariableRef};
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
)::Dict{NTuple{3, Int}, ConstraintRef}
    coverage = Dict{NTuple{3, Int}, ConstraintRef}()
    for s in scenarios
        for (o, d) in mapping.Omega_s[s]
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            terms = AffExpr(0.0)
            found = false
            for pair in get_valid_jk_pairs(mapping, o, d)
                requires_no_vehicle_route(pair) && continue
                j, k = pair
                haskey(x, (s, o, d, j, k)) || continue
                add_to_expression!(terms, x[(s, o, d, j, k)])
                found = true
            end
            found || continue
            coverage[(s, o, d)] = @constraint(m, terms == 1)
        end
    end
    return coverage
end
