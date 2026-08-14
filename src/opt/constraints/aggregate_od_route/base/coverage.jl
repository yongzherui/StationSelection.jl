"""
Coverage constraints for `AggregateODRouteBaseFormulation`'s `y`/`x`/`θ` master.
"""

export add_aggregate_od_route_base_coverage_constraints!

"""
    add_aggregate_od_route_base_coverage_constraints!(m, data, mapping, x; scenarios=1:n_scenarios(data)) -> Dict{Tuple{Int,Int}, ConstraintRef}

One row per demand group `(s,p)` with `s in scenarios`: exactly one feasible `(j,k)`
assignment, `sum_{(j,k)} x[s,p,j,k] == 1`. `scenarios` mirrors
`add_aggregate_od_route_base_assignment_variables!`'s own kwarg -- used by
`AggregateODRouteBendersYXFormulation`'s per-scenario subproblem.
"""
function add_aggregate_od_route_base_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    x::Dict{NTuple{4, Int}, VariableRef};
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
)::Dict{Tuple{Int, Int}, ConstraintRef}
    coverage = Dict{Tuple{Int, Int}, ConstraintRef}()
    for s in scenarios
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            terms = AffExpr(0.0)
            found = false
            for pair in get_valid_jk_pairs(mapping, o, d)
                requires_no_vehicle_route(pair) && continue
                j, k = pair
                haskey(x, (s, p, j, k)) || continue
                add_to_expression!(terms, x[(s, p, j, k)])
                found = true
            end
            found || continue
            coverage[(s, p)] = @constraint(m, terms == 1)
        end
    end
    return coverage
end
