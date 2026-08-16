"""
Coverage constraints for `AggregateODRouteBaseFormulation`'s `y`/`x`/`θ` master.
"""

export add_aggregate_od_route_base_coverage_constraints!

"""
    add_aggregate_od_route_base_coverage_constraints!(m, data, mapping, x, x_walk; scenarios=1:n_scenarios(data)) -> Dict{Tuple{Int,Int}, ConstraintRef}

One row per demand group `(s,p)` with `s in scenarios`: exactly one feasible assignment,
`sum_{(j,k)} x[s,p,j,k] + x_walk[s,p] == 1` (the `x_walk` term omitted when no such key
exists, i.e. direct walking isn't available for that OD). `scenarios` mirrors
`add_assignment_variables!`'s own kwarg -- used by
`AggregateODRouteBendersYXFormulation`'s per-scenario subproblem.

Every row is guaranteed at least one term: `build_model` calls
`aggregate_od_route_validate_feasible_coverage` before any variable exists, which proves
every positive-demand `(s,p)` has either a real `(j,k)` with finite routing cost (so
`x[s,p,j,k]` exists) or `WALK_ONLY_PAIR` (so `x_walk[s,p]` exists).
"""
function add_aggregate_od_route_base_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    x::Dict{NTuple{4, Int}, VariableRef},
    x_walk::Dict{Tuple{Int, Int}, VariableRef};
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
)::Dict{Tuple{Int, Int}, ConstraintRef}
    coverage = Dict{Tuple{Int, Int}, ConstraintRef}()
    for s in scenarios
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            terms = AffExpr(0.0)
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, k = pair
                haskey(x, (s, p, j, k)) || continue
                add_to_expression!(terms, x[(s, p, j, k)])
            end
            haskey(x_walk, (s, p)) && add_to_expression!(terms, x_walk[(s, p)])
            coverage[(s, p)] = @constraint(m, terms == 1)
        end
    end
    return coverage
end
