"""
Coverage constraints for the joint routing+assignment CG master, built directly off
`AggregateODRouteMap`.
"""

export add_joint_routing_assignment_coverage_constraints!

"""
    add_joint_routing_assignment_coverage_constraints!(m, data, mapping, x_walk) -> Dict{Tuple{Int,Int}, ConstraintRef}

One row per demand group `(s,p)`: `x_walk[(s,p)] >= 1` when a direct-walk option exists
for that group (mirrors `add_aggregate_od_route_base_coverage_constraints!`'s own
`x_walk` term), else the placeholder `0 >= 1`. Route-column (`theta`) coefficients are
added later, by `add_joint_routing_assignment_column!` -- unlike `x_walk`, `theta` columns
are discovered dynamically by CG, so their rows can't be written as a closed-form
expression up front and instead get their coefficients patched in via
`set_normalized_coefficient` on top of the row built here. There is no slack variable:
`build_model` calls `aggregate_od_route_validate_feasible_coverage` before this, and
always seeds every two-stop route right after, so every row is guaranteed to pick up at
least one term before the model is solved -- see that function's docstring for why an
uncovered row is a build-time error rather than a runtime slack.
"""
function add_joint_routing_assignment_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    x_walk::Dict{Tuple{Int, Int}, VariableRef},
)::Dict{Tuple{Int, Int}, ConstraintRef}
    coverage = Dict{Tuple{Int, Int}, ConstraintRef}()
    for s in 1:n_scenarios(data)
        for p in eachindex(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            key2 = (s, p)
            terms = AffExpr(0.0)
            haskey(x_walk, key2) && add_to_expression!(terms, x_walk[key2])
            coverage[key2] = @constraint(m, terms >= 1)
        end
    end
    return coverage
end
