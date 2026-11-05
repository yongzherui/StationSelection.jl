"""
Coverage constraints for the joint routing+assignment CG master, built directly off
`AggregateODRouteMap`.
"""

export add_joint_routing_assignment_coverage_constraints!

"""
    add_joint_routing_assignment_coverage_constraints!(m, data, mapping, x_walk;
        scenarios=1:n_scenarios(data)) -> Dict{Tuple{Int,Int}, ConstraintRef}

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

`scenarios` restricts row creation to a subset, mirroring `add_walk_variables!`'s own
kwarg of the same name (and defaulting the same way, so every existing caller is
unchanged): `AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation`'s build
(`optimize/aggregate_od_route/benders/build_subproblem.jl`) makes one model per scenario,
since the joint model's second stage separates exactly by scenario. Pass the same
`scenarios` here as to `add_walk_variables!` -- a coverage row whose group has no
`x_walk` variable because that scenario was excluded there would be the placeholder
`0 >= 1`, i.e. an instantly infeasible model.
"""
function add_joint_routing_assignment_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    x_walk::Dict{Tuple{Int, Int}, VariableRef};
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
)::Dict{Tuple{Int, Int}, ConstraintRef}
    coverage = Dict{Tuple{Int, Int}, ConstraintRef}()
    for s in scenarios
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

export add_joint_routing_assignment_unserved_slack!

"""
    add_joint_routing_assignment_unserved_slack!(m, mapping, coverage, penalty)
        -> Dict{Tuple{Int,Int}, VariableRef}

The `unserved_penalty` variant (see `AggregateODRouteJointRoutingAssignmentFormulation`):
one continuous `u[(s,p)] >= 0` per coverage row, entered into that row with coefficient 1
and into the objective at `penalty * Q_s[s][p]`. Continuous even in the integer-recovery
MIP: with `theta`/`x_walk` binary, `u` settles at 0 or 1 at any optimum, since a partial
slack only pays for coverage the row does not need. Call after the objective is set.

The coverage dual of a row is then capped at `penalty * Q`, so the pricer never values a
group above what leaving it unserved costs.
"""
function add_joint_routing_assignment_unserved_slack!(
    m::Model,
    mapping::AggregateODRouteMap,
    coverage::Dict{Tuple{Int, Int}, ConstraintRef},
    penalty::Float64,
)::Dict{Tuple{Int, Int}, VariableRef}
    isfinite(penalty) && penalty > 0 || throw(ArgumentError("unserved slack needs a finite positive penalty, got $penalty"))
    unserved = Dict{Tuple{Int, Int}, VariableRef}()
    for (key2, con) in coverage
        s, p = key2
        u = @variable(m, lower_bound = 0.0, base_name = "u_unserved[$s,$p]")
        set_normalized_coefficient(con, u, 1.0)
        set_objective_coefficient(m, u, penalty * mapping.Q_s[s][p])
        unserved[key2] = u
    end
    return unserved
end
