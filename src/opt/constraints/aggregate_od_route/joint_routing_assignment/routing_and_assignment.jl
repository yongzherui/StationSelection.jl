"""
Route-column (theta) machinery for the joint routing+assignment CG master: cost, the
incremental column-adder, and its `add_columns!` CGSolver hook. Column theta variables
carry routing and assignment jointly (no separate `x`), hence this file's name.

`column`'s type annotation is intentionally omitted below: `JointRoutingAssignmentRouteColumn`
is defined in `label_setting/joint_routing_assignment/types.jl`, included well
after `opt/constraints.jl` (see `src/StationSelection.jl`'s include order) -- annotating it
here would be a forward type reference Julia can't resolve yet.
"""

export joint_routing_assignment_column_cost
export add_joint_routing_assignment_column!

"""
    joint_routing_assignment_column_cost(m, data, mapping, column) -> Float64

The column's true objective coefficient: `route_regularization_weight*(tau + repositioning_time)`
plus the demand-weighted walking cost of the concrete assignments it carries.
`route_regularization_weight`/`repositioning_time`/`walk_cost_weight` are read off `m`
(stashed there at build time by `build_model`), matching
`aggregate_od_route_column_objective_coefficient`'s calling convention for the other
formulation. `column.assignments` entries are `(p, j, k)`; `p` indexes
`mapping.Omega_s[column.metadata["scenario"]]`.
"""
function joint_routing_assignment_column_cost(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    column,
)::Float64
    s = Int(column.metadata["scenario"])
    omega = mapping.Omega_s[s]
    walk = 0.0
    for (p, j, k) in column.assignments
        o, d = omega[p]
        demand = mapping.Q_s[s][p]
        walk += demand * od_pair_walking_cost(data, o, d, (j, k))
    end
    beta = Float64(m[:joint_routing_assignment_route_regularization_weight])
    rho = Float64(m[:joint_routing_assignment_repositioning_time])
    w = Float64(m[:joint_routing_assignment_walk_cost_weight])
    return beta * (column.tau + rho) + w * walk
end

"""
    add_joint_routing_assignment_column!(m, data, mapping, column) -> (theta, action)

`action` is `:added`, or `:skipped` when an identical assignment signature
(`_joint_routing_assignment_column_signature`, `pricing/.../labels.jl`, unchanged) is
already in the pool at no greater `tau`. `theta`/`columns`/`column_signatures` all live
on `m[...]` (mirrors `add_aggregate_od_route_base_column!`'s
`m[:aggregate_od_route_base_theta]` convention, `constraints/aggregate_od_route/base/route_activation.jl`)
-- no separate master wrapper struct.

New `theta` variables read `m[:joint_routing_assignment_relax_integrality]` (stashed by
`_build_joint_routing_assignment_model`, `optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl`)
rather than taking the flag as an explicit argument: this function is called both from
that build path's own seeding loop and from `add_columns!` below (a fixed-signature
`CGSolver` hook mid-CG), so a single build-time flag on `m` is what keeps every column --
seeded or CG-discovered -- consistent with whichever domain the rest of that master's
variables were built with (continuous for the LP master, `Bin` for the integer-recovery
rebuild).
"""
function add_joint_routing_assignment_column!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    column,
)
    theta = m[:joint_routing_assignment_theta]
    columns = m[:joint_routing_assignment_columns]
    signatures = m[:joint_routing_assignment_column_signatures]

    signature = _joint_routing_assignment_column_signature(column)
    existing_id = get(signatures, signature, nothing)
    if !isnothing(existing_id)
        columns[existing_id].tau <= column.tau + 1e-9 && return theta[existing_id], :skipped
    end

    s = Int(column.metadata["scenario"])
    coverage = m[:joint_routing_assignment_coverage]
    pickup_link = m[:joint_routing_assignment_pickup_link]
    dropoff_link = m[:joint_routing_assignment_dropoff_link]

    relax_integrality = Bool(m[:joint_routing_assignment_relax_integrality])
    theta_var = relax_integrality ?
        @variable(m, lower_bound = 0.0, base_name = "theta[$(column.id)]") :
        @variable(m, binary = true, base_name = "theta[$(column.id)]")
    theta[column.id] = theta_var
    columns[column.id] = column
    signatures[signature] = column.id
    set_objective_coefficient(m, theta_var, joint_routing_assignment_column_cost(m, data, mapping, column))

    for (p, j, k) in column.assignments
        key2 = (s, p)
        haskey(coverage, key2) && set_normalized_coefficient(coverage[key2], theta_var, 1.0)
        haskey(pickup_link, (key2, j)) && set_normalized_coefficient(pickup_link[(key2, j)], theta_var, 1.0)
        haskey(dropoff_link, (key2, k)) && set_normalized_coefficient(dropoff_link[(key2, k)], theta_var, 1.0)
    end
    return theta_var, :added
end

# CGSolver hook real logic (dispatched from
# optimize/aggregate_od_route/column_generation/dispatch.jl) -- count excludes :skipped
# columns. `data` is stashed on `m` at build time (build_joint_routing_assignment.jl); this
# hook only ever receives `build_result`/`mapping`/`m`, no explicit `data` argument.
function _aggregate_od_route_add_columns!(
    ::AggregateODRouteJointRoutingAssignmentFormulation,
    build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model, columns,
)::Int
    data = m[:joint_routing_assignment_data]
    added_count = 0
    for column in columns
        _theta, action = add_joint_routing_assignment_column!(m, data, mapping, column)
        action === :skipped || (added_count += 1)
    end
    return added_count
end
