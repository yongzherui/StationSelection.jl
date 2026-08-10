"""
Constraint-declaration helpers for the passenger free-assignment column-generation master, plus
its incremental column-adder -- mirrors `add_aggregate_od_route_column!`'s placement precedent in
`constraints/aggregate_od_route/core.jl`: an incremental column-adder that also declares one
`@variable` (the column's own `theta[r]`) lives in `constraints/`, not `variables/`.

`master_data`/`master`/`column` type annotations are intentionally omitted (see the parallel note
in `variables/aggregate_od_route/column_generation/master.jl`): `PassengerFreeAssignmentMasterData`,
`PassengerFreeAssignmentMaster`, and `PassengerFreeAssignmentRouteColumn` are all defined under
`pricing/passenger_free_assignment/`, included well after `opt/constraints.jl`.
"""

export add_passenger_coverage_constraints!
export add_passenger_station_linking_constraints!
export add_passenger_station_budget_constraint!
export add_passenger_free_assignment_column!

"""
    add_passenger_coverage_constraints!(m, master_data, v) -> Dict{Int,ConstraintRef}

One row per passenger, `v[p] >= 1` -- route-column and `x_same` coefficients are added later, by
`add_passenger_same_station_variables!` and `add_passenger_free_assignment_column!` respectively.
"""
function add_passenger_coverage_constraints!(
    m::Model,
    master_data,
    v::Dict{Int, VariableRef},
)::Dict{Int, ConstraintRef}
    coverage = Dict{Int, ConstraintRef}()
    for p in master_data.passengers
        coverage[p.id] = @constraint(m, v[p.id] >= 1)
    end
    return coverage
end

"""
    add_passenger_station_linking_constraints!(m, master_data, y) -> (pickup_link, dropoff_link)

Disaggregated `(p, j)`/`(p, k)` linking rows, written as `-y[j] <= 0` (not `0 <= y[j]`) so the
normalized form JuMP stores is unambiguous: a column's `theta` coefficient of `+1.0`, added later
via `set_normalized_coefficient`, then yields exactly `theta - y[j] <= 0`.
"""
function add_passenger_station_linking_constraints!(
    m::Model,
    master_data,
    y::Vector{VariableRef},
)
    pickup_link = Dict{Tuple{Int, Int}, ConstraintRef}()
    dropoff_link = Dict{Tuple{Int, Int}, ConstraintRef}()
    for p in master_data.passengers
        for j in master_data.feasible_pickups[p.id]
            pickup_link[(p.id, j)] = @constraint(m, -y[j] <= 0.0)
        end
        for k in master_data.feasible_dropoffs[p.id]
            dropoff_link[(p.id, k)] = @constraint(m, -y[k] <= 0.0)
        end
    end
    return pickup_link, dropoff_link
end

"""
    add_passenger_station_budget_constraint!(m, master_data) -> ConstraintRef

`sum(y) == master_data.l`. Not `add_station_limit_constraint!` (`constraints/base.jl`): that
helper takes an unused `data::StationSelectionData` parameter purely for its type signature, which
`build_passenger_free_assignment_master` has no other reason to carry.
"""
function add_passenger_station_budget_constraint!(m::Model, master_data)
    con = @constraint(m, sum(m[:y]) == master_data.l)
    m[:station_budget] = con
    return con
end

"""
    add_passenger_free_assignment_column!(master, column) -> (theta, action)

`action` is `:added`, or `:skipped` when an identical assignment signature is already in the pool
at no greater `tau` (the pool keeps the cheapest route per signature, mirroring the pricer's own
dedup rule).
"""
function add_passenger_free_assignment_column!(
    master,
    column,
)
    signature = _passenger_free_assignment_column_signature(column)
    existing_id = get(master.column_signatures, signature, nothing)
    if !isnothing(existing_id)
        master.columns[existing_id].tau <= column.tau + 1e-9 &&
            return master.theta[existing_id], :skipped
    end

    m = master.model
    md = master.master_data
    theta = @variable(m, lower_bound=0.0, base_name="theta[$(column.id)]")
    master.theta[column.id] = theta
    master.columns[column.id] = column
    master.column_signatures[signature] = column.id

    set_objective_coefficient(m, theta, passenger_free_assignment_column_cost(column, md))
    for (p, j, k) in column.assignments
        haskey(master.coverage, p) || continue
        set_normalized_coefficient(master.coverage[p], theta, 1.0)
        haskey(master.pickup_link, (p, j)) &&
            set_normalized_coefficient(master.pickup_link[(p, j)], theta, 1.0)
        haskey(master.dropoff_link, (p, k)) &&
            set_normalized_coefficient(master.dropoff_link[(p, k)], theta, 1.0)
    end
    return theta, :added
end
