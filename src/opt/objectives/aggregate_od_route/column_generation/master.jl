"""
Objective assembly for the passenger free-assignment column-generation master.

`master_data`'s type annotation is intentionally omitted (see the parallel note in
`variables/aggregate_od_route/column_generation/master.jl`): `PassengerFreeAssignmentMasterData`
is defined in `pricing/passenger_free_assignment/master.jl`, included after `opt/objective.jl`.
"""

export set_passenger_free_assignment_objective!

"""
    set_passenger_free_assignment_objective!(m, master_data, v, x_same)

Base objective before any route columns exist: unserved-passenger penalty plus same-station
walking cost. Route `theta` coefficients are added later, per-column, via
`set_objective_coefficient` inside `add_passenger_free_assignment_column!`.
"""
function set_passenger_free_assignment_objective!(
    m::Model,
    master_data,
    v::Dict{Int, VariableRef},
    x_same::Dict{Tuple{Int, Int}, VariableRef},
)
    @objective(m, Min,
        sum(master_data.unserved_penalty * v[p.id] for p in master_data.passengers; init=0.0) +
        sum(master_data.walk_cost_weight * master_data.same_station_walk_cost[key] * var
            for (key, var) in x_same; init=0.0),
    )
    return nothing
end
