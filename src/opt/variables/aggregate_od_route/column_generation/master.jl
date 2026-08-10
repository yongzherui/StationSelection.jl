"""
Variable-declaration helpers for the passenger free-assignment column-generation master
(`pricing/passenger_free_assignment/master.jl`'s `build_passenger_free_assignment_master`), extracted so this
formulation's JuMP `@variable` calls live under `variables/`, not `optimize/` -- matching the
convention `AggregateODRouteModel`'s own `build.jl` already follows.

`master_data`'s type annotation is intentionally omitted below: `PassengerFreeAssignmentMasterData`
is defined in `pricing/passenger_free_assignment/master.jl`, included well after this file (see
`src/StationSelection.jl`'s include order -- `opt/variables.jl` precedes `opt/optimize.jl`), so
annotating it here would be a forward type reference JuMP's `function` parsing can't resolve.
"""

export add_passenger_free_assignment_station_variables!
export add_passenger_slack_variables!
export add_passenger_same_station_variables!

"""
    add_passenger_free_assignment_station_variables!(m, master_data; relax_integrality) -> Vector{VariableRef}

Station-selection `y[j]`, one per `master_data.nodes`. Not `add_station_selection_variables!`
(`variables/base.jl`): that helper takes `data::StationSelectionData` purely to read `n`, which
`build_passenger_free_assignment_master` never receives (only `master_data`) -- `master_data.nodes`
already gives the same `1:n`, so this stays a thin, passenger-master-specific sibling rather than
forcing a signature change through `build_passenger_free_assignment_master`'s six other call
sites (`run_passenger_free_assignment_column_generation`, one test, four diagnostic scripts).
"""
function add_passenger_free_assignment_station_variables!(
    m::Model,
    master_data;
    relax_integrality::Bool=true,
)::Vector{VariableRef}
    n = length(master_data.nodes)
    y = relax_integrality ?
        @variable(m, [1:n], lower_bound=0.0, upper_bound=1.0, base_name="y") :
        @variable(m, [1:n], Bin, base_name="y")
    m[:y] = y
    return y
end

"""
    add_passenger_slack_variables!(m, master_data) -> Dict{Int,VariableRef}

Per-passenger unserved slack `v[p] >= 0` -- see `PassengerFreeAssignmentMasterData`'s own
docstring for why this exists (RMP feasibility from an empty column pool).
"""
function add_passenger_slack_variables!(m::Model, master_data)::Dict{Int, VariableRef}
    v = Dict{Int, VariableRef}()
    for p in master_data.passengers
        v[p.id] = @variable(m, lower_bound=0.0, base_name="v[$(p.id)]")
    end
    m[:v] = v
    return v
end

"""
    add_passenger_same_station_variables!(m, master_data, coverage, pickup_link, dropoff_link) -> Dict{Tuple{Int,Int},VariableRef}

Same-station ("no vehicle route") assignment variables `x_same[p,j] >= 0`, wired directly into the
already-built `coverage`/`pickup_link`/`dropoff_link` rows they share with route columns -- see
`build_passenger_free_assignment_master`'s own docstring for why sharing those rows, rather than
giving `x_same` its own, is load-bearing. No explicit upper bound: `x_same[p,j] <= y[j] <= 1`
already follows from the pickup row, so stating it again would only add a dual variable the dual
selector would have to carry for nothing.
"""
function add_passenger_same_station_variables!(
    m::Model,
    master_data,
    coverage::Dict{Int, ConstraintRef},
    pickup_link::Dict{Tuple{Int, Int}, ConstraintRef},
    dropoff_link::Dict{Tuple{Int, Int}, ConstraintRef},
)
    x_same = Dict{Tuple{Int, Int}, VariableRef}()
    for p in master_data.passengers
        for j in master_data.same_station_options[p.id]
            x = @variable(m, lower_bound=0.0, base_name="x_same[$(p.id),$j]")
            x_same[(p.id, j)] = x
            set_normalized_coefficient(coverage[p.id], x, 1.0)
            haskey(pickup_link, (p.id, j)) && set_normalized_coefficient(pickup_link[(p.id, j)], x, 1.0)
            haskey(dropoff_link, (p.id, j)) && set_normalized_coefficient(dropoff_link[(p.id, j)], x, 1.0)
        end
    end
    m[:x_same] = x_same
    return x_same
end
