"""
Export optimization variable values from solved models.

This module provides functions to export variable values from OptResult
to CSV files for post-hoc analysis. Uses multiple dispatch on mapping types
to handle model-specific exports.

# Exports

Common (all models):
- `station_id_mapping.csv`: Array index ↔ station ID
- `scenario_info.csv`: Scenario labels, start/end times
- `station_selection.csv`: y variables (selected stations)
- `scenario_activation.csv`: z variables (active stations per scenario)
- `assignment_variables.csv`: x variables (OD → station pair assignments)
- `variable_export_metadata.json`: Summary statistics

# Usage

```julia
result = run_opt(problem, formulation, DirectMIPSolver())
export_variables(result, output_dir)
```
"""

using DataFrames
using CSV
using JSON
using JuMP

export export_variables

"""
    _exported_station_id(array_idx_to_station_id, idx) -> Int

Map an assignment array index to a station ID for CSV export, with `0`
reserved for [`WALK_ONLY_PAIR`](@ref) (no station used — direct walk).
Real station IDs are always >= 1, so `0` is unambiguous.
"""
_exported_station_id(array_idx_to_station_id, idx::Int)::Int =
    idx == 0 ? 0 : array_idx_to_station_id[idx]


"""
    export_variables(result::OptResult, output_dir::String)

Export optimization variable values from a solved OptResult to CSV files.

Creates a `variable_exports/` subdirectory in `output_dir` containing:
- Common variable exports (y, z, x) for all models
- Model-specific exports based on the mapping type

# Arguments
- `result::OptResult`: Result from `run_opt`
- `output_dir::String`: Directory where `variable_exports/` will be created
"""
function export_variables(result::OptResult, output_dir::String)
    export_dir = joinpath(output_dir, "variable_exports")
    mkpath(export_dir)

    println("\n  Exporting optimization variables...")

    mapping = result.mapping
    m = result.model

    metadata = Dict{String, Any}()

    # Common exports for all models
    export_station_mapping(mapping, export_dir)
    export_scenario_info(mapping, export_dir)
    n_selected = export_station_selection(m, mapping, export_dir)
    n_activated = export_scenario_activation(m, mapping, export_dir)
    n_assignments = export_assignment_variables(m, mapping, export_dir)

    metadata["n_stations_selected"] = n_selected
    metadata["n_scenario_activations"] = n_activated
    metadata["n_assignments"] = n_assignments

    # Model-specific exports via dispatch
    export_model_specific_variables(result, mapping, export_dir, metadata)

    # Save metadata
    open(joinpath(export_dir, "variable_export_metadata.json"), "w") do f
        JSON.print(f, metadata, 4)
    end
    println("    ✓ variable_export_metadata.json")

    println("  ✓ Variables exported to: $export_dir")
end


# =============================================================================
# Common Export Functions
# =============================================================================

"""
    export_station_mapping(mapping::AbstractStationSelectionMap, export_dir::String)

Export array index to station ID mapping.
"""
function export_station_mapping(mapping::AbstractStationSelectionMap, export_dir::String)
    array_idx_to_station_id = mapping.array_idx_to_station_id
    df = DataFrame(
        array_idx = 1:length(array_idx_to_station_id),
        station_id = array_idx_to_station_id
    )
    CSV.write(joinpath(export_dir, "station_id_mapping.csv"), df)
    println("    ✓ station_id_mapping.csv")
end


"""
    export_scenario_info(mapping::AbstractStationSelectionMap, export_dir::String)

Export scenario information (labels, start times, end times).
"""
function export_scenario_info(mapping::AbstractStationSelectionMap, export_dir::String)
    scenarios = mapping.scenarios
    df = DataFrame(
        scenario_idx = 1:length(scenarios),
        label = [s.label for s in scenarios],
        start_time = [isnothing(s.start_time) ? "" : string(s.start_time) for s in scenarios],
        end_time = [isnothing(s.end_time) ? "" : string(s.end_time) for s in scenarios]
    )
    CSV.write(joinpath(export_dir, "scenario_info.csv"), df)
    println("    ✓ scenario_info.csv")
end


"""
    export_station_selection(m::JuMP.Model, mapping::AbstractStationSelectionMap, export_dir::String) -> Int

Export station selection variables (y). Returns count of selected stations.
"""
function export_station_selection(m::JuMP.Model, mapping::AbstractStationSelectionMap, export_dir::String)
    if !haskey(m.obj_dict, :y)
        return 0
    end

    y = m[:y]
    n = length(y)
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for i in 1:n
        val = JuMP.value(y[i])
        push!(rows, (
            array_idx = i,
            station_id = array_idx_to_station_id[i],
            selected = val > 0.5 ? 1 : 0,
            value = val
        ))
    end

    df = DataFrame(rows)
    n_selected = sum(df.selected)
    CSV.write(joinpath(export_dir, "station_selection.csv"), df)
    println("    ✓ station_selection.csv ($n_selected selected)")

    return n_selected
end


"""
    export_scenario_activation(m::JuMP.Model, mapping::AbstractStationSelectionMap, export_dir::String) -> Int

Export scenario activation variables (z). Returns count of activations.
"""
function export_scenario_activation(m::JuMP.Model, mapping::AbstractStationSelectionMap, export_dir::String)
    if !haskey(m.obj_dict, :z)
        return 0
    end

    z = m[:z]
    n_stations, n_scenarios = size(z)
    array_idx_to_station_id = mapping.array_idx_to_station_id
    array_idx_to_scenario_label = mapping.array_idx_to_scenario_label

    rows = []
    for i in 1:n_stations
        for s in 1:n_scenarios
            val = JuMP.value(z[i, s])
            if val > 0.5
                push!(rows, (
                    station_idx = i,
                    station_id = array_idx_to_station_id[i],
                    scenario_idx = s,
                    scenario_label = array_idx_to_scenario_label[s],
                    value = val
                ))
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "scenario_activation.csv"), df)
    println("    ✓ scenario_activation.csv ($(nrow(df)) activations)")

    return nrow(df)
end


# =============================================================================
# Assignment Variable Export - Dispatched by Mapping Type
# =============================================================================

"""
    export_assignment_variables(m::JuMP.Model, mapping::ClusteringTwoStageODMap, export_dir::String) -> Int

Export assignment variables for TwoStageODPolicy.
Structure: x[s][p] → Vector over valid (pickup, dropoff) pairs
"""
function export_assignment_variables(m::JuMP.Model, mapping::ClusteringTwoStageODMap, export_dir::String)
    if !haskey(m.obj_dict, :x)
        return 0
    end

    x = m[:x]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for (s, x_s) in enumerate(x)
        od_pairs = mapping.Omega_s[s]
        for (p, x_od) in x_s
            o, d = od_pairs[p]
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (pair_idx, var) in enumerate(x_od)
                val = JuMP.value(var)
                if val > 0.5
                    j, k = valid_pairs[pair_idx]
                    push!(rows, (
                        scenario = s,
                        od_idx = p,
                        origin_id = array_idx_to_station_id[o],
                        dest_id = array_idx_to_station_id[d],
                        pickup_idx = j,
                        dropoff_idx = k,
                        pickup_id = _exported_station_id(array_idx_to_station_id, j),
                        dropoff_id = _exported_station_id(array_idx_to_station_id, k),
                        value = round(Int, val)
                    ))
                end
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "assignment_variables.csv"), df)
    println("    ✓ assignment_variables.csv ($(nrow(df)) assignments)")

    return nrow(df)
end


"""
    export_assignment_variables(m::JuMP.Model, mapping::ClusteringBaseModelMap, export_dir::String) -> Int

Export assignment variables for SingleStagePolicy.
Structure: x[i] → Vector over admissible cluster centers j.
"""
function export_assignment_variables(m::JuMP.Model, mapping::ClusteringBaseModelMap, export_dir::String)
    if !haskey(m.obj_dict, :x)
        return 0
    end

    x = m[:x]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for i in 1:mapping.n_stations
        valid_js = get_valid_j_assignments(mapping, i)
        for (j_idx, var) in enumerate(x[i])
            val = JuMP.value(var)
            if val > 0.5
                j = valid_js[j_idx]
                push!(rows, (
                    station_idx = i,
                    station_id = array_idx_to_station_id[i],
                    medoid_idx = j,
                    medoid_id = array_idx_to_station_id[j],
                    request_count = mapping.request_counts[i],
                    value = val
                ))
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "assignment_variables.csv"), df)
    println("    ✓ assignment_variables.csv ($(nrow(df)) assignments)")

    return nrow(df)
end

"""
    export_assignment_variables(m::JuMP.Model, mapping::ClusteringTwoStageStationMap, export_dir::String) -> Int

Export assignment variables for TwoStagePolicy.
Structure: x[s][i_idx] → Vector over admissible cluster centers j.
"""
function export_assignment_variables(m::JuMP.Model, mapping::ClusteringTwoStageStationMap, export_dir::String)
    if !haskey(m.obj_dict, :x)
        return 0
    end

    x = m[:x]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for (s, x_s) in enumerate(x)
        demand_points = mapping.I_s[s]
        for (i_idx, x_i) in x_s
            i = demand_points[i_idx]
            valid_js = get_valid_j_assignments(mapping, i)
            for (j_idx, var) in enumerate(x_i)
                val = JuMP.value(var)
                if val > 0.5
                    j = valid_js[j_idx]
                    push!(rows, (
                        scenario = s,
                        demand_station_idx = i,
                        demand_station_id = array_idx_to_station_id[i],
                        assigned_station_idx = j,
                        assigned_station_id = array_idx_to_station_id[j],
                        endpoint_count = mapping.q_s[s][i],
                        value = val
                    ))
                end
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "assignment_variables.csv"), df)
    println("    ✓ assignment_variables.csv ($(nrow(df)) assignments)")

    return nrow(df)
end


# =============================================================================
# Model-Specific Export Functions - Dispatched by Mapping Type
# =============================================================================

"""
    export_model_specific_variables(result::OptResult, mapping::ClusteringTwoStageODMap, export_dir::String, metadata::Dict)

Export TwoStageODPolicy specific metadata (no additional variable files).
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::ClusteringTwoStageODMap,
    export_dir::String,
    metadata::Dict
)
    m = result.model
    metadata["model_type"] = "ClusteringTwoStageODModel"
    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)

    # Count OD pairs for metadata
    total_od_pairs = sum(length(od_pairs) for (_, od_pairs) in mapping.Omega_s)
    metadata["n_od_scenario_pairs"] = total_od_pairs

    # Export route activations when flow regularizer is present (no-op otherwise)
    n_flow_activations = export_flow_activation_variables(m, mapping, export_dir)
    metadata["n_flow_activations"] = n_flow_activations
end


"""
    export_model_specific_variables(result::OptResult, mapping::ClusteringBaseModelMap, export_dir::String, metadata::Dict)

Export SingleStagePolicy specific metadata.
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::ClusteringBaseModelMap,
    export_dir::String,
    metadata::Dict
)
    metadata["model_type"] = "ClusteringBaseModel"
    metadata["n_stations"] = mapping.n_stations
    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)
end

"""
    export_model_specific_variables(result::OptResult, mapping::ClusteringTwoStageStationMap, export_dir::String, metadata::Dict)

Export TwoStagePolicy specific metadata.
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::ClusteringTwoStageStationMap,
    export_dir::String,
    metadata::Dict
)
    metadata["model_type"] = "ClusteringTwoStageStationModel"
    metadata["n_stations"] = mapping.n_stations
    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)
    metadata["n_endpoint_groups"] = sum(length(v) for v in values(mapping.I_s))
end


"""
    export_flow_activation_variables(m, mapping::ClusteringTwoStageODMap, export_dir) -> Int

Export route activation variables (f_flow) to `flow_activation.csv`.

Columns: scenario, pickup_id, dropoff_id, value

Only activated routes (value > 0.5) are written. The (j, k) array indices
are converted to station IDs via `mapping.array_idx_to_station_id`.
Returns the count of activated routes written.
"""
function export_flow_activation_variables(
    m::JuMP.Model,
    mapping::ClusteringTwoStageODMap,
    export_dir::String
)
    if !haskey(m.obj_dict, :f_flow)
        println("    ✓ flow_activation.csv (0 routes — f_flow not present)")
        return 0
    end

    f_flow = m[:f_flow]
    id_map  = mapping.array_idx_to_station_id

    rows = []
    for (s, route_dict) in enumerate(f_flow)
        for ((j, k), var) in route_dict
            val = JuMP.value(var)
            val > 0.5 || continue
            push!(rows, (
                scenario   = s,
                pickup_id  = id_map[j],
                dropoff_id = id_map[k],
                value      = val
            ))
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "flow_activation.csv"), df)
    println("    ✓ flow_activation.csv ($(nrow(df)) activated routes)")
    return nrow(df)
end


# =============================================================================
# AggregateODRouteMap exports (AggregateODRouteProblem, RouteCoveringProblem)
# =============================================================================

"""
    export_assignment_variables(m, mapping::AggregateODRouteMap, export_dir) -> Int

Export assignment variables for `AggregateODRouteBaseFormulation`. Structure: a flat
`x[(s, p, j, k)]::VariableRef` Dict (see `add_assignment_variables!` in
`variables/assignment.jl`), unlike `ClusteringTwoStageODMap`'s nested
`x[s][p]::Vector`. Adds a `demand` column from `mapping.Q_s`.
"""
function export_assignment_variables(
    m::JuMP.Model,
    mapping::AggregateODRouteMap,
    export_dir::String
)
    if !haskey(m.obj_dict, :x)
        return 0
    end

    x = m[:x]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for ((s, od_idx, j, k), var) in x
        val = JuMP.value(var)
        if val > 0.5
            o, d = mapping.Omega_s[s][od_idx]
            demand = get(mapping.Q_s[s], (o, d), 0)
            push!(rows, (
                scenario = s,
                od_idx = od_idx,
                origin_id = array_idx_to_station_id[o],
                dest_id = array_idx_to_station_id[d],
                pickup_idx = j,
                dropoff_idx = k,
                pickup_id = _exported_station_id(array_idx_to_station_id, j),
                dropoff_id = _exported_station_id(array_idx_to_station_id, k),
                demand = demand,
                value = round(Int, val)
            ))
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "assignment_variables.csv"), df)
    println("    ✓ assignment_variables.csv ($(nrow(df)) assignments)")
    return nrow(df)
end


"""
    _export_route_columns(mapping::AggregateODRouteMap, export_dir) -> Int

Export the scenario-independent pool of AggregateODRouteColumns to
`route_columns.csv`. One row per column. Station order is taken from
`column.metadata["route"]` when present (CG-generated columns); for
singleton columns (metadata only has "initialization"=>"singleton", no
"route" key) the two stations of the column's single od_pair are used.
"""
function _export_route_columns(mapping::AggregateODRouteMap, export_dir::String)
    id_map = mapping.array_idx_to_station_id
    rows = []
    for column in mapping.columns
        station_idxs = haskey(column.metadata, "route") ?
            collect(column.metadata["route"]) :
            collect(column.od_pairs[1])
        push!(rows, (
            column_id = column.id,
            n_stations = length(station_idxs),
            n_od_pairs = length(column.od_pairs),
            station_ids = join((id_map[idx] for idx in station_idxs), "|"),
            od_pairs = join(("$(id_map[j])-$(id_map[k])" for (j, k) in column.od_pairs), ";"),
            tau = column.tau,
            initialization = string(get(column.metadata, "initialization", ""))
        ))
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "route_columns.csv"), df)
    println("    ✓ route_columns.csv ($(nrow(df)) columns in pool)")
    return nrow(df)
end


"""
    _export_route_activations(m, mapping::AggregateODRouteMap, export_dir) -> Int

Export per-scenario route activation (route_theta) to `route_activations.csv`.
Only rows with value > 0.5 (route_theta is built for every (column, scenario)
pair regardless of activation — see add_route_variables!).
"""
function _export_route_activations(
    m::JuMP.Model,
    mapping::AggregateODRouteMap,
    export_dir::String
)
    empty_df() = DataFrame(scenario=Int[], column_id=Int[], value=Int[])

    if !haskey(m.obj_dict, :route_theta)
        CSV.write(joinpath(export_dir, "route_activations.csv"), empty_df())
        println("    ✓ route_activations.csv (0 activations — route_theta not present)")
        return 0
    end

    rows = []
    for ((column_id, s), var) in m[:route_theta]
        val = JuMP.value(var)
        val > 0.5 || continue
        push!(rows, (scenario = s, column_id = column_id, value = round(Int, val)))
    end

    df = isempty(rows) ? empty_df() : sort!(DataFrame(rows), [:scenario, :column_id])
    CSV.write(joinpath(export_dir, "route_activations.csv"), df)
    println("    ✓ route_activations.csv ($(nrow(df)) activations)")
    return nrow(df)
end



"""
    export_model_specific_variables(result, mapping::AggregateODRouteMap, export_dir, metadata)

Both live AggregateODRoute formulations share `mapping::AggregateODRouteMap`, so this one
method has to serve both -- a second method on the same signature would not be an overload,
it would silently overwrite the first at load time. It therefore redispatches on the
formulation stashed on the model at build time (`m[:aggregate_od_route_formulation]`),
exactly as the `CGSolver` hooks in
`optimize/aggregate_od_route/column_generation/dispatch.jl` do. A model carrying no such
key (`RouteCoveringProblem` reuses this mapping but is not built by either `build_model`)
falls back to the `AggregateODRouteBaseFormulation` exports, preserving the behaviour this
function had before the split.
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::AggregateODRouteMap,
    export_dir::String,
    metadata::Dict
)
    formulation = get(result.model.obj_dict, :aggregate_od_route_formulation, nothing)
    return _aggregate_od_route_export_model_specific_variables(
        formulation, result, mapping, export_dir, metadata,
    )
end


"""
    _aggregate_od_route_export_model_specific_variables(::AggregateODRouteBaseFormulation, ...)

`AggregateODRouteBaseFormulation`/`RouteCoveringProblem` exports: route_columns.csv (the
up-front enumerated pool, read off `mapping.columns`) and route_activations.csv
(`m[:route_theta]`, keyed `(column_id, scenario)`). Neither name exists on the joint CG
path, which is why that formulation gets its own method below rather than sharing this one.
"""
function _aggregate_od_route_export_model_specific_variables(
    ::Union{Nothing, AggregateODRouteBaseFormulation},
    result::OptResult,
    mapping::AggregateODRouteMap,
    export_dir::String,
    metadata::Dict
)
    metadata["model_type"] = "AggregateODRouteProblem"
    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)
    metadata["n_route_columns_in_pool"] = length(mapping.columns)

    n_columns = _export_route_columns(mapping, export_dir)
    n_activations = _export_route_activations(result.model, mapping, export_dir)
    metadata["n_route_columns_exported"] = n_columns
    metadata["n_route_activations"] = n_activations
end


# =============================================================================
# AggregateODRouteJointRoutingAssignmentFormulation exports (CGSolver)
# =============================================================================

"""
Selection tolerance for joint `theta`/`x_walk`. Deliberately not the `0.5` the Base
exporter uses: `theta` is binary only after `CGSolver`'s integer recovery
(`recover_integer_solution=true`), and on an LP-master result it is fractional, where a
`0.5` cut would silently discard most of the solution instead of reporting it.
"""
const JOINT_ROUTING_ASSIGNMENT_EXPORT_TOL = 1e-6

"""
    _joint_has_primal(m) -> Bool

Whether `JuMP.value` can be called on `m` at all. `CGSolver` can return a result whose
model was never solved to a point (`SOLVE_NOT_SOLVED`), and `value` throws there rather
than returning a sentinel, so every joint export below is gated on this and writes
correctly-headed empty frames when it is false.
"""
_joint_has_primal(m::JuMP.Model) =
    JuMP.result_count(m) > 0 &&
    JuMP.primal_status(m) in (MOI.FEASIBLE_POINT, MOI.NEARLY_FEASIBLE_POINT)

"""
    _joint_selected_columns(m) -> Vector{Tuple{Int, Float64}}

The `(column_id, theta_value)` pairs above [`JOINT_ROUTING_ASSIGNMENT_EXPORT_TOL`], sorted
by id. `m[:joint_routing_assignment_theta]` is keyed by **column id alone**, not
`(column_id, scenario)` as the Base formulation's `route_theta` is: a joint column belongs
to exactly one scenario (it is part of the column's dedup signature, see
`add_joint_routing_assignment_column!`), which is recovered from
`column.metadata["scenario"]` rather than from the variable key.
"""
function _joint_selected_columns(m::JuMP.Model)
    theta = m[:joint_routing_assignment_theta]
    selected = Tuple{Int, Float64}[]
    for (column_id, var) in theta
        val = JuMP.value(var)
        val > JOINT_ROUTING_ASSIGNMENT_EXPORT_TOL || continue
        push!(selected, (column_id, val))
    end
    sort!(selected; by = first)
    return selected
end

"""
    _export_joint_route_activations(m, mapping, selected, export_dir) -> Int

One row per selected route column, written to `route_activations.csv`.

`route_station_ids` is `column.route` -- the physical stop sequence in visit order --
translated through `mapping.array_idx_to_station_id` and joined with `|`. `is_elementary`
records whether that sequence visits each station at most once: the pricer's labels are
revisit-tolerant, so a column may legitimately repeat a station, and whether the columns
that end up *selected* do so is not something the pool size or the objective can answer.
`column_cost` is the true objective coefficient via `joint_routing_assignment_column_cost`,
i.e. `mu*(tau + rho)` plus the demand-weighted walking cost of the assignments it carries.
"""
function _export_joint_route_activations(
    m::JuMP.Model,
    mapping::AggregateODRouteMap,
    selected::Vector{Tuple{Int, Float64}},
    export_dir::String,
)
    empty_df() = DataFrame(
        scenario=Int[], column_id=Int[], theta_value=Float64[], is_integral=Bool[],
        tau=Float64[], column_cost=Float64[], n_stops=Int[], n_distinct_stations=Int[],
        is_elementary=Bool[], route_station_ids=String[], n_assignments=Int[],
        total_demand=Int[], initialization=String[],
    )

    data = m[:joint_routing_assignment_data]
    columns = m[:joint_routing_assignment_columns]
    id_map = mapping.array_idx_to_station_id

    rows = []
    for (column_id, val) in selected
        column = columns[column_id]
        s = Int(column.metadata["scenario"])
        route = column.route
        push!(rows, (
            scenario = s,
            column_id = column_id,
            theta_value = val,
            is_integral = abs(val - round(val)) <= JOINT_ROUTING_ASSIGNMENT_EXPORT_TOL,
            tau = column.tau,
            column_cost = joint_routing_assignment_column_cost(m, data, mapping, column),
            n_stops = length(route),
            n_distinct_stations = length(Set(route)),
            is_elementary = length(Set(route)) == length(route),
            route_station_ids = join((id_map[idx] for idx in route), "|"),
            n_assignments = length(column.assignments),
            total_demand = sum(mapping.Q_s[s][p] for (p, _, _) in column.assignments; init = 0),
            initialization = string(get(column.metadata, "initialization", "")),
        ))
    end

    df = isempty(rows) ? empty_df() : sort!(DataFrame(rows), [:scenario, :column_id])
    CSV.write(joinpath(export_dir, "route_activations.csv"), df)
    println("    ✓ route_activations.csv ($(nrow(df)) selected columns)")
    return nrow(df)
end

"""
    _export_joint_route_assignments(m, mapping, selected, export_dir) -> Int

One row per `(selected column, passenger assignment)` pair, written to
`route_assignments.csv` -- the long form of `column.assignments`, whose entries are
`(p, pickup, dropoff)` with `p` indexing `mapping.Omega_s[scenario]` and pickup/dropoff
being station array indices. This is the table that says who rides which route and where
they board and alight; the joint formulation has no `x` variable, so it exists nowhere else.

`pickup_stop_position`/`dropoff_stop_position` are the indices route replay actually
certified, read from `column.metadata["assignment_positions"]`
(`_replay_joint_routing_assignment_route` records them at the point they are known). They
are **not** re-derived here from the station pair: on a revisiting route the certifying
dropoff is the earliest ride-limit-feasible index and its pickup is the most recent prior
visit to the origin, so a `findfirst`/`findlast` scan of `route` gets both ends wrong in
general and agrees only by coincidence on an elementary route. Columns built by a path that
does not record the key (`darp`, `darp_modified`, direct enumeration) export `0`, which is
distinguishable from any real 1-based position.
"""
function _export_joint_route_assignments(
    m::JuMP.Model,
    mapping::AggregateODRouteMap,
    selected::Vector{Tuple{Int, Float64}},
    export_dir::String,
)
    empty_df() = DataFrame(
        scenario=Int[], column_id=Int[], theta_value=Float64[], p=Int[],
        origin_id=Int[], dest_id=Int[], demand=Int[],
        pickup_idx=Int[], dropoff_idx=Int[], pickup_id=Int[], dropoff_id=Int[],
        walk_cost=Float64[], pickup_stop_position=Int[], dropoff_stop_position=Int[],
    )

    data = m[:joint_routing_assignment_data]
    columns = m[:joint_routing_assignment_columns]
    id_map = mapping.array_idx_to_station_id

    rows = []
    for (column_id, val) in selected
        column = columns[column_id]
        s = Int(column.metadata["scenario"])
        omega = mapping.Omega_s[s]
        positions = get(column.metadata, "assignment_positions",
                        Dict{Int, Tuple{Int, Int}}())
        for (p, j, k) in column.assignments
            pickup_pos, dropoff_pos = get(positions, p, (0, 0))
            o, d = omega[p]
            push!(rows, (
                scenario = s,
                column_id = column_id,
                theta_value = val,
                p = p,
                origin_id = id_map[o],
                dest_id = id_map[d],
                demand = mapping.Q_s[s][p],
                pickup_idx = j,
                dropoff_idx = k,
                pickup_id = _exported_station_id(id_map, j),
                dropoff_id = _exported_station_id(id_map, k),
                walk_cost = od_pair_walking_cost(data, o, d, (j, k)),
                pickup_stop_position = pickup_pos,
                dropoff_stop_position = dropoff_pos,
            ))
        end
    end

    df = isempty(rows) ? empty_df() : sort!(DataFrame(rows), [:scenario, :column_id, :p])
    CSV.write(joinpath(export_dir, "route_assignments.csv"), df)
    println("    ✓ route_assignments.csv ($(nrow(df)) route-borne assignments)")
    return nrow(df)
end

"""
    _export_joint_walk_only_assignments(m, mapping, export_dir) -> Int

The `x_walk` half of the solution, written to `walk_only_assignments.csv`. Every
positive-demand group is covered by route columns **or** by a direct walk (the coverage
constraint), so without this file the export silently drops demand and a genuinely
walk-served group is indistinguishable from an export bug.
"""
function _export_joint_walk_only_assignments(
    m::JuMP.Model,
    mapping::AggregateODRouteMap,
    export_dir::String,
)
    empty_df() = DataFrame(
        scenario=Int[], p=Int[], origin_id=Int[], dest_id=Int[],
        demand=Int[], value=Float64[], walk_cost=Float64[],
    )

    if !haskey(m.obj_dict, :x_walk) || !_joint_has_primal(m)
        CSV.write(joinpath(export_dir, "walk_only_assignments.csv"), empty_df())
        println("    ✓ walk_only_assignments.csv (0 — no x_walk or no primal solution)")
        return 0
    end

    data = m[:joint_routing_assignment_data]
    id_map = mapping.array_idx_to_station_id

    rows = []
    for ((s, p), var) in m[:x_walk]
        val = JuMP.value(var)
        val > JOINT_ROUTING_ASSIGNMENT_EXPORT_TOL || continue
        o, d = mapping.Omega_s[s][p]
        push!(rows, (
            scenario = s,
            p = p,
            origin_id = id_map[o],
            dest_id = id_map[d],
            demand = mapping.Q_s[s][p],
            value = val,
            walk_cost = od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR),
        ))
    end

    df = isempty(rows) ? empty_df() : sort!(DataFrame(rows), [:scenario, :p])
    CSV.write(joinpath(export_dir, "walk_only_assignments.csv"), df)
    println("    ✓ walk_only_assignments.csv ($(nrow(df)) direct walks)")
    return nrow(df)
end

"""
    _joint_export_self_checks!(metadata, m, mapping, selected)

Two invariants recomputed from the exported quantities alone, recorded in
`variable_export_metadata.json`. Both are cheap, and both fail loudly if this exporter has
misread the index conventions (`p` vs station array index vs station id is easy to get
wrong here) -- which matters because the alternative way to notice is re-running the solve.

- `coverage_shortfall_max`: over every `(s, p)`, `max(0, 1 - (sum(theta over covering
  columns) + x_walk))`. The coverage constraint is `>= 1`, so a shortfall is a genuine
  violation while an excess is not -- overcoverage is feasible and merely paid for. The
  check is therefore one-sided, and `coverage_max` records the largest total seen so
  overcoverage stays visible without being flagged as an error.
- `objective_residual`: `sum(theta * column_cost) + sum(x_walk * walk cost)` against
  `result.objective_value`. Reconstructs the objective from the exported rows only.
"""
function _joint_export_self_checks!(
    metadata::Dict,
    result::OptResult,
    m::JuMP.Model,
    mapping::AggregateODRouteMap,
    selected::Vector{Tuple{Int, Float64}},
)
    data = m[:joint_routing_assignment_data]
    columns = m[:joint_routing_assignment_columns]
    walk_cost_weight = Float64(m[:joint_routing_assignment_walk_cost_weight])

    coverage = Dict{Tuple{Int, Int}, Float64}()
    for s in 1:n_scenarios(data), p in 1:length(mapping.Omega_s[s])
        coverage[(s, p)] = 0.0
    end

    objective = 0.0
    for (column_id, val) in selected
        column = columns[column_id]
        s = Int(column.metadata["scenario"])
        objective += val * joint_routing_assignment_column_cost(m, data, mapping, column)
        for (p, _, _) in column.assignments
            haskey(coverage, (s, p)) && (coverage[(s, p)] += val)
        end
    end

    n_walk = 0
    if haskey(m.obj_dict, :x_walk)
        for ((s, p), var) in m[:x_walk]
            val = JuMP.value(var)
            val > JOINT_ROUTING_ASSIGNMENT_EXPORT_TOL || continue
            n_walk += 1
            haskey(coverage, (s, p)) && (coverage[(s, p)] += val)
            o, d = mapping.Omega_s[s][p]
            objective += val * walk_cost_weight * mapping.Q_s[s][p] *
                od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)
        end
    end

    metadata["coverage_shortfall_max"] =
        isempty(coverage) ? 0.0 : maximum(max(0.0, 1.0 - v) for v in values(coverage))
    metadata["coverage_max"] =
        isempty(coverage) ? 0.0 : maximum(values(coverage))
    metadata["objective_reconstructed"] = objective
    if !isnothing(result.objective_value)
        metadata["objective_residual"] = abs(objective - result.objective_value)
    end
    metadata["n_walk_only_assignments"] = n_walk
    return nothing
end

"""
    _aggregate_od_route_export_model_specific_variables(::AggregateODRouteJointRoutingAssignmentFormulation, ...)

Joint routing+assignment CG exports. Writes route_activations.csv (one row per selected
`theta`, with the decoded stop sequence and whether it is elementary), route_assignments.csv
(the per-passenger `(p, pickup, dropoff)` triples those columns carry), and
walk_only_assignments.csv (the `x_walk` complement), plus the self-check fields
`_joint_export_self_checks!` documents.

Reads `m[:joint_routing_assignment_{theta,columns}]`, **not** `mapping.columns` /
`m[:route_theta]`: the CG path never registers columns onto the mapping
(`_register_aggregate_od_route_column_metadata!` is only called from the Base formulation's
`route_activation.jl`), so `mapping.columns` holds the unrelated singleton seed pool here.

`result.model` is the last model optimized, which under `recover_integer_solution=true` is
the rebuilt integer-recovery MIP rather than the LP master -- the right object, and the one
whose `theta` are binary. That rebuild re-runs the column adder and so re-deduplicates, so
its pool can be a strict subset of the LP pool; both sizes are recorded rather than assumed
equal. Column ids survive the rebuild (`column.id` travels on the column and keys the dict).
"""
function _aggregate_od_route_export_model_specific_variables(
    ::AggregateODRouteJointRoutingAssignmentFormulation,
    result::OptResult,
    mapping::AggregateODRouteMap,
    export_dir::String,
    metadata::Dict
)
    m = result.model
    metadata["model_type"] = "AggregateODRouteJointRoutingAssignment"
    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)
    metadata["n_route_columns_in_pool"] = length(m[:joint_routing_assignment_columns])
    metadata["theta_relaxed"] = Bool(m[:joint_routing_assignment_relax_integrality])

    if !_joint_has_primal(m)
        metadata["has_primal_solution"] = false
        empty = Tuple{Int, Float64}[]
        metadata["n_route_activations"] = _export_joint_route_activations(m, mapping, empty, export_dir)
        metadata["n_route_assignments"] = _export_joint_route_assignments(m, mapping, empty, export_dir)
        metadata["n_walk_only_assignments"] = _export_joint_walk_only_assignments(m, mapping, export_dir)
        println("    ! no primal solution on the final model — wrote empty route exports")
        return nothing
    end

    metadata["has_primal_solution"] = true
    selected = _joint_selected_columns(m)
    metadata["n_route_activations"] =
        _export_joint_route_activations(m, mapping, selected, export_dir)
    metadata["n_route_assignments"] =
        _export_joint_route_assignments(m, mapping, selected, export_dir)
    _export_joint_walk_only_assignments(m, mapping, export_dir)
    _joint_export_self_checks!(metadata, result, m, mapping, selected)

    # The study question this export exists to answer, precomputed so a reader does not
    # have to re-derive it from route_station_ids.
    columns = m[:joint_routing_assignment_columns]
    n_elementary = count(
        length(Set(columns[id].route)) == length(columns[id].route) for (id, _) in selected;
        init = 0,
    )
    metadata["n_selected_elementary"] = n_elementary
    metadata["n_selected_non_elementary"] = length(selected) - n_elementary
    return nothing
end
