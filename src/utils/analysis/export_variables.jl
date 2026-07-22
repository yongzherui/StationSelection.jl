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
result = run_opt(data, model, DirectSolver(...))
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
Structure: x[s][od_idx] → Vector over valid (pickup, dropoff) pairs
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
        for (od_idx, x_od) in x_s
            o, d = od_pairs[od_idx]
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (pair_idx, var) in enumerate(x_od)
                val = JuMP.value(var)
                if val > 0.5
                    j, k = valid_pairs[pair_idx]
                    push!(rows, (
                        scenario = s,
                        od_idx = od_idx,
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


# =============================================================================
# ExactDARPRouteODMap exports (ExactDARPRouteModel)
# =============================================================================

"""
    export_assignment_variables(m, mapping::ExactDARPRouteODMap, export_dir) -> Int

Export assignment variables for ExactDARPRouteModel.
"""
function export_assignment_variables(
    m::JuMP.Model,
    mapping::ExactDARPRouteODMap,
    export_dir::String
)
    if !haskey(m.obj_dict, :x)
        return 0
    end

    x      = m[:x]
    id_map = mapping.array_idx_to_station_id
    rows   = []

    for (s, x_s) in enumerate(x)
        for (t_id, x_t) in x_s
            od_pairs = _time_od_pairs(mapping, s, t_id)
            for (od_idx, x_od) in x_t
                isempty(x_od) && continue
                o, d = od_pairs[od_idx]
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (pair_idx, (j, k)) in enumerate(valid_pairs)
                    val = JuMP.value(x_od[pair_idx])
                    val > 0 || continue
                    push!(rows, (
                        scenario   = s,
                        t_id       = t_id,
                        od_idx     = od_idx,
                        origin_id  = id_map[o],
                        dest_id    = id_map[d],
                        pickup_id  = _exported_station_id(id_map, j),
                        dropoff_id = _exported_station_id(id_map, k),
                        value      = round(Int, val)
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
    export_model_specific_variables(result, mapping::ExactDARPRouteODMap, export_dir, metadata)

Export ExactDARPRouteModel-specific variables: theta_r_ts.csv.
Alpha values are fixed parameters (not variables); the input CSV is the source of record.
"""
function export_model_specific_variables(
    result    :: OptResult,
    mapping   :: ExactDARPRouteODMap,
    export_dir :: String,
    metadata  :: Dict
)
    metadata["model_type"] = "ExactDARPRouteModel"
    metadata["n_routes"]   = sum(
        sum(length(v) for v in values(rs); init = 0)
        for rs in values(mapping.routes_s); init = 0
    )

    n_theta = _arm_export_theta_r_ts(result.model, mapping, export_dir)
    metadata["n_theta_r_ts_nonzero"] = n_theta
end


"""
    _arm_export_theta_r_ts(m, mapping::ExactDARPRouteODMap, export_dir) -> Int

Export theta_r_ts variables to `theta_r_ts.csv`.
Columns: scenario, t_id, route_id, route_idx, n_stops, is_direct, station_ids, travel_time, value.
Only rows with value > 0.5 are written.
"""
function _arm_export_theta_r_ts(
    m          :: JuMP.Model,
    mapping    :: ExactDARPRouteODMap,
    export_dir :: String
)
    if !haskey(m.obj_dict, :theta_r_ts)
        println("    ✓ theta_r_ts.csv (0 deployments — theta_r_ts not present)")
        return 0
    end

    rows = []
    for ((s, t_id, r_idx), var) in m[:theta_r_ts]
        val = JuMP.value(var)
        val > 0.5 || continue
        route = mapping.routes_s[s][t_id][r_idx]
        n_stops = length(route.station_indices)
        push!(rows, (
            scenario    = s,
            t_id        = t_id,
            route_id    = route.id,
            route_idx   = r_idx,
            n_stops     = n_stops,
            is_direct   = n_stops == 2,
            station_ids = join((mapping.array_idx_to_station_id[idx] for idx in route.station_indices), "|"),
            travel_time = route.travel_time,
            value       = round(Int, val)
        ))
    end

    if isempty(rows)
        df = DataFrame(scenario=Int[], t_id=Int[], route_id=Int[], route_idx=Int[],
                       n_stops=Int[], is_direct=Bool[], station_ids=String[],
                       travel_time=Float64[], value=Int[])
        CSV.write(joinpath(export_dir, "theta_r_ts.csv"), df)
        println("    ✓ theta_r_ts.csv (0 deployments)")
        return 0
    end

    df = DataFrame(rows)
    sort!(df, [:scenario, :t_id, :route_id])
    CSV.write(joinpath(export_dir, "theta_r_ts.csv"), df)
    n_direct   = count(r -> r.is_direct,  eachrow(df))
    n_multileg = count(r -> !r.is_direct, eachrow(df))
    println("    ✓ theta_r_ts.csv ($(nrow(df)) deployments: $n_direct direct, $n_multileg multi-leg)")
    return nrow(df)
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
# AggregateODRouteMap exports (AggregateODRouteModel, RouteCoveringProblem)
# =============================================================================

"""
    export_assignment_variables(m, mapping::AggregateODRouteMap, export_dir) -> Int

Export assignment variables for AggregateODRouteModel / RouteCoveringProblem.
Structure: x[s][od_idx] → Vector over valid (pickup, dropoff) pairs (same
shape as ClusteringTwoStageODMap). Adds a `demand` column from mapping.Q_s.
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
    for (s, x_s) in enumerate(x)
        od_pairs = mapping.Omega_s[s]
        for (od_idx, x_od) in x_s
            isempty(x_od) && continue
            o, d = od_pairs[od_idx]
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            demand = get(mapping.Q_s[s], (o, d), 0)
            for (pair_idx, var) in enumerate(x_od)
                val = JuMP.value(var)
                if val > 0.5
                    j, k = valid_pairs[pair_idx]
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

Export per-scenario route activation (theta_compat) to `route_activations.csv`.
Only rows with value > 0.5 (theta_compat is built for every (column, scenario)
pair regardless of activation — see add_aggregate_od_route_theta_variables!).
"""
function _export_route_activations(
    m::JuMP.Model,
    mapping::AggregateODRouteMap,
    export_dir::String
)
    empty_df() = DataFrame(scenario=Int[], column_id=Int[], value=Int[])

    if !haskey(m.obj_dict, :theta_compat)
        CSV.write(joinpath(export_dir, "route_activations.csv"), empty_df())
        println("    ✓ route_activations.csv (0 activations — theta_compat not present)")
        return 0
    end

    rows = []
    for ((column_id, s), var) in m[:theta_compat]
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

Export AggregateODRouteModel/RouteCoveringProblem-specific variables:
route_columns.csv (static route pool) and route_activations.csv
(per-scenario theta_compat activations).
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::AggregateODRouteMap,
    export_dir::String,
    metadata::Dict
)
    metadata["model_type"] = "AggregateODRouteModel"
    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)
    metadata["n_route_columns_in_pool"] = length(mapping.columns)

    n_columns = _export_route_columns(mapping, export_dir)
    n_activations = _export_route_activations(result.model, mapping, export_dir)
    metadata["n_route_columns_exported"] = n_columns
    metadata["n_route_activations"] = n_activations
end
