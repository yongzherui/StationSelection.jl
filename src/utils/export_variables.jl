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
result = run_opt(model, data; ...)
export_variables(result, output_dir)
```
"""

using DataFrames
using CSV
using JSON
using JuMP

export export_variables


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

Export assignment variables for ClusteringTwoStageODModel.
Structure: x[s][od_idx] → Vector (sparse) or Matrix (dense)
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
            if x_od isa Vector
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (pair_idx, var) in enumerate(x_od)
                    val = JuMP.value(var)
                    if val > 0.5
                        j, k = valid_pairs[pair_idx]
                        push!(rows, (
                            scenario = s,
                            od_idx = od_idx,
                            origin_id = o,
                            dest_id = d,
                            pickup_idx = j,
                            dropoff_idx = k,
                            pickup_id = array_idx_to_station_id[j],
                            dropoff_id = array_idx_to_station_id[k],
                            value = val
                        ))
                    end
                end
            else
                n = size(x_od, 1)
                for j in 1:n, k in 1:n
                    val = JuMP.value(x_od[j, k])
                    if val > 0.5
                        push!(rows, (
                            scenario = s,
                            od_idx = od_idx,
                            origin_id = o,
                            dest_id = d,
                            pickup_idx = j,
                            dropoff_idx = k,
                            pickup_id = array_idx_to_station_id[j],
                            dropoff_id = array_idx_to_station_id[k],
                            value = val
                        ))
                    end
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

Export assignment variables for ClusteringBaseModel.
Structure: x[i,j] matrix (station-to-station assignment)
"""
function export_assignment_variables(m::JuMP.Model, mapping::ClusteringBaseModelMap, export_dir::String)
    if !haskey(m.obj_dict, :x)
        return 0
    end

    x = m[:x]
    n = mapping.n_stations
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for i in 1:n, j in 1:n
        val = JuMP.value(x[i, j])
        if val > 0.5
            push!(rows, (
                station_idx = i,
                station_id = array_idx_to_station_id[i],
                medoid_idx = j,
                medoid_id = array_idx_to_station_id[j],
                value = val
            ))
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

Export ClusteringTwoStageODModel specific metadata (no additional variable files).
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

Export ClusteringBaseModel specific metadata.
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::ClusteringBaseModelMap,
    export_dir::String,
    metadata::Dict
)
    metadata["model_type"] = "ClusteringBaseModel"
    metadata["n_stations"] = mapping.n_stations
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
