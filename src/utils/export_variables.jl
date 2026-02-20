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

TwoStageSingleDetourModel additional:
- `flow_variables.csv`: f variables (vehicle flows)
- `same_source_pooling.csv`: u variables (same-source pooling)
- `same_dest_pooling.csv`: v variables (same-destination pooling)
- `detour_triplets_same_source.csv`: (j,k,l) detour definitions
- `detour_quadruplets_same_dest.csv`: (j,k,l,t') detour definitions

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
    export_assignment_variables(m::JuMP.Model, mapping::TwoStageSingleDetourMap, export_dir::String) -> Int

Export assignment variables for TwoStageSingleDetourModel.
Structure: x[s][t][(o,d)] → Vector (sparse) or Matrix (dense)
"""
function export_assignment_variables(m::JuMP.Model, mapping::TwoStageSingleDetourMap, export_dir::String)
    if !haskey(m.obj_dict, :x)
        return 0
    end

    x = m[:x]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    use_sparse = has_walking_distance_limit(mapping)

    rows = []
    for (s, time_dict) in enumerate(x)
        for (t, od_dict) in time_dict
            for (od, x_od) in od_dict
                o, d = od
                if use_sparse
                    valid_pairs = get_valid_jk_pairs(mapping, o, d)
                    for (pair_idx, var) in enumerate(x_od)
                        val = JuMP.value(var)
                        if val > 0.5
                            j, k = valid_pairs[pair_idx]
                            push!(rows, (
                                scenario = s,
                                time_id = t,
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
                                time_id = t,
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
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "assignment_variables.csv"), df)
    println("    ✓ assignment_variables.csv ($(nrow(df)) assignments)")

    return nrow(df)
end


"""
    export_assignment_variables(m::JuMP.Model,
        mapping::Union{ClusteringTwoStageODMap, CorridorTwoStageODMap}, export_dir::String) -> Int

Export assignment variables for ClusteringTwoStageODModel and corridor models.
Structure: x[s][od_idx] → Vector (sparse) or Matrix (dense)
"""
function export_assignment_variables(m::JuMP.Model, mapping::Union{ClusteringTwoStageODMap, CorridorTwoStageODMap}, export_dir::String)
    if !haskey(m.obj_dict, :x)
        return 0
    end

    x = m[:x]
    array_idx_to_station_id = mapping.array_idx_to_station_id
    use_sparse = has_walking_distance_limit(mapping)

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
    export_model_specific_variables(result::OptResult, mapping::TwoStageSingleDetourMap, export_dir::String, metadata::Dict)

Export TwoStageSingleDetourModel specific variables (f, u, v).
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::TwoStageSingleDetourMap,
    export_dir::String,
    metadata::Dict
)
    m = result.model
    detour_combos = result.detour_combos

    if isnothing(detour_combos)
        metadata["model_type"] = "TwoStageSingleDetourModel"
        return
    end

    array_idx_to_station_id = mapping.array_idx_to_station_id
    Xi_same_source = detour_combos.same_source
    Xi_same_dest = detour_combos.same_dest

    # Export flow variables (f)
    n_flows = export_flow_variables(m, mapping, export_dir)

    # Export same-source pooling (u)
    n_same_source = export_same_source_pooling(m, mapping, Xi_same_source, export_dir)

    # Export same-dest pooling (v)
    n_same_dest = export_same_dest_pooling(m, mapping, Xi_same_dest, export_dir)

    metadata["model_type"] = "TwoStageSingleDetourModel"
    metadata["time_window_sec"] = mapping.time_window
    metadata["n_same_source_triplets"] = length(Xi_same_source)
    metadata["n_same_dest_quadruplets"] = length(Xi_same_dest)
    metadata["n_activated_flows"] = n_flows
    metadata["n_activated_same_source"] = n_same_source
    metadata["n_activated_same_dest"] = n_same_dest
end


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
    metadata["model_type"] = "ClusteringTwoStageODModel"
    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)

    # Count OD pairs for metadata
    total_od_pairs = sum(length(od_pairs) for (_, od_pairs) in mapping.Omega_s)
    metadata["n_od_scenario_pairs"] = total_od_pairs
end


"""
    export_model_specific_variables(result::OptResult, mapping::CorridorTwoStageODMap, export_dir::String, metadata::Dict)

Export corridor model specific variables (f_corridor, and α if present).
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::CorridorTwoStageODMap,
    export_dir::String,
    metadata::Dict
)
    m = result.model

    # Distinguish Z vs X model by presence of α variables
    if haskey(m.obj_dict, :α)
        metadata["model_type"] = "ZCorridorODModel"
    else
        metadata["model_type"] = "XCorridorODModel"
    end

    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)
    metadata["n_clusters"] = mapping.n_clusters
    metadata["n_corridors"] = length(mapping.corridor_indices)

    n_cluster_activations = export_cluster_activation_variables(m, mapping, export_dir)
    n_corridor_uses = export_corridor_usage_variables(m, mapping, export_dir)

    metadata["n_cluster_activations"] = n_cluster_activations
    metadata["n_corridor_uses"] = n_corridor_uses
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
    export_assignment_variables(m::JuMP.Model, mapping::TransportationMap, export_dir::String) -> Int

Export assignment variables (x_pick, x_drop) for TransportationModel.
"""
function export_assignment_variables(m::JuMP.Model, mapping::TransportationMap, export_dir::String)
    if !haskey(m.obj_dict, :x_pick) || !haskey(m.obj_dict, :x_drop)
        return 0
    end

    x_pick = m[:x_pick]
    x_drop = m[:x_drop]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        for s in mapping.anchor_scenarios[g_idx]
            # Pickup assignments
            for ((i, j), var) in x_pick[g_idx][s]
                val = JuMP.value(var)
                if val > 0.5
                    push!(rows, (
                        type = "pickup",
                        anchor_idx = g_idx,
                        zone_a = zone_a,
                        zone_b = zone_b,
                        scenario = s,
                        origin_or_dest_id = i,
                        station_idx = j,
                        station_id = array_idx_to_station_id[j],
                        value = val
                    ))
                end
            end

            # Dropoff assignments
            for ((i, k), var) in x_drop[g_idx][s]
                val = JuMP.value(var)
                if val > 0.5
                    push!(rows, (
                        type = "dropoff",
                        anchor_idx = g_idx,
                        zone_a = zone_a,
                        zone_b = zone_b,
                        scenario = s,
                        origin_or_dest_id = i,
                        station_idx = k,
                        station_id = array_idx_to_station_id[k],
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


"""
    export_model_specific_variables(result::OptResult, mapping::TransportationMap, export_dir::String, metadata::Dict)

Export TransportationModel specific variables (p, d, f_transport, u_anchor).
"""
function export_model_specific_variables(
    result::OptResult,
    mapping::TransportationMap,
    export_dir::String,
    metadata::Dict
)
    m = result.model

    metadata["model_type"] = "TransportationModel"
    metadata["has_walking_limit"] = has_walking_distance_limit(mapping)
    metadata["n_clusters"] = mapping.n_clusters
    metadata["n_active_anchors"] = length(mapping.active_anchors)

    # Export flow variables
    n_flows = _export_transportation_flow(m, mapping, export_dir)
    metadata["n_active_flows"] = n_flows

    # Export anchor activation
    n_activations = _export_transportation_activation(m, mapping, export_dir)
    metadata["n_anchor_activations"] = n_activations

    # Export aggregation (p, d)
    n_aggregations = _export_transportation_aggregation(m, mapping, export_dir)
    metadata["n_aggregation_entries"] = n_aggregations
end


"""
Export transportation flow variables (f_transport).
"""
function _export_transportation_flow(m::JuMP.Model, mapping::TransportationMap, export_dir::String)
    if !haskey(m.obj_dict, :f_transport)
        println("    ✓ transportation_flow.csv (0 flows)")
        return 0
    end

    f_transport = m[:f_transport]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        for s in mapping.anchor_scenarios[g_idx]
            for ((j, k), var) in f_transport[g_idx][s]
                val = JuMP.value(var)
                if val > 1e-6
                    push!(rows, (
                        anchor_idx = g_idx,
                        zone_a = zone_a,
                        zone_b = zone_b,
                        scenario = s,
                        j_idx = j,
                        k_idx = k,
                        j_id = array_idx_to_station_id[j],
                        k_id = array_idx_to_station_id[k],
                        value = val
                    ))
                end
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "transportation_flow.csv"), df)
    println("    ✓ transportation_flow.csv ($(nrow(df)) flows)")

    return nrow(df)
end


"""
Export transportation anchor activation variables (u_anchor).
"""
function _export_transportation_activation(m::JuMP.Model, mapping::TransportationMap, export_dir::String)
    if !haskey(m.obj_dict, :u_anchor)
        println("    ✓ anchor_activation.csv (0 activations)")
        return 0
    end

    u_anchor = m[:u_anchor]

    rows = []
    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        for s in mapping.anchor_scenarios[g_idx]
            val = JuMP.value(u_anchor[g_idx][s])
            if val > 0.5
                push!(rows, (
                    anchor_idx = g_idx,
                    zone_a = zone_a,
                    zone_b = zone_b,
                    scenario = s,
                    value = val
                ))
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "anchor_activation.csv"), df)
    println("    ✓ anchor_activation.csv ($(nrow(df)) activations)")

    return nrow(df)
end


"""
Export transportation aggregation variables (p_agg, d_agg).
"""
function _export_transportation_aggregation(m::JuMP.Model, mapping::TransportationMap, export_dir::String)
    if !haskey(m.obj_dict, :p_agg) || !haskey(m.obj_dict, :d_agg)
        println("    ✓ transportation_aggregation.csv (0 entries)")
        return 0
    end

    p_agg = m[:p_agg]
    d_agg = m[:d_agg]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        for s in mapping.anchor_scenarios[g_idx]
            for (j, var) in p_agg[g_idx][s]
                val = JuMP.value(var)
                if val > 1e-6
                    push!(rows, (
                        type = "pickup",
                        anchor_idx = g_idx,
                        zone_a = zone_a,
                        zone_b = zone_b,
                        scenario = s,
                        station_idx = j,
                        station_id = array_idx_to_station_id[j],
                        value = val
                    ))
                end
            end
            for (k, var) in d_agg[g_idx][s]
                val = JuMP.value(var)
                if val > 1e-6
                    push!(rows, (
                        type = "dropoff",
                        anchor_idx = g_idx,
                        zone_a = zone_a,
                        zone_b = zone_b,
                        scenario = s,
                        station_idx = k,
                        station_id = array_idx_to_station_id[k],
                        value = val
                    ))
                end
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "transportation_aggregation.csv"), df)
    println("    ✓ transportation_aggregation.csv ($(nrow(df)) entries)")

    return nrow(df)
end


# =============================================================================
# TwoStageSingleDetourModel Helper Functions
# =============================================================================

"""
Export flow variables (f) for TwoStageSingleDetourModel.
"""
function export_flow_variables(m::JuMP.Model, mapping::TwoStageSingleDetourMap, export_dir::String)
    if !haskey(m.obj_dict, :f)
        println("    ✓ flow_variables.csv (0 flows)")
        return 0
    end

    f = m[:f]
    array_idx_to_station_id = mapping.array_idx_to_station_id

    rows = []
    for (s, time_dict) in enumerate(f)
        for (t, f_st) in time_dict
            if f_st isa Dict
                for ((j, k), var) in f_st
                    val = JuMP.value(var)
                    if val > 0.5
                        push!(rows, (
                            scenario = s,
                            time_id = t,
                            j_array = j,
                            k_array = k,
                            j_id = array_idx_to_station_id[j],
                            k_id = array_idx_to_station_id[k],
                            value = val
                        ))
                    end
                end
            else
                n = size(f_st, 1)
                for j in 1:n, k in 1:n
                    val = JuMP.value(f_st[j, k])
                    if val > 0.5
                        push!(rows, (
                            scenario = s,
                            time_id = t,
                            j_array = j,
                            k_array = k,
                            j_id = array_idx_to_station_id[j],
                            k_id = array_idx_to_station_id[k],
                            value = val
                        ))
                    end
                end
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "flow_variables.csv"), df)
    println("    ✓ flow_variables.csv ($(nrow(df)) flows)")

    return nrow(df)
end


# =============================================================================
# Corridor Model Helper Functions
# =============================================================================

"""
Export cluster activation variables (α) for corridor models.
"""
function export_cluster_activation_variables(m::JuMP.Model, mapping::CorridorTwoStageODMap, export_dir::String)
    if !haskey(m.obj_dict, :α)
        println("    ✓ cluster_activation.csv (0 activations)")
        return 0
    end

    α = m[:α]
    n_clusters, n_scenarios = size(α)
    rows = []
    for a in 1:n_clusters, s in 1:n_scenarios
        val = JuMP.value(α[a, s])
        if val > 1e-6
            push!(rows, (
                cluster_idx = a,
                scenario = s,
                value = val
            ))
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "cluster_activation.csv"), df)
    println("    ✓ cluster_activation.csv ($(nrow(df)) activations)")

    return nrow(df)
end

"""
Export corridor usage variables (f_corridor) for corridor models.
"""
function export_corridor_usage_variables(m::JuMP.Model, mapping::CorridorTwoStageODMap, export_dir::String)
    if !haskey(m.obj_dict, :f_corridor)
        println("    ✓ corridor_usage.csv (0 corridors)")
        return 0
    end

    f_corridor = m[:f_corridor]
    n_corridors, n_scenarios = size(f_corridor)
    corridor_indices = mapping.corridor_indices

    rows = []
    for g in 1:n_corridors, s in 1:n_scenarios
        val = JuMP.value(f_corridor[g, s])
        if val > 0.5
            a, b = corridor_indices[g]
            push!(rows, (
                corridor_idx = g,
                cluster_a = a,
                cluster_b = b,
                scenario = s,
                value = val
            ))
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "corridor_usage.csv"), df)
    println("    ✓ corridor_usage.csv ($(nrow(df)) corridors)")

    return nrow(df)
end


"""
Export same-source pooling variables (u) for TwoStageSingleDetourModel.
"""
function export_same_source_pooling(
    m::JuMP.Model,
    mapping::TwoStageSingleDetourMap,
    Xi_same_source::Vector{Tuple{Int, Int, Int}},
    export_dir::String
)
    if !haskey(m.obj_dict, :u) || isempty(Xi_same_source)
        println("    ✓ same_source_pooling.csv (0 activations)")
        return 0
    end

    u = m[:u]
    use_sparse = has_walking_distance_limit(mapping)

    rows = []
    for (s, time_dict) in enumerate(u)
        for (t, u_st) in time_dict
            if isempty(u_st)
                continue
            end

            if use_sparse
                feasible_indices = get(mapping.feasible_same_source[s], t, Int[])
                for (local_idx, xi_idx) in enumerate(feasible_indices)
                    if local_idx <= length(u_st)
                        val = JuMP.value(u_st[local_idx])
                        if val > 0.5
                            j_id, k_id, l_id = Xi_same_source[xi_idx]
                            push!(rows, (
                                scenario = s,
                                time_id = t,
                                xi_idx = xi_idx,
                                j_id = j_id,
                                k_id = k_id,
                                l_id = l_id,
                                value = val
                            ))
                        end
                    end
                end
            else
                for idx in 1:min(length(u_st), length(Xi_same_source))
                    val = JuMP.value(u_st[idx])
                    if val > 0.5
                        j_id, k_id, l_id = Xi_same_source[idx]
                        push!(rows, (
                            scenario = s,
                            time_id = t,
                            xi_idx = idx,
                            j_id = j_id,
                            k_id = k_id,
                            l_id = l_id,
                            value = val
                        ))
                    end
                end
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "same_source_pooling.csv"), df)
    println("    ✓ same_source_pooling.csv ($(nrow(df)) activations)")

    return nrow(df)
end


"""
Export same-destination pooling variables (v) for TwoStageSingleDetourModel.
"""
function export_same_dest_pooling(
    m::JuMP.Model,
    mapping::TwoStageSingleDetourMap,
    Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}},
    export_dir::String
)
    if !haskey(m.obj_dict, :v) || isempty(Xi_same_dest)
        println("    ✓ same_dest_pooling.csv (0 activations)")
        return 0
    end

    v = m[:v]
    use_sparse = has_walking_distance_limit(mapping)

    rows = []
    for (s, time_dict) in enumerate(v)
        for (t, v_st) in time_dict
            if isempty(v_st)
                continue
            end

            if use_sparse
                feasible_indices = get(mapping.feasible_same_dest[s], t, Int[])
                for (local_idx, xi_idx) in enumerate(feasible_indices)
                    if local_idx <= length(v_st)
                        val = JuMP.value(v_st[local_idx])
                        if val > 0.5
                            j_id, k_id, l_id, time_delta = Xi_same_dest[xi_idx]
                            push!(rows, (
                                scenario = s,
                                time_id = t,
                                xi_idx = xi_idx,
                                j_id = j_id,
                                k_id = k_id,
                                l_id = l_id,
                                time_delta = time_delta,
                                value = val
                            ))
                        end
                    end
                end
            else
                for idx in 1:min(length(v_st), length(Xi_same_dest))
                    val = JuMP.value(v_st[idx])
                    if val > 0.5
                        j_id, k_id, l_id, time_delta = Xi_same_dest[idx]
                        push!(rows, (
                            scenario = s,
                            time_id = t,
                            xi_idx = idx,
                            j_id = j_id,
                            k_id = k_id,
                            l_id = l_id,
                            time_delta = time_delta,
                            value = val
                        ))
                    end
                end
            end
        end
    end

    df = DataFrame(rows)
    CSV.write(joinpath(export_dir, "same_dest_pooling.csv"), df)
    println("    ✓ same_dest_pooling.csv ($(nrow(df)) activations)")

    return nrow(df)
end
