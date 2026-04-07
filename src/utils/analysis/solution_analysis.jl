"""
Solution analysis utilities for station selection optimization.

This module provides functions to analyze and annotate orders with solution data,
calculate model-predicted walking distances, in-vehicle times, and vehicle routing distances.

# Functions

Annotation:
- `annotate_orders_with_solution`: Add solution columns to orders DataFrame

Calculations:
- `calculate_model_walking_distance`: Total walking distance from solution
- `calculate_model_in_vehicle_time`: Total in-vehicle time from solution
- `calculate_model_vehicle_routing_distance`: Total vehicle routing distance

# Usage

```julia
result = run_opt(model, data; ...)
annotated = annotate_orders_with_solution(result, data)
walking = calculate_model_walking_distance(annotated)
ivt = calculate_model_in_vehicle_time(annotated)
vrd = calculate_model_vehicle_routing_distance(result, data)
```
"""

using DataFrames
using Dates
using JuMP

export annotate_orders_with_solution
export calculate_model_walking_distance
export calculate_model_in_vehicle_time
export calculate_model_vehicle_routing_distance


# =============================================================================
# Order Annotation - Main Entry Point
# =============================================================================

"""
    annotate_orders_with_solution(result::OptResult, data::StationSelectionData) -> DataFrame

Annotate orders from scenarios with solution information.

Returns a DataFrame with all orders from all scenarios, plus columns:
- `scenario_idx`: Scenario index (1-based)
- `assigned_pickup_id`: Assigned pickup station ID
- `assigned_dropoff_id`: Assigned dropoff station ID
- `walking_distance_pickup`: Walking distance from origin to pickup
- `walking_distance_dropoff`: Walking distance from dropoff to destination
- `walking_distance_total`: Total walking distance
- `in_vehicle_time_direct`: Direct in-vehicle time (pickup → dropoff)
- `in_vehicle_time_actual`: Actual in-vehicle time
"""
function annotate_orders_with_solution(result::OptResult, data::StationSelectionData)
    return annotate_orders_with_solution(result, result.mapping, data)
end


# =============================================================================
# ClusteringTwoStageODMap Implementation
# =============================================================================

"""
    annotate_orders_with_solution(result::OptResult, mapping::ClusteringTwoStageODMap, data::StationSelectionData) -> DataFrame

Annotate orders for ClusteringTwoStageODModel. No pooling support.
"""
function annotate_orders_with_solution(
    result::OptResult,
    mapping::ClusteringTwoStageODMap,
    data::StationSelectionData
)
    m = result.model

    # Pre-extract assignment variable values
    assignments = extract_assignments_clustering(m, mapping)

    # Process each scenario
    all_rows = []

    for (s, scenario) in enumerate(data.scenarios)
        for row in eachrow(scenario.requests)
            o = row.origin_idx
            d = row.dest_idx

            # Find assignment for this order
            assignment = find_assignment_clustering(assignments, s, o, d)

            if isnothing(assignment)
                push!(all_rows, create_unassigned_row_clustering(row, s))
                continue
            end

            j_idx, k_idx = assignment.pickup_idx, assignment.dropoff_idx
            j_id = mapping.array_idx_to_station_id[j_idx]
            k_id = mapping.array_idx_to_station_id[k_idx]

            # Calculate walking distances
            walking_pickup = get_walking_cost(data, o, j_idx)
            walking_dropoff = get_walking_cost(data, k_idx, d)
            walking_total = walking_pickup + walking_dropoff

            # Calculate direct in-vehicle time
            ivt_direct = has_routing_costs(data) ? get_routing_cost(data, j_idx, k_idx) : 0.0

            push!(all_rows, create_annotated_row_clustering(
                row, s, j_id, k_id,
                walking_pickup, walking_dropoff, walking_total,
                ivt_direct
            ))
        end
    end

    return DataFrame(all_rows)
end


# =============================================================================
# Helper Functions - Assignment Extraction
# =============================================================================

"""
Extract a representative positive assignment for each ClusteringTwoStageODMap OD pair.
When demand is split across station pairs, this returns the first positive pair for
order-level annotation.
"""
function extract_assignments_clustering(m::JuMP.Model, mapping::ClusteringTwoStageODMap)
    if !haskey(m.obj_dict, :x)
        return Dict{Tuple{Int, Int, Int}, NamedTuple}()
    end

    x = m[:x]
    assignments = Dict{Tuple{Int, Int, Int}, NamedTuple}()

    for (s, x_s) in enumerate(x)
        od_pairs = mapping.Omega_s[s]
        for (od_idx, x_od) in x_s
            o, d = od_pairs[od_idx]

            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            idx = findfirst(i -> JuMP.value(x_od[i]) > 0.5, eachindex(x_od))
            if !isnothing(idx)
                j, k = valid_pairs[idx]
                assignments[(s, o, d)] = (pickup_idx=j, dropoff_idx=k)
            end
        end
    end

    return assignments
end


"""
Find assignment for a specific order (clustering model).
"""
function find_assignment_clustering(assignments::Dict, s::Int, o::Int, d::Int)
    return get(assignments, (s, o, d), nothing)
end


# =============================================================================
# Helper Functions - Row Creation
# =============================================================================

"""
Create annotated row for clustering model (no pooling).
"""
function create_annotated_row_clustering(
    original_row, s, pickup_id, dropoff_id,
    walking_pickup, walking_dropoff, walking_total, ivt_direct
)
    row_dict = Dict{Symbol, Any}()

    for col in propertynames(original_row)
        row_dict[col] = original_row[col]
    end

    row_dict[:scenario_idx] = s
    row_dict[:assigned_pickup_id] = pickup_id
    row_dict[:assigned_dropoff_id] = dropoff_id
    row_dict[:walking_distance_pickup] = walking_pickup
    row_dict[:walking_distance_dropoff] = walking_dropoff
    row_dict[:walking_distance_total] = walking_total
    row_dict[:in_vehicle_time_direct] = ivt_direct
    row_dict[:in_vehicle_time_actual] = ivt_direct

    return row_dict
end


"""
Create row for unassigned order (clustering model).
"""
function create_unassigned_row_clustering(original_row, s)
    row_dict = Dict{Symbol, Any}()

    for col in propertynames(original_row)
        row_dict[col] = original_row[col]
    end

    row_dict[:scenario_idx] = s
    row_dict[:assigned_pickup_id] = missing
    row_dict[:assigned_dropoff_id] = missing
    row_dict[:walking_distance_pickup] = missing
    row_dict[:walking_distance_dropoff] = missing
    row_dict[:walking_distance_total] = missing
    row_dict[:in_vehicle_time_direct] = missing
    row_dict[:in_vehicle_time_actual] = missing

    return row_dict
end


# =============================================================================
# Calculation Functions
# =============================================================================

"""
    calculate_model_walking_distance(annotated_orders::DataFrame) -> Float64

Calculate total walking distance from annotated orders.
"""
function calculate_model_walking_distance(annotated_orders::DataFrame)
    return sum(skipmissing(annotated_orders.walking_distance_total))
end


"""
    calculate_model_in_vehicle_time(annotated_orders::DataFrame) -> Float64

Calculate total in-vehicle time from annotated orders (using actual time).
"""
function calculate_model_in_vehicle_time(annotated_orders::DataFrame)
    return sum(skipmissing(annotated_orders.in_vehicle_time_actual))
end


"""
    calculate_model_vehicle_routing_distance(result::OptResult, data::StationSelectionData; with_pooling::Bool=true) -> Float64

Calculate total vehicle routing distance from solution.

# Arguments
- `result::OptResult`: Optimization result
- `data::StationSelectionData`: Problem data with routing costs
- `with_pooling::Bool`: Ignored for ClusteringTwoStageODModel (no pooling).
"""
function calculate_model_vehicle_routing_distance(
    result::OptResult,
    data::StationSelectionData;
    with_pooling::Bool=true
)
    return calculate_model_vehicle_routing_distance(result, result.mapping, data; with_pooling=with_pooling)
end


"""
Vehicle routing distance for ClusteringTwoStageODMap.
"""
function calculate_model_vehicle_routing_distance(
    result::OptResult,
    mapping::ClusteringTwoStageODMap,
    data::StationSelectionData;
    with_pooling::Bool=true
)
    if !has_routing_costs(data)
        @warn "Routing costs not available, returning 0"
        return 0.0
    end

    # ClusteringTwoStageODModel has no pooling, always use assignments
    return calculate_vrd_from_assignments_clustering(result.model, mapping, data)
end


"""
Calculate VRD from assignment variables for clustering model.
"""
function calculate_vrd_from_assignments_clustering(
    m::JuMP.Model,
    mapping::ClusteringTwoStageODMap,
    data::StationSelectionData
)
    if !haskey(m.obj_dict, :x)
        return 0.0
    end

    x = m[:x]
    total_distance = 0.0

    for (s, x_s) in enumerate(x)
        od_pairs = mapping.Omega_s[s]
        for (od_idx, x_od) in x_s
            o, d = od_pairs[od_idx]

            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (idx, (j, k)) in enumerate(valid_pairs)
                val = JuMP.value(x_od[idx])
                val > 0.5 || continue
                total_distance += get_routing_cost(data, j, k) * val
            end
        end
    end

    return total_distance
end
