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
- `time_id`: Time window ID within scenario
- `assigned_pickup_id`: Assigned pickup station ID
- `assigned_dropoff_id`: Assigned dropoff station ID
- `walking_distance_pickup`: Walking distance from origin to pickup
- `walking_distance_dropoff`: Walking distance from dropoff to destination
- `walking_distance_total`: Total walking distance
- `in_vehicle_time_direct`: Direct in-vehicle time (pickup → dropoff)
- `is_pooled`: Whether this order is part of a pooling route
- `pooling_type`: "same_source", "same_dest", or missing
- `pooling_role`: "primary" or "secondary" (who gets the detour)
- `pooling_xi_idx`: Index into Xi_same_source or Xi_same_dest
- `pooling_j_id`, `pooling_k_id`, `pooling_l_id`: Triplet station IDs
- `pooling_time_delta`: Time delta for same-dest pooling
- `in_vehicle_time_actual`: Actual in-vehicle time (including any detour)
- `detour_time`: Additional time due to pooling detour
"""
function annotate_orders_with_solution(result::OptResult, data::StationSelectionData)
    return annotate_orders_with_solution(result, result.mapping, data)
end


# =============================================================================
# TwoStageSingleDetourMap Implementation
# =============================================================================

"""
    annotate_orders_with_solution(result::OptResult, mapping::TwoStageSingleDetourMap, data::StationSelectionData) -> DataFrame

Annotate orders for TwoStageSingleDetourModel.
"""
function annotate_orders_with_solution(
    result::OptResult,
    mapping::TwoStageSingleDetourMap,
    data::StationSelectionData
)
    m = result.model
    detour_combos = result.detour_combos

    # Pre-extract assignment variable values
    assignments = extract_assignments(m, mapping)

    # Pre-extract pooling if available
    same_source_pooling = Dict{Tuple{Int, Int}, Vector{NamedTuple}}()
    same_dest_pooling = Dict{Tuple{Int, Int}, Vector{NamedTuple}}()

    if !isnothing(detour_combos) && haskey(m.obj_dict, :u)
        same_source_pooling = extract_same_source_pooling(m, mapping, detour_combos.same_source)
    end
    if !isnothing(detour_combos) && haskey(m.obj_dict, :v)
        same_dest_pooling = extract_same_dest_pooling(m, mapping, detour_combos.same_dest)
    end

    # Process each scenario
    all_rows = []

    for (s, scenario) in enumerate(data.scenarios)
        scenario_start_time = scenario.start_time
        time_window = mapping.time_window

        for row in eachrow(scenario.requests)
            order_id = row.order_id
            o = row.start_station_id
            d = row.end_station_id

            # Compute time_id
            time_id = compute_time_id(row.request_time, scenario_start_time, time_window)

            # Find assignment for this order
            assignment = find_assignment(assignments, s, time_id, o, d)

            if isnothing(assignment)
                # No assignment found - add row with missing values
                push!(all_rows, create_unassigned_row(row, s, time_id))
                continue
            end

            j_id, k_id = assignment.pickup_id, assignment.dropoff_id

            # Calculate walking distances
            walking_pickup = get_walking_cost(data, o, j_id)
            walking_dropoff = get_walking_cost(data, k_id, d)
            walking_total = walking_pickup + walking_dropoff

            # Calculate direct in-vehicle time
            ivt_direct = has_routing_costs(data) ? get_routing_cost(data, j_id, k_id) : 0.0

            # Check for pooling
            pooling_info = find_pooling_for_order(
                same_source_pooling, same_dest_pooling,
                s, time_id, j_id, k_id
            )

            if isnothing(pooling_info)
                # Not pooled
                push!(all_rows, create_annotated_row(
                    row, s, time_id, j_id, k_id,
                    walking_pickup, walking_dropoff, walking_total,
                    ivt_direct, false, missing, missing, missing,
                    missing, missing, missing, missing, ivt_direct, 0.0
                ))
            else
                # Pooled - calculate actual in-vehicle time
                ivt_actual, detour_time = calculate_pooled_ivt(
                    data, pooling_info, j_id, k_id
                )

                push!(all_rows, create_annotated_row(
                    row, s, time_id, j_id, k_id,
                    walking_pickup, walking_dropoff, walking_total,
                    ivt_direct, true, pooling_info.type, pooling_info.role,
                    pooling_info.xi_idx, pooling_info.j_id, pooling_info.k_id,
                    pooling_info.l_id, pooling_info.time_delta, ivt_actual, detour_time
                ))
            end
        end
    end

    return DataFrame(all_rows)
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
            o = row.start_station_id
            d = row.end_station_id

            # Find assignment for this order
            assignment = find_assignment_clustering(assignments, s, o, d)

            if isnothing(assignment)
                push!(all_rows, create_unassigned_row_clustering(row, s))
                continue
            end

            j_id, k_id = assignment.pickup_id, assignment.dropoff_id

            # Calculate walking distances
            walking_pickup = get_walking_cost(data, o, j_id)
            walking_dropoff = get_walking_cost(data, k_id, d)
            walking_total = walking_pickup + walking_dropoff

            # Calculate direct in-vehicle time
            ivt_direct = has_routing_costs(data) ? get_routing_cost(data, j_id, k_id) : 0.0

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
Extract all active assignments from solved model (TwoStageSingleDetourMap).
Returns Dict[(s, t, o, d)] → (pickup_id, dropoff_id)
"""
function extract_assignments(m::JuMP.Model, mapping::TwoStageSingleDetourMap)
    if !haskey(m.obj_dict, :x)
        return Dict{Tuple{Int, Int, Int, Int}, NamedTuple}()
    end

    x = m[:x]
    assignments = Dict{Tuple{Int, Int, Int, Int}, NamedTuple}()
    use_sparse = has_walking_distance_limit(mapping)

    for (s, time_dict) in enumerate(x)
        for (t, od_dict) in time_dict
            for (od, x_od) in od_dict
                o, d = od

                if use_sparse
                    valid_pairs = get_valid_jk_pairs(mapping, o, d)
                    idx = findfirst(i -> JuMP.value(x_od[i]) > 0.5, eachindex(x_od))
                    if !isnothing(idx)
                        j, k = valid_pairs[idx]
                        j_id = mapping.array_idx_to_station_id[j]
                        k_id = mapping.array_idx_to_station_id[k]
                        assignments[(s, t, o, d)] = (pickup_id=j_id, dropoff_id=k_id)
                    end
                else
                    n = size(x_od, 1)
                    for j in 1:n, k in 1:n
                        if JuMP.value(x_od[j, k]) > 0.5
                            j_id = mapping.array_idx_to_station_id[j]
                            k_id = mapping.array_idx_to_station_id[k]
                            assignments[(s, t, o, d)] = (pickup_id=j_id, dropoff_id=k_id)
                            break
                        end
                    end
                end
            end
        end
    end

    return assignments
end


"""
Extract assignments for ClusteringTwoStageODMap.
Returns Dict[(s, o, d)] → (pickup_id, dropoff_id)
"""
function extract_assignments_clustering(m::JuMP.Model, mapping::ClusteringTwoStageODMap)
    if !haskey(m.obj_dict, :x)
        return Dict{Tuple{Int, Int, Int}, NamedTuple}()
    end

    x = m[:x]
    assignments = Dict{Tuple{Int, Int, Int}, NamedTuple}()
    use_sparse = has_walking_distance_limit(mapping)

    for (s, x_s) in enumerate(x)
        od_pairs = mapping.Omega_s[s]
        for (od_idx, x_od) in x_s
            o, d = od_pairs[od_idx]

            if x_od isa Vector
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                idx = findfirst(i -> JuMP.value(x_od[i]) > 0.5, eachindex(x_od))
                if !isnothing(idx)
                    j, k = valid_pairs[idx]
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]
                    assignments[(s, o, d)] = (pickup_id=j_id, dropoff_id=k_id)
                end
            else
                n = size(x_od, 1)
                for j in 1:n, k in 1:n
                    if JuMP.value(x_od[j, k]) > 0.5
                        j_id = mapping.array_idx_to_station_id[j]
                        k_id = mapping.array_idx_to_station_id[k]
                        assignments[(s, o, d)] = (pickup_id=j_id, dropoff_id=k_id)
                        break
                    end
                end
            end
        end
    end

    return assignments
end


"""
Find assignment for a specific order.
"""
function find_assignment(assignments::Dict, s::Int, t::Int, o::Int, d::Int)
    return get(assignments, (s, t, o, d), nothing)
end


"""
Find assignment for a specific order (clustering model).
"""
function find_assignment_clustering(assignments::Dict, s::Int, o::Int, d::Int)
    return get(assignments, (s, o, d), nothing)
end


# =============================================================================
# Helper Functions - Pooling Extraction
# =============================================================================

"""
Extract active same-source pooling variables.
Returns Dict[(s, t)] → Vector of activated pooling tuples
"""
function extract_same_source_pooling(
    m::JuMP.Model,
    mapping::TwoStageSingleDetourMap,
    Xi_same_source::Vector{Tuple{Int, Int, Int}}
)
    u = m[:u]
    use_sparse = has_walking_distance_limit(mapping)
    pooling = Dict{Tuple{Int, Int}, Vector{NamedTuple}}()

    for (s, time_dict) in enumerate(u)
        for (t, u_st) in time_dict
            if isempty(u_st)
                continue
            end

            activated = NamedTuple[]

            if use_sparse
                feasible_indices = get(mapping.feasible_same_source[s], t, Int[])
                for (local_idx, xi_idx) in enumerate(feasible_indices)
                    if local_idx <= length(u_st) && JuMP.value(u_st[local_idx]) > 0.5
                        j_id, k_id, l_id = Xi_same_source[xi_idx]
                        push!(activated, (xi_idx=xi_idx, j_id=j_id, k_id=k_id, l_id=l_id))
                    end
                end
            else
                for (idx, var) in enumerate(u_st)
                    if idx <= length(Xi_same_source) && JuMP.value(var) > 0.5
                        j_id, k_id, l_id = Xi_same_source[idx]
                        push!(activated, (xi_idx=idx, j_id=j_id, k_id=k_id, l_id=l_id))
                    end
                end
            end

            if !isempty(activated)
                pooling[(s, t)] = activated
            end
        end
    end

    return pooling
end


"""
Extract active same-dest pooling variables.
Returns Dict[(s, t)] → Vector of activated pooling tuples
"""
function extract_same_dest_pooling(
    m::JuMP.Model,
    mapping::TwoStageSingleDetourMap,
    Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
)
    v = m[:v]
    use_sparse = has_walking_distance_limit(mapping)
    pooling = Dict{Tuple{Int, Int}, Vector{NamedTuple}}()

    for (s, time_dict) in enumerate(v)
        for (t, v_st) in time_dict
            if isempty(v_st)
                continue
            end

            activated = NamedTuple[]

            if use_sparse
                feasible_indices = get(mapping.feasible_same_dest[s], t, Int[])
                for (local_idx, xi_idx) in enumerate(feasible_indices)
                    if local_idx <= length(v_st) && JuMP.value(v_st[local_idx]) > 0.5
                        j_id, k_id, l_id, time_delta = Xi_same_dest[xi_idx]
                        push!(activated, (xi_idx=xi_idx, j_id=j_id, k_id=k_id, l_id=l_id, time_delta=time_delta))
                    end
                end
            else
                for (idx, var) in enumerate(v_st)
                    if idx <= length(Xi_same_dest) && JuMP.value(var) > 0.5
                        j_id, k_id, l_id, time_delta = Xi_same_dest[idx]
                        push!(activated, (xi_idx=idx, j_id=j_id, k_id=k_id, l_id=l_id, time_delta=time_delta))
                    end
                end
            end

            if !isempty(activated)
                pooling[(s, t)] = activated
            end
        end
    end

    return pooling
end


"""
Find if order (j_id → k_id) participates in any pooling at (s, t).
"""
function find_pooling_for_order(
    same_source_pooling::Dict,
    same_dest_pooling::Dict,
    s::Int, t::Int, j_id::Int, k_id::Int
)
    # Check same-source pooling: trips (j→k) and (j→l) are pooled, route is j→k→l
    # Order j→k is "primary" (dropped first, no detour)
    # Order j→l is "secondary" (dropped at l via k, experiences detour)
    if haskey(same_source_pooling, (s, t))
        for p in same_source_pooling[(s, t)]
            if p.j_id == j_id && p.k_id == k_id
                # This order j→k is primary (dropped first)
                return (type="same_source", role="primary",
                        xi_idx=p.xi_idx, j_id=p.j_id, k_id=p.k_id, l_id=p.l_id, time_delta=missing)
            elseif p.j_id == j_id && p.l_id == k_id
                # This order j→l is secondary (experiences detour via k)
                return (type="same_source", role="secondary",
                        xi_idx=p.xi_idx, j_id=p.j_id, k_id=p.k_id, l_id=p.l_id, time_delta=missing)
            end
        end
    end

    # Check same-dest pooling: trips (j→l) at t and (k→l) at t+t' pooled, route is j→k→l
    # Order j→l is "primary" (picked up first, experiences detour via k)
    # Order k→l is "secondary" (picked up at k, direct to l)
    if haskey(same_dest_pooling, (s, t))
        for p in same_dest_pooling[(s, t)]
            if p.j_id == j_id && p.l_id == k_id
                # Wait, need to check if this order is j→l (primary) or k→l (secondary)
                # At time t, order is j→l (primary)
                return (type="same_dest", role="primary",
                        xi_idx=p.xi_idx, j_id=p.j_id, k_id=p.k_id, l_id=p.l_id, time_delta=p.time_delta)
            end
        end
    end

    # Also check if this order is the secondary in a same-dest pooling (at time t+t')
    for ((s2, t2), poolings) in same_dest_pooling
        if s2 != s
            continue
        end
        for p in poolings
            # Order k→l at time t2+time_delta is secondary
            if p.k_id == j_id && p.l_id == k_id && t == t2 + p.time_delta
                return (type="same_dest", role="secondary",
                        xi_idx=p.xi_idx, j_id=p.j_id, k_id=p.k_id, l_id=p.l_id, time_delta=p.time_delta)
            end
        end
    end

    return nothing
end


"""
Calculate actual in-vehicle time for a pooled order.
Returns (actual_ivt, detour_time).
"""
function calculate_pooled_ivt(data::StationSelectionData, pooling_info, j_id::Int, k_id::Int)
    if !has_routing_costs(data)
        return (0.0, 0.0)
    end

    # Direct time from assigned pickup to dropoff
    direct_time = get_routing_cost(data, j_id, k_id)

    if pooling_info.type == "same_source"
        if pooling_info.role == "primary"
            # Primary: j→k, dropped first, no detour
            return (direct_time, 0.0)
        else
            # Secondary: j→l, experiences detour via k
            # Actual route: j→k→l, order wants j→l
            actual_time = get_routing_cost(data, pooling_info.j_id, pooling_info.k_id) +
                          get_routing_cost(data, pooling_info.k_id, pooling_info.l_id)
            direct_jl = get_routing_cost(data, pooling_info.j_id, pooling_info.l_id)
            detour_time = actual_time - direct_jl
            return (actual_time, detour_time)
        end
    elseif pooling_info.type == "same_dest"
        if pooling_info.role == "primary"
            # Primary: j→l at time t, picked first, experiences detour via k
            # Actual route: j→k→l, order wants j→l
            actual_time = get_routing_cost(data, pooling_info.j_id, pooling_info.k_id) +
                          get_routing_cost(data, pooling_info.k_id, pooling_info.l_id)
            direct_jl = get_routing_cost(data, pooling_info.j_id, pooling_info.l_id)
            detour_time = actual_time - direct_jl
            return (actual_time, detour_time)
        else
            # Secondary: k→l, picked second, direct to l
            return (direct_time, 0.0)
        end
    end

    return (direct_time, 0.0)
end


# =============================================================================
# Helper Functions - Time Computation
# =============================================================================

"""
Compute time_id for an order.
"""
function compute_time_id(request_time, scenario_start_time::DateTime, time_window::Int)
    if request_time isa String
        request_time = DateTime(request_time, "yyyy-mm-dd HH:MM:SS")
    end
    time_diff_seconds = (request_time - scenario_start_time) / Dates.Second(1)
    return floor(Int, time_diff_seconds / time_window)
end

function compute_time_id(request_time, scenario_start_time::Nothing, time_window::Int)
    return 0  # No time window, single time slot
end


# =============================================================================
# Helper Functions - Row Creation
# =============================================================================

"""
Create annotated row with all solution columns.
"""
function create_annotated_row(
    original_row, s, time_id, pickup_id, dropoff_id,
    walking_pickup, walking_dropoff, walking_total,
    ivt_direct, is_pooled, pooling_type, pooling_role,
    pooling_xi_idx, pooling_j_id, pooling_k_id, pooling_l_id,
    pooling_time_delta, ivt_actual, detour_time
)
    row_dict = Dict{Symbol, Any}()

    # Copy original columns
    for col in propertynames(original_row)
        row_dict[col] = original_row[col]
    end

    # Add solution columns
    row_dict[:scenario_idx] = s
    row_dict[:time_id] = time_id
    row_dict[:assigned_pickup_id] = pickup_id
    row_dict[:assigned_dropoff_id] = dropoff_id
    row_dict[:walking_distance_pickup] = walking_pickup
    row_dict[:walking_distance_dropoff] = walking_dropoff
    row_dict[:walking_distance_total] = walking_total
    row_dict[:in_vehicle_time_direct] = ivt_direct
    row_dict[:is_pooled] = is_pooled
    row_dict[:pooling_type] = pooling_type
    row_dict[:pooling_role] = pooling_role
    row_dict[:pooling_xi_idx] = pooling_xi_idx
    row_dict[:pooling_j_id] = pooling_j_id
    row_dict[:pooling_k_id] = pooling_k_id
    row_dict[:pooling_l_id] = pooling_l_id
    row_dict[:pooling_time_delta] = pooling_time_delta
    row_dict[:in_vehicle_time_actual] = ivt_actual
    row_dict[:detour_time] = detour_time

    return row_dict
end


"""
Create row for unassigned order.
"""
function create_unassigned_row(original_row, s, time_id)
    row_dict = Dict{Symbol, Any}()

    for col in propertynames(original_row)
        row_dict[col] = original_row[col]
    end

    row_dict[:scenario_idx] = s
    row_dict[:time_id] = time_id
    row_dict[:assigned_pickup_id] = missing
    row_dict[:assigned_dropoff_id] = missing
    row_dict[:walking_distance_pickup] = missing
    row_dict[:walking_distance_dropoff] = missing
    row_dict[:walking_distance_total] = missing
    row_dict[:in_vehicle_time_direct] = missing
    row_dict[:is_pooled] = false
    row_dict[:pooling_type] = missing
    row_dict[:pooling_role] = missing
    row_dict[:pooling_xi_idx] = missing
    row_dict[:pooling_j_id] = missing
    row_dict[:pooling_k_id] = missing
    row_dict[:pooling_l_id] = missing
    row_dict[:pooling_time_delta] = missing
    row_dict[:in_vehicle_time_actual] = missing
    row_dict[:detour_time] = missing

    return row_dict
end


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
- `with_pooling::Bool`: If true, use flow variables (accounts for pooling savings).
                        If false, sum direct pickup-dropoff routes for each assignment.
"""
function calculate_model_vehicle_routing_distance(
    result::OptResult,
    data::StationSelectionData;
    with_pooling::Bool=true
)
    return calculate_model_vehicle_routing_distance(result, result.mapping, data; with_pooling=with_pooling)
end


"""
Vehicle routing distance for TwoStageSingleDetourMap.
"""
function calculate_model_vehicle_routing_distance(
    result::OptResult,
    mapping::TwoStageSingleDetourMap,
    data::StationSelectionData;
    with_pooling::Bool=true
)
    if !has_routing_costs(data)
        @warn "Routing costs not available, returning 0"
        return 0.0
    end

    m = result.model

    if with_pooling
        # Use flow variables f[s][t][j,k] to get actual vehicle movements
        return calculate_vrd_from_flows(m, mapping, data)
    else
        # Sum direct routes from assignments
        return calculate_vrd_from_assignments(m, mapping, data)
    end
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
Calculate VRD from flow variables.
"""
function calculate_vrd_from_flows(m::JuMP.Model, mapping::TwoStageSingleDetourMap, data::StationSelectionData)
    if !haskey(m.obj_dict, :f)
        return 0.0
    end

    f = m[:f]
    total_distance = 0.0

    for (s, time_dict) in enumerate(f)
        for (t, f_st) in time_dict
            if f_st isa Dict
                for ((j, k), var) in f_st
                    val = JuMP.value(var)
                    if val > 0.5
                        j_id = mapping.array_idx_to_station_id[j]
                        k_id = mapping.array_idx_to_station_id[k]
                        total_distance += get_routing_cost(data, j_id, k_id) * val
                    end
                end
            else
                n = size(f_st, 1)
                for j in 1:n, k in 1:n
                    val = JuMP.value(f_st[j, k])
                    if val > 0.5
                        j_id = mapping.array_idx_to_station_id[j]
                        k_id = mapping.array_idx_to_station_id[k]
                        total_distance += get_routing_cost(data, j_id, k_id) * val
                    end
                end
            end
        end
    end

    return total_distance
end


"""
Calculate VRD from assignment variables (no pooling).
"""
function calculate_vrd_from_assignments(m::JuMP.Model, mapping::TwoStageSingleDetourMap, data::StationSelectionData)
    if !haskey(m.obj_dict, :x)
        return 0.0
    end

    x = m[:x]
    use_sparse = has_walking_distance_limit(mapping)
    total_distance = 0.0

    for (s, time_dict) in enumerate(x)
        for (t, od_dict) in time_dict
            for (od, x_od) in od_dict
                o, d = od

                # Get demand count for this OD pair at this time
                q = get(get(mapping.Q_s_t[s], t, Dict{Tuple{Int,Int}, Int}()), od, 0)

                if use_sparse
                    valid_pairs = get_valid_jk_pairs(mapping, o, d)
                    for (pair_idx, var) in enumerate(x_od)
                        val = JuMP.value(var)
                        if val > 0.5
                            j, k = valid_pairs[pair_idx]
                            j_id = mapping.array_idx_to_station_id[j]
                            k_id = mapping.array_idx_to_station_id[k]
                            # Multiply by demand count (each request gets its own vehicle trip without pooling)
                            total_distance += get_routing_cost(data, j_id, k_id) * q
                            break
                        end
                    end
                else
                    n = size(x_od, 1)
                    for j in 1:n, k in 1:n
                        val = JuMP.value(x_od[j, k])
                        if val > 0.5
                            j_id = mapping.array_idx_to_station_id[j]
                            k_id = mapping.array_idx_to_station_id[k]
                            total_distance += get_routing_cost(data, j_id, k_id) * q
                            break
                        end
                    end
                end
            end
        end
    end

    return total_distance
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
    use_sparse = has_walking_distance_limit(mapping)
    total_distance = 0.0

    for (s, x_s) in enumerate(x)
        od_pairs = mapping.Omega_s[s]
        for (od_idx, x_od) in x_s
            o, d = od_pairs[od_idx]

            # Get demand count (Q_s uses OD tuple as key)
            q = get(mapping.Q_s[s], (o, d), 1)

            if x_od isa Vector
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                idx = findfirst(i -> JuMP.value(x_od[i]) > 0.5, eachindex(x_od))
                if !isnothing(idx)
                    j, k = valid_pairs[idx]
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]
                    total_distance += get_routing_cost(data, j_id, k_id) * q
                end
            else
                n = size(x_od, 1)
                for j in 1:n, k in 1:n
                    if JuMP.value(x_od[j, k]) > 0.5
                        j_id = mapping.array_idx_to_station_id[j]
                        k_id = mapping.array_idx_to_station_id[k]
                        total_distance += get_routing_cost(data, j_id, k_id) * q
                        break
                    end
                end
            end
        end
    end

    return total_distance
end
