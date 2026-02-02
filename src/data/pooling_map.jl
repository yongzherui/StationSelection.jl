"""
Pooling scenario origin-destination time mapping for TwoStageSingleDetourModel.

This module provides data structures for mapping scenarios and time windows
to origin-destination pairs for pooling optimization.

Note: Detour combinations (Xi) are computed separately via find_detour_combinations,
find_same_source_detour_combinations, and find_same_dest_detour_combinations
in detour_combinations.jl.
"""

using DataFrames
using Dates

export TwoStageSingleDetourMap
export create_two_stage_single_detour_map
export create_station_id_mappings, create_scenario_label_mappings
export compute_time_to_od_count_mapping
export has_walking_distance_limit, get_valid_jk_pairs, get_valid_f_pairs
export get_max_walking_distance
export get_feasible_same_source_indices, get_feasible_same_dest_indices


"""
    TwoStageSingleDetourMap

Maps scenarios, time windows, and origin-destination pairs for pooling optimization.

# Fields
- `station_id_to_array_idx::Dict{Int, Int}`: Station ID → array index mapping
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID mapping
- `scenarios::Vector{ScenarioData}`: Reference to scenario data
- `scenario_label_to_array_idx::Dict{String, Int}`: Scenario label → array index
- `array_idx_to_scenario_label::Vector{String}`: Array index → scenario label
- `time_window::Int`: Time discretization window in seconds
- `Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}`: Maps (scenario, time) → OD pairs
- `Q_s_t::Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}`: Demand count q_{od,s,t} - number of requests per OD pair per scenario per time
- `max_walking_distance::Union{Float64, Nothing}`: Maximum walking distance constraint (optional)
- `valid_jk_pairs::Dict{Tuple{Int,Int}, Vector{Tuple{Int,Int}}}`: Maps OD pair (o,d) → valid (j,k) station pairs (array indices).
  Only populated when max_walking_distance is set. j is valid if walking_cost(o,j) ≤ max_walking_distance,
  k is valid if walking_cost(k,d) ≤ max_walking_distance.
- `valid_f_pairs_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}`: Maps (scenario, time) → valid (j,k) pairs.
  Only populated when max_walking_distance is set.
- `feasible_same_source::Dict{Int, Dict{Int, Vector{Int}}}`: Maps (scenario, time) → feasible same-source detour indices.
  Only populated when max_walking_distance is set.
- `feasible_same_dest::Dict{Int, Dict{Int, Vector{Int}}}`: Maps (scenario, time) → feasible same-dest detour indices.
  Only populated when max_walking_distance is set.
"""
struct TwoStageSingleDetourMap <: AbstractPoolingMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    time_window::Int

    # Omega[scenario_id][time_id] = [(o1, d1), (o2, d2), ...]
    Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}

    # Q[scenario_id][time_id][(o, d)] = count of requests for OD pair (o,d)
    Q_s_t::Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}

    # Walking distance constraint
    max_walking_distance::Union{Float64, Nothing}

    # valid_jk_pairs[(o, d)] = [(j1, k1), (j2, k2), ...] as array indices
    # Only populated when max_walking_distance is set
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    # Union of valid (j, k) pairs across ODs for each (scenario, time)
    # Only populated when max_walking_distance is set
    valid_f_pairs_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}

    # Feasible detour indices for each (scenario, time)
    # Only populated when max_walking_distance is set
    feasible_same_source::Dict{Int, Dict{Int, Vector{Int}}}
    feasible_same_dest::Dict{Int, Dict{Int, Vector{Int}}}
end

"""
    create_station_id_mappings(station_ids::Vector{Int}) -> (Dict{Int, Int}, Vector{Int})

Create bidirectional mappings between station IDs and array indices.

# Returns
- Tuple of (id_to_idx::Dict, idx_to_id::Vector)
"""
function create_station_id_mappings(station_ids::Vector{Int})
    station_id_to_array_idx = Dict{Int, Int}()
    array_idx_to_station_id = Vector{Int}()

    for (idx, station_id) in enumerate(station_ids)
        station_id_to_array_idx[station_id] = idx
        push!(array_idx_to_station_id, station_id)
    end

    return station_id_to_array_idx, array_idx_to_station_id
end


"""
    create_scenario_label_mappings(scenarios::Vector{ScenarioData}) -> (Dict{String, Int}, Vector{String})

Create bidirectional mappings between scenario labels and array indices.

# Returns
- Tuple of (label_to_idx::Dict, idx_to_label::Vector)
"""
function create_scenario_label_mappings(scenarios::Vector{ScenarioData})
    scenario_label_to_array_idx = Dict{String, Int}()
    array_idx_to_scenario_label = Vector{String}()

    for (idx, scenario) in enumerate(scenarios)
        scenario_label_to_array_idx[scenario.label] = idx
        push!(array_idx_to_scenario_label, scenario.label)
    end

    return scenario_label_to_array_idx, array_idx_to_scenario_label
end


"""
    compute_time_to_od_count_mapping(
        scenario_data::ScenarioData,
        time_window::Int;
        time_column::Symbol=:request_time
    ) -> Dict{Int, Dict{Tuple{Int, Int}, Int}}

Compute mapping from time IDs to origin-destination pair counts for a single scenario.

The time_id is computed as floor((request_time - scenario_start_time) / time_window).

# Arguments
- `scenario_data::ScenarioData`: Scenario containing requests
- `time_window::Int`: Time discretization window in seconds
- `time_column::Symbol`: Column name for request time (default :request_time)

# Returns
- Dict mapping time_id → Dict mapping (origin, destination) → count
"""
function compute_time_to_od_count_mapping(
    scenario_data::ScenarioData,
    time_window::Int;
    time_column::Symbol=:request_time
)::Dict{Int, Dict{Tuple{Int, Int}, Int}}

    time_to_od_count = Dict{Int, Dict{Tuple{Int, Int}, Int}}()

    scenario_start_time = scenario_data.start_time
    if isnothing(scenario_start_time)
        error("scenario_start_time cannot be nothing for time-based OD mapping")
    end

    for row in eachrow(scenario_data.requests)
        o = row.start_station_id
        d = row.end_station_id

        # Get the request time
        request_time = row[time_column]

        # Handle both DateTime and String types
        if request_time isa String
            request_time = DateTime(request_time, "yyyy-mm-dd HH:MM:SS")
        end

        # Compute time relative to scenario start in seconds
        time_diff_seconds = (request_time - scenario_start_time) / Dates.Second(1)
        time_id = floor(Int, time_diff_seconds / time_window)

        if !haskey(time_to_od_count, time_id)
            time_to_od_count[time_id] = Dict{Tuple{Int, Int}, Int}()
        end

        od_pair = (o, d)
        time_to_od_count[time_id][od_pair] = get(time_to_od_count[time_id], od_pair, 0) + 1
    end

    return time_to_od_count
end


"""
    compute_valid_jk_pairs(
        od_pairs::Set{Tuple{Int, Int}},
        data::StationSelectionData,
        station_id_to_array_idx::Dict{Int, Int},
        array_idx_to_station_id::Vector{Int},
        max_walking_distance::Union{Float64, Nothing}
    ) -> Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

Compute valid (j, k) pickup/dropoff station pairs for each OD pair based on walking distance.
If max_walking_distance is nothing, all (j, k) pairs are returned for each OD pair.

For an OD pair (o, d):
- j is a valid pickup station if walking_cost(o, j) ≤ max_walking_distance
- k is a valid dropoff station if walking_cost(k, d) ≤ max_walking_distance
- The pair (j, k) is valid if both j and k are valid

Returns a dictionary mapping (o, d) → [(j1, k1), (j2, k2), ...] where j, k are array indices.
"""
function compute_valid_jk_pairs(
    od_pairs::Set{Tuple{Int, Int}},
    data::StationSelectionData,
    station_id_to_array_idx::Dict{Int, Int},
    array_idx_to_station_id::Vector{Int},
    max_walking_distance::Union{Float64, Nothing}
)::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    n = length(array_idx_to_station_id)
    valid_jk_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()

    if isnothing(max_walking_distance)
        all_pairs = [(j, k) for j in 1:n for k in 1:n]
        for od in od_pairs
            valid_jk_pairs[od] = all_pairs
        end
        return valid_jk_pairs
    end

    for (o, d) in od_pairs
        # Find valid pickup stations j for origin o
        valid_j = Int[]
        for j in 1:n
            j_id = array_idx_to_station_id[j]
            if get_walking_cost(data, o, j_id) <= max_walking_distance
                push!(valid_j, j)
            end
        end

        # Find valid dropoff stations k for destination d
        valid_k = Int[]
        for k in 1:n
            k_id = array_idx_to_station_id[k]
            if get_walking_cost(data, k_id, d) <= max_walking_distance
                push!(valid_k, k)
            end
        end

        # Generate all valid (j, k) pairs
        pairs = Tuple{Int, Int}[]
        for j in valid_j
            for k in valid_k
                push!(pairs, (j, k))
            end
        end

        valid_jk_pairs[(o, d)] = pairs
    end

    return valid_jk_pairs
end


"""
    create_two_stage_single_detour_map(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData;
        Xi_same_source::Vector{Tuple{Int, Int, Int}}=Tuple{Int, Int, Int}[],
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}=Tuple{Int, Int, Int, Int}[]
    ) -> TwoStageSingleDetourMap

Create a pooling scenario map with OD pairs organized by scenario and time.

If model.max_walking_distance is set, also computes:
- Valid (j, k) station pairs for each OD pair based on walking distance constraints
- Feasible detour indices for same-source and same-dest detours

# Arguments
- `model`: The TwoStageSingleDetourModel
- `data`: Station selection data
- `Xi_same_source`: Same-source detour triplets (optional, needed for feasible detour computation)
- `Xi_same_dest`: Same-dest detour quadruplets (optional, needed for feasible detour computation)
"""
function create_two_stage_single_detour_map(
    model::TwoStageSingleDetourModel,
    data::StationSelectionData;
    Xi_same_source::Vector{Tuple{Int, Int, Int}}=Tuple{Int, Int, Int}[],
    Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}=Tuple{Int, Int, Int, Int}[]
)::TwoStageSingleDetourMap

    # Create station ID mappings
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    # Create scenario label mappings
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    time_window = floor(Int, model.time_window)

    # Compute Omega_s_t and Q_s_t for all scenarios
    Omega_s_t = Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}()
    Q_s_t = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}()

    # Collect all unique OD pairs across all scenarios for valid_jk_pairs computation
    all_od_pairs = Set{Tuple{Int, Int}}()

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        # Get OD pair counts (single pass through data)
        time_to_od_count = compute_time_to_od_count_mapping(scenario_data, time_window)

        # Derive unique OD pairs from count dictionary keys
        Omega_s_t[scenario_id] = Dict{Int, Vector{Tuple{Int, Int}}}()
        for (time_id, od_count_dict) in time_to_od_count
            od_pairs = collect(keys(od_count_dict))
            Omega_s_t[scenario_id][time_id] = od_pairs
            union!(all_od_pairs, od_pairs)
        end

        # Store the counts
        Q_s_t[scenario_id] = time_to_od_count
    end

    max_walking_distance = model.max_walking_distance
    valid_jk_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()
    valid_f_pairs_s_t = Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}()
    feasible_same_source = Dict{Int, Dict{Int, Vector{Int}}}()
    feasible_same_dest = Dict{Int, Dict{Int, Vector{Int}}}()

    if !isnothing(max_walking_distance)
        valid_jk_pairs = compute_valid_jk_pairs(
            all_od_pairs, data, station_id_to_array_idx, array_idx_to_station_id, max_walking_distance
        )
        # Build per-(scenario,time) valid f pairs as union of OD valid pairs
        for (scenario_id, time_dict) in Omega_s_t
            valid_f_pairs_s_t[scenario_id] = Dict{Int, Vector{Tuple{Int, Int}}}()
            for (time_id, od_vector) in time_dict
                pair_set = Set{Tuple{Int, Int}}()
                for od in od_vector
                    for pair in get(valid_jk_pairs, od, Tuple{Int, Int}[])
                        push!(pair_set, pair)
                    end
                end
                valid_f_pairs_s_t[scenario_id][time_id] = collect(pair_set)
            end
        end

        # Compute feasible detours if Xi is provided
        if !isempty(Xi_same_source) || !isempty(Xi_same_dest)
            feasible_same_source, feasible_same_dest = compute_feasible_detours(
                Omega_s_t, valid_jk_pairs, station_id_to_array_idx,
                Xi_same_source, Xi_same_dest
            )
        end
    end

    return TwoStageSingleDetourMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        time_window,
        Omega_s_t,
        Q_s_t,
        max_walking_distance,
        valid_jk_pairs,
        valid_f_pairs_s_t,
        feasible_same_source,
        feasible_same_dest
    )


"""
    compute_feasible_detours(
        Omega_s_t, valid_jk_pairs, station_id_to_array_idx,
        Xi_same_source, Xi_same_dest
    ) -> (feasible_same_source, feasible_same_dest)

Compute which detour combinations are feasible for each (scenario, time) given walking limits.
"""
function compute_feasible_detours(
    Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}},
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    station_id_to_array_idx::Dict{Int, Int},
    Xi_same_source::Vector{Tuple{Int, Int, Int}},
    Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
)
    S = length(Omega_s_t)

    feasible_same_source = Dict{Int, Dict{Int, Vector{Int}}}()
    feasible_same_dest = Dict{Int, Dict{Int, Vector{Int}}}()

    for s in 1:S
        feasible_same_source[s] = Dict{Int, Vector{Int}}()
        feasible_same_dest[s] = Dict{Int, Vector{Int}}()

        for (time_id, od_vector) in Omega_s_t[s]
            # Same-source feasibility
            if length(od_vector) > 1 && !isempty(Xi_same_source)
                feasible_indices = Int[]
                for (idx, (j_id, k_id, l_id)) in enumerate(Xi_same_source)
                    j = station_id_to_array_idx[j_id]
                    k = station_id_to_array_idx[k_id]
                    l = station_id_to_array_idx[l_id]

                    # Check if edges (j,k) and (j,l) exist for any OD
                    has_jk = any(od -> (j, k) in get(valid_jk_pairs, od, Tuple{Int,Int}[]), od_vector)
                    has_jl = any(od -> (j, l) in get(valid_jk_pairs, od, Tuple{Int,Int}[]), od_vector)

                    if has_jk && has_jl
                        push!(feasible_indices, idx)
                    end
                end
                feasible_same_source[s][time_id] = feasible_indices
            else
                feasible_same_source[s][time_id] = Int[]
            end

            # Same-dest feasibility
            if !isempty(Xi_same_dest)
                feasible_indices = Int[]
                for (idx, (j_id, k_id, l_id, time_delta)) in enumerate(Xi_same_dest)
                    future_time_id = time_id + time_delta
                    if !haskey(Omega_s_t[s], future_time_id)
                        continue
                    end

                    future_od_vector = Omega_s_t[s][future_time_id]

                    j = station_id_to_array_idx[j_id]
                    k = station_id_to_array_idx[k_id]
                    l = station_id_to_array_idx[l_id]

                    # Check if edge (j,l) exists for any OD at time t
                    has_jl = any(od -> (j, l) in get(valid_jk_pairs, od, Tuple{Int,Int}[]), od_vector)
                    # Check if edge (k,l) exists for any OD at time t+t'
                    has_kl = any(od -> (k, l) in get(valid_jk_pairs, od, Tuple{Int,Int}[]), future_od_vector)

                    if has_jl && has_kl
                        push!(feasible_indices, idx)
                    end
                end
                feasible_same_dest[s][time_id] = feasible_indices
            else
                feasible_same_dest[s][time_id] = Int[]
            end
        end
    end

    return feasible_same_source, feasible_same_dest
end


"""
    has_walking_distance_limit(mapping::TwoStageSingleDetourMap) -> Bool

Check if the mapping has valid (j, k) pairs computed based on walking distance limits.
"""
has_walking_distance_limit(mapping::TwoStageSingleDetourMap) = !isnothing(mapping.max_walking_distance)

"""
    get_max_walking_distance(mapping::TwoStageSingleDetourMap) -> Union{Float64, Nothing}

Return the walking distance limit (if any) for this mapping.
"""
get_max_walking_distance(mapping::TwoStageSingleDetourMap) = mapping.max_walking_distance


"""
    get_valid_jk_pairs(mapping::TwoStageSingleDetourMap, o::Int, d::Int) -> Vector{Tuple{Int, Int}}

Get the valid (j, k) station pairs for an OD pair. Returns all pairs if no walking limit is set.
"""
function get_valid_jk_pairs(mapping::TwoStageSingleDetourMap, o::Int, d::Int)
    if has_walking_distance_limit(mapping)
        return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
    else
        # No limit - return all pairs
        n = length(mapping.array_idx_to_station_id)
        return [(j, k) for j in 1:n for k in 1:n]
    end
end


"""
    get_valid_f_pairs(mapping::TwoStageSingleDetourMap, s::Int, time_id::Int) -> Vector{Tuple{Int, Int}}

Get valid (j,k) pairs for a (scenario, time) based on OD walking limits.
Returns all pairs if no walking limit is set.
"""
function get_valid_f_pairs(mapping::TwoStageSingleDetourMap, s::Int, time_id::Int)
    if has_walking_distance_limit(mapping)
        return get(get(mapping.valid_f_pairs_s_t, s, Dict{Int,Vector{Tuple{Int, Int}}}()), time_id, Tuple{Int, Int}[])
    else
        n = length(mapping.array_idx_to_station_id)
        return [(j, k) for j in 1:n for k in 1:n]
    end
end


"""
    get_feasible_same_source_indices(mapping::TwoStageSingleDetourMap, s::Int, time_id::Int, n_same_source::Int) -> Vector{Int}

Get feasible same-source detour indices for a (scenario, time) pair.
Returns all indices 1:n_same_source if no walking limit is set.
"""
function get_feasible_same_source_indices(mapping::TwoStageSingleDetourMap, s::Int, time_id::Int, n_same_source::Int)
    if has_walking_distance_limit(mapping)
        return get(get(mapping.feasible_same_source, s, Dict{Int,Vector{Int}}()), time_id, Int[])
    else
        return collect(1:n_same_source)
    end
end


"""
    get_feasible_same_dest_indices(mapping::TwoStageSingleDetourMap, s::Int, time_id::Int, n_same_dest::Int) -> Vector{Int}

Get feasible same-dest detour indices for a (scenario, time) pair.
Returns indices based on time validity if no walking limit is set.
"""
function get_feasible_same_dest_indices(mapping::TwoStageSingleDetourMap, s::Int, time_id::Int, n_same_dest::Int)
    if has_walking_distance_limit(mapping)
        return get(get(mapping.feasible_same_dest, s, Dict{Int,Vector{Int}}()), time_id, Int[])
    else
        # No walking limit case is handled separately in variable creation
        return Int[]
    end
end
