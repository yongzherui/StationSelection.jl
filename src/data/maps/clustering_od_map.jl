"""
Clustering OD mapping for ClusteringTwoStageODFormulation/ClusteringTwoStageODFlowRegularizerFormulation.

This module provides data structures for mapping scenarios to origin-destination pairs
for the clustering two-stage optimization (without time dimension).
"""

using DataFrames

export ClusteringTwoStageODMap
export create_clustering_two_stage_od_map
export WALK_ONLY_PAIR
export is_walk_only_pair
export od_pair_walking_cost

"""
    WALK_ONLY_PAIR

Sentinel `(j, k)` pair meaning "no station/vehicle used — walk directly from
origin to destination." Station indices start at 1, so `(0, 0)` cannot collide
with a real station pair.
"""
const WALK_ONLY_PAIR = (0, 0)

is_walk_only_pair(pair::Tuple{Int, Int}) = pair == WALK_ONLY_PAIR

"""
    od_pair_walking_cost(data, o, d, pair) -> Float64

Walking cost of assigning OD pair (o, d) to station pair `pair`. For a real
station pair (j, k) this is walk(o, j) + walk(k, d). For [`WALK_ONLY_PAIR`]
it is the direct walk(o, d) — no station is used.
"""
function od_pair_walking_cost(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pair::Tuple{Int, Int},
)::Float64
    is_walk_only_pair(pair) && return get_walking_cost(data, o, d)
    j, k = pair
    return get_walking_cost(data, o, j) + get_walking_cost(data, k, d)
end

"""
    ClusteringTwoStageODMap

Maps scenarios to origin-destination pairs for clustering optimization.

# Fields
- `station_id_to_array_idx::Dict{Int, Int}`: Station ID → array index mapping
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID mapping
- `scenarios::Vector{ScenarioData}`: Reference to scenario data
- `scenario_label_to_array_idx::Dict{String, Int}`: Scenario label → array index
- `array_idx_to_scenario_label::Vector{String}`: Array index → scenario label
- `Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}`: Maps scenario → OD pairs with positive
  demand; position `p` within `Omega_s[s]` is the demand group's index.
- `Q_s::Dict{Int, Vector{Int}}`: Demand count `Q_s[s][p]` for demand group `p`, dense and
  parallel to `Omega_s[s]` (every `p` has positive demand by construction).
- `max_walking_distance::Float64`: Maximum walking distance constraint
- `valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}`: Maps OD pair (o,d) → valid (j,k) station pairs
"""
struct ClusteringTwoStageODMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    # Omega[scenario_id] = [(o1, d1), (o2, d2), ...]; position p -> (o,d)
    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}

    # Q[scenario_id][p] = count of requests for OD pair Omega_s[scenario_id][p]
    Q_s::Dict{Int, Vector{Int}}

    # Walking distance constraint
    max_walking_distance::Float64
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
end


"""
    compute_valid_jk_pairs(
        all_od_pairs, data, max_walking_distance
    ) -> Dict{Tuple{Int,Int}, Vector{Tuple{Int,Int}}}

For each OD index pair (origin_idx,dest_idx), compute which station index pairs
(pickup_idx, dropoff_idx) satisfy both walking distance limits.

`j == k` (same-station) pairs are never produced: boarding and alighting at the
same station needs no vehicle trip, and by the triangle inequality any `j` within
`max_walking_distance` of *both* `o` and `d` implies `dist(o,d) <= 2 *
max_walking_distance` -- exactly `WALK_ONLY_PAIR`'s own availability condition
below. So whenever a same-station pair would have been valid, `WALK_ONLY_PAIR`
already covers the same case at equal-or-lower cost and without requiring any
station to be open at all; same-station pairs would only ever add a
dominated, identically-costed alternative.
"""
function compute_valid_jk_pairs(
    all_od_pairs::Set{Tuple{Int, Int}},
    data::StationSelectionData,
    max_walking_distance::Float64;
    allow_walk_only::Bool=false,
)::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
    n = data.n_stations
    valid_jk_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()

    for (o, d) in all_od_pairs
        walk_only_available = allow_walk_only && get_walking_cost(data, o, d) <= 2 * max_walking_distance
        pairs = Tuple{Int, Int}[]
        for j in 1:n
            get_walking_cost(data, o, j) <= max_walking_distance || continue
            for k in 1:n
                # station pairs must be distinct: j==k would mean boarding and
                # alighting at the same station, i.e. no vehicle trip at all --
                # handled separately below via WALK_ONLY_PAIR when it's
                # available for this OD (see this function's docstring for why
                # a real same-station pair is never needed).
                j == k && continue
                get_walking_cost(data, k, d) <= max_walking_distance || continue
                push!(pairs, (j, k))
            end
        end
        # Walk-only option (opt-in): skip stations entirely if the direct walk
        # is within 2 * max_walking_distance (the same budget a
        # station-mediated trip would use: up to max_walking_distance on each
        # leg). Off by default — real walking costs obey the triangle
        # inequality, so this condition is satisfied almost any time a real
        # 2-hop station pair exists, and callers that don't expect a
        # station-free option (e.g. route-pool generation) must opt in
        # deliberately.
        if walk_only_available
            push!(pairs, WALK_ONLY_PAIR)
        end
        valid_jk_pairs[(o, d)] = pairs
    end

    return valid_jk_pairs
end


"""
    compute_scenario_od_count(scenario_data::ScenarioData) -> Dict{Tuple{Int, Int}, Int}

Compute OD pair demand counts for a single scenario (aggregated across all times).

# Arguments
- `scenario_data::ScenarioData`: Scenario containing requests

# Returns
- Dict mapping (origin_idx, dest_idx) → count
"""
function compute_scenario_od_count(scenario_data::ScenarioData)::Dict{Tuple{Int, Int}, Int}
    od_count = Dict{Tuple{Int, Int}, Int}()
    _require_indexed_request_columns(scenario_data.requests)

    for row in eachrow(scenario_data.requests)
        o = row.origin_idx
        d = row.dest_idx
        od_pair = (o, d)
        od_count[od_pair] = get(od_count, od_pair, 0) + 1
    end

    return od_count
end


"""
    create_clustering_two_stage_od_map(
        problem::StationSelectionProblem,
        formulation::AbstractClusteringTwoStageODFormulation,
        data::StationSelectionData
    ) -> ClusteringTwoStageODMap

Create a clustering scenario OD map with OD pairs organized by scenario. Shared by both
`ClusteringTwoStageODFormulation` and `ClusteringTwoStageODFlowRegularizerFormulation` --
flow regularization doesn't change the OD mapping, only the objective/variable/constraint
set `_build_clustering!` adds.
"""
function create_clustering_two_stage_od_map(
    problem::StationSelectionProblem,
    formulation::AbstractClusteringTwoStageODFormulation,
    data::StationSelectionData
)::ClusteringTwoStageODMap

    # Create scenario label mappings
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    # Compute Omega_s and Q_s for all scenarios
    Omega_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s = Dict{Int, Vector{Int}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        # Get OD pair counts (aggregated across all times)
        od_count = compute_scenario_od_count(scenario_data)

        # Store unique OD pairs (position p -> (o,d)) and their parallel demand counts
        Omega_s[scenario_id] = collect(keys(od_count))
        Q_s[scenario_id] = [od_count[pair] for pair in Omega_s[scenario_id]]
        union!(all_od_pairs, Omega_s[scenario_id])
    end

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs,
        data,
        problem.max_walking_distance
    )

    return ClusteringTwoStageODMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        problem.max_walking_distance,
        valid_jk_pairs
    )
end

"""
    has_walking_distance_limit(mapping::ClusteringTwoStageODMap) -> Bool

Check if the mapping has valid (j, k) pairs computed based on walking distance limits.
"""
has_walking_distance_limit(mapping::ClusteringTwoStageODMap) = true

"""
    get_valid_jk_pairs(mapping::ClusteringTwoStageODMap, o::Int, d::Int) -> Vector{Tuple{Int, Int}}

Get the valid (j, k) station pairs for an OD pair.
"""
function get_valid_jk_pairs(mapping::ClusteringTwoStageODMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
end
