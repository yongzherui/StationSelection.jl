"""
Robust OD map for RobustTotalDemandCapModel.

Stores the spatial structure (stations, scenarios, valid station pairs) alongside
the pre-calibrated demand bounds needed to build the robust counterpart.
"""

using DataFrames

export RobustTotalDemandCapMap
export create_robust_total_demand_cap_map

"""
    RobustTotalDemandCapMap <: AbstractClusteringMap

OD mapping for the robust total-demand-cap model.

# Fields (spatial — same as ClusteringTwoStageODMap)
- `station_id_to_array_idx`, `array_idx_to_station_id`: station ID ↔ index
- `scenarios`: reference to ScenarioData vector
- `scenario_label_to_array_idx`, `array_idx_to_scenario_label`
- `Omega_s`: active OD pairs per scenario index (union of all pairs with q̂ > 0)
- `max_walking_distance`, `valid_jk_pairs`

# Fields (robust)
- `q_low::Dict{Int, Dict{Tuple{Int,Int}, Float64}}` — q̲_ods per scenario
- `q_hat::Dict{Int, Dict{Tuple{Int,Int}, Float64}}` — q̂_ods per scenario
- `B::Vector{Float64}` — budget B_s per scenario
"""
struct RobustTotalDemandCapMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}

    max_walking_distance::Float64
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    # Robust demand data
    q_low::Dict{Int, Dict{Tuple{Int,Int}, Float64}}
    q_hat::Dict{Int, Dict{Tuple{Int,Int}, Float64}}
    B::Vector{Float64}
end


"""
    create_robust_total_demand_cap_map(
        model::RobustTotalDemandCapModel,
        data::StationSelectionData
    ) -> RobustTotalDemandCapMap

Build the robust OD map.

Omega_s is derived from q_hat: any OD pair for which q̂_ods > 0 in scenario s
is included (i.e., pairs that ever appear in the historical data for that period).
valid_jk_pairs uses the same walking-distance filter as ClusteringTwoStageODMap.
"""
function create_robust_total_demand_cap_map(
        model::RobustTotalDemandCapModel,
        data::StationSelectionData
    )::RobustTotalDemandCapMap

    S = length(data.scenarios)

    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    # Build Omega_s from q_hat: include all (o,d) with q_hat > 0 in that scenario
    Omega_s = Dict{Int, Vector{Tuple{Int,Int}}}()
    all_od_pairs = Set{Tuple{Int,Int}}()

    for s in 1:S
        q_hat_s = get(model.q_hat, s, Dict{Tuple{Int,Int}, Float64}())
        active = [(o, d) for ((o, d), v) in q_hat_s if v > 0.0]
        # Also include pairs that appear in q_low (even if q_hat = 0)
        q_low_s = get(model.q_low, s, Dict{Tuple{Int,Int}, Float64}())
        for od in keys(q_low_s)
            od in active || push!(active, od)
        end
        Omega_s[s] = active
        union!(all_od_pairs, active)
    end

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs, data, model.max_walking_distance
    )

    return RobustTotalDemandCapMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        model.max_walking_distance,
        valid_jk_pairs,
        model.q_low,
        model.q_hat,
        model.B,
    )
end

has_walking_distance_limit(::RobustTotalDemandCapMap) = true

function get_valid_jk_pairs(mapping::RobustTotalDemandCapMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int,Int}[])
end
