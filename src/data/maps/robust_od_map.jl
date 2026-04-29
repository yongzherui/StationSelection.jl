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

The model stores demand bounds indexed by *period* (1-4, one per time-of-day
window).  `data.scenarios` has one entry per (date × period) instance —
potentially hundreds of scenarios.  This function maps each scenario to its
period via `start_time`, then expands the period-indexed model bounds to
scenario-indexed ones so the rest of the build pipeline can iterate `1:S`.

Omega_s[s] = active OD pairs in scenario s (those with q̂ > 0 for that period).
valid_jk_pairs uses the same walking-distance filter as ClusteringTwoStageODMap.
"""

# Period windows matching demand_bounds.jl's _PERIOD_WINDOWS.
const _ROBUST_PERIOD_WINDOWS = [(6, 10), (10, 15), (15, 20), (20, 24)]

function _period_from_scenario(sc::ScenarioData)::Int
    h = isnothing(sc.start_time) ? -1 : Dates.hour(sc.start_time)
    for (idx, (lo, hi)) in enumerate(_ROBUST_PERIOD_WINDOWS)
        lo <= h < hi && return idx
    end
    return 1  # fallback: use morning bounds for any unclassified scenario
end

function create_robust_total_demand_cap_map(
        model::RobustTotalDemandCapModel,
        data::StationSelectionData
    )::RobustTotalDemandCapMap

    S = length(data.scenarios)

    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    # Map each scenario index → period (1-4) using start_time.
    # model.q_hat, q_low, B are all period-indexed (keys 1-4).
    scenario_to_period = [_period_from_scenario(data.scenarios[s]) for s in 1:S]

    # Build scenario-indexed Omega_s, q_low_map, q_hat_map, B_vec
    Omega_s     = Dict{Int, Vector{Tuple{Int,Int}}}()
    q_low_map   = Dict{Int, Dict{Tuple{Int,Int}, Float64}}()
    q_hat_map   = Dict{Int, Dict{Tuple{Int,Int}, Float64}}()
    B_vec       = Vector{Float64}(undef, S)
    all_od_pairs = Set{Tuple{Int,Int}}()

    for s in 1:S
        p = scenario_to_period[s]

        q_hat_p = get(model.q_hat, p, Dict{Tuple{Int,Int}, Float64}())
        q_low_p = get(model.q_low, p, Dict{Tuple{Int,Int}, Float64}())

        active = [(o, d) for ((o, d), v) in q_hat_p if v > 0.0]
        for od in keys(q_low_p)
            od ∉ active && push!(active, od)
        end

        Omega_s[s]   = active
        q_low_map[s] = q_low_p
        q_hat_map[s] = q_hat_p
        B_vec[s]     = p <= length(model.B) ? model.B[p] : 0.0

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
        q_low_map,
        q_hat_map,
        B_vec,
    )
end

has_walking_distance_limit(::RobustTotalDemandCapMap) = true

function get_valid_jk_pairs(mapping::RobustTotalDemandCapMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int,Int}[])
end
