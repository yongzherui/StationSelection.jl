"""
Nominal OD mapping for NominalTwoStageODModel.

Mirrors ClusteringTwoStageODMap but stores mean daily demand (Float64) per OD pair,
averaging raw request counts by ScenarioData.n_days.  Intended for use with
create_period_aggregated_data, which produces 4 period-aggregated scenarios so that
n_days = (end_date - start_date + 1) is the averaging denominator.
"""

using DataFrames

export NominalTwoStageODMap
export create_nominal_two_stage_od_map

"""
    NominalTwoStageODMap <: AbstractClusteringMap

OD mapping for NominalTwoStageODModel with mean-daily demand.

# Fields
- `station_id_to_array_idx`, `array_idx_to_station_id`: station ID ↔ index
- `scenarios`: reference to ScenarioData vector
- `scenario_label_to_array_idx`, `array_idx_to_scenario_label`
- `Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}`: OD pairs with positive demand per scenario
- `Q_s::Dict{Int, Dict{Tuple{Int, Int}, Float64}}`: Mean daily demand per OD pair per scenario
- `max_walking_distance::Float64`: Walking-distance filter
- `valid_jk_pairs`: valid (pickup, dropoff) station pairs per OD pair
"""
struct NominalTwoStageODMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}

    # Q[scenario_id][(o, d)] = mean daily demand for OD pair (o,d)
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Float64}}

    max_walking_distance::Float64
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
end


"""
    create_nominal_two_stage_od_map(
        model::NominalTwoStageODModel,
        data::StationSelectionData
    ) -> NominalTwoStageODMap

Build the nominal OD map, averaging raw OD counts by ScenarioData.n_days.
"""
function create_nominal_two_stage_od_map(
    model::NominalTwoStageODModel,
    data::StationSelectionData
)::NominalTwoStageODMap

    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    Omega_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s = Dict{Int, Dict{Tuple{Int, Int}, Float64}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        od_count = compute_scenario_od_count(scenario_data)
        n = scenario_data.n_days
        Q_s[scenario_id] = Dict(od => Float64(cnt) / n for (od, cnt) in od_count)
        Omega_s[scenario_id] = collect(keys(Q_s[scenario_id]))
        union!(all_od_pairs, Omega_s[scenario_id])
    end

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs,
        data,
        model.max_walking_distance
    )

    return NominalTwoStageODMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        model.max_walking_distance,
        valid_jk_pairs,
    )
end

has_walking_distance_limit(::NominalTwoStageODMap) = true

function get_valid_jk_pairs(mapping::NominalTwoStageODMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
end
