"""
Nominal OD mapping for NominalTwoStageODModel.

Mirrors ClusteringTwoStageODMap but stores mean daily demand (Float64) per OD pair,
averaging raw request counts by ScenarioData.n_days.  Intended for use with
create_period_aggregated_data, which produces 4 period-aggregated scenarios so that
n_days = (end_date - start_date + 1) is the averaging denominator.
"""

using DataFrames
using Dates

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

function _scenario_od_active_days(
    scenario_data::ScenarioData
)::Dict{Tuple{Int, Int}, Int}
    active_dates = Dict{Tuple{Int, Int}, Set{Date}}()
    _require_indexed_request_columns(scenario_data.requests)

    for row in eachrow(scenario_data.requests)
        od = (row.origin_idx, row.dest_idx)
        push!(get!(active_dates, od, Set{Date}()), Date(row.request_time))
    end

    return Dict(od => length(days) for (od, days) in active_dates)
end

function _gravity_prior_probability(
    feasible_od_pairs::Vector{Tuple{Int, Int}},
    empirical_mean::Dict{Tuple{Int, Int}, Float64},
    gravity_uniform_mix::Float64,
)::Dict{Tuple{Int, Int}, Float64}
    n_pairs = length(feasible_od_pairs)
    n_pairs > 0 || return Dict{Tuple{Int, Int}, Float64}()

    origin_totals = Dict{Int, Float64}()
    dest_totals = Dict{Int, Float64}()
    total_mean = 0.0

    for ((o, d), q) in empirical_mean
        q > 0 || continue
        origin_totals[o] = get(origin_totals, o, 0.0) + q
        dest_totals[d] = get(dest_totals, d, 0.0) + q
        total_mean += q
    end

    uniform_prob = 1.0 / n_pairs
    gravity_prob = Dict{Tuple{Int, Int}, Float64}()
    gravity_mass = 0.0

    for (o, d) in feasible_od_pairs
        prob = if total_mean > 0
            get(origin_totals, o, 0.0) * get(dest_totals, d, 0.0) / total_mean^2
        else
            0.0
        end
        gravity_prob[(o, d)] = prob
        gravity_mass += prob
    end

    prior_prob = Dict{Tuple{Int, Int}, Float64}()
    for od in feasible_od_pairs
        gravity_component = gravity_mass > 0 ? gravity_prob[od] / gravity_mass : uniform_prob
        prior_prob[od] = (1 - gravity_uniform_mix) * gravity_component + gravity_uniform_mix * uniform_prob
    end

    return prior_prob
end

"""
    create_nominal_two_stage_od_map(
        model::SmoothedNominalTwoStageODModel,
        data::StationSelectionData
    ) -> NominalTwoStageODMap

Build the nominal OD map with demand smoothing.

For each scenario, every walk-feasible OD pair receives positive mean demand via
empirical-Bayes shrinkage toward a gravity prior with a small uniform mixture.
"""
function create_nominal_two_stage_od_map(
    model::SmoothedNominalTwoStageODModel,
    data::StationSelectionData
)::NominalTwoStageODMap

    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    n = data.n_stations
    candidate_od_pairs = Set{Tuple{Int, Int}}((o, d) for o in 1:n for d in 1:n if o != d)
    valid_jk_pairs = compute_valid_jk_pairs(
        candidate_od_pairs,
        data,
        model.max_walking_distance
    )
    feasible_od_pairs = sort!(
        [(o, d) for (o, d) in candidate_od_pairs if !isempty(get(valid_jk_pairs, (o, d), Tuple{Int, Int}[]))];
        by=identity,
    )

    Omega_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s = Dict{Int, Dict{Tuple{Int, Int}, Float64}}()

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        od_count = compute_scenario_od_count(scenario_data)
        active_days = _scenario_od_active_days(scenario_data)
        n_days = scenario_data.n_days
        empirical_mean = Dict(od => Float64(cnt) / n_days for (od, cnt) in od_count)

        prior_prob = _gravity_prior_probability(
            feasible_od_pairs,
            empirical_mean,
            model.gravity_uniform_mix,
        )
        total_empirical_mean = sum(values(empirical_mean))
        prior_mass = model.pseudo_demand_fraction * total_empirical_mean

        smoothed_q = Dict{Tuple{Int, Int}, Float64}()
        for od in feasible_od_pairs
            q_emp = get(empirical_mean, od, 0.0)
            n_active = Float64(get(active_days, od, 0))
            q_prior = prior_mass * get(prior_prob, od, 0.0)
            weight_emp = n_active / (n_active + model.smoothing_tau)
            weight_prior = model.smoothing_tau / (n_active + model.smoothing_tau)
            smoothed_q[od] = weight_emp * q_emp + weight_prior * q_prior
        end

        Omega_s[scenario_id] = feasible_od_pairs
        Q_s[scenario_id] = smoothed_q
    end

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
