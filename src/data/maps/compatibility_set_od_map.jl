"""
OD mapping and initial restricted column pool for CompatibilitySetModel.
"""

export CompatibilitySetODMap
export create_compatibility_set_od_map

mutable struct CompatibilitySetODMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}
    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}
    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}}
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
    active_jk_s::Dict{Int, Vector{Tuple{Int, Int}}}
    columns::Vector{CompatibilityColumn}
    column_ids::Set{Int}
    columns_by_pair::Dict{Tuple{Int, Int}, Vector{Int}}
    max_walking_distance::Float64
end

has_walking_distance_limit(mapping::CompatibilitySetODMap) = true

get_valid_jk_pairs(mapping::CompatibilitySetODMap, o::Int, d::Int) =
    get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])

function _compatibility_active_jk_by_s(
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}},
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
)::Dict{Int, Vector{Tuple{Int, Int}}}
    active_jk_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    for s in sort!(collect(keys(Q_s)))
        jk_set = Set{Tuple{Int, Int}}()
        for ((o, d), demand) in Q_s[s]
            demand > 0 || continue
            union!(jk_set, get(valid_jk_pairs, (o, d), Tuple{Int, Int}[]))
        end
        active_jk_s[s] = sort!(collect(jk_set))
    end
    return active_jk_s
end

function _register_compatibility_column_metadata!(
    mapping::CompatibilitySetODMap,
    column::CompatibilityColumn,
)
    column.id in mapping.column_ids &&
        throw(ArgumentError("compatibility column id $(column.id) already exists"))
    push!(mapping.columns, column)
    push!(mapping.column_ids, column.id)
    for pair in column.od_pairs
        push!(get!(mapping.columns_by_pair, pair, Int[]), column.id)
    end
    return column
end

function _singleton_compatibility_columns(
    active_jk_s::Dict{Int, Vector{Tuple{Int, Int}}},
    data::StationSelectionData,
)::Vector{CompatibilityColumn}
    has_routing_costs(data) ||
        throw(ArgumentError("CompatibilitySetModel singleton initialization requires routing_costs"))

    all_pairs = Set{Tuple{Int, Int}}()
    for pairs in values(active_jk_s)
        union!(all_pairs, pairs)
    end

    missing_pairs = Tuple{Int, Int}[]
    columns = CompatibilityColumn[]
    next_id = 1
    for (j, k) in sort!(collect(all_pairs))
        tau = get_routing_cost(data, j, k)
        if !isfinite(tau)
            push!(missing_pairs, (j, k))
            continue
        end
        push!(columns, CompatibilityColumn(
            next_id,
            [(j, k)],
            tau;
            metadata=Dict{String, Any}("initialization" => "singleton"),
        ))
        next_id += 1
    end

    isempty(missing_pairs) ||
        throw(ArgumentError("missing finite routing costs for singleton compatibility columns: $(missing_pairs)"))
    return columns
end

function create_compatibility_set_od_map(
    model::AnyCompatibilitySetModel,
    data::StationSelectionData,
)::CompatibilitySetODMap
    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    Omega_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for (s, scenario_data) in enumerate(data.scenarios)
        od_count = compute_scenario_od_count(scenario_data)
        Omega_s[s] = sort!(collect(keys(od_count)))
        Q_s[s] = od_count
        union!(all_od_pairs, Omega_s[s])
    end

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs,
        data,
        model.max_walking_distance,
    )
    active_jk_s = _compatibility_active_jk_by_s(Q_s, valid_jk_pairs)
    initial_columns = isnothing(model.initial_columns) ?
        _singleton_compatibility_columns(active_jk_s, data) :
        model.initial_columns

    mapping = CompatibilitySetODMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        valid_jk_pairs,
        active_jk_s,
        CompatibilityColumn[],
        Set{Int}(),
        Dict{Tuple{Int, Int}, Vector{Int}}(),
        model.max_walking_distance,
    )
    for column in initial_columns
        _register_compatibility_column_metadata!(mapping, column)
    end
    return mapping
end
