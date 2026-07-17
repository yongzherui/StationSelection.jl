"""
OD mapping and initial restricted column pool for AggregateODRouteModel.
"""

export AggregateODRouteMap
export create_aggregate_od_route_map
export assert_no_walk_only_pairs

mutable struct AggregateODRouteMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}
    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}
    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}}
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
    active_jk_s::Dict{Int, Vector{Tuple{Int, Int}}}
    columns::Vector{AggregateODRouteColumn}
    column_ids::Set{Int}
    columns_by_pair::Dict{Tuple{Int, Int}, Vector{Int}}
    max_walking_distance::Float64
end

has_walking_distance_limit(mapping::AggregateODRouteMap) = true

"""
    assert_no_walk_only_pairs(mapping::AggregateODRouteMap, context::AbstractString)

WALK_ONLY_PAIR assignments (from `allow_walk_only=true`) are wired through the
direct-solve / column-generation build path (`_build_aggregate_od_route_core!`)
and the FreeAggregateODAssignmentPolicy Benders (BendersXY) path. The
NearestOpen assignment policy and its Benders paths (BendersY, and BendersXY
with NearestOpen) build their own `y[j]`/`y[k]`-indexed ranking/domination
constraints outside those paths and do not yet know how to handle a
station-free pair. Fail loudly and early instead of erroring deep inside a
solver with a cryptic index-0 BoundsError.
"""
function assert_no_walk_only_pairs(mapping::AggregateODRouteMap, context::AbstractString)::Nothing
    any(any(is_walk_only_pair, pairs) for pairs in values(mapping.valid_jk_pairs)) &&
        throw(ArgumentError(
            "$context does not yet support walk-only (station-free) assignments; " *
            "set allow_walk_only=false, or use the default FreeAggregateODAssignmentPolicy " *
            "direct-solve / column-generation path instead."
        ))
    return nothing
end

get_valid_jk_pairs(mapping::AggregateODRouteMap, o::Int, d::Int) =
    get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])

function _aggregate_od_route_active_jk_by_s(
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

function _register_aggregate_od_route_column_metadata!(
    mapping::AggregateODRouteMap,
    column::AggregateODRouteColumn,
)
    column.id in mapping.column_ids &&
        throw(ArgumentError("aggregate OD route column id $(column.id) already exists"))
    push!(mapping.columns, column)
    push!(mapping.column_ids, column.id)
    for pair in column.od_pairs
        push!(get!(mapping.columns_by_pair, pair, Int[]), column.id)
    end
    return column
end

function _singleton_aggregate_od_route_columns(
    active_jk_s::Dict{Int, Vector{Tuple{Int, Int}}},
    data::StationSelectionData,
)::Vector{AggregateODRouteColumn}
    has_routing_costs(data) ||
        throw(ArgumentError("AggregateODRouteModel singleton initialization requires routing_costs"))

    all_pairs = Set{Tuple{Int, Int}}()
    for pairs in values(active_jk_s)
        union!(all_pairs, pairs)
    end

    missing_pairs = Tuple{Int, Int}[]
    columns = AggregateODRouteColumn[]
    next_id = 1
    for (j, k) in sort!(collect(all_pairs))
        is_walk_only_pair((j, k)) && continue
        tau = get_routing_cost(data, j, k)
        if !isfinite(tau)
            push!(missing_pairs, (j, k))
            continue
        end
        push!(columns, AggregateODRouteColumn(
            next_id,
            [(j, k)],
            tau;
            metadata=Dict{String, Any}("initialization" => "singleton"),
        ))
        next_id += 1
    end

    isempty(missing_pairs) ||
        throw(ArgumentError("missing finite routing costs for singleton aggregate OD route columns: $(missing_pairs)"))
    return columns
end

function create_aggregate_od_route_map(
    model::AnyAggregateODRouteModel,
    data::StationSelectionData,
)::AggregateODRouteMap
    base_model = model isa RouteCoveringProblem ? model.base : model
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
        base_model.max_walking_distance;
        allow_walk_only=base_model.allow_walk_only,
    )
    if model isa RouteCoveringProblem
        _apply_route_covering_assignments!(valid_jk_pairs, Q_s, model)
    end
    active_jk_s = _aggregate_od_route_active_jk_by_s(Q_s, valid_jk_pairs)
    initial_columns = isnothing(base_model.initial_columns) ?
        _singleton_aggregate_od_route_columns(active_jk_s, data) :
        base_model.initial_columns

    mapping = AggregateODRouteMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        valid_jk_pairs,
        active_jk_s,
        AggregateODRouteColumn[],
        Set{Int}(),
        Dict{Tuple{Int, Int}, Vector{Int}}(),
        base_model.max_walking_distance,
    )
    for column in initial_columns
        _register_aggregate_od_route_column_metadata!(mapping, column)
    end
    return mapping
end

function _apply_route_covering_assignments!(
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}},
    model::RouteCoveringProblem,
)::Nothing
    open_set = Set(model.open_stations)
    for (s, q_s) in Q_s
        for ((o, d), demand) in q_s
            demand > 0 || continue
            key = (s, o, d)
            haskey(model.fixed_assignments, key) ||
                throw(ArgumentError("missing fixed assignment for scenario/OD $(key)"))
            assigned = model.fixed_assignments[key]
            is_walk_only_pair(assigned) ||
                assigned[1] in open_set && assigned[2] in open_set ||
                throw(ArgumentError("fixed assignment $(assigned) for $(key) uses a station that is not open"))
            feasible = get(valid_jk_pairs, (o, d), Tuple{Int, Int}[])
            assigned in feasible ||
                throw(ArgumentError("fixed assignment $(assigned) is infeasible for OD $((o, d))"))
            valid_jk_pairs[(o, d)] = [assigned]
        end
    end
    return nothing
end
