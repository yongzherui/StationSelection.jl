"""
OD mapping and initial restricted column pool for the aggregate-OD-route problem.
"""

export AggregateODRouteColumn
export AggregateODRouteMap
export create_aggregate_od_route_map
export assert_no_walk_only_pairs

"""
    AggregateODRouteColumn

A route column for the aggregate-OD-route problem: which station pairs it serves and at
what cost. Lives here (not `opt/problems/` or `opt/formulations/`) because it's a plain
data/value type -- the item type of `AggregateODRouteMap.columns` below, its dominant
consumer -- not a `Problem` or `Formulation`. Used as the θ pool for
`AggregateODRouteBaseFormulation` (built via `enumerate_aggregate_od_route_columns`);
`AggregateODRouteJointRoutingAssignmentFormulation` uses a different, richer type,
`JointRoutingAssignmentRouteColumn` (`route`/`assignments`, not just `od_pairs`), for its
own real θ pool -- the two are not interchangeable despite the similar name.
"""
struct AggregateODRouteColumn
    id::Int
    od_pairs::Vector{Tuple{Int, Int}}
    tau::Float64
    metadata::Dict{String, Any}

    function AggregateODRouteColumn(
            id::Int,
            od_pairs::AbstractVector{<:Tuple{Int, Int}},
            tau::Number;
            metadata::Dict{String, Any}=Dict{String, Any}()
        )
        id > 0 || throw(ArgumentError("column id must be positive"))
        isempty(od_pairs) && throw(ArgumentError("aggregate OD route column must cover at least one OD pair"))
        tau >= 0 || throw(ArgumentError("tau must be non-negative"))
        unique_pairs = unique(Tuple{Int, Int}.(od_pairs))
        new(id, unique_pairs, Float64(tau), metadata)
    end
end

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
        throw(ArgumentError("AggregateODRouteProblem singleton initialization requires routing_costs"))

    all_pairs = Set{Tuple{Int, Int}}()
    for pairs in values(active_jk_s)
        union!(all_pairs, pairs)
    end

    missing_pairs = Tuple{Int, Int}[]
    columns = AggregateODRouteColumn[]
    next_id = 1
    for (j, k) in sort!(collect(all_pairs))
        requires_no_vehicle_route((j, k)) && continue
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

"""
    create_aggregate_od_route_map(problem::StationSelectionProblem, formulation,
                                   data::StationSelectionData; initial_columns=nothing)
        -> AggregateODRouteMap

Build the `AggregateODRouteMap` for `AggregateODRouteBaseFormulation`/
`AggregateODRouteJointRoutingAssignmentFormulation`, reading `problem.max_walking_distance`
and `formulation.allow_walk_only`. (`RouteCoveringProblem`'s fixed-assignment variant of
this, `_apply_route_covering_assignments!`, was removed along with `AggregateODRouteProblem`
-- `RouteCoveringProblem` is currently unwired, see `StationSelection.jl`'s include
comments.)
"""
function create_aggregate_od_route_map(
    problem::StationSelectionProblem,
    formulation::AnyAggregateODRouteFormulation,
    data::StationSelectionData;
    initial_columns::Union{Nothing, AbstractVector}=nothing,
)::AggregateODRouteMap
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
        problem.max_walking_distance;
        allow_walk_only=formulation.allow_walk_only,
        allow_same_station=true,
    )
    active_jk_s = _aggregate_od_route_active_jk_by_s(Q_s, valid_jk_pairs)
    resolved_initial_columns = isnothing(initial_columns) ?
        _singleton_aggregate_od_route_columns(active_jk_s, data) :
        initial_columns

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
        problem.max_walking_distance,
    )
    for column in resolved_initial_columns
        _register_aggregate_od_route_column_metadata!(mapping, column)
    end
    return mapping
end
