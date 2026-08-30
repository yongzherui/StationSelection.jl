"""
OD mapping and initial restricted column pool for the aggregate-OD-route problem.
"""

export AggregateODRouteColumn
export AggregateODRouteMap
export create_aggregate_od_route_map
export aggregate_od_route_validate_feasible_coverage

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

"""
    AggregateODRouteMap

# OD demand-group indexing

`Omega_s[s]::Vector{Tuple{Int,Int}}` maps scenario `s` to its demand-positive
`(o,d)` pairs; position `p` within `Omega_s[s]` is that demand group's index
(`p`), used throughout the aggregate-OD-route master/pricer as `(s,p)` instead
of the raw `(s,o,d)` triple. `Q_s[s]::Vector{Int}` is dense and parallel to
`Omega_s[s]` (`Q_s[s][p]` = demand for `Omega_s[s][p]`), since every position
has positive demand by construction. `valid_jk_pairs` stays keyed by the raw
`(o,d)` tuple -- station-pair feasibility depends on geography, not on `s`/`p`.
"""
mutable struct AggregateODRouteMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}
    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}
    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}
    Q_s::Dict{Int, Vector{Int}}
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
    active_jk_s::Dict{Int, Vector{Tuple{Int, Int}}}
    columns::Vector{AggregateODRouteColumn}
    column_ids::Set{Int}
    columns_by_pair::Dict{Tuple{Int, Int}, Vector{Int}}
    max_walking_distance::Float64
end

has_walking_distance_limit(mapping::AggregateODRouteMap) = true

get_valid_jk_pairs(mapping::AggregateODRouteMap, o::Int, d::Int) =
    get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])

function _aggregate_od_route_active_jk_by_s(
    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}},
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
)::Dict{Int, Vector{Tuple{Int, Int}}}
    active_jk_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    for s in sort!(collect(keys(Omega_s)))
        jk_set = Set{Tuple{Int, Int}}()
        for (o, d) in Omega_s[s]
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

"""
    _aggregate_od_route_allow_walk_only(formulation) -> Bool

Resolve the "is direct walking (`WALK_ONLY_PAIR`) available" flag per formulation type.
`AggregateODRouteBendersYXFormulation` (unwired, see `opt/optimize.jl`'s include
comments) carries it as a genuine opt-in `allow_walk_only::Bool` field. Both *live*
aggregate-OD-route formulations -- `AggregateODRouteBaseFormulation` and
`AggregateODRouteJointRoutingAssignmentFormulation` -- carry no such field: direct
walking is mandatory for both (see their own docstrings), since
`compute_valid_jk_pairs` no longer produces same-station pairs at all, making
`WALK_ONLY_PAIR` the *only* station-free coverage option left, and each formulation's
build-time feasibility guarantee (`aggregate_od_route_validate_feasible_coverage`)
assumes it's always on.
"""
_aggregate_od_route_allow_walk_only(formulation) = formulation.allow_walk_only
_aggregate_od_route_allow_walk_only(::AggregateODRouteJointRoutingAssignmentFormulation) = true
_aggregate_od_route_allow_walk_only(::AggregateODRouteBaseFormulation) = true
_aggregate_od_route_allow_walk_only(::AggregateODRouteFeasibilityFormulation) = true

"""
    aggregate_od_route_validate_feasible_coverage(data, mapping)

Throw `ArgumentError` unless every positive-demand group `(s,p)` has at least one
coverage option: a real `(j,k)`, `j != k`, pair with finite `get_routing_cost(data,j,k)`,
or `WALK_ONLY_PAIR`. Shared by both live aggregate-OD-route `build_model` paths --
`AggregateODRouteBaseFormulation` (`x_walk`/`x`/`θ`, `DirectMIPSolver`) and
`AggregateODRouteJointRoutingAssignmentFormulation` (`x_walk`/`θ`, `CGSolver`) -- neither
of which carries an unserved-demand slack, so a demand group with neither option would
otherwise leave its coverage row permanently infeasible: a genuine data/geometry problem
(no station in walking range of either endpoint, and direct walking either disabled or
out of range), not something a slack variable should paper over. Note this is a
*necessary* condition, not sufficient on its own for `AggregateODRouteBaseFormulation`:
a finite `get_routing_cost` only proves the pair is walkable-to, not that
`enumerate_aggregate_od_route_columns` actually emitted a route covering it -- the same
`detour_factor >= 1.0` two-stop certification argument
`joint_routing_assignment_two_stop_seed_columns` documents applies to that enumeration
too, so in practice it does, but this function only checks the necessary part. Called
once by each `build_model`, right after the map is built and before any JuMP variables
exist, so the failure is immediate and cheap.
"""
function aggregate_od_route_validate_feasible_coverage(
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)
    unroutable = Tuple{Int, Int}[]
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            covered = false
            for pair in get_valid_jk_pairs(mapping, o, d)
                if is_walk_only_pair(pair) || isfinite(get_routing_cost(data, pair[1], pair[2]))
                    covered = true
                    break
                end
            end
            covered || push!(unroutable, (s, p))
        end
    end
    isempty(unroutable) || throw(ArgumentError(
        "aggregate OD route: no feasible coverage option (no real station pair with " *
        "finite routing cost, no direct walk) for demand group(s) (scenario,p) = " *
        "$(unroutable)",
    ))
    return nothing
end

"""
    create_aggregate_od_route_map(problem::StationSelectionProblem, formulation,
                                   data::StationSelectionData; initial_columns=nothing)
        -> AggregateODRouteMap

Build the `AggregateODRouteMap` for `AggregateODRouteBaseFormulation`/
`AggregateODRouteJointRoutingAssignmentFormulation`/`AggregateODRouteFeasibilityFormulation`,
reading `problem.max_walking_distance` and `_aggregate_od_route_allow_walk_only(formulation)`
-- the only two things this function actually needs from `formulation`, which is why its
parameter type is the broad `AbstractFormulation` rather than the narrower
`AnyAggregateODRouteFormulation` Union: that Union is reserved for formulations sharing the
full route-column encoding-detail field set (`enumerate_aggregate_od_route_columns` and
friends), which `AggregateODRouteFeasibilityFormulation` deliberately doesn't carry, and
this function was never one of the callers that needed it. (`RouteCoveringProblem`'s
fixed-assignment variant of this, `_apply_route_covering_assignments!`, was removed along
with `AggregateODRouteProblem` -- `RouteCoveringProblem` is currently unwired, see
`StationSelection.jl`'s include comments.)
"""
function create_aggregate_od_route_map(
    problem::StationSelectionProblem,
    formulation::AbstractFormulation,
    data::StationSelectionData;
    initial_columns::Union{Nothing, AbstractVector}=nothing,
)::AggregateODRouteMap
    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    Omega_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s = Dict{Int, Vector{Int}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for (s, scenario_data) in enumerate(data.scenarios)
        od_count = compute_scenario_od_count(scenario_data)
        Omega_s[s] = sort!(collect(keys(od_count)))
        Q_s[s] = [od_count[pair] for pair in Omega_s[s]]
        union!(all_od_pairs, Omega_s[s])
    end

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs,
        data,
        problem.max_walking_distance;
        allow_walk_only=_aggregate_od_route_allow_walk_only(formulation),
    )
    active_jk_s = _aggregate_od_route_active_jk_by_s(Omega_s, valid_jk_pairs)
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
