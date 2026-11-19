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
`Omega_s[s]`. `valid_jk_pairs` stays keyed by the raw
`(o,d)` tuple -- station-pair feasibility depends on geography, not on `s`/`p`.

# Demand per OD group: `Q_s[s][p]` (`_aggregate_od_route_demand_weighting`)

`Q_s[s][p]` is the number of requests on OD pair `p` in scenario `s` (`:count`) or 1
(`:unit`). The Joint formulation chooses through its `demand_weighting` field, **default
`:count` since 2026-11-03**; `AggregateODRouteBaseFormulation` and the feasibility
formulation stay `:unit` (Base's own CG pricer was never made demand-aware). `pax_num` is
not read: one request is one rider, as in the Clustering maps.

History: from 2026-09-15 (`6000078`) to 2026-11-03 the whole family was `:unit`
unconditionally. That retired a pricer/master drift (the master multiplied walking by `Q`,
the pricer used `Q` only as a `> 0` gate) by removing `Q`. `:count` is sound only because
the pricer (`joint_routing_assignment_pricing_candidates`) and the Benders completions now
multiply walking by `Q` too. Do not reintroduce `Q` on one side alone; the per-column
reduced-cost assertion in the CG master fires if the two disagree.

`Q` scales walking only. Coverage stays `>= 1` and a route's travel is charged once, so at a
fixed `walk_cost_weight` `:count` weights walking more than `:unit` by the mean requests per
group (about 2.1 on the Zhuzhou k30 training windows).
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
    walking_limit_s::Float64
    # `problem.door_to_door_limit_ratio`: already applied to `valid_jk_pairs`, and read again by
    # the joint pricer's per-candidate ride limit (`joint_routing_assignment_ride_limit`).
    # One copy, so the two cannot disagree -- the pricer's seeding relies on it.
    door_to_door_limit_ratio::Float64
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

**OFF for every live formulation, because the simulator does not implement it.**
MicroTransitSimulator has no direct-walk service mode: the only path that completes a
request without a vehicle needs `assigned_pickup_station == assigned_dropoff_station`
(`request_walking_handlers.jl`, logged as `WALKING_ONLY_TRANSFER_AT_PUDO`), which is
incidental geometry under `fixed_closest_selected` rather than a decision. And the only
thing crossing the selection/simulation boundary is the station set --
`walk_only_assignments.csv` is never read. So with this on, selection credited itself for
serving ~25% of demand groups on foot while the simulator put every one of them in a
vehicle, and "which formulation picks a better station set" was partly measuring which
one had an escape hatch. Measured on a 4x4 grid at `walking_limit_s=110`: the routing
arm walked 20-21 of 81 groups; the simulator's incidental path fired for 0-5 of 30
requests, for an unrelated reason.

Everything downstream was already written for this: `compute_valid_jk_pairs` takes the
flag, `add_walk_variables!` creates no `x_walk` for a group without the fallback,
`add_aggregate_od_route_endpoint_feasibility_constraints!` then marks BOTH endpoints
required (the strictly correct behaviour with no fallback), and
`aggregate_od_route_validate_feasible_coverage`'s own error text names "direct walking
either disabled or out of range".

Turning it back on is only sound once the simulator grows a real walk-only mode; until
then it makes selection optimise a service this pipeline cannot deliver.

There is deliberately no generic fallback: a formulation added without a method here
should fail with a `MethodError` naming this function, not by reaching for a field.
"""
_aggregate_od_route_allow_walk_only(::AnyJointRoutingAssignmentFormulation) = false
_aggregate_od_route_allow_walk_only(::AggregateODRouteBaseFormulation) = false
_aggregate_od_route_allow_walk_only(::AggregateODRouteFeasibilityFormulation) = false

"""
    _aggregate_od_route_demand_weighting(formulation) -> Symbol

`:count` or `:unit`, the `Q_s` rule (see "Demand per OD group" on `AggregateODRouteMap`).
Joint reads its field; Base and the feasibility formulation are `:unit`. No generic fallback,
for the same reason as `_aggregate_od_route_allow_walk_only`.
"""
_aggregate_od_route_demand_weighting(f::AnyJointRoutingAssignmentFormulation) = f.demand_weighting
_aggregate_od_route_demand_weighting(::AggregateODRouteBaseFormulation) = :unit

"""Seats per vehicle in requests, or `nothing` (unlimited). Only the joint monolith carries
one (`vehicle_capacity`); every other formulation is uncapacitated."""
_aggregate_od_route_vehicle_capacity(formulation) = nothing
_aggregate_od_route_vehicle_capacity(f::AggregateODRouteJointRoutingAssignmentFormulation) = f.vehicle_capacity
_aggregate_od_route_demand_weighting(::AggregateODRouteFeasibilityFormulation) = :unit

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
`ride_time_limit_ratio >= 1.0` two-stop certification argument
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
reading `problem.walking_limit_s` and `_aggregate_od_route_allow_walk_only(formulation)`
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
    demand_weighting = _aggregate_od_route_demand_weighting(formulation)
    capacity = _aggregate_od_route_vehicle_capacity(formulation)
    all_od_pairs = Set{Tuple{Int, Int}}()

    for (s, scenario_data) in enumerate(data.scenarios)
        od_count = compute_scenario_od_count(scenario_data)
        # With a vehicle capacity, a group of more requests than seats can ride no route: it
        # is dropped from the instance (owner's choice, 2026-11-14), not left uncoverable.
        Omega_s[s] = sort!([od for od in keys(od_count) if isnothing(capacity) || od_count[od] <= capacity])
        # See "Demand per OD group" on `AggregateODRouteMap`.
        Q_s[s] = demand_weighting === :count ?
            [od_count[od] for od in Omega_s[s]] :
            ones(Int, length(Omega_s[s]))
        union!(all_od_pairs, Omega_s[s])
    end

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs,
        data,
        problem.walking_limit_s;
        allow_walk_only=_aggregate_od_route_allow_walk_only(formulation),
        door_to_door_limit_ratio=problem.door_to_door_limit_ratio,
    )
    check_door_to_door_servable(valid_jk_pairs, data,
                                problem.walking_limit_s, problem.door_to_door_limit_ratio)
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
        problem.walking_limit_s,
        problem.door_to_door_limit_ratio,
    )
    for column in resolved_initial_columns
        _register_aggregate_od_route_column_metadata!(mapping, column)
    end
    return mapping
end
