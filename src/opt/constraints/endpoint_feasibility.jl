"""
Endpoint feasibility constraints for the aggregate-OD-route master, shared across
`AggregateODRouteBaseFormulation` and `AggregateODRouteJointRoutingAssignmentFormulation`
(both builds pass through here -- see `optimize/aggregate_od_route/base_shared.jl`'s
`_aggregate_od_route_base_master_core!` and `optimize/aggregate_od_route/
column_generation/build_joint_routing_assignment.jl`'s `_build_joint_routing_assignment_model`).

`aggregate_od_route_validate_feasible_coverage` (`data/maps/aggregate_od_route_map.jl`) is
a *build-time* check on the demand-group/candidate-pair data alone -- it says nothing
about whether the `k`-station budget forced on `y` (`Σy == k`, `add_station_limit_constraint!`)
can actually be met. `Σ_{j reachable within max_walking_distance} y[j] >= 1` is a genuine
necessary condition on `y` itself, so writing it as a real `@constraint` lets a solve that
can't possibly satisfy it fail immediately (LP infeasible on the very first `optimize!`)
instead of only surfacing after CG has spent several pricing rounds discovering it has no
column left to offer, or (worse) after `recover_integer_solution`'s restricted MIP goes
infeasible over an incomplete column pool and gets misread as "this instance is
infeasible" when it's really "the pool never got a chance to prove it isn't" -- see
`notes/2026-08-28_study5_dominance_fix_pilot_infeasible_repro.md`.
"""

export add_aggregate_od_route_endpoint_feasibility_constraints!

"""
    _assert_symmetric_walking_costs(data)

`add_aggregate_od_route_endpoint_feasibility_constraints!` consolidates a location's
pickup-side and dropoff-side row into one, which is only sound if `get_walking_cost(data,
point, j) == get_walking_cost(data, j, point)` for every `point`/`j` -- otherwise a
station reachable *from* a location but not reachable *to* it (or vice versa) would wrongly
satisfy both directions' requirement off a single row. Every built-in generator derives
`walking_costs` from geographic distance, which is always symmetric, but the field itself
(`StationSelectionData.walking_costs::Matrix{Float64}`) carries no such guarantee -- so
this is checked, not assumed, once per build.
"""
function _assert_symmetric_walking_costs(data::StationSelectionData)
    isapprox(data.walking_costs, data.walking_costs'; atol=1e-9) || throw(ArgumentError(
        "aggregate OD route: walking_costs is not symmetric, so " *
        "add_aggregate_od_route_endpoint_feasibility_constraints!'s consolidated " *
        "per-location rows would be unsound (pickup- and dropoff-side reachability " *
        "would silently satisfy each other instead of being checked independently)",
    ))
    return nothing
end

"""
    add_aggregate_od_route_endpoint_feasibility_constraints!(m, data, mapping, y)
        -> Dict{Int, ConstraintRef}

One row per *required* location: `Σ_{j : walking_cost(point, j) <= max_walking_distance}
y[j] >= 1`.

A demand group `(s,p)` with origin `o` and destination `d` marks both `o` and `d` required
only when that group has **no** `WALK_ONLY_PAIR` fallback (`is_walk_only_pair` absent from
`get_valid_jk_pairs(mapping, o, d)`). When the fallback *is* available, the group can
legally be served by direct walking with zero stations built anywhere near it --
`_aggregate_od_route_allow_walk_only` is unconditionally `true` for both live
formulations -- so forcing a station near `o`/`d` there would be unsound: it would reject
solutions the model is supposed to allow.

Consolidated per location, not per demand group, and not split by pickup/dropoff side:
reachability depends only on `(point, max_walking_distance)` under symmetric walking costs
(enforced by `_assert_symmetric_walking_costs`), so every demand group touching a location
-- as origin or destination -- contributes to the same one row. This also means a location
is required (gets a row) the moment *any* demand group needs it there, even if some other
demand group that happens to share the same `o`/`d` location has its own walk-only
fallback and would, on its own, never have required the row -- one group's fallback only
ever excuses that group, never a different demand group sitting at the same location.

Every candidate list this builds is guaranteed non-empty by the time it's called: each
required location belongs to a demand group `aggregate_od_route_validate_feasible_coverage`
already proved has a real `(j,k)` pair with finite routing cost, and that pair only exists
in `mapping.valid_jk_pairs` because both its stations already passed this same
`max_walking_distance` reachability check (`compute_valid_jk_pairs`) -- so there is always
at least one term to sum. Both `build_model` entry points call the validator before this.
"""
function add_aggregate_od_route_endpoint_feasibility_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        y::Vector{VariableRef},
    )::Dict{Int, ConstraintRef}
    _assert_symmetric_walking_costs(data)
    n = data.n_stations
    required = Set{Int}()
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
            push!(required, o)
            push!(required, d)
        end
    end

    endpoint_feasibility = Dict{Int, ConstraintRef}()
    for point in sort!(collect(required))
        candidates = [j for j in 1:n if get_walking_cost(data, point, j) <= mapping.max_walking_distance]
        endpoint_feasibility[point] = @constraint(m, sum(y[j] for j in candidates; init=0.0) >= 1)
    end
    return endpoint_feasibility
end
