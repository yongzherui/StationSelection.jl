"""
Exhaustive route-column enumeration for `AggregateODRouteColumn`/`θ`, used by
`AggregateODRouteBaseFormulation` (`y`/`x`/`θ`, `x` decoupled from routing) builds
that need the whole column universe up front rather than iteratively priced columns.

The traversal is deliberately independent of the pricing search orchestration: the
pricer is optimized for finding negative-reduced-cost columns under a particular dual
vector, while direct solves need a dual-free route universe. This module does a plain
bounded depth-first search over the same pricing-label transitions
(the pricing graph/duals in `../types.jl`, the label type in `types.jl`, and the
transitions in `labels.jl`) -- station-age/wait/detour rules are
genuine physical feasibility, not reduced-cost pruning -- but applies neither dominance
nor reduced-cost pruning itself, and uses uniform positive rewards so nothing gets
pruned. Every label serving at least one active pair is emitted as a candidate column;
the cheapest column per distinct served-pairs signature is kept.

(Recovered from an external reorganization that archived the original copy of this
file alongside genuinely-dead Benders scaffolding -- see `opt/optimize.jl`'s comment
above its Benders include block. Adapted here to the current `AggregateODRouteProblem`
struct, which no longer carries CG-specific fields, so it no longer needs the old
`_base_aggregate_od_route_model`/`_copy_with_initial_columns` indirection for
`RouteCoveringProblem`.)
"""

export enumerate_aggregate_od_route_columns

function _all_active_aggregate_od_route_pairs(mapping::AggregateODRouteMap)::Vector{Tuple{Int, Int}}
    pairs = Set{Tuple{Int, Int}}()
    for scenario_pairs in values(mapping.active_jk_s)
        union!(pairs, scenario_pairs)
    end
    filter!(!is_walk_only_pair, pairs)
    return sort!(collect(pairs))
end

"""
    _deduplicate_aggregate_od_route_columns(columns) -> Vector{AggregateODRouteColumn}

Keep the cheapest (`tau`) column per distinct `od_pairs` signature, renumbered
`1:length(...)` so the result is ready to seed a fresh `AggregateODRouteMap`.
"""
function _deduplicate_aggregate_od_route_columns(
    columns::Vector{AggregateODRouteColumn},
)::Vector{AggregateODRouteColumn}
    best = Dict{Any, AggregateODRouteColumn}()
    for column in columns
        signature = _aggregate_od_route_column_signature(column)
        incumbent = get(best, signature, nothing)
        if isnothing(incumbent) || column.tau < incumbent.tau - 1e-9
            best[signature] = column
        end
    end
    out = AggregateODRouteColumn[]
    next_id = 1
    for column in sort!(collect(values(best)); by=c -> (length(c.od_pairs), c.tau, string(c.od_pairs)))
        push!(out, AggregateODRouteColumn(
            next_id, column.od_pairs, column.tau; metadata=copy(column.metadata),
        ))
        next_id += 1
    end
    return out
end

"""
    _od_route_relevant_nodes(active_pairs) -> Vector{Int}

Only stations that are an origin or destination of some active OD pair can ever help
serve a pair -- routing costs are direct point-to-point (no underlying road graph to
transit through), so restricting the search to these nodes is lossless, not a
heuristic prune.
"""
function _od_route_relevant_nodes(active_pairs::Vector{Tuple{Int, Int}})::Vector{Int}
    nodes = Set{Int}()
    for (j, k) in active_pairs
        push!(nodes, j)
        push!(nodes, k)
    end
    return sort!(collect(nodes))
end

function _od_route_travel_lookup(
    data::StationSelectionData,
    nodes::Vector{Int},
)::Dict{Tuple{Int, Int}, Float64}
    travel = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        cost = get_routing_cost(data, i, j)
        isfinite(cost) ||
            throw(ArgumentError("missing finite routing cost for station arc $((i, j))"))
        travel[(i, j)] = cost
    end
    return travel
end

"""
    _enumerate_aggregate_od_route_raw_columns(mapping, data, route_regularization_weight,
        repositioning_time, max_wait_time, detour_factor, max_stops_cap, max_routes,
        time_limit_sec) -> Vector{AggregateODRouteColumn}

The DFS itself, *before* `_deduplicate_aggregate_od_route_columns` collapses it to one
column per distinct `served_pairs` signature -- one `AggregateODRouteColumn` per distinct
label/physical route the search visits (`metadata["route"]` carries the node sequence).
`_enumerate_aggregate_od_route_columns_core` (below) is a thin wrapper over this that
dedupes for `AggregateODRouteBaseFormulation`'s own use.

Exposed raw because `AggregateODRouteJointRoutingAssignmentFormulation`'s own exhaustive
enumerator (`label_setting/joint_routing_assignment/exact/enumeration.jl`) needs the same
physical-route universe but a *different* per-route expansion (every combination of
per-passenger assignment choices, not one column per served-pairs signature) -- collapsing
by signature here first would silently discard physically distinct routes (different node
order, different `tau`, potentially different passenger certifications) that happen to
share a served-pairs signature. The two formulations' physical feasibility rule is
identical (`detour_factor * routing_cost(j, k)` per `(j, k)` pair, independent of which
formulation or which passenger uses it -- see `joint_routing_assignment/pricing_round.jl`'s
own `ride_limit` computation), so reusing this DFS verbatim, rather than re-deriving an
equivalent one against `JointRoutingAssignmentPricingLabel`'s transitions, is exact, not an
approximation.
"""
function _enumerate_aggregate_od_route_raw_columns(
        mapping::AggregateODRouteMap,
        data::StationSelectionData,
        route_regularization_weight::Float64,
        repositioning_time::Float64,
        max_wait_time::Float64,
        detour_factor::Float64,
        max_stops_cap::Int,
        max_routes::Int,
        time_limit_sec::Float64,
    )::Vector{AggregateODRouteColumn}
    active_pairs = _all_active_aggregate_od_route_pairs(mapping)
    isempty(active_pairs) && return AggregateODRouteColumn[]

    nodes = _od_route_relevant_nodes(active_pairs)
    max_stops = _resolve_aggregate_od_route_max_stops(max_stops_cap)
    travel = _od_route_travel_lookup(data, nodes)
    pricing_data = RouteCoveringPricingData(
        0,
        nodes,
        travel,
        active_pairs,
        route_regularization_weight,
        repositioning_time,
        max_wait_time,
        detour_factor,
        max_stops,
        max_stops_cap != typemax(Int),
        false,  # compensated_dominance: irrelevant -- this DFS never calls dominance at all
    )
    # Uniform positive rewards make every active pair visible to the shared pricing
    # transitions. They do not prune or rank this DFS.
    enumeration_duals = RouteCoveringPricingDuals(Dict(pair => 1.0 for pair in active_pairs))

    t_start = time()
    exhausted = true  # flips to false the moment the time budget runs out; every level of the recursion checks and unwinds on it
    columns = AggregateODRouteColumn[]
    next_id = 1

    # Depth-first visit of every reachable label. Unlike the pricer
    # (`_run_label_setting`/`engine.jl`), there is no dominance test, no
    # priority queue, and no reduced-cost pruning here -- every label the
    # transitions in `labels.jl` say is physically reachable gets visited, in
    # whatever order the plain recursion produces.
    function visit!(label::RouteCoveringPricingLabel)
        if time() - t_start > time_limit_sec
            exhausted = false
            return
        end
        # Any label that has certified at least one pair becomes a column
        # immediately -- unlike the pricer's `best_by_signature`, nothing here
        # dedupes across labels sharing a signature as the walk goes; that
        # cleanup is `_deduplicate_aggregate_od_route_columns`'s job, once,
        # after the whole DFS finishes (see below).
        if !isempty(label.served_pairs)
            push!(columns, AggregateODRouteColumn(
                next_id,
                collect(label.served_pairs),
                label.tau;
                metadata=Dict{String, Any}(
                    "initialization" => "enumeration",
                    "route" => Tuple(label.route),
                ),
            ))
            next_id += 1
            length(columns) <= max_routes ||
                throw(ArgumentError("route enumeration exceeded max_routes=$(max_routes)"))
        end
        # The only other pruning this DFS does: stop extending once at the
        # stops cap. Everything else `_route_covering_candidate_next_nodes`
        # offers (genuine physical feasibility -- wait time, ride limit) is
        # followed unconditionally.
        label.route_length >= max_stops && return
        next_nodes = _route_covering_candidate_next_nodes(label, pricing_data, enumeration_duals)
        for next_node in next_nodes
            for child in extend_route_covering_pricing_label(label, next_node, pricing_data, enumeration_duals)
                visit!(child)
                exhausted || return  # time ran out somewhere below -- unwind the whole recursion, not just this frame
            end
        end
    end

    for initial_label in initial_route_covering_pricing_labels(pricing_data, enumeration_duals)
        visit!(initial_label)
        exhausted || break
    end

    exhausted || throw(ArgumentError(
        "route enumeration did not complete within time_limit_sec=$(time_limit_sec)"
    ))

    append!(columns, mapping.columns)
    return columns
end

"""
    _enumerate_aggregate_od_route_columns_core(mapping, data, route_regularization_weight,
        repositioning_time, max_wait_time, detour_factor, max_stops_cap, max_routes,
        time_limit_sec) -> Vector{AggregateODRouteColumn}

Shared body behind both `enumerate_aggregate_od_route_columns` methods below -- the
`AnyAggregateODRouteProblem` (single combined object) and `StationSelectionProblem`/
formulation (split) call sites differ only in where the encoding-detail scalars come
from; both resolve `mapping` and those scalars up front and delegate here. Thin wrapper
over `_enumerate_aggregate_od_route_raw_columns` (above) plus the final
served-pairs-signature dedup `AggregateODRouteBaseFormulation`'s own `θ` pool wants.
"""
function _enumerate_aggregate_od_route_columns_core(
        mapping::AggregateODRouteMap,
        data::StationSelectionData,
        route_regularization_weight::Float64,
        repositioning_time::Float64,
        max_wait_time::Float64,
        detour_factor::Float64,
        max_stops_cap::Int,
        max_routes::Int,
        time_limit_sec::Float64,
    )::Vector{AggregateODRouteColumn}
    columns = _enumerate_aggregate_od_route_raw_columns(
        mapping, data, route_regularization_weight, repositioning_time, max_wait_time,
        detour_factor, max_stops_cap, max_routes, time_limit_sec,
    )
    return _deduplicate_aggregate_od_route_columns(columns)
end

"""
    enumerate_aggregate_od_route_columns(problem::StationSelectionProblem, formulation,
        data::StationSelectionData; max_routes=10_000, time_limit_sec=30.0)
        -> Vector{AggregateODRouteColumn}

`StationSelectionProblem`/formulation-split counterpart of the `AnyAggregateODRouteProblem`
method above -- reads encoding-detail scalars off `formulation` directly.
"""
function enumerate_aggregate_od_route_columns(
    problem::StationSelectionProblem,
    formulation::AnyAggregateODRouteFormulation,
    data::StationSelectionData;
    max_routes::Int=10_000,
    time_limit_sec::Float64=30.0,
)::Vector{AggregateODRouteColumn}
    max_routes > 0 || throw(ArgumentError("max_routes must be positive"))
    time_limit_sec > 0 || throw(ArgumentError("time_limit_sec must be positive"))

    mapping = create_aggregate_od_route_map(problem, formulation, data)
    return _enumerate_aggregate_od_route_columns_core(
        mapping, data,
        formulation.route_regularization_weight,
        formulation.repositioning_time,
        formulation.max_wait_time,
        formulation.detour_factor,
        formulation.max_stops,
        max_routes, time_limit_sec,
    )
end
