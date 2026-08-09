"""
Station-simple (elementary-route) label-setting pricer for AggregateODRouteModel:
an alternative to the revisit-tolerant search in `labels.jl`/`search.jl`. A route
may never revisit a station, so a certified `(j,k)` pair settles permanently the
first (and only) time `k` is visited after `j` -- there is no need for a
`station_age` Dict tracking every past visit, only `live_origin_age` for stations
already on the route whose destination hasn't been reached yet.

Because `visited` only grows and is part of the dominance signature (exact match,
not subset), a dominating label's `served_pairs` need not be compared separately:
identical `(current, visited)` plus reduced-cost/time/live-origin-age domination
already implies every future extension available to the dominated label is at
least as good from the dominating one, since which nodes remain reachable depends
only on `visited` and rewards not yet banked are fully described by
`live_origin_age`. Do not add a served-pairs comparison here without re-deriving
that invariant; see `_dominates_aggregate_od_route_station_simple_label` below.

This shares `AggregateODRoutePricingData`/`AggregateODRoutePricingDuals` and the
`_aggregate_od_route_travel`/`_direct_ride_limit` helpers from `data.jl`, and
produces `AggregateODRouteColumn`s via the same `_aggregate_od_route_column_from_label`
convention as the revisit-tolerant pricer (a new method dispatched on this file's
label type).
"""

export AggregateODRouteStationSimpleLabel
export aggregate_od_route_pricing_by_station_simple_label_setting

struct AggregateODRouteStationSimpleLabel
    current::Int
    route::Vector{Int}
    visited::Set{Int}
    time::Float64
    live_origin_age::Dict{Int, Float64}
    served_pairs::Set{Tuple{Int, Int}}
    tau::Float64
    reduced_cost::Float64
end

"""
`live_origin_age` is stored sparse -- `(age_idx, age_val, age_mask)`, the same
representation `_make_sparse_station_ages` (`mechanics.jl`) builds for the
revisit-tolerant pricer and for the PFA station-simple pricer -- rather than a
dense `Vector` over every node, since a partial elementary route typically has
only a handful of live pickup clocks regardless of network size.
"""
struct AggregateODRouteStationSimpleBitsets
    visited_bits::BitSet
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    age_mask::UInt64
end

function _make_aggregate_od_route_station_simple_bitsets(
    label::AggregateODRouteStationSimpleLabel,
    node_index::Dict{Int, Int},
)::AggregateODRouteStationSimpleBitsets
    visited_bits = BitSet()
    for node in label.visited
        push!(visited_bits, node_index[node])
    end

    age_idx, age_val, age_mask = _make_sparse_station_ages(label.live_origin_age, node_index)

    return AggregateODRouteStationSimpleBitsets(visited_bits, age_idx, age_val, age_mask)
end

_aggregate_od_route_station_simple_dominance_signature(
    label::AggregateODRouteStationSimpleLabel,
    bs::AggregateODRouteStationSimpleBitsets,
) = (label.current, bs.visited_bits)

function _aggregate_od_route_station_simple_order_key(
    label::AggregateODRouteStationSimpleLabel,
    label_id::AggregateODRouteLabelId,
)::AggregateODRouteLabelOrderKey
    return (label.reduced_cost, label.time, length(label.route), label_id)
end

function _dominates_aggregate_od_route_station_simple_label(
    a::AggregateODRouteStationSimpleLabel,
    b::AggregateODRouteStationSimpleLabel,
    abs::AggregateODRouteStationSimpleBitsets,
    bbs::AggregateODRouteStationSimpleBitsets,
)::Bool
    a.current == b.current || return false
    abs.visited_bits == bbs.visited_bits || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    a.time <= b.time + 1e-9 || return false
    # dom(b) ⊆ dom(a) and age_a(j) <= age_b(j) for j in dom(b) -- shared with the
    # revisit-tolerant bitset dominance and the PFA station-simple pricer via
    # `mechanics.jl`.
    _sparse_station_ages_dominate(
        abs.age_idx, abs.age_val, abs.age_mask, bbs.age_idx, bbs.age_val, bbs.age_mask,
    ) || return false
    return true
end

function _add_aggregate_od_route_station_simple_label_to_bucket!(
    bucket::SortedDict{AggregateODRouteLabelOrderKey, AggregateODRouteLabelId},
    live_labels::Dict{Int, AggregateODRouteStationSimpleLabel},
    label_bitsets::Dict{Int, AggregateODRouteStationSimpleBitsets},
    label::AggregateODRouteStationSimpleLabel,
    label_id::Int,
    label_bs::AggregateODRouteStationSimpleBitsets,
)
    inserted = true
    dominated_ids = Int[]
    switched = false

    for (_existing_key, existing_id) in pairs(bucket)
        existing_label = live_labels[existing_id]
        existing_bs = label_bitsets[existing_id]

        if !switched && label.reduced_cost > existing_label.reduced_cost + 1e-9
            if _dominates_aggregate_od_route_station_simple_label(existing_label, label, existing_bs, label_bs)
                inserted = false
                break
            end
            continue
        end

        switched = true
        if _dominates_aggregate_od_route_station_simple_label(label, existing_label, label_bs, existing_bs)
            push!(dominated_ids, existing_id)
        end
    end

    if inserted
        for id in dominated_ids
            delete!(bucket, _aggregate_od_route_station_simple_order_key(live_labels[id], id))
            delete!(live_labels, id)
            delete!(label_bitsets, id)
        end
        bucket[_aggregate_od_route_station_simple_order_key(label, label_id)] = label_id
        label_bitsets[label_id] = label_bs
    end
    return inserted, length(dominated_ids)
end

function _initial_aggregate_od_route_station_simple_labels(
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Vector{AggregateODRouteStationSimpleLabel}
    positive_origins = Set{Int}(
        pair[1] for pair in pricing_data.active_pairs if get(duals.sigma, pair, 0.0) > 1e-9
    )
    labels = AggregateODRouteStationSimpleLabel[]
    for node in pricing_data.nodes
        live = Dict{Int, Float64}()
        node in positive_origins && (live[node] = 0.0)
        push!(labels, AggregateODRouteStationSimpleLabel(
            node,
            [node],
            Set{Int}([node]),
            0.0,
            live,
            Set{Tuple{Int, Int}}(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
        ))
    end
    return labels
end

function _aggregate_od_route_station_simple_prune_live_origins(
    live_origin_age::Dict{Int, Float64},
    current::Int,
    visited::Set{Int},
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Dict{Int, Float64}
    remaining = Dict{Int, Float64}()
    for (origin, age) in live_origin_age
        can_still_reward = false
        for pair in pricing_data.active_pairs
            pair[1] == origin || continue
            pair[2] in visited && continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            age + _aggregate_od_route_travel(pricing_data, current, pair[2]) <=
                _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            can_still_reward = true
            break
        end
        can_still_reward && (remaining[origin] = age)
    end
    return remaining
end

function _aggregate_od_route_station_simple_candidate_next_nodes(
    label::AggregateODRouteStationSimpleLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Vector{Int}
    candidates = Int[]
    for next_node in pricing_data.nodes
        next_node in label.visited && continue
        travel_time = _aggregate_od_route_travel(pricing_data, label.current, next_node)

        is_useful = false
        for (origin, age) in label.live_origin_age
            pair = (origin, next_node)
            dual = get(duals.sigma, pair, 0.0)
            dual > 1e-9 || continue
            age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            is_useful = true
            break
        end

        if !is_useful && label.time + travel_time <= pricing_data.max_wait_time + 1e-9
            for pair in pricing_data.active_pairs
                pair[1] == next_node || continue
                pair[2] in label.visited && continue
                get(duals.sigma, pair, 0.0) > 1e-9 || continue
                is_useful = true
                break
            end
        end

        is_useful && push!(candidates, next_node)
    end
    return candidates
end

function _extend_aggregate_od_route_station_simple_label(
    label::AggregateODRouteStationSimpleLabel,
    next_node::Int,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::AggregateODRouteStationSimpleLabel
    next_node in label.visited && throw(ArgumentError("station-simple extension cannot revisit $next_node"))

    travel_time = _aggregate_od_route_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_route = vcat(label.route, next_node)
    visited = copy(label.visited)
    push!(visited, next_node)

    served_pairs = copy(label.served_pairs)
    reward = 0.0
    for (origin, age) in label.live_origin_age
        pair = (origin, next_node)
        dual = get(duals.sigma, pair, 0.0)
        dual > 1e-9 || continue
        age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        if pair ∉ served_pairs
            push!(served_pairs, pair)
            reward += dual
        end
    end

    live = Dict(origin => age + travel_time for (origin, age) in label.live_origin_age)
    opens_origin = any(
        pair -> pair[1] == next_node && get(duals.sigma, pair, 0.0) > 1e-9,
        pricing_data.active_pairs,
    )
    if opens_origin && arrival_time <= pricing_data.max_wait_time + 1e-9
        live[next_node] = 0.0
    end
    live = _aggregate_od_route_station_simple_prune_live_origins(live, next_node, visited, pricing_data, duals)

    new_tau = label.tau + travel_time
    return AggregateODRouteStationSimpleLabel(
        next_node,
        new_route,
        visited,
        arrival_time,
        live,
        served_pairs,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
    )
end

# Upper bound on collectible reward from this label onward. Already-open origins
# need the detour check against their current age; not-yet-visited origins only
# need to still be reachable within the pickup window (ignoring detour, since
# triangle inequality makes direct travel a lower bound on any routed arrival).
function _aggregate_od_route_station_simple_future_reward_bound(
    label::AggregateODRouteStationSimpleLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Float64
    ub = 0.0
    for pair in pricing_data.active_pairs
        dual = get(duals.sigma, pair, 0.0)
        dual > 1e-9 || continue
        pair[2] in label.visited && continue
        if pair[1] in label.visited
            age = get(label.live_origin_age, pair[1], Inf)
            isinf(age) && continue
            age + _aggregate_od_route_travel(pricing_data, label.current, pair[2]) <=
                _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        else
            label.time + _aggregate_od_route_travel(pricing_data, label.current, pair[1]) <=
                pricing_data.max_wait_time + 1e-9 || continue
        end
        ub += dual
    end
    return ub
end

_aggregate_od_route_station_simple_label_priority(
    label::AggregateODRouteStationSimpleLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
) = label.reduced_cost - _aggregate_od_route_station_simple_future_reward_bound(label, pricing_data, duals)

_aggregate_od_route_column_from_label(
    label::AggregateODRouteStationSimpleLabel,
    column_id::Int,
    scenario::Int,
)::AggregateODRouteColumn = AggregateODRouteColumn(
    column_id,
    collect(label.served_pairs),
    label.tau;
    metadata=Dict{String, Any}(
        "scenario" => scenario,
        "route" => Tuple(label.route),
        "reduced_cost" => label.reduced_cost,
    ),
)

function _enumerate_aggregate_od_route_station_simple_pricing_labels(
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals;
    time_limit::Float64,
    reduced_cost_tol::Float64,
    profile::Bool=false,
    stop_if=label -> false,
)
    frontier = PriorityQueue{Int, Float64}()
    live_labels = Dict{Int, AggregateODRouteStationSimpleLabel}()
    label_bitsets = Dict{Int, AggregateODRouteStationSimpleBitsets}()
    dominance_buckets = Dict{Tuple{Int, BitSet}, SortedDict{AggregateODRouteLabelOrderKey, AggregateODRouteLabelId}}()
    best_by_signature = Dict{Any, AggregateODRouteStationSimpleLabel}()

    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))

    exhausted = true
    t_start = time()
    next_label_id = 1
    labels_generated = 0
    labels_rejected_by_dominance = 0
    labels_removed_by_dominance = 0
    stale_pops = 0
    max_frontier_size = 0
    max_live_labels = 0
    t_queue = UInt64(0)
    t_candidates = UInt64(0)
    t_extension = UInt64(0)
    t_dominance = UInt64(0)

    function add_label!(label::AggregateODRouteStationSimpleLabel)
        label_id = next_label_id
        next_label_id += 1
        labels_generated += 1
        live_labels[label_id] = label
        label_bs = _make_aggregate_od_route_station_simple_bitsets(label, node_index)
        signature = _aggregate_od_route_station_simple_dominance_signature(label, label_bs)
        bucket = get!(() -> SortedDict{AggregateODRouteLabelOrderKey, AggregateODRouteLabelId}(), dominance_buckets, signature)

        t0 = profile ? time_ns() : UInt64(0)
        inserted, removed = _add_aggregate_od_route_station_simple_label_to_bucket!(
            bucket, live_labels, label_bitsets, label, label_id, label_bs,
        )
        profile && (t_dominance += time_ns() - t0)
        labels_removed_by_dominance += removed

        if !inserted
            delete!(live_labels, label_id)
            labels_rejected_by_dominance += 1
            return nothing
        end

        t0 = profile ? time_ns() : UInt64(0)
        push!(frontier, label_id => _aggregate_od_route_station_simple_label_priority(label, pricing_data, duals))
        profile && (t_queue += time_ns() - t0)
        max_frontier_size = max(max_frontier_size, length(frontier))
        max_live_labels = max(max_live_labels, length(live_labels))
        return nothing
    end

    for label in _initial_aggregate_od_route_station_simple_labels(pricing_data, duals)
        add_label!(label)
    end

    while !isempty(frontier)
        if time() - t_start > time_limit
            exhausted = false
            break
        end

        t0 = profile ? time_ns() : UInt64(0)
        label_id = popfirst!(frontier).first
        profile && (t_queue += time_ns() - t0)
        label = get(live_labels, label_id, nothing)
        if isnothing(label)
            stale_pops += 1
            continue
        end

        if !isempty(label.served_pairs)
            signature = _aggregate_od_route_column_signature(label.served_pairs)
            incumbent = get(best_by_signature, signature, nothing)
            if isnothing(incumbent) || label.tau < incumbent.tau - 1e-9
                best_by_signature[signature] = label
                if stop_if(label)
                    exhausted = false
                    break
                end
            end
        end

        length(label.route) >= pricing_data.max_stops && continue
        _aggregate_od_route_station_simple_label_priority(label, pricing_data, duals) >= -reduced_cost_tol && continue

        t0 = profile ? time_ns() : UInt64(0)
        next_nodes = _aggregate_od_route_station_simple_candidate_next_nodes(label, pricing_data, duals)
        profile && (t_candidates += time_ns() - t0)

        for next_node in next_nodes
            t0 = profile ? time_ns() : UInt64(0)
            child = _extend_aggregate_od_route_station_simple_label(label, next_node, pricing_data, duals)
            profile && (t_extension += time_ns() - t0)
            add_label!(child)
        end
    end

    stats = (
        labels_generated=labels_generated,
        labels_rejected_by_dominance=labels_rejected_by_dominance,
        labels_removed_by_dominance=labels_removed_by_dominance,
        stale_pops=stale_pops,
        max_frontier_size=max_frontier_size,
        max_live_labels=max_live_labels,
        t_queue_sec=t_queue * 1e-9,
        t_candidates_sec=t_candidates * 1e-9,
        t_extension_sec=t_extension * 1e-9,
        t_dominance_sec=t_dominance * 1e-9,
    )
    return collect(values(best_by_signature)), exhausted, stats
end

function aggregate_od_route_pricing_by_station_simple_label_setting(
    pricing_data::AggregateODRoutePricingData,
    existing_columns::Vector{AggregateODRouteColumn},
    duals::AggregateODRoutePricingDuals;
    next_column_id::Int,
    reduced_cost_tol::Float64=1e-6,
    max_new_columns::Int=1,
    n_candidates::Int=max_new_columns,
    time_limit::Float64=30.0,
    profile::Bool=false,
)
    max_new_columns > 0 || throw(ArgumentError("max_new_columns must be positive"))
    n_candidates >= max_new_columns || throw(ArgumentError("n_candidates must be >= max_new_columns"))
    time_limit > 0 || throw(ArgumentError("time_limit must be positive"))

    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        signature = _aggregate_od_route_column_signature(column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    scored_by_signature = Dict{Any, Tuple{Float64, AggregateODRouteStationSimpleLabel}}()

    function accept_pricing_label!(label::AggregateODRouteStationSimpleLabel)
        isempty(label.served_pairs) && return false
        label.reduced_cost < -reduced_cost_tol || return false
        signature = _aggregate_od_route_column_signature(label.served_pairs)
        label.tau < get(best_pool_tau, signature, Inf) - 1e-9 || return false
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                label.reduced_cost < current[1] - 1e-9 ||
                (abs(label.reduced_cost - current[1]) <= 1e-9 && label.tau < current[2].tau - 1e-9)
            scored_by_signature[signature] = (label.reduced_cost, label)
        end
        return length(scored_by_signature) >= n_candidates
    end

    labels, exhausted, stats = _enumerate_aggregate_od_route_station_simple_pricing_labels(
        pricing_data,
        duals;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        profile=profile,
        stop_if=accept_pricing_label!,
    )

    for label in labels
        isempty(label.served_pairs) && continue
        label.reduced_cost < -reduced_cost_tol || continue
        signature = _aggregate_od_route_column_signature(label.served_pairs)
        label.tau < get(best_pool_tau, signature, Inf) - 1e-9 || continue
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                label.reduced_cost < current[1] - 1e-9 ||
                (abs(label.reduced_cost - current[1]) <= 1e-9 && label.tau < current[2].tau - 1e-9)
            scored_by_signature[signature] = (label.reduced_cost, label)
        end
    end

    scored = collect(values(scored_by_signature))
    sort!(scored, by=entry -> (entry[1], entry[2].tau, string(entry[2].route)))
    scored = scored[1:min(length(scored), n_candidates)]
    scored = scored[1:min(length(scored), max_new_columns)]

    columns = AggregateODRouteColumn[]
    column_id = next_column_id
    for (_, label) in scored
        push!(columns, _aggregate_od_route_column_from_label(label, column_id, pricing_data.scenario))
        column_id += 1
    end
    return columns, exhausted, stats
end
