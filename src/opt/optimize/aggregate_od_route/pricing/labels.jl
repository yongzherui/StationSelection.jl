"""
Core label-DP primitives for AggregateODRouteModel pricing: label creation,
extension, and dominance. `search.jl` orchestrates these into a full pricing
pass; this file is the one to audit for "is the label search correct".

# Operational contract of the column being priced

This is deliberately **not** a finite-capacity passenger-loading problem. A
column is one unlimited-capacity vehicle route under the following synchronized
service assumptions:

- the route clock and every passenger's wait clock start at `t = 0`;
- a visit to origin station `j` can pick up every relevant `(j,k)` passenger
  when the visit's arrival time is at most `max_wait_time`;
- after that pickup, `(j,k)` is certified by a later visit to `k` when elapsed
  onboard time is at most `detour_factor * routing_cost(j,k)`; and
- certifying one pair consumes no capacity and does not prevent the same route
  from certifying any other pair whose independent wait/detour tests pass.

Accordingly, `station_age[j]` is the elapsed time since the most recent eligible
pickup visit to `j`, not a vehicle-load state. `served_pairs` is the set of all
pairs independently certified by the stop sequence. It can be broad: for
example, `[1,2,3,4]` can certify all six forward pairs when their time tests
pass. This high overlap is intended model behavior and is also why the route
master has a potentially large set-covering LP/IP gap: pricing gives a column
the sum of the dual rewards of every pair it certifies, whereas the final MIP
must purchase the whole route.

Do not add load resources or capacity dominance rules here unless the model's
operational semantics are intentionally being changed. Conversely, any future
finite-capacity version must add passenger quantities and leg-by-leg onboard
load state; pair certification alone is not a capacity formulation.
"""

export initial_aggregate_od_route_pricing_labels
export extend_aggregate_od_route_pricing_label

function initial_aggregate_od_route_pricing_labels(
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Vector{AggregateODRoutePricingLabel}
    endpoints = Set{Int}()
    for (j, k) in pricing_data.active_pairs
        push!(endpoints, j)
        push!(endpoints, k)
    end

    labels = AggregateODRoutePricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, AggregateODRoutePricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            Set{Tuple{Int, Int}}(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
            1,
        ))
    end
    return labels
end

function _has_useful_live_aggregate_od_route_origin(
    label::AggregateODRoutePricingLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Bool
    for (station, age) in label.station_age
        for pair in pricing_data.active_pairs
            pair[1] == station || continue
            pair ∈ label.served_pairs && continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            t_to_dest = pair[2] == label.current ? 0.0 : _aggregate_od_route_travel(pricing_data, label.current, pair[2])
            age + t_to_dest <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            return true
        end
    end
    return false
end

function _aggregate_od_route_candidate_next_nodes(
    label::AggregateODRoutePricingLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals;
    max_visits_per_node::Int=pricing_data.max_visits_per_node,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_aggregate_od_route_origin(label, pricing_data, duals)
        return Int[]
    end
    for pair in pricing_data.active_pairs
        get(duals.sigma, pair, 0.0) > 1e-9 || continue
        origin, destination = pair
        remembered = pair ∈ label.served_pairs

        if !remembered && origin != label.current
            arrival_time = label.time + _aggregate_od_route_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end

        remembered && continue
        destination == label.current && continue
        origin_age = get(label.station_age, origin, Inf)
        origin_age + _aggregate_od_route_travel(pricing_data, label.current, destination) <=
            _direct_ride_limit(pricing_data, pair) + 1e-9 && push!(candidate_nodes, destination)
    end

    if max_visits_per_node < typemax(Int)
        visit_counts = Dict{Int, Int}()
        for node in label.route
            visit_counts[node] = get(visit_counts, node, 0) + 1
        end
        filter!(node -> get(visit_counts, node, 0) < max_visits_per_node, candidate_nodes)
    end

    return sort!(collect(candidate_nodes))
end

function extend_aggregate_od_route_pricing_label(
    label::AggregateODRoutePricingLabel,
    next_node::Int,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Vector{AggregateODRoutePricingLabel}
    travel_time = _aggregate_od_route_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)
    aged_station = Dict(station => age + travel_time for (station, age) in label.station_age)

    certified_pairs, reward =
        _certify_aggregate_od_route_pairs_at_node(
            next_node,
            label.station_age,
            travel_time,
            label.served_pairs,
            pricing_data,
            duals,
        )
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        aged_station[next_node] = 0.0
    end
    aged_station = _prune_irrelevant_aggregate_od_route_station_ages(
        aged_station,
        certified_pairs,
        pricing_data,
        duals,
        next_node,
    )

    child = AggregateODRoutePricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified_pairs,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )

    return AggregateODRoutePricingLabel[child]
end

_aggregate_od_route_dominance_signature(label::AggregateODRoutePricingLabel) = label.current

function _aggregate_od_route_label_order_key(
    label::AggregateODRoutePricingLabel,
    label_id::AggregateODRouteLabelId,
)::AggregateODRouteLabelOrderKey
    return (
        label.reduced_cost,
        label.time,
        label.route_length,
        label_id,
    )
end

_create_aggregate_od_route_dominance_bucket() = SortedDict{AggregateODRouteLabelOrderKey, AggregateODRouteLabelId}()

function _make_aggregate_od_route_label_bitsets(
    label::AggregateODRoutePricingLabel,
    pair_index::Dict{Tuple{Int, Int}, Int},
    n_pairs::Int,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::AggregateODRouteLabelBitsets
    served_bits = BitSet()
    for pair in label.served_pairs
        push!(served_bits, pair_index[pair])
    end

    station_age = fill(Inf, n_nodes)
    for (station, age) in label.station_age
        station_age[node_index[station]] = age
    end

    return AggregateODRouteLabelBitsets(served_bits, station_age)
end

function _dominates_aggregate_od_route_label(
    a::AggregateODRoutePricingLabel,
    b::AggregateODRoutePricingLabel,
    bounded_max_stops::Bool,
)::Bool
    _aggregate_od_route_dominance_signature(a) == _aggregate_od_route_dominance_signature(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    issubset(a.served_pairs, b.served_pairs) || return false
    all_stations = union(keys(a.station_age), keys(b.station_age), (a.current, b.current))
    for station in all_stations
        get(a.station_age, station, Inf) <= get(b.station_age, station, Inf) + 1e-9 || return false
    end
    return true
end

function _dominates_aggregate_od_route_label(
    a::AggregateODRoutePricingLabel,
    b::AggregateODRoutePricingLabel,
    abs::AggregateODRouteLabelBitsets,
    bbs::AggregateODRouteLabelBitsets,
    bounded_max_stops::Bool,
)::Bool
    _aggregate_od_route_dominance_signature(a) == _aggregate_od_route_dominance_signature(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    issubset(abs.served_bits, bbs.served_bits) || return false
    @inbounds for i in eachindex(abs.station_age)
        abs.station_age[i] <= bbs.station_age[i] + 1e-9 || return false
    end
    return true
end

function _add_aggregate_od_route_label_to_bucket!(
    bucket::SortedDict{AggregateODRouteLabelOrderKey, AggregateODRouteLabelId},
    live_labels::Dict{Int, AggregateODRoutePricingLabel},
    label_bitsets::Dict{Int, AggregateODRouteLabelBitsets},
    label::AggregateODRoutePricingLabel,
    label_id::Int,
    label_bs::AggregateODRouteLabelBitsets,
    bounded_max_stops::Bool,
)
    inserted = true
    dominated_ids = Int[]
    switched = false

    for (_existing_key, existing_id) in pairs(bucket)
        existing_label = live_labels[existing_id]
        existing_bs = label_bitsets[existing_id]

        if !switched && label.reduced_cost > existing_label.reduced_cost + 1e-9
            if _dominates_aggregate_od_route_label(existing_label, label, existing_bs, label_bs, bounded_max_stops)
                inserted = false
                break
            end
            continue
        end

        switched = true
        if _dominates_aggregate_od_route_label(label, existing_label, label_bs, existing_bs, bounded_max_stops)
            push!(dominated_ids, existing_id)
        end
    end

    if inserted
        for id in dominated_ids
            delete!(bucket, _aggregate_od_route_label_order_key(live_labels[id], id))
            delete!(live_labels, id)
            delete!(label_bitsets, id)
        end
        bucket[_aggregate_od_route_label_order_key(label, label_id)] = label_id
        label_bitsets[label_id] = label_bs
    end
    return inserted, length(dominated_ids)
end

function _aggregate_od_route_label_priority(
    label::AggregateODRoutePricingLabel,
    duals::AggregateODRoutePricingDuals,
)::Float64
    return label.reduced_cost
end

function _aggregate_od_route_column_signature(pairs)::Tuple{Vararg{Tuple{Int, Int}}}
    return Tuple(sort!(collect(pairs)))
end

_aggregate_od_route_column_signature(column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature(column.od_pairs)

function _aggregate_od_route_column_from_label(
    label::AggregateODRoutePricingLabel,
    column_id::Int,
    scenario::Int,
)::AggregateODRouteColumn
    return AggregateODRouteColumn(
        column_id,
        collect(label.served_pairs),
        label.tau;
        metadata=Dict{String, Any}(
            "scenario" => scenario,
            "route" => Tuple(label.route),
            "reduced_cost" => label.reduced_cost,
        ),
    )
end
