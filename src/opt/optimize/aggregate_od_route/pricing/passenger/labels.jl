"""
Core label-DP primitives for passenger free-assignment pricing: label creation,
extension, and dominance. `search.jl` orchestrates these into a full pricing
pass; this file is the one to audit for "is the label search correct".

# Operational contract of the column being priced

Same unlimited-capacity, synchronized-start physical route as
`AggregateODRoutePricingLabel` (see that file's module docstring for the shared
station-age/wait/detour rules) -- this pricer changes only what a route gets
*credit* for:

- a visit to origin `j` within `max_wait_time` opens a live pickup clock for `j`,
  exactly as before;
- a later visit to `k` certifies `(p, j, k)` for every passenger `p` with a
  positive-reward candidate on that pair, when elapsed time since the pickup is
  at most that candidate's own `ride_limit` (passenger-specific, not a single
  `detour_factor * travel_time` shared by all pairs);
- but a passenger is not "served" by the union of everything certified -- of all
  the assignments the finished route certifies for `p`, only the single best one
  counts. `activated_reward_layers` tracks running per-passenger maxima (via the
  prefix-layer encoding from `data.jl`) so labels can compare/dominate on this
  without carrying dense per-passenger reward vectors or a discrete "which
  assignment is current best" pointer -- reaching a strictly better destination
  later activates only the incremental layers between the old and new reward, so
  the layer weight sum is always exactly `sum(max reward per passenger)`, never a
  double count. Reaching a worse destination afterwards activates nothing (its
  prefix mask is already a subset of what's active).

No onboard-passenger state, capacity resource, or drop-off subset enumeration is
modeled -- unbounded capacity means certifying `p`'s assignment never prevents
certifying anyone else's, so there is nothing to branch on beyond the physical
route itself. The concrete `(j, k)` a passenger is ultimately assigned to is
reconstructed only for finished, negative-reduced-cost routes (`search.jl`'s
route replay) -- not tracked while labels are being extended.
"""

export initial_passenger_free_assignment_pricing_labels
export extend_passenger_free_assignment_pricing_label

function initial_passenger_free_assignment_pricing_labels(
    pricing_data::PassengerFreeAssignmentPricingData,
)::Vector{PassengerFreeAssignmentPricingLabel}
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end

    labels = PassengerFreeAssignmentPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, PassengerFreeAssignmentPricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            RewardLayerBitset(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
            1,
        ))
    end
    return labels
end

function _has_useful_live_passenger_free_assignment_origin(
    label::PassengerFreeAssignmentPricingLabel,
    pricing_data::PassengerFreeAssignmentPricingData,
)::Bool
    for (station, age) in label.station_age
        opportunities = get(pricing_data.assignments_by_origin, station, PassengerAssignmentOpportunity[])
        for opp in opportunities
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            t_to_dest = opp.destination == label.current ? 0.0 :
                _passenger_free_assignment_travel(pricing_data, label.current, opp.destination)
            age + t_to_dest <= opp.ride_limit + 1e-9 || continue
            return true
        end
    end
    return false
end

"""
    is_useful_destination(label, k)

A station `k` is worth visiting next as a destination if some *currently live*
origin age can still certify a currently-inactive layer there. A station `j` is
worth visiting as an origin (only while still inside the pickup window) if
visiting it could open a live clock that later unlocks a currently-inactive
layer, judged via `origin_layer_mask` (the union of everything reachable from
that origin, an optimistic pre-filter -- true feasibility from that origin is
re-checked once its age actually becomes live).
"""
function _passenger_free_assignment_candidate_next_nodes(
    label::PassengerFreeAssignmentPricingLabel,
    pricing_data::PassengerFreeAssignmentPricingData;
    max_visits_per_node::Int=pricing_data.max_visits_per_node,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_passenger_free_assignment_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, mask) in pricing_data.origin_layer_mask
            origin == label.current && continue
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            arrival_time = label.time + _passenger_free_assignment_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    # Driven from the label's *live origins* rather than from every destination
    # group: only a live origin can make a destination useful, and pruning keeps
    # the live set small, whereas `assignments_by_destination` spans all
    # `~P * n^2` opportunities regardless of label state.
    for (origin, origin_age) in label.station_age
        for opp in get(pricing_data.assignments_by_origin, origin, PassengerAssignmentOpportunity[])
            opp.destination == label.current && continue
            opp.destination in candidate_nodes && continue
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            origin_age + _passenger_free_assignment_travel(pricing_data, label.current, opp.destination) <=
                opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
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

function extend_passenger_free_assignment_pricing_label(
    label::PassengerFreeAssignmentPricingLabel,
    next_node::Int,
    pricing_data::PassengerFreeAssignmentPricingData,
)::Vector{PassengerFreeAssignmentPricingLabel}
    travel_time = _passenger_free_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)

    certified_layers, reward = _certify_passenger_free_assignment_layers_at_node(
        next_node,
        label.station_age,
        travel_time,
        label.activated_reward_layers,
        pricing_data,
    )

    # Age, reset, and prune in ONE pass. Previously this built an aged Dict and
    # then a second pruned Dict from it -- two allocations and two traversals per
    # extension, on the hottest path in the search.
    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        station == next_node && continue  # handled by the reset below
        aged = age + travel_time
        _passenger_free_assignment_age_is_useful(station, aged, certified_layers, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        # A fresh clock at the arrival station: age 0 is the most useful an age can
        # be, but it still only earns a slot if it can certify something new.
        _passenger_free_assignment_age_is_useful(next_node, 0.0, certified_layers, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    elseif haskey(label.station_age, next_node)
        # Past the cutoff the visit creates no new clock, so `next_node`'s existing
        # clock (if any) just ages like the rest.
        aged = label.station_age[next_node] + travel_time
        _passenger_free_assignment_age_is_useful(next_node, aged, certified_layers, pricing_data, next_node) &&
            (aged_station[next_node] = aged)
    end

    child = PassengerFreeAssignmentPricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified_layers,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )

    return PassengerFreeAssignmentPricingLabel[child]
end

_passenger_free_assignment_dominance_signature(label::PassengerFreeAssignmentPricingLabel) = label.current

"""
The cheap bookkeeping signature used by the label search itself (dedup within
`best_by_signature`, see `search.jl`) -- deliberately *not* the final column
signature. Two labels can share this signature (same running per-passenger
maxima) while representing different physical routes with different concrete
`(p, j, k)` assignments and therefore different station-linking coefficients;
only route replay on a finished, accepted route (section 13) resolves which
concrete assignment each passenger actually gets.
"""
_passenger_free_assignment_layer_signature(label::PassengerFreeAssignmentPricingLabel) = label.activated_reward_layers

function _passenger_free_assignment_label_order_key(
    label::PassengerFreeAssignmentPricingLabel,
    label_id::PassengerFreeAssignmentLabelId,
)::PassengerFreeAssignmentLabelOrderKey
    return (
        label.reduced_cost,
        label.time,
        label.route_length,
        label_id,
    )
end

_create_passenger_free_assignment_dominance_bucket() =
    SortedDict{PassengerFreeAssignmentLabelOrderKey, PassengerFreeAssignmentLabelId}()

function _make_passenger_free_assignment_label_bitsets(
    label::PassengerFreeAssignmentPricingLabel,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::PassengerFreeAssignmentLabelBitsets
    n_live = length(label.station_age)
    age_idx = Vector{Int32}(undef, n_live)
    age_val = Vector{Float64}(undef, n_live)
    i = 1
    for (station, age) in label.station_age
        age_idx[i] = Int32(node_index[station])
        age_val[i] = age
        i += 1
    end
    perm = sortperm(age_idx)
    return PassengerFreeAssignmentLabelBitsets(
        copy(label.activated_reward_layers), age_idx[perm], age_val[perm],
    )
end

function _dominates_passenger_free_assignment_label(
    a::PassengerFreeAssignmentPricingLabel,
    b::PassengerFreeAssignmentPricingLabel,
    bounded_max_stops::Bool,
)::Bool
    _passenger_free_assignment_dominance_signature(a) == _passenger_free_assignment_dominance_signature(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    issubset(a.activated_reward_layers, b.activated_reward_layers) || return false
    all_stations = union(keys(a.station_age), keys(b.station_age), (a.current, b.current))
    for station in all_stations
        get(a.station_age, station, Inf) <= get(b.station_age, station, Inf) + 1e-9 || return false
    end
    return true
end

function _dominates_passenger_free_assignment_label(
    a::PassengerFreeAssignmentPricingLabel,
    b::PassengerFreeAssignmentPricingLabel,
    abs::PassengerFreeAssignmentLabelBitsets,
    bbs::PassengerFreeAssignmentLabelBitsets,
    bounded_max_stops::Bool,
)::Bool
    _passenger_free_assignment_dominance_signature(a) == _passenger_free_assignment_dominance_signature(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    # Cheap necessary condition before the (more expensive) subset test: a's mask
    # cannot be a subset of b's if it has more bits set.
    length(abs.activated_bits) <= length(bbs.activated_bits) || return false
    issubset(abs.activated_bits, bbs.activated_bits) || return false
    # `dom(age_b) subseteq dom(age_a)` and `age_a(j) <= age_b(j)` for all j in
    # dom(age_b) -- exactly equivalent to the dense "all stations, missing = Inf"
    # rule (a live age absent from `a` compares as Inf and always fails; one
    # absent from `b` compares as <= Inf and always passes), but costs O(#live)
    # instead of O(n_nodes). Both arrays are sorted, so one merge walk suffices.
    length(abs.age_idx) >= length(bbs.age_idx) || return false
    ia = 1
    na = length(abs.age_idx)
    @inbounds for ib in eachindex(bbs.age_idx)
        j = bbs.age_idx[ib]
        while ia <= na && abs.age_idx[ia] < j
            ia += 1
        end
        (ia <= na && abs.age_idx[ia] == j) || return false
        abs.age_val[ia] <= bbs.age_val[ib] + 1e-9 || return false
    end
    return true
end

function _add_passenger_free_assignment_label_to_bucket!(
    bucket::SortedDict{PassengerFreeAssignmentLabelOrderKey, PassengerFreeAssignmentLabelId},
    live_labels::Dict{Int, PassengerFreeAssignmentPricingLabel},
    label_bitsets::Dict{Int, PassengerFreeAssignmentLabelBitsets},
    label::PassengerFreeAssignmentPricingLabel,
    label_id::Int,
    label_bs::PassengerFreeAssignmentLabelBitsets,
    bounded_max_stops::Bool,
)
    inserted = true
    dominated_ids = Int[]
    switched = false

    for (_existing_key, existing_id) in pairs(bucket)
        existing_label = live_labels[existing_id]
        existing_bs = label_bitsets[existing_id]

        if !switched && label.reduced_cost > existing_label.reduced_cost + 1e-9
            if _dominates_passenger_free_assignment_label(existing_label, label, existing_bs, label_bs, bounded_max_stops)
                inserted = false
                break
            end
            continue
        end

        switched = true
        if _dominates_passenger_free_assignment_label(label, existing_label, label_bs, existing_bs, bounded_max_stops)
            push!(dominated_ids, existing_id)
        end
    end

    if inserted
        for id in dominated_ids
            delete!(bucket, _passenger_free_assignment_label_order_key(live_labels[id], id))
            delete!(live_labels, id)
            delete!(label_bitsets, id)
        end
        bucket[_passenger_free_assignment_label_order_key(label, label_id)] = label_id
        label_bitsets[label_id] = label_bs
    end
    return inserted, length(dominated_ids)
end

function _passenger_free_assignment_column_signature(assignments)::Tuple{Vararg{Tuple{Int, Int, Int}}}
    return Tuple(sort!(collect(assignments)))
end

_passenger_free_assignment_column_signature(column::PassengerFreeAssignmentRouteColumn) =
    _passenger_free_assignment_column_signature(column.assignments)
