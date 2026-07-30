"""
Station-simple (elementary-route) label-setting pricer for the passenger
free-assignment subproblem: an alternative to the revisit-tolerant search in
`labels.jl`/`search.jl` in which a physical route may never revisit a station.

# What elementarity changes (and what it does not)

Only the *route universe* changes -- the reward contract is identical to the
revisit-tolerant pricer (see `labels.jl`'s module docstring): a visit to origin
`j` within `max_wait_time` opens a live pickup clock; a later visit to `k`
certifies `(p, j, k)` for passengers whose clock survives the ride limit; and a
passenger banks only its single best certified reward, tracked incrementally via
`activated_reward_layers`.

Crucially, the per-passenger *maximum* reward means elementarity does NOT let us
drop the layer/age bookkeeping the way the aggregate pair-based `station_simple.jl`
drops `station_age` for `live_origin_age`: reaching a strictly better dropoff later
still activates incremental layers, so a live clock stays useful even after it has
already certified something. What elementarity removes is clock *resets* -- a
station is visited exactly once, so a clock only ages and is never reopened.

# Why this is faster

Two levers:

1. **Fewer extensions.** Candidate generation drops any already-visited node, so
   the branching factor shrinks as a route grows.

2. **Stronger dominance from a subset-visited rule.** Buckets are keyed on
   `current` alone, and dominance adds the elementary resource `U_a ⊆ U_b`: a
   label that has visited a subset of another's stations has fewer forbidden
   futures, so it dominates whenever it is otherwise no worse. This is strictly
   stronger than the revisit-tolerant pricer, which ignores `visited` entirely
   when uncapped, and prunes the "wandered" labels an elementary search would
   otherwise carry.

   An earlier version keyed buckets on the exact `(current, visited)` pair, which
   makes each bucket tiny but lets labels with differing visited sets never
   compare -- measured letting the live population balloon 3-6x. The subset rule
   above recovers that cross-domination; see
   `notes/2026-07-30_passenger_station_simple_pricing.md`.

# Correctness caveat

Restricting to elementary routes restricts the column universe the master problem
prices over. Where the model's optimum genuinely wants a revisiting route this
pricer is a *heuristic* -- it can terminate CG with a weaker LP bound or miss
improving columns (the aggregate pair-based `use_station_simple` did exactly this
on some instance families). It is therefore opt-in and off by default; validate
the LP bound against the revisit-tolerant pricer before relying on it.

# Reuse

Shares `PassengerFreeAssignmentPricingData` (no new data struct) and the
`_passenger_free_assignment_travel`, `_certify_passenger_free_assignment_layers_at_node`,
`_passenger_free_assignment_age_is_useful`, `_has_useful_live_passenger_free_assignment_origin`,
`_passenger_free_assignment_compensation`, and `_passenger_free_assignment_remaining_reward_bound`
primitives from `data.jl`/`labels.jl`/`search.jl`. Emits the same
`PassengerFreeAssignmentRouteColumn` via the identical route-replay path
(`_passenger_free_assignment_column_from_route`), since replay is agnostic to how
the physical route was found and replays an elementary route unchanged.
"""

export PassengerFreeAssignmentStationSimpleLabel
export passenger_free_assignment_pricing_by_station_simple_label_setting

"""
A partial elementary route. Same fields as `PassengerFreeAssignmentPricingLabel`
minus `visited_mask` (the `UInt64` distinct-station budget mask, which had a
64-station ceiling and is now subsumed) and plus an authoritative `visited` set.
`route_length == length(visited)` always holds here.
"""
struct PassengerFreeAssignmentStationSimpleLabel
    current::Int
    route::Vector{Int}
    visited::Set{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    activated_reward_layers::RewardLayerBitset
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

"""
Hot-path mirror of a station-simple label. `visited_bits` supplies the compact
visited-subset resource used by dominance; `activated_bits`/`age_idx`/`age_val` mirror
`PassengerFreeAssignmentLabelBitsets` so the shared reward bound and the
compensated-layer/age dominance tests apply unchanged. `age_idx` is sorted
ascending, `age_val` parallel to it.
"""
struct PassengerFreeAssignmentStationSimpleBitsets
    visited_bits::BitSet
    activated_bits::BitSet
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
end

function _make_passenger_free_assignment_station_simple_bitsets(
    label::PassengerFreeAssignmentStationSimpleLabel,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::PassengerFreeAssignmentStationSimpleBitsets
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
    visited_bits = BitSet()
    for node in label.visited
        push!(visited_bits, node_index[node])
    end
    return PassengerFreeAssignmentStationSimpleBitsets(
        visited_bits, copy(label.activated_reward_layers), age_idx[perm], age_val[perm],
    )
end

# Bucket on `current` alone: labels at the same station compare regardless of
# their visited sets, so the subset-visited dominance in
# `_dominates_passenger_free_assignment_station_simple_label` can actually fire
# across differing routes (that cross-domination is the whole point of the subset
# rule over exact `(current, visited)` bucketing). `bs` is unused but kept in the
# signature so the call site matches the exact-visited variant's shape.
_passenger_free_assignment_station_simple_signature(
    label::PassengerFreeAssignmentStationSimpleLabel,
    ::PassengerFreeAssignmentStationSimpleBitsets,
) = label.current

_passenger_free_assignment_station_simple_entry_order_key(entry) =
    (entry.label.reduced_cost, entry.label.time, entry.label.route_length, entry.id)

struct PassengerFreeAssignmentStationSimpleBucketEntry
    id::PassengerFreeAssignmentLabelId
    label::PassengerFreeAssignmentStationSimpleLabel
    bitsets::PassengerFreeAssignmentStationSimpleBitsets
end

const PassengerFreeAssignmentStationSimpleDominanceBucket =
    Vector{PassengerFreeAssignmentStationSimpleBucketEntry}

"""
    _dominates_passenger_free_assignment_station_simple_label(a, b, abs, bbs, layer_weight)

`a` dominates `b`: every completion of `b` has a counterpart from `a` at least as
good. Callers only ever compare labels drawn from the same `current` bucket, so
`a.current == b.current` is re-checked only as a cheap guard.

The visited resource is a **subset** test, `U_a ⊆ U_b`, not equality. For an
elementary route `visited` is the set of forbidden future stations, so if `a` has
visited a subset of what `b` has, every station `b` may still visit `a` may visit
too -- hence every completion feasible for `b` is feasible from `a`. This is
strictly stronger than the exact-`(current, visited)` bucketing it replaced: a
"lean" label (visited a subset) can now kill a "wandered" one that forbade itself
extra stations for no gain, which the exact rule structurally could not, and which
was measured letting the live-label population balloon 3-6x (see the note). Because
`U_a ⊆ U_b` implies `route_length_a <= route_length_b`, the `max_stops` resource is
subsumed and needs no separate check. The remaining conditions are the
revisit-tolerant pricer's:

  - `time_a <= time_b`;
  - the compensated reward-layer budget `rc_a + w(A_a ∖ A_b) <= rc_b` (see
    `_passenger_free_assignment_compensation` and the dominance docstring in
    `labels.jl` for why this, not `issubset` on layers, is the sound test);
  - every live station age in `a` is no larger than `b`'s (sparse merge walk).
"""
function _dominates_passenger_free_assignment_station_simple_label(
    a::PassengerFreeAssignmentStationSimpleLabel,
    b::PassengerFreeAssignmentStationSimpleLabel,
    abs::PassengerFreeAssignmentStationSimpleBitsets,
    bbs::PassengerFreeAssignmentStationSimpleBitsets,
    layer_weight::Vector{Float64},
)::Bool
    a.current == b.current || return false
    issubset(abs.visited_bits, bbs.visited_bits) || return false
    a.time <= b.time + 1e-9 || return false
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    _passenger_free_assignment_compensation(
        abs.activated_bits, bbs.activated_bits, layer_weight, budget,
    ) <= budget || return false
    # `dom(age_b) ⊆ dom(age_a)` and `age_a(j) <= age_b(j)` for j in dom(age_b) --
    # the same O(#live) merge as the revisit-tolerant bitset dominance.
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

"""
Insert `label` into its dominance bucket unless an incumbent dominates it, evict
the incumbents it dominates. Identical structure to
`_add_passenger_free_assignment_label_to_bucket!` (reduced-cost-ordered `Vector`,
split-at-`rc` walk, in-order eviction); see that function's docstring.
"""
function _add_passenger_free_assignment_station_simple_label_to_bucket!(
    bucket::PassengerFreeAssignmentStationSimpleDominanceBucket,
    live_labels::Dict{Int, PassengerFreeAssignmentStationSimpleLabel},
    label::PassengerFreeAssignmentStationSimpleLabel,
    label_id::Int,
    label_bs::PassengerFreeAssignmentStationSimpleBitsets,
    layer_weight::Vector{Float64},
    dominated::Vector{Int},
)
    inserted = true
    empty!(dominated)
    switched = false

    @inbounds for i in eachindex(bucket)
        entry = bucket[i]
        existing_label = entry.label

        if !switched && label.reduced_cost > existing_label.reduced_cost + 1e-9
            if _dominates_passenger_free_assignment_station_simple_label(
                    existing_label, label, entry.bitsets, label_bs, layer_weight)
                inserted = false
                break
            end
            continue
        end

        switched = true
        if _dominates_passenger_free_assignment_station_simple_label(
                label, existing_label, label_bs, entry.bitsets, layer_weight)
            push!(dominated, i)
        end
    end

    n_dominated = length(dominated)
    if inserted
        @inbounds for i in dominated
            delete!(live_labels, bucket[i].id)
        end
        deleteat!(bucket, dominated)
        new_entry = PassengerFreeAssignmentStationSimpleBucketEntry(label_id, label, label_bs)
        pos = searchsortedfirst(bucket, new_entry; by=_passenger_free_assignment_station_simple_entry_order_key)
        insert!(bucket, pos, new_entry)
    end
    return inserted, n_dominated
end

function _initial_passenger_free_assignment_station_simple_labels(
    pricing_data::PassengerFreeAssignmentPricingData,
)::Vector{PassengerFreeAssignmentStationSimpleLabel}
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end

    labels = PassengerFreeAssignmentStationSimpleLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, PassengerFreeAssignmentStationSimpleLabel(
            node,
            [node],
            Set{Int}([node]),
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

"""
Candidate next nodes for an elementary label: the revisit-tolerant
`_passenger_free_assignment_candidate_next_nodes` restricted to unvisited nodes,
with the `max_visits_per_node` and station-budget branches removed (both
subsumed by elementarity).
"""
function _passenger_free_assignment_station_simple_candidate_next_nodes(
    label::PassengerFreeAssignmentStationSimpleLabel,
    pricing_data::PassengerFreeAssignmentPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_passenger_free_assignment_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, mask) in pricing_data.origin_layer_mask
            origin in label.visited && continue  # elementary: no revisit (also excludes current)
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            arrival_time = label.time + _passenger_free_assignment_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    for (origin, origin_age) in label.station_age
        for opp in get(pricing_data.assignments_by_origin, origin, PassengerAssignmentOpportunity[])
            opp.destination in label.visited && continue
            opp.destination in candidate_nodes && continue
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            origin_age + _passenger_free_assignment_travel(pricing_data, label.current, opp.destination) <=
                opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
    end

    return sort!(collect(candidate_nodes))
end

"""
Extend an elementary label to `next_node` (which must be unvisited). Reward
certification and clock aging are identical to
`extend_passenger_free_assignment_pricing_label`; the revisit-tolerant version's
special handling of a re-visited `next_node` clock is simply unreachable here
(`next_node ∉ visited ⊇ keys(station_age)`), so it is omitted.
"""
function _extend_passenger_free_assignment_station_simple_label(
    label::PassengerFreeAssignmentStationSimpleLabel,
    next_node::Int,
    pricing_data::PassengerFreeAssignmentPricingData,
)::PassengerFreeAssignmentStationSimpleLabel
    next_node in label.visited &&
        throw(ArgumentError("station-simple extension cannot revisit $next_node"))

    travel_time = _passenger_free_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)
    new_visited = copy(label.visited)
    push!(new_visited, next_node)

    certified_layers, reward = _certify_passenger_free_assignment_layers_at_node(
        next_node,
        label.station_age,
        travel_time,
        label.activated_reward_layers,
        pricing_data,
    )

    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        aged = age + travel_time
        _passenger_free_assignment_age_is_useful(station, aged, certified_layers, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        _passenger_free_assignment_age_is_useful(next_node, 0.0, certified_layers, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    end

    return PassengerFreeAssignmentStationSimpleLabel(
        next_node,
        new_route,
        new_visited,
        arrival_time,
        aged_station,
        certified_layers,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )
end

function _enumerate_passenger_free_assignment_station_simple_pricing_labels(
    pricing_data::PassengerFreeAssignmentPricingData;
    time_limit::Float64,
    reduced_cost_tol::Float64,
    use_reduced_cost_pruning::Bool=true,
    profile::Bool=false,
    stop_if=label -> false,
)
    frontier = PriorityQueue{Int, Float64}()
    live_labels = Dict{Int, PassengerFreeAssignmentStationSimpleLabel}()
    dominance_buckets = Dict{Int, PassengerFreeAssignmentStationSimpleDominanceBucket}()
    best_by_signature = Dict{Any, PassengerFreeAssignmentStationSimpleLabel}()

    n_nodes = length(pricing_data.nodes)
    search_index = _build_passenger_free_assignment_search_index(pricing_data)
    bound_workspace = _create_passenger_free_assignment_bound_workspace(n_nodes)
    node_index = search_index.node_index
    # Reused across every bucket insertion: indices of the entries the incoming
    # label dominates, in ascending order.
    dominated_scratch = Int[]

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

    # The reward bound reads only `current`/`time`/`activated_reward_layers` and the
    # bitsets' `age_idx`/`age_val`, so it applies unchanged to station-simple labels
    # (its `label`/`label_bs` annotations were loosened for exactly this reuse).
    remaining_reward_bound(label, label_bs) =
        _passenger_free_assignment_remaining_reward_bound(
            label, label_bs, pricing_data, search_index, bound_workspace,
        )

    label_priority(label, label_bs) = label.reduced_cost - remaining_reward_bound(label, label_bs)

    function add_label!(label::PassengerFreeAssignmentStationSimpleLabel)
        label_id = next_label_id
        next_label_id += 1
        labels_generated += 1
        live_labels[label_id] = label
        label_bs = _make_passenger_free_assignment_station_simple_bitsets(label, node_index, n_nodes)
        signature = _passenger_free_assignment_station_simple_signature(label, label_bs)
        bucket = get!(() -> PassengerFreeAssignmentStationSimpleDominanceBucket(), dominance_buckets, signature)
        t0 = profile ? time_ns() : UInt64(0)
        inserted, removed = _add_passenger_free_assignment_station_simple_label_to_bucket!(
            bucket, live_labels, label, label_id, label_bs,
            pricing_data.layer_weight, dominated_scratch,
        )
        profile && (t_dominance += time_ns() - t0)
        labels_removed_by_dominance += removed
        if inserted
            t0 = profile ? time_ns() : UInt64(0)
            enqueue!(frontier, label_id => label_priority(label, label_bs))
            profile && (t_queue += time_ns() - t0)
            max_frontier_size = max(max_frontier_size, length(frontier))
            max_live_labels = max(max_live_labels, length(live_labels))
        else
            delete!(live_labels, label_id)
            labels_rejected_by_dominance += 1
        end
        return label_bs
    end

    for label in _initial_passenger_free_assignment_station_simple_labels(pricing_data)
        add_label!(label)
    end

    while !isempty(frontier)
        if time() - t_start > time_limit
            exhausted = false
            break
        end

        t0 = profile ? time_ns() : UInt64(0)
        label_id, popped_priority = dequeue_pair!(frontier)
        profile && (t_queue += time_ns() - t0)
        if !haskey(live_labels, label_id)
            stale_pops += 1
            continue
        end
        label = live_labels[label_id]

        if !isempty(label.activated_reward_layers)
            signature = label.activated_reward_layers
            incumbent = get(best_by_signature, signature, nothing)
            if isnothing(incumbent) || label.tau < incumbent.tau - 1e-9
                best_by_signature[signature] = label
                if stop_if(label)
                    exhausted = false
                    break
                end
            end
        end

        label.route_length >= pricing_data.max_stops && continue
        if use_reduced_cost_pruning
            popped_priority >= -reduced_cost_tol && continue
        end

        t0 = profile ? time_ns() : UInt64(0)
        next_nodes = _passenger_free_assignment_station_simple_candidate_next_nodes(label, pricing_data)
        profile && (t_candidates += time_ns() - t0)

        for next_node in next_nodes
            t0 = profile ? time_ns() : UInt64(0)
            child = _extend_passenger_free_assignment_station_simple_label(label, next_node, pricing_data)
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

"""
    passenger_free_assignment_pricing_by_station_simple_label_setting(pricing_data, existing_columns; kwargs...)

Elementary-route counterpart of `passenger_free_assignment_pricing_by_label_setting`:
runs the station-simple label search, replays every finished candidate route to
recover concrete assignments (via the shared
`_passenger_free_assignment_column_from_route`), and returns up to
`max_new_columns` improving, pool-novel `PassengerFreeAssignmentRouteColumn`s.
Acceptance/dedup operates on the real assignment signature, exactly as in the
revisit-tolerant driver.
"""
function passenger_free_assignment_pricing_by_station_simple_label_setting(
    pricing_data::PassengerFreeAssignmentPricingData,
    existing_columns::Vector{PassengerFreeAssignmentRouteColumn};
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
        signature = _passenger_free_assignment_column_signature(column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    scored_by_signature = Dict{Any, NamedTuple}()

    function try_accept_route!(route::Vector{Int}, label_reduced_cost::Float64)::Bool
        assignments, tau, reduced_cost = _passenger_free_assignment_column_from_route(
            route, pricing_data; label_reduced_cost=label_reduced_cost,
        )
        isempty(assignments) && return false
        reduced_cost < -reduced_cost_tol || return false
        signature = _passenger_free_assignment_column_signature(assignments)
        tau < get(best_pool_tau, signature, Inf) - 1e-9 || return false
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                reduced_cost < current.reduced_cost - 1e-9 ||
                (abs(reduced_cost - current.reduced_cost) <= 1e-9 && tau < current.tau - 1e-9)
            scored_by_signature[signature] = (reduced_cost=reduced_cost, route=route, assignments=assignments, tau=tau)
        end
        return length(scored_by_signature) >= n_candidates
    end

    labels, exhausted, stats = _enumerate_passenger_free_assignment_station_simple_pricing_labels(
        pricing_data;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        profile=profile,
        stop_if=label -> try_accept_route!(label.route, label.reduced_cost),
    )

    for label in labels
        try_accept_route!(label.route, label.reduced_cost)
    end

    scored = collect(values(scored_by_signature))
    sort!(scored, by=entry -> (entry.reduced_cost, entry.tau, string(entry.route)))
    scored = scored[1:min(length(scored), n_candidates)]
    scored = scored[1:min(length(scored), max_new_columns)]

    columns = PassengerFreeAssignmentRouteColumn[]
    column_id = next_column_id
    for entry in scored
        push!(columns, PassengerFreeAssignmentRouteColumn(
            column_id,
            entry.route,
            entry.assignments,
            entry.tau;
            metadata=Dict{String, Any}(
                "scenario" => pricing_data.scenario,
                "route" => Tuple(entry.route),
                "reduced_cost" => entry.reduced_cost,
            ),
        ))
        column_id += 1
    end
    return columns, exhausted, stats
end
