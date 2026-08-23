"""
Core label-DP primitives for AggregateODRouteProblem pricing: label creation,
extension, and dominance. `exact.jl` orchestrates these into a full pricing
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

export initial_route_covering_pricing_labels
export extend_route_covering_pricing_label

function initial_route_covering_pricing_labels(
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{RouteCoveringPricingLabel}
    # A route can only ever serve an active pair by visiting one of that
    # pair's two endpoints, so seeding a label at every other node would just
    # waste search on routes that can never certify anything.
    endpoints = Set{Int}()
    for (j, k) in pricing_data.active_pairs
        push!(endpoints, j)
        push!(endpoints, k)
    end

    labels = RouteCoveringPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        # One depth-1 label per relevant node: route so far is just `[node]`,
        # `time = 0` (route clock starts here), this node's own pickup clock
        # starts live at age 0, nothing served yet, and `tau`/`reduced_cost`
        # already carry the fixed `repositioning_time` cost every route pays
        # regardless of length.
        push!(labels, RouteCoveringPricingLabel(
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

# True iff `label` still has a live pickup clock (a station visited, whose
# clock hasn't been pruned) that could still reach some uncertified pair's
# destination in time. Used only after the pickup cutoff has passed (see
# below) to decide whether a route with no more pickups ahead is nonetheless
# still worth extending toward a dropoff it already set up.
function _has_useful_live_route_covering_origin(
    label::RouteCoveringPricingLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Bool
    for (station, age) in label.station_age
        for pair in pricing_data.active_pairs
            pair[1] == station || continue                              # station must be this pair's origin
            pair ∈ label.served_pairs && continue                       # already certified, nothing to gain
            get(duals.sigma, pair, 0.0) > 1e-9 || continue               # not worth pursuing under current duals
            t_to_dest = pair[2] == label.current ? 0.0 : _route_covering_travel(pricing_data, label.current, pair[2])
            age + t_to_dest <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue  # dropoff still reachable in time
            return true
        end
    end
    return false
end

# Every node the search may legally extend `label` to next: either a fresh
# pickup (a pair's origin, reachable before its wait cutoff) or a dropoff for
# a pickup clock already live (a pair's destination, reachable within its
# ride limit). Both are gated on `get(duals.sigma, pair, 0.0) > 1e-9` -- a
# pair with no positive dual reward can't improve reduced cost, so there's no
# point ever visiting either of its endpoints for its sake.
function _route_covering_candidate_next_nodes(
    label::RouteCoveringPricingLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    # Once no fresh pickup is possible (past cutoff), the route is only worth
    # continuing if some already-open pickup clock can still reach a
    # dropoff -- otherwise every future extension is dead weight.
    if past_pickup_cutoff && !_has_useful_live_route_covering_origin(label, pricing_data, duals)
        return Int[]
    end
    for pair in pricing_data.active_pairs
        get(duals.sigma, pair, 0.0) > 1e-9 || continue
        origin, destination = pair
        remembered = pair ∈ label.served_pairs

        # Fresh-pickup candidate: not yet certified, not already standing at
        # the origin, and reachable before this pair's wait cutoff.
        if !remembered && origin != label.current
            arrival_time = label.time + _route_covering_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end

        # Dropoff candidate: skip if already certified or already standing at
        # the destination; otherwise offer it if the origin's pickup clock
        # (if any -- `Inf` if never visited) can still reach it in time.
        remembered && continue
        destination == label.current && continue
        origin_age = get(label.station_age, origin, Inf)
        origin_age + _route_covering_travel(pricing_data, label.current, destination) <=
            _direct_ride_limit(pricing_data, pair) + 1e-9 && push!(candidate_nodes, destination)
    end

    return sort!(collect(candidate_nodes))
end

# Public wrapper: this pricer only ever produces one child per next-node (no
# branching within a single extension), but `_run_label_setting`'s engine
# calls this as a `label -> [children...]`-shaped hook so pricers that *can*
# branch (e.g. multiple ways to serve the same node) share the same call
# site. See `_extend_route_covering_pricing_label` for the actual mechanics.
function extend_route_covering_pricing_label(
    label::RouteCoveringPricingLabel,
    next_node::Int,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{RouteCoveringPricingLabel}
    return RouteCoveringPricingLabel[
        _extend_route_covering_pricing_label(label, next_node, pricing_data, duals),
    ]
end

function _extend_route_covering_pricing_label(
    label::RouteCoveringPricingLabel,
    next_node::Int,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::RouteCoveringPricingLabel
    travel_time = _route_covering_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)
    # Every existing live pickup clock ages by the travel time to get here --
    # this happens unconditionally, before checking what this visit certifies.
    aged_station = Dict(station => age + travel_time for (station, age) in label.station_age)

    # Certify every pair whose destination is `next_node` and whose origin's
    # (aged) pickup clock still beats its ride limit; `reward` is the summed
    # dual value of everything just certified (see data.jl).
    certified_pairs, reward =
        _certify_route_covering_pairs_at_node(
            next_node,
            label.station_age,
            travel_time,
            label.served_pairs,
            pricing_data,
            duals,
        )
    # This visit also opens a fresh pickup clock at `next_node` itself, but
    # only if we're still within its own wait cutoff -- arriving late means
    # `next_node` can pick up nothing, so no clock is worth starting.
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        aged_station[next_node] = 0.0
    end
    # Drop any clock (including the one possibly just opened above) that can
    # no longer reach any uncertified pair's destination in time -- keeps the
    # label's state lean for the dominance/bitset machinery downstream.
    aged_station = _prune_irrelevant_route_covering_station_ages(
        aged_station,
        certified_pairs,
        pricing_data,
        duals,
        next_node,
    )

    return RouteCoveringPricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified_pairs,
        new_tau,
        # Reduced cost accrues the travel's regularized cost and is credited
        # back the reward of whatever just got certified -- this is the
        # quantity the search's priority/pruning is driven by, not `tau`.
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )

end

_route_covering_state(label::RouteCoveringPricingLabel) = label.current

function _route_covering_label_order_key(
    label::RouteCoveringPricingLabel,
    label_id::RouteCoveringLabelId,
)::RouteCoveringLabelOrderKey
    return (
        label.reduced_cost,
        label.time,
        label.route_length,
        label_id,
    )
end


function _make_route_covering_label_bitsets(
    label::RouteCoveringPricingLabel,
    pair_index::Dict{Tuple{Int, Int}, Int},
    n_pairs::Int,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::RouteCoveringLabelBitsets
    served_bits = BitSet()
    for pair in label.served_pairs
        push!(served_bits, pair_index[pair])
    end

    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)

    return RouteCoveringLabelBitsets(served_bits, age_idx, age_val, age_mask)
end

RouteCoveringDominanceFilters(label::RouteCoveringPricingLabel, bs::RouteCoveringLabelBitsets) =
    RouteCoveringDominanceFilters(label.reduced_cost, label.time, bs.age_mask,
        Int32(label.route_length), Int32(length(bs.age_idx)))

PricingLabelEntry(id::Int, label::RouteCoveringPricingLabel, bs::RouteCoveringLabelBitsets) =
    PricingLabelEntry(RouteCoveringDominanceFilters(label, bs), id, label, bs)

function _dominates_route_covering_label(
    a::RouteCoveringPricingLabel,
    b::RouteCoveringPricingLabel,
    bounded_max_stops::Bool;
    pair_weight::Dict{Tuple{Int, Int}, Float64}=Dict{Tuple{Int, Int}, Float64}(),
    compensated_dominance::Bool=true,
)::Bool
    _route_covering_state(a) == _route_covering_state(b) || return false          # must share current node
    (!bounded_max_stops || a.route_length <= b.route_length) || return false      # a can't have used more stops
    a.time <= b.time + 1e-9 || return false                                       # a can't be running later
    # `a` is allowed a strictly better (lower) reduced cost than `b` -- that
    # surplus is exactly the "budget" `a` can spend below to still dominate
    # despite otherwise-worse served-pairs coverage (see the loop below).
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    # See `_pricing_dominates_at_state` below for the compensated-vs-plain-subset
    # soundness argument; this is its `Set`-based counterpart for tests/callers
    # working straight off labels rather than bitsets.
    compensation = 0.0
    for pair in a.served_pairs
        pair in b.served_pairs && continue                    # `b` already has this pair too, free
        compensated_dominance || return false                 # plain mode: `a` must be a subset of `b`, full stop
        compensation += get(pair_weight, pair, 0.0)            # compensated mode: charge `a`'s reduced-cost budget
        compensation > budget && return false                 # for holding a pair `b` lacks
    end
    # Every station either label has a live pickup clock for: `a`'s clock
    # can't be older than `b`'s (an older clock is strictly worse -- less
    # time left before the ride limit), on pain of not dominating.
    all_stations = union(keys(a.station_age), keys(b.station_age), (a.current, b.current))
    for station in all_stations
        get(a.station_age, station, Inf) <= get(b.station_age, station, Inf) + 1e-9 || return false
    end
    return true
end

"""
State-scan-equivalent bitset dominance, expressed by delegating to
`_pricing_dominates_at_state` rather than reimplementing it a third time --
matching the convention `joint_routing_assignment/exact/labels.jl`'s twin already
follows. The state check stays here (label-level, cheap, and outside what
`_pricing_dominates_at_state` tests, since a state's label-list scan already
guarantees it for every pair it's called on)."""
function _dominates_route_covering_label(
    a::RouteCoveringPricingLabel,
    b::RouteCoveringPricingLabel,
    abs::RouteCoveringLabelBitsets,
    bbs::RouteCoveringLabelBitsets,
    bounded_max_stops::Bool;
    weight::Vector{Float64}=Float64[],
    compensated_dominance::Bool=true,
)::Bool
    _route_covering_state(a) == _route_covering_state(b) || return false
    return _pricing_dominates_at_state(
        RouteCoveringDominanceFilters(a, abs), abs,
        RouteCoveringDominanceFilters(b, bbs), bbs,
        weight,
        RouteCoveringDominanceRules{bounded_max_stops, compensated_dominance}(),
    )
end

"""
    _pricing_dominates_at_state(af, abs, bf, bbs, weight, rules)

The dominance predicate as the state's label-list scan calls it (see
`joint_routing_assignment/exact/labels.jl`'s twin for the full condition-ordering
rationale, which applies unchanged here). `weight` is the per-pair reward
(`RouteCoveringSearchContext.positive_pair_rewards`, indexed exactly like
`served_bits`) the reward-diff test in `_bitset_diff_weight`
(`label_setting/utils.jl`) charges `a` for holding pairs `b` lacks.
"""
@inline function _pricing_dominates_at_state(
    af::RouteCoveringDominanceFilters, abs::RouteCoveringLabelBitsets,
    bf::RouteCoveringDominanceFilters, bbs::RouteCoveringLabelBitsets,
    weight::Vector{Float64},
    ::RouteCoveringDominanceRules{BoundedStops, Compensated},
)::Bool where {BoundedStops, Compensated}
    af.time <= bf.time + 1e-9 || return false                          # a can't be running later
    af.n_live_ages >= bf.n_live_ages || return false                   # cheap prefilter before the real support check below
    bf.age_mask & ~af.age_mask == 0 || return false                    # b's live stations must be a subset of a's (folded-bit prefilter)
    BoundedStops && af.route_length > bf.route_length && return false  # a can't have used more stops (only checked if the cap is finite)
    budget = bf.reduced_cost - af.reduced_cost + 1e-9                  # a's reduced-cost surplus over b, spendable below
    budget >= 0.0 || return false
    # Sorted-merge walk: for every station b has a live clock at (age_idx is
    # ascending in both), the matching entry in a must exist and be no older.
    # `ia` only ever advances, never resets, since both arrays are sorted --
    # this is what makes the whole check O(n_live_ages) instead of O(n^2).
    ia = 1
    na = Int(af.n_live_ages)
    @inbounds for ib in Base.OneTo(Int(bf.n_live_ages))
        idx = bbs.age_idx[ib]
        while ia <= na && abs.age_idx[ia] < idx
            ia += 1
        end
        ia <= na && abs.age_idx[ia] == idx || return false  # b has a live station a doesn't -- a can't dominate
        abs.age_val[ia] <= bbs.age_val[ib] + 1e-9 || return false  # a's clock at this station must be no older
    end
    # Final, most expensive check last: does a's served-pairs surplus over b
    # (weighted by dual reward) fit inside the reduced-cost budget computed
    # above? `Val(Compensated)` picks the compensated-vs-plain-subset rule at
    # compile time (see `_bitset_diff_weight`, `label_setting/utils.jl`).
    _bitset_diff_weight(abs.served_bits, bbs.served_bits, weight, budget, Val(Compensated)) <= budget || return false
    return true
end

function _route_covering_label_priority(
    label::RouteCoveringPricingLabel,
    duals::RouteCoveringPricingDuals,
)::Float64
    return label.reduced_cost
end

function _aggregate_od_route_column_signature(pairs)::Tuple{Vararg{Tuple{Int, Int}}}
    return Tuple(sort!(collect(pairs)))
end

_aggregate_od_route_column_signature(column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature(column.od_pairs)

function _aggregate_od_route_column_from_label(
    label::RouteCoveringPricingLabel,
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
