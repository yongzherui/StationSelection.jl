"""
Core label-DP primitives for the elementary-route (station-simple) pricer:
label creation, extension, and dominance. `station_simple.jl` orchestrates
these into a full pricing pass; this file is the one to audit for "is the
label search correct" -- see `../exact/labels.jl` for the same role on the
revisit-tolerant pricer.

An alternative to the revisit-tolerant search in `../exact/labels.jl`/`../exact/exact.jl`: a
route may never revisit a station, so a certified `(j,k)` pair settles
permanently the first (and only) time `k` is visited after `j` -- there is no
need for a `station_age` Dict tracking every past visit, only
`live_origin_age` for stations already on the route whose destination hasn't
been reached yet.

Because `visited` only grows and is part of the dominance signature (exact match,
not subset), a dominating label's `served_pairs` need not be compared separately:
identical `(current, visited)` plus reduced-cost/time/live-origin-age domination
already implies every future extension available to the dominated label is at
least as good from the dominating one, since which nodes remain reachable depends
only on `visited` and rewards not yet banked are fully described by
`live_origin_age`. Do not add a served-pairs comparison here without re-deriving
that invariant; see `_dominates_route_covering_station_simple_label` below.

# Why this is faster

Fewer extensions, since candidate generation drops any already-visited node --
the branching factor shrinks as a route grows.

# Correctness caveat

Restricting to elementary routes restricts the column universe the master
problem prices over. Where the model's optimum genuinely wants a revisiting
route this pricer is a *heuristic* -- it can terminate CG with a weaker LP
bound or miss improving columns. It is therefore opt-in and off by default (not
currently reachable from any `pricing_round.jl`); validate the LP bound against
the revisit-tolerant pricer before relying on it.

# Reuse

Shares `RouteCoveringPricingData`/`RouteCoveringPricingDuals` and the
`_route_covering_travel`/`_direct_ride_limit` helpers from `../data.jl`, and
produces `AggregateODRouteColumn`s via the same `_aggregate_od_route_column_from_label`
convention as the revisit-tolerant pricer (a new method dispatched on this
file's label type, at the bottom of this file).
"""

function _make_route_covering_station_simple_bitsets(
    label::RouteCoveringStationSimpleLabel,
    node_index::Dict{Int, Int},
)::RouteCoveringStationSimpleBitsets
    visited_bits = BitSet()
    for node in label.visited
        push!(visited_bits, node_index[node])
    end

    age_idx, age_val, age_mask = _make_sparse_station_ages(label.live_origin_age, node_index)

    return RouteCoveringStationSimpleBitsets(visited_bits, age_idx, age_val, age_mask)
end

_route_covering_station_simple_state(
    label::RouteCoveringStationSimpleLabel,
    bs::RouteCoveringStationSimpleBitsets,
) = (label.current, bs.visited_bits)

function _dominates_route_covering_station_simple_label(
    a::RouteCoveringStationSimpleLabel,
    b::RouteCoveringStationSimpleLabel,
    abs::RouteCoveringStationSimpleBitsets,
    bbs::RouteCoveringStationSimpleBitsets,
)::Bool
    a.current == b.current || return false
    abs.visited_bits == bbs.visited_bits || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    a.time <= b.time + 1e-9 || return false
    # dom(b) ⊆ dom(a) and age_a(j) <= age_b(j) for j in dom(b) -- shared with the
    # revisit-tolerant bitset dominance and the PFA station-simple pricer via
    # `label_setting/utils.jl`.
    _sparse_station_ages_dominate(
        abs.age_idx, abs.age_val, abs.age_mask, bbs.age_idx, bbs.age_val, bbs.age_mask,
    ) || return false
    return true
end

RouteCoveringStationSimpleDominanceFilters(
    label::RouteCoveringStationSimpleLabel, ::RouteCoveringStationSimpleBitsets,
) = RouteCoveringStationSimpleDominanceFilters(
    label.reduced_cost, label.time, Int32(length(label.route)),
)

PricingLabelEntry(id::Int, label::RouteCoveringStationSimpleLabel, bs::RouteCoveringStationSimpleBitsets) =
    PricingLabelEntry(RouteCoveringStationSimpleDominanceFilters(label, bs), id, label, bs)

"""
State-scan fast path for `_dominates_route_covering_station_simple_label`:
identical dominance test, minus the `current`/`visited_bits` check, which the
state itself already guarantees for every pair this is called on (see the
4-argument method's docstring above, and `_dominates_joint_routing_assignment_at_state`
in `joint_routing_assignment/exact/labels.jl` for the same convention on the other
pricer).
"""
@inline function _pricing_dominates_at_state(
    af::RouteCoveringStationSimpleDominanceFilters, abs::RouteCoveringStationSimpleBitsets,
    bf::RouteCoveringStationSimpleDominanceFilters, bbs::RouteCoveringStationSimpleBitsets,
    ::RouteCoveringStationSimpleDominanceRules,
)::Bool
    af.time <= bf.time + 1e-9 || return false
    af.reduced_cost <= bf.reduced_cost + 1e-9 || return false
    _sparse_station_ages_dominate(
        abs.age_idx, abs.age_val, abs.age_mask, bbs.age_idx, bbs.age_val, bbs.age_mask,
    ) || return false
    return true
end

function _initial_route_covering_station_simple_labels(
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{RouteCoveringStationSimpleLabel}
    # Only stations that are some pair's origin with positive dual reward are
    # worth opening a pickup clock at from the start; every node still gets a
    # depth-1 label (a route could pass through anywhere), just not all of
    # them start with a live clock.
    positive_origins = Set{Int}(
        pair[1] for pair in pricing_data.active_pairs if get(duals.sigma, pair, 0.0) > 1e-9
    )
    labels = RouteCoveringStationSimpleLabel[]
    for node in pricing_data.nodes
        live = Dict{Int, Float64}()
        node in positive_origins && (live[node] = 0.0)
        push!(labels, RouteCoveringStationSimpleLabel(
            node,
            [node],
            Set{Int}([node]),  # visited starts with just this node
            0.0,
            live,
            Set{Tuple{Int, Int}}(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
        ))
    end
    return labels
end

# Drop any live pickup clock that can no longer reach an uncertified,
# positive-dual pair's destination in time -- once a station's clock is
# useless it's pure dead weight in the label state and dominance signature.
# Unlike the revisit-tolerant pricer, a destination already `visited` can
# never be certified later even by a different clock (a station is visited
# exactly once here), so `pair[2] in visited` alone is enough to rule a pair
# out, with no need to also check `served_pairs`.
function _route_covering_station_simple_prune_live_origins(
    live_origin_age::Dict{Int, Float64},
    current::Int,
    visited::Set{Int},
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Dict{Int, Float64}
    remaining = Dict{Int, Float64}()
    for (origin, age) in live_origin_age
        can_still_reward = false
        for pair in pricing_data.active_pairs
            pair[1] == origin || continue
            pair[2] in visited && continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            age + _route_covering_travel(pricing_data, current, pair[2]) <=
                _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            can_still_reward = true
            break
        end
        can_still_reward && (remaining[origin] = age)
    end
    return remaining
end

# Every not-yet-visited node worth extending to next: either a dropoff for a
# clock already live, or (if not that) a fresh origin the route could still
# open in time. `next_node in label.visited && continue` up front is the one
# thing that makes this elementary rather than revisit-tolerant -- everywhere
# else the logic mirrors the revisit-tolerant candidate search.
function _route_covering_station_simple_candidate_next_nodes(
    label::RouteCoveringStationSimpleLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{Int}
    candidates = Int[]
    for next_node in pricing_data.nodes
        next_node in label.visited && continue
        travel_time = _route_covering_travel(pricing_data, label.current, next_node)

        # Is `next_node` a live-clock's dropoff, reachable within its ride limit?
        is_useful = false
        for (origin, age) in label.live_origin_age
            pair = (origin, next_node)
            dual = get(duals.sigma, pair, 0.0)
            dual > 1e-9 || continue
            age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            is_useful = true
            break
        end

        # Not a dropoff -- is it worth visiting to open a fresh pickup clock
        # (reachable before the wait cutoff, and it's some uncertifiable
        # pair's origin)?
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

function _extend_route_covering_station_simple_label(
    label::RouteCoveringStationSimpleLabel,
    next_node::Int,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::RouteCoveringStationSimpleLabel
    next_node in label.visited && throw(ArgumentError("station-simple extension cannot revisit $next_node"))

    travel_time = _route_covering_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_route = vcat(label.route, next_node)
    visited = copy(label.visited)
    push!(visited, next_node)

    # Certify every live clock's pair whose destination is `next_node` and
    # whose ride limit isn't exceeded -- same certification rule as the
    # revisit-tolerant pricer, just scanning `live_origin_age` (this label's
    # explicit clock set) instead of `_certify_route_covering_pairs_at_node`'s
    # scan over every active pair.
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

    # Every existing clock ages by the travel time, same as the
    # revisit-tolerant pricer; then, if this visit opens a fresh clock (some
    # pair has `next_node` as its origin, positive dual) and we're still
    # within the wait cutoff, that clock starts live at age 0.
    live = Dict(origin => age + travel_time for (origin, age) in label.live_origin_age)
    opens_origin = any(
        pair -> pair[1] == next_node && get(duals.sigma, pair, 0.0) > 1e-9,
        pricing_data.active_pairs,
    )
    if opens_origin && arrival_time <= pricing_data.max_wait_time + 1e-9
        live[next_node] = 0.0
    end
    live = _route_covering_station_simple_prune_live_origins(live, next_node, visited, pricing_data, duals)

    new_tau = label.tau + travel_time
    return RouteCoveringStationSimpleLabel(
        next_node,
        new_route,
        visited,
        arrival_time,
        live,
        served_pairs,
        new_tau,
        # Same reduced-cost update rule as the revisit-tolerant pricer: pay
        # the regularized travel cost, credit back what was just certified.
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
    )
end

# Upper bound on collectible reward from this label onward. Already-open origins
# need the detour check against their current age; not-yet-visited origins only
# need to still be reachable within the pickup window (ignoring detour, since
# triangle inequality makes direct travel a lower bound on any routed arrival).
function _route_covering_station_simple_future_reward_bound(
    label::RouteCoveringStationSimpleLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Float64
    ub = 0.0
    for pair in pricing_data.active_pairs
        dual = get(duals.sigma, pair, 0.0)
        dual > 1e-9 || continue
        pair[2] in label.visited && continue
        if pair[1] in label.visited
            age = get(label.live_origin_age, pair[1], Inf)
            isinf(age) && continue
            age + _route_covering_travel(pricing_data, label.current, pair[2]) <=
                _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        else
            label.time + _route_covering_travel(pricing_data, label.current, pair[1]) <=
                pricing_data.max_wait_time + 1e-9 || continue
        end
        ub += dual
    end
    return ub
end

_route_covering_station_simple_label_priority(
    label::RouteCoveringStationSimpleLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
) = label.reduced_cost - _route_covering_station_simple_future_reward_bound(label, pricing_data, duals)

_aggregate_od_route_column_from_label(
    label::RouteCoveringStationSimpleLabel,
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
