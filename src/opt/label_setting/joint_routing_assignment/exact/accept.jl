"""
Route replay: how a finished label's physical route is turned into concrete
per-passenger assignments -- the only place concrete `(j, k)` pairs are
recovered, since the label itself only tracks a cheap reward-layer proxy
during search (`types.jl`, section 16). Pure logic, no `ctx` and no hook
methods -- `hooks.jl`'s `_pricing_candidate_from_label` is what calls
`_joint_routing_assignment_column_from_route` once per surviving label; this
file has no role in *deciding* dominance or termination, only in turning an
already-found label into a concrete, replayed answer.
"""

# ── route replay ─────────────────────────────────────────────────────────────
"""
    _replay_joint_routing_assignment_route(route, pricing_data) -> Dict{Int, Tuple{Int,Int,Float64,Int,Int}}

Replay a finished physical route from `t = 0`, independently of any label's
(possibly dominance-pruned) station-age history, and return each passenger's
best certified assignment as
`passenger => (origin, destination, reward, pickup_position, dropoff_position)`.

The two positions are 1-based indices into `route`, and they are recorded here because
here is the only place they are known. A route may visit a station more than once (labels
are revisit-tolerant), and then the station alone does not identify the visit: the
certifying dropoff is the *earliest* index at which the ride-limit check passes, while its
pickup is the *most recent* prior visit to the origin -- the freshest clock, since
`pickup_time` is overwritten on every re-visit inside the wait window. Recovering them
after the fact from the station pair alone (e.g. `findfirst`/`findlast` over `route`) gets
both ends wrong in general, and can only agree by coincidence on an elementary route.
This is the only place concrete `(j, k)` pairs are recovered -- label expansion
never materializes them (section 16), since only a small number of finished
candidate routes ever need this, not every intermediate label.

Ties on reward are broken lexicographically by `(origin, destination)` for
determinism.

Clocks are held as **absolute pickup times**, not ages. Ageing every live clock by
the same `travel_time` at each stop used to be written as a comprehension, which
built a brand-new `Dict` per stop of every route replayed -- and replay runs once
per improving route, which in a harvesting configuration is once per accepted
column. Storing `pickup_time[j]` and deriving `age = elapsed_time - pickup_time[j]`
at the point of use is the same arithmetic with no rebuild: a station never seen
reads back as `-Inf`, so its age is `Inf`, exactly as the missing-key default was.
"""
function _replay_joint_routing_assignment_route(
    route::Vector{Int},
    pricing_data::JointRoutingAssignmentPricingData,
)::Dict{Int, Tuple{Int, Int, Float64, Int, Int}}
    best = Dict{Int, Tuple{Int, Int, Float64, Int, Int}}()
    isempty(route) && return best

    pickup_time = Dict{Int, Float64}()
    # Which visit set the clock now held in `pickup_time`, so a certified assignment can
    # name the actual boarding stop rather than the station's first appearance.
    pickup_position = Dict{Int, Int}()
    current = route[1]
    elapsed_time = 0.0
    pickup_time[current] = 0.0  # t = 0 is always within the (non-negative) pickup window
    pickup_position[current] = 1

    for idx in 2:length(route)
        next_node = route[idx]
        travel_time = _joint_routing_assignment_travel(pricing_data, current, next_node)
        elapsed_time += travel_time

        for opp in get(pricing_data.assignments_by_destination, next_node, PassengerAssignmentOpportunity[])
            origin_age = elapsed_time - get(pickup_time, opp.origin, -Inf)
            origin_age <= opp.ride_limit + 1e-9 || continue
            current_best = get(best, opp.p, nothing)
            if isnothing(current_best) || opp.reward > current_best[3] + 1e-9 ||
                    (abs(opp.reward - current_best[3]) <= 1e-9 && (opp.origin, opp.destination) < (current_best[1], current_best[2]))
                best[opp.p] = (opp.origin, opp.destination, opp.reward,
                               get(pickup_position, opp.origin, 0), idx)
            end
        end

        if elapsed_time <= pricing_data.max_wait_time + 1e-9
            pickup_time[next_node] = elapsed_time  # a fresh clock, i.e. age 0 from here
            pickup_position[next_node] = idx
        end
        current = next_node
    end

    return best
end

"""
    _joint_routing_assignment_column_from_route(route, pricing_data)

Route replay plus per-passenger argmax selection (spec section 13): returns
`(assignments, tau, reduced_cost, positions)` where `assignments` is
`[(p, j_p*, k_p*), ...]` for every passenger with a positive certified reward,
`positions` maps each such `p` to the `(pickup_position, dropoff_position)` indices into
`route` that replay actually certified (see above -- they are not recoverable from the
station pair once a route revisits a station). `positions` is appended last so existing
three-way destructurings of this function keep working unchanged,
`tau` is the route's physical travel time, and `reduced_cost` is recomputed
directly from the selected assignments' rewards (not copied from any label).
When `label_reduced_cost` is supplied, asserts the two agree within tolerance
-- the correctness invariant from spec section 7/13, checked on every finished
route rather than only in tests, since replay is cheap relative to the search
that produced the route in the first place.
"""
function _joint_routing_assignment_column_from_route(
    route::Vector{Int},
    pricing_data::JointRoutingAssignmentPricingData;
    label_reduced_cost::Union{Float64, Nothing}=nothing,
)
    best = _replay_joint_routing_assignment_route(route, pricing_data)
    assignments = Tuple{Int, Int, Int}[(p, v[1], v[2]) for (p, v) in best]
    positions = Dict{Int, Tuple{Int, Int}}(p => (v[4], v[5]) for (p, v) in best)
    reward_sum = sum((v[3] for (_p, v) in best); init=0.0)
    tau = length(route) < 2 ? 0.0 :
        sum(_joint_routing_assignment_travel(pricing_data, route[i], route[i + 1]) for i in 1:(length(route) - 1))
    reduced_cost = pricing_data.route_regularization_weight * (tau + pricing_data.repositioning_time) - reward_sum

    if !isnothing(label_reduced_cost)
        @assert isapprox(reduced_cost, label_reduced_cost; atol=1e-6) (
            "reconstructed reduced cost $(reduced_cost) does not match the searched label's " *
            "$(label_reduced_cost) for route $(route) -- reward-layer accounting is inconsistent " *
            "with a direct passenger-by-passenger recomputation"
        )
    end

    return assignments, tau, reduced_cost, positions
end

# ── column signature ──────────────────────────────────────────────────────────
function _joint_routing_assignment_column_signature(assignments)::Tuple{Vararg{Tuple{Int, Int, Int}}}
    return Tuple(sort!(collect(assignments)))
end

_joint_routing_assignment_column_signature(column::JointRoutingAssignmentRouteColumn) =
    _joint_routing_assignment_column_signature(column.assignments)
