"""
`:exact_no_anticipation` -- the `exact` pricer's route universe and reward model, with the
simulator's timing rules for riders who walk to their pickup stop (owner, 2026-11-18).

Why: the `exact` pricer lets a vehicle collect a rider at a stop as soon as it gets there,
whatever the rider's walk, and charges the walk only inside the door-to-door budget. In the
simulator a rider boards only after walking to the stop, a vehicle with riders on board
never waits, and the door-to-door guarantee (G2) runs from the request. On the model's own
G2 3.0 routes 48-67% of later boardings happen before the rider could have arrived
(experiments/2026-11-17_g2_time_blind, finding 7).

Clock. Every rider of a scenario requests at t = 0 (the scenario's shared start, as in the
`exact` model). A route's first stop is reached empty; `first_boarding` (on
`CGPricingConfig`) says when the vehicle leaves it:

  - `:start_now` (M1) -- at t0 = 0. The first stop's riders board at once; their walk is
    not waited for and their G2 is the `exact` rule, walk + ride + egress walk <= budget.
  - `:after_first_walker` (M2) -- the empty vehicle waits for the first rider to arrive and
    leaves the moment that rider boards: t0 = walk(o, j1) of that rider. Only riders who
    are already there board with them: walk in (t0 - 5 s, t0].
  - `:after_last_walker` (M3) -- the empty vehicle times its departure t0 so the riders it takes
    at the first stop have gathered: every first-stop rider needs walk(o, j1) <= t0 (the
    simulator lets an idle vehicle defer its departure until after a walker arrives). Riders
    who arrive earlier stand at the stop; G2 runs from the request, so that counts.

After the first stop the vehicle never waits. A rider boards a later visit of `j` at time v
only if they have walked there: walk(o, j) <= v. Every rider (first stop under M2, every
later stop) meets G2 counted from the request: drop-off time T + walk(k, d) <=
door_to_door_ratio * drive(o, d). The in-vehicle limit is `detour_factor * drive(j, k)`.
The only pickup limit is the pickup deadline: no rider boards later than the formulation's
`pickup_deadline_s` after their request (the simulator's rule). Clocks open only while
time <= the last rider's deadline, and an M2/M3 start t0 <= the deadline of every rider it takes.

Pickup clocks. As in `exact`, a visit to `j` opens a clock and a drop-off at `k` credits
`(p, j, k)` if the clock's age (ride time) fits; only the latest visit's clock is kept, and
it is the best one under every rule above (latest visit = most walkers arrived, shortest
ride). The first stop's clock is held separately (`root_age`) and never reset, because its
riders follow a different rule; a later visit of the first stop opens an ordinary clock
alongside it.

Not modelled: a vehicle that empties mid-route and could then wait before its next pickup.

Request times (2026-11-18, M4). With `demand_weighting = :request` every request is its own group with its own
request time r_p (seconds from the scenario start; `aggregate_od_route_request_time`), and every rule above shifts by
r_p: the rider reaches stop j at `ready = r_p + walk(o, j)` (the walk, plus arriving at r_p), G2 runs to
`r_p + door_to_door_ratio * drive(o, d) - walk(k, d)` and the pickup deadline closes at `latest = r_p + pickup_deadline_s`. The
walking COST is still the walk alone. Under `:count` / `:unit` every r_p = 0 and nothing changes. `:start_now` (M1)
is rejected with request times (it boards first-stop riders at t0 = 0, before they ask).

Because `latest` differs per rider, the latest visit of a stop no longer admits everyone an earlier visit did (a
rider whose pickup deadline closes between the two visits can use only the earlier one), so a label keeps every
useful visit clock of a stop, not only the latest (an older one is kept only for riders whose `latest` precedes the
next visit).
"""

"Rider `p` via `(origin, destination)` with this pricer's three timing limits."
struct NoAnticipationOpportunity
    p::Int
    origin::Int
    destination::Int
    ride_cap::Float64       # detour_factor * drive(j, k): the in-vehicle limit on its own
    deadline::Float64       # r_p + door_to_door_ratio * drive(o, d) - walk(k, d) (Inf without G2)
    walk_in::Float64        # walk(o, j)
    ready::Float64          # r_p + walk(o, j): when the rider is at j
    latest::Float64         # r_p + pickup_deadline_s: the pickup deadline, the rider boards no later
    reward::Float64
    layer_mask::RewardLayerBitset
end

"""
One scenario's pricing data: the `exact` pricer's (`base`: travel times, reward layers,
weights, pickup_deadline_s, max_stops) plus the per-rider timing limits.
`by_origin[j]` lists opportunity indices sorted by `ready` ascending.
"""
struct NoAnticipationPricingData
    base::JointRoutingAssignmentPricingData
    first_walker::Bool          # M2 or M3: the vehicle waits (empty) at its first stop and leaves at t0 > 0
    gather::Bool                # M3: every first-stop rider with walk <= t0 boards (else only walk in (t0 - 5 s, t0])
    horizon::Float64            # the last rider's pickup deadline (max latest)
    opps::Vector{NoAnticipationOpportunity}
    by_origin::Dict{Int, Vector{Int}}
    by_destination::Dict{Int, Vector{Int}}
    origins::Vector{Int}
end

"Riders arriving within this many seconds of the first rider board with them (M2)."
const NO_ANTICIPATION_FIRST_WALKER_TOL = 5.0
const NO_ANTICIPATION_EPS = 1e-6

"""
A partial route. `time` is the clock (seconds from the requests), `start_time` = t0 (when the
vehicle left its first stop, `route[1]`), `root_age` = time - t0 while the first stop's clock
can still serve someone (else Inf), `station_age[j]` the clocks (ages, youngest first) of the useful later visits of j.
"""
struct NoAnticipationLabel
    current::Int
    route::Vector{Int}
    time::Float64
    start_time::Float64
    root_age::Float64
    station_age::Dict{Int, Vector{Float64}}
    activated_reward_layers::RewardLayerBitset
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

struct NoAnticipationFilters
    reduced_cost::Float64
    time::Float64
    route_length::Int32
end

"""
Per-label dominance data. `stations`/`ages` are the live later-visit clocks sorted by station; `riders[n]` lists, for
station `stations[n]`, every opportunity there with a layer this label has not paid and whose G2 it can still meet
(time + direct drive to its drop-off): `(ready, latest, ride slack = ride_cap - that drive)`. `future_*` summarise the
riders this label cannot take even reaching their stop directly (ready > time + drive) but some later visit could
still deliver: max of `latest - drive`, min `ready`, min drive. `root_riders` the (ready, latest) of unpaid first-stop
riders the first-stop clock could still deliver, whatever their admission.
"Could still deliver" uses this label's own time: a label it is compared with is no earlier, so it cannot deliver
the others either.
"""
struct NoAnticipationBitsets
    stations::Vector{Int}
    ages::Vector{Vector{Float64}}
    riders::Vector{Vector{NTuple{3, Float64}}}
    future_latest::Float64
    future_min_ready::Float64
    future_min_d::Float64
    root_riders::Vector{NTuple{2, Float64}}
end

PricingLabelEntry(id::Int, label::NoAnticipationLabel, bs::NoAnticipationBitsets) =
    PricingLabelEntry(NoAnticipationFilters(label.reduced_cost, label.time, Int32(label.route_length)), id, label, bs)

struct NoAnticipationSearchContext{D<:Function} <: AbstractPricingSearchContext{
    NoAnticipationFilters, NoAnticipationLabel, NoAnticipationBitsets, Tuple{Int, UInt64}, RewardLayerBitset,
}
    pricing_data::NoAnticipationPricingData
    dominates::D
end
