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
Every rider requests at t = 0, so all share one pickup deadline, the formulation's
`pickup_deadline_s` (900 s): pickup clocks open only while time <= pickup_deadline_s, and an
M2/M3 start t0 <= pickup_deadline_s (owner, 2026-11-18: the pickup deadline is the only pickup limit).

Pickup clocks. As in `exact`, a visit to `j` opens a clock and a drop-off at `k` credits
`(p, j, k)` if the clock's age (ride time) fits; only the latest visit's clock is kept, and
it is the best one under every rule above (latest visit = most walkers arrived, shortest
ride). The first stop's clock is held separately (`root_age`) and never reset, because its
riders follow a different rule; a later visit of the first stop opens an ordinary clock
alongside it.

Not modelled: a vehicle that empties mid-route and could then wait before its next pickup.
"""

"Rider `p` via `(origin, destination)` with this pricer's three timing limits."
struct NoAnticipationOpportunity
    p::Int
    origin::Int
    destination::Int
    ride_cap::Float64       # detour_factor * drive(j, k): the in-vehicle limit on its own
    deadline::Float64       # door_to_door_ratio * drive(o, d) - walk(k, d), from the request (Inf without G2)
    walk_in::Float64        # walk(o, j)
    reward::Float64
    layer_mask::RewardLayerBitset
end

"""
One scenario's pricing data: the `exact` pricer's (`base`: travel times, reward layers,
weights, pickup_deadline_s, max_stops) plus the per-rider timing limits.
`by_origin[j]` lists opportunity indices sorted by `walk_in` ascending.
"""
struct NoAnticipationPricingData
    base::JointRoutingAssignmentPricingData
    first_walker::Bool          # M2 or M3: the vehicle waits (empty) at its first stop and leaves at t0 > 0
    gather::Bool                # M3: every first-stop rider with walk <= t0 boards (else only walk in (t0 - 5 s, t0])
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
can still serve someone (else Inf), `station_age` the clocks of later visits.
"""
struct NoAnticipationLabel
    current::Int
    route::Vector{Int}
    time::Float64
    start_time::Float64
    root_age::Float64
    station_age::Dict{Int, Float64}
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
Per-label dominance data. `stations`/`ages` are the live later-visit clocks sorted by
station; `gaps[i]` is the smallest walk of a rider at `stations[i]`, with a layer this label
has not activated, that the clock cannot take (walk > its visit time) but could still deliver
in time if it could take them; `future_walk` the smallest walk of such a rider, at any origin,
that this label could not take even reaching that origin directly (walk > time +
drive(current, origin)) but that some later visit could still deliver; `root_walks` the walks
of unpaid first-stop riders the first-stop clock could still deliver, whatever their walk.
"Could still deliver" uses this label's own time and clock ages: a label it is compared with
is no earlier and its clocks no younger, so it cannot deliver them either.
"""
struct NoAnticipationBitsets
    stations::Vector{Int}
    ages::Vector{Float64}
    gaps::Vector{Float64}
    future_walk::Float64
    root_walks::Vector{Float64}     # sorted walks of first-stop riders this label has not paid and could still deliver
end

PricingLabelEntry(id::Int, label::NoAnticipationLabel, bs::NoAnticipationBitsets) =
    PricingLabelEntry(NoAnticipationFilters(label.reduced_cost, label.time, Int32(label.route_length)), id, label, bs)

struct NoAnticipationSearchContext{D<:Function} <: AbstractPricingSearchContext{
    NoAnticipationFilters, NoAnticipationLabel, NoAnticipationBitsets, Int, RewardLayerBitset,
}
    pricing_data::NoAnticipationPricingData
    dominates::D
end
