"""
`:exact_no_anticipation` -- the `exact` pricer's route universe and reward model, with the
simulator's timing rules for riders who walk to their pickup stop (owner, 2026-11-18).

Why: the `exact` pricer lets a vehicle collect a rider at a stop as soon as it gets there,
whatever the rider's walk, and charges the walk only inside the door-to-door budget. In the
simulator a rider boards only after walking to the stop, a vehicle with riders on board
never waits, and the door-to-door limit runs from the request. On the model's own
door-to-door limit 3.0 routes 48-67% of later boardings happen before the rider could have arrived
(experiments/2026-11-17_g2_time_blind, finding 7).

Clock. Every rider of a scenario requests at t = 0 (the scenario's shared start, as in the
`exact` model). A route's first stop is reached empty; `first_boarding` (on
`CGPricingConfig`) says when the vehicle leaves it:

  - `:start_now` (M1) -- at t0 = 0. The first stop's riders board at once; their walk is
    not waited for and their door-to-door limit is the `exact` rule, walk + ride + egress walk <= budget.
  - `:after_first_walker` (M2) -- the empty vehicle waits for the first rider to arrive and
    leaves the moment that rider boards: t0 = walk(o, j1) of that rider. Only riders who
    are already there board with them: walk in (t0 - 5 s, t0].
  - `:after_last_walker` (M3) -- the empty vehicle times its departure t0 so the riders it takes
    at the first stop have gathered: every first-stop rider needs walk(o, j1) <= t0 (the
    simulator lets an idle vehicle defer its departure until after a walker arrives). Riders
    who arrive earlier stand at the stop; the door-to-door limit runs from the request, so that counts.
  - `:start_now_walk_limit` (M4, owner 2026-11-19) -- M1 with a cap: the vehicle leaves at
    t0 = 0 and takes only first-stop riders with walk(o, j1) <= `first_stop_walk_limit`
    (60/120/300 s), so it runs ahead of a first-stop rider by at most that much. Their door-to-door
    limit is M1's. Master seeds are two-stop singletons only (`seeding.jl`, `singletons_only`):
    a lone rider's `[j, k]` stays valid at any walk (the vehicle could wait for them), but a
    seed bundle could pool first-stop riders the pricer forbids.

Under M2/M3 every route starts on a TRIGGER: a rider pair (p, j1, k) whose walk sets t0 (the
first walker under M2, the last under M3). The trigger must be carried -- dropped off at k with
the first stop's clock -- and its reward counts whatever its sign (owner, 2026-11-19: whoever
sets the start time is on board). A label starts from (first stop, t0) only; WHICH rider is the
trigger is decided when the route is finished (owner's suggestion, 2026-11-19): the label keeps,
per rider, the best trigger reward dropped off so far, and a finished route pays
min over riders p of (p's credited reward - p's trigger reward), i.e. nothing when a rider with
walk t0 is dropped off on their best pair. Every valid pair with demand is a possible trigger, not only
the positive-reward ones: the best route can need a rider who is not worth carrying on their own
to time the departure, and leaving those start times out made CG certify too early (set 183:
a column at rc -152 was never generated). Other first-stop riders stay optional.

After the first stop the vehicle never waits. A rider boards a later visit of `j` at time v
only if they have walked there: walk(o, j) <= v. Every rider (first stop under M2, every
later stop) meets the door-to-door limit counted from the request: drop-off time T + walk(k, d) <=
door_to_door_limit_ratio * drive(o, d). The in-vehicle limit is `ride_time_limit_ratio * drive(j, k)`.
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
    ride_cap::Float64       # ride_time_limit_ratio * drive(j, k): the in-vehicle limit on its own
    deadline::Float64       # door_to_door_limit_ratio * drive(o, d) - walk(k, d), from the request (Inf without the door-to-door limit)
    walk_in::Float64        # walk(o, j)
    reward::Float64
    layer_mask::RewardLayerBitset
end

"""
A possible M2/M3 trigger: rider `p` boarding at the first stop `origin` and dropped off at `destination`. Their walk sets
the departure (t0 = walk_in); they must be carried, and `reward` (the master's rho for the pair, ANY sign) counts in the
route's reduced cost (owner, 2026-11-19: the rider who sets the start time is on board).
"""
struct NoAnticipationTrigger
    p::Int
    origin::Int
    destination::Int
    ride_cap::Float64
    deadline::Float64
    walk_in::Float64
    reward::Float64
end

"""
One scenario's pricing data: the `exact` pricer's (`base`: travel times, reward layers,
weights, pickup_deadline_s, max_stops) plus the per-rider timing limits.
`by_origin[j]` lists opportunity indices sorted by `walk_in` ascending.
`triggers` (M2/M3 only) lists every rider pair that may set a route's departure, whatever its reward;
`p_layers[p]` is the union of rider `p`'s reward layers (pre-activated on a label whose trigger is `p`, so
`p` is not credited a second time).
"""
struct NoAnticipationPricingData
    base::JointRoutingAssignmentPricingData
    first_walker::Bool          # M2 or M3: the vehicle waits (empty) at its first stop and leaves at t0 > 0
    gather::Bool                # M3: every first-stop rider with walk <= t0 boards (else only walk in (t0 - 5 s, t0])
    first_walk_limit::Float64   # M4: first-stop riders need walk <= this (Inf for M1/M2/M3)
    opps::Vector{NoAnticipationOpportunity}
    by_origin::Dict{Int, Vector{Int}}
    by_destination::Dict{Int, Vector{Int}}
    origins::Vector{Int}
    triggers::Vector{NoAnticipationTrigger}
    p_layers::Dict{Int, RewardLayerBitset}
    starts::Vector{Tuple{Int, Float64}}         # M2/M3: distinct (first stop, t0 = some trigger's walk), t0 <= pickup deadline
    start_triggers::Vector{Vector{Int}}         # triggers (indices into `triggers`) whose walk is that start's t0
    travel::Matrix{Float64}                     # dense drive times between the pricing nodes (Inf off the node set)
    scratch_ev::Vector{Tuple{Float64, Int, Float64}}   # bound workspace (one search per pricing data, never shared)
    scratch_w::Vector{Float64}                  # bound workspace, indexed by rider p
    scratch_touched::Vector{Int}
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
    start::Int                  # M2/M3: index into pd.starts (0 under M1)
    trig_r::Dict{Int, Float64}  # M2/M3: rider p => best reward among p's triggers of this start dropped off so far
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
    NoAnticipationFilters, NoAnticipationLabel, NoAnticipationBitsets, Tuple{Int, UInt64}, Tuple{RewardLayerBitset, Int, Float64},
}
    pricing_data::NoAnticipationPricingData
    dominates::D
end
