# Passenger free-assignment pricing

New pricer under `src/opt/optimize/aggregate_od_route/pricing/passenger/`
(`types.jl`, `data.jl`, `labels.jl`, `search.jl`), adapted from the aggregate
OD route pricer (`../types.jl`, `../data.jl`, `../labels.jl`, `../search.jl`)
without touching that pricer's files, since they're consumed throughout the
Benders/CG stack (`benders/y.jl`, `yz.jl`, `yzh.jl`,
`pricing/column_generation.jl`, `pricing/station_simple.jl`,
`enumeration.jl`) and carry a different dual/reward structure (aggregate
per-pair dual credit vs. per-passenger reward). This is a sibling subproblem
solver, not a drop-in replacement -- no master-problem/CG wiring is included
here, only the label search and the `PassengerFreeAssignmentRouteColumn`s it
produces.

## Why reward layers encode a per-passenger maximum

A route can independently certify many `(j, k)` assignments for the same
passenger `p`, but under unbounded capacity with no onboard commitment, `p`
ultimately picks exactly one -- the best one. Naively summing every certified
reward would credit a route for options it never actually delivers. Splitting
`p`'s distinct positive rewards `0 = v_p0 < v_p1 < ... < v_pmp` into
incremental layers `δ_ph = v_ph - v_p,h-1` and mapping each candidate reward
`v_pq` to the *prefix* `{(p,1),...,(p,q)}` makes "activate a layer set" and
"hold the running maximum reward" the same operation: prefix sums telescope,
so the weight of an activated prefix is always exactly the reward level it
represents, and OR-ing in another prefix (a bitset union) can only ever add
the layers strictly between the old and new levels.

## Why a better future destination upgrades rather than double-counts

Certifying `(p, j, k_2)` with `ρ = 10` after already certifying `(p, j, k_1)`
with `ρ = 4` computes `new_layers = mask(10) & ~activated`. Since `mask(4) =
{layer 1}` and `mask(10) = {layer 1, layer 2}` are nested (both prefixes of
the same sorted sequence), `new_layers = {layer 2}`, worth `10 - 4 = 6`. The
running total becomes `4 + 6 = 10`, matching `v_pq` for the new best
assignment exactly -- never `4 + 10 = 14`. Symmetrically, reaching a *worse*
destination afterwards contributes a mask that's already a subset of what's
active, so `new_layers` is empty and nothing changes. This is also why several
different origins certifying the same passenger at the same destination can
just have their prefix masks OR'd together in one batch (`data.jl`,
`_certify_passenger_free_assignment_layers_at_node`): the union of nested
prefixes is just the largest one, so batching is equivalent to processing them
one at a time and cannot double-credit an overlapping layer.

## Why no onboard passenger state is necessary

Capacity is unbounded and passengers don't interact through it or through
service time, so certifying `p`'s assignment never uses up a resource another
passenger's assignment needs. The only thing a label needs to remember is
*how good the best option found so far is* (the activated-layer bitset) and
*which origins still have a live pickup clock* (`station_age`, reused verbatim
from the aggregate pricer) -- there is no pickup/dropoff decision to branch on,
no vehicle load to track, and no drop-off subset to enumerate. This is exactly
why `_prune_irrelevant_passenger_free_assignment_station_ages` can freely
delete a station age once it can't unlock anything new: it's a potential
certification opportunity, not an irrevocable commitment, so nothing is lost
by forgetting it early.

## Why final assignments must still be reconstructed

Labels only ever carry *how much* reward is certified (the bitset's total
weight), never *which* concrete `(j, k)` earned it -- two different physical
routes, or even two different origins on the same route, can reach the same
activated-layer signature while assigning completely different pickup/dropoff
stations to the same passenger, and a master problem needs the real stations
to build linking coefficients. `search.jl`'s route replay
(`_replay_passenger_free_assignment_route`) re-walks a *finished* route from
`t = 0` and picks each passenger's true argmax `(j, k)` only at that point
(spec section 13), verifying its recomputed reduced cost against the label's
via `@assert isapprox(...; atol=1e-6)` in
`_passenger_free_assignment_column_from_route`. Doing this only for finished
candidate routes -- not during label expansion -- is what keeps the label
search itself cheap (bitset ops only, no per-label assignment bookkeeping).

## Search-level signature vs. final column signature

The label search's own `best_by_signature` bookkeeping
(`_enumerate_passenger_free_assignment_pricing_labels`) dedups on the cheap
`activated_reward_layers` bitset, exactly mirroring the aggregate pricer's use
of `served_pairs` as an internal proxy signature. The top-level driver
(`passenger_free_assignment_pricing_by_label_setting`) re-keys every surviving
label's *replayed* result on the real `Set{(passenger, pickup, dropoff)}`
signature before deciding what's pool-novel and improving -- per spec section
12, the layer signature is deliberately never used as the final column
identity.
