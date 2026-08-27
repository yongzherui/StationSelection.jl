"""
Reward-layer preprocessing and physical-graph helpers for the passenger
free-assignment pricer.

# Reward layers, in one paragraph

Each passenger `p` can be certified by many different `(j, k)` pairs, each worth
a different reward `ρ_pjk`, but a route only gets credit for `p`'s *best*
certified assignment, not the sum. To let a bitset-based label carry that
"running maximum" cheaply, each passenger's distinct positive rewards
`0 = v_p0 < v_p1 < ... < v_pmp` are split into incremental layers
`δ_ph = v_ph - v_p,h-1`. Certifying an assignment worth `v_pq` activates the
*prefix* `{(p,1),...,(p,q)}`; the prefix sums telescope back to `v_pq`, and
OR-ing another assignment's prefix into an already-activated one can only add
the incremental layers between the old and new levels -- so summing activated
layer weights across every passenger is exactly `sum(B_p(L) for p)`, never a
double count. See the module docstring in `exact/labels.jl` for the label-level
contract this preprocessing feeds.
"""

export create_joint_routing_assignment_pricing_data
export coarsen_passenger_assignment_rewards

"""
    coarsen_passenger_assignment_rewards(candidates, levels; tol=1e-9)

Round each positive assignment reward *up* to one of at most `levels` retained
values for that passenger. The maximum value is always retained. Since the
transformed reward is never smaller than the exact reward, pricing with the
transformed candidates is a relaxation (`relaxed_rc <= exact_rc` route by
route). Returned routes must still be replayed against exact pricing data before
they are admitted to the master.

The retained values are selected at evenly-spaced targets over the passenger's
observed reward range. `levels == 0` returns a copy without coarsening.
"""
# ── reward-layer preprocessing: coarsening (optional relaxation) ────────────
function coarsen_passenger_assignment_rewards(
    candidates::AbstractVector{PassengerAssignmentCandidate},
    levels::Int;
    tol::Float64=1e-9,
)::Vector{PassengerAssignmentCandidate}
    levels >= 0 || throw(ArgumentError("reward coarsening levels must be nonnegative"))
    levels == 0 && return collect(PassengerAssignmentCandidate, candidates)

    values_by_passenger = Dict{Int, Vector{Float64}}()
    for candidate in candidates
        candidate.reward > tol || continue
        push!(get!(() -> Float64[], values_by_passenger, candidate.p), candidate.reward)
    end
    retained_by_passenger = Dict{Int, Vector{Float64}}()
    for (passenger, raw_values) in values_by_passenger
        values = Float64[]
        for value in sort(raw_values)
            (!isempty(values) && value - values[end] <= tol) || push!(values, value)
        end
        if length(values) <= levels
            retained_by_passenger[passenger] = values
            continue
        end
        retained = Float64[]
        lo, hi = first(values), last(values)
        for level in 1:levels
            target = lo + (hi - lo) * level / levels
            index = findfirst(value -> value >= target - tol, values)
            isnothing(index) && continue
            value = values[index]
            (isempty(retained) || value > retained[end] + tol) && push!(retained, value)
        end
        (isempty(retained) || retained[end] < hi - tol) && push!(retained, hi)
        retained_by_passenger[passenger] = retained
    end

    transformed = PassengerAssignmentCandidate[]
    sizehint!(transformed, length(candidates))
    for candidate in candidates
        if candidate.reward <= tol
            push!(transformed, candidate)
            continue
        end
        retained = retained_by_passenger[candidate.p]
        index = findfirst(value -> value >= candidate.reward - tol, retained)
        rounded = isnothing(index) ? last(retained) : retained[index]
        push!(transformed, PassengerAssignmentCandidate(
            candidate.p, candidate.origin, candidate.destination,
            candidate.ride_limit, rounded,
        ))
    end
    return transformed
end

# ── small physical-graph / bitset helpers ───────────────────────────────────
function _joint_routing_assignment_travel(
    pricing_data::JointRoutingAssignmentPricingData, u::Int, v::Int,
)::Float64
    cost = get(pricing_data.travel_cost, (u, v), Inf)
    isfinite(cost) || throw(ArgumentError("missing finite routing cost for station arc $((u, v))"))
    return cost
end

function _sum_layer_weights(pricing_data::JointRoutingAssignmentPricingData, mask::RewardLayerBitset)::Float64
    total = 0.0
    @inbounds for layer in mask
        total += pricing_data.layer_weight[layer]
    end
    return total
end

_has_inactive_layer(mask::RewardLayerBitset, activated::RewardLayerBitset)::Bool = !issubset(mask, activated)

"""
Turn raw `(p, j, k, R_pjk, ρ_pjk)` candidates into global reward layers.

Returns `(layer_weight, assignment_layer_mask, positive_candidates)`:
- `layer_weight[layer_id] = δ_ph`;
- `assignment_layer_mask[(p, j, k)]` is the prefix mask for that candidate;
- `positive_candidates` is `candidates` filtered to `reward > 0`, the only ones
  pricing needs to represent.

Reward values are grouped per passenger by tolerance (`1e-9`) rather than exact
`==`, since two candidates can carry the "same" reward computed along different
arithmetic paths.
"""
# ── reward-layer preprocessing: layers from raw candidates ──────────────────
function _build_passenger_reward_layers(
    candidates::AbstractVector{PassengerAssignmentCandidate};
    tol::Float64=1e-9,
)
    positive = filter(c -> c.reward > tol, candidates)

    rewards_by_passenger = Dict{Int, Vector{Float64}}()
    for c in positive
        push!(get!(() -> Float64[], rewards_by_passenger, c.p), c.reward)
    end

    layer_weight = Float64[]
    passenger_layer_ids = Dict{Int, Vector{Int}}()
    passenger_layer_values = Dict{Int, Vector{Float64}}()
    for (p, rewards) in rewards_by_passenger
        sorted_rewards = sort(rewards)
        values = Float64[]
        for v in sorted_rewards
            (!isempty(values) && v - values[end] <= tol) && continue
            push!(values, v)
        end
        ids = Int[]
        prev = 0.0
        for v in values
            push!(layer_weight, v - prev)
            push!(ids, length(layer_weight))
            prev = v
        end
        passenger_layer_ids[p] = ids
        passenger_layer_values[p] = values
    end

    assignment_layer_mask = Dict{Tuple{Int, Int, Int}, RewardLayerBitset}()
    for c in positive
        values = passenger_layer_values[c.p]
        ids = passenger_layer_ids[c.p]
        q = findfirst(v -> abs(v - c.reward) <= tol, values)
        q === nothing && throw(ArgumentError(
            "internal error: reward $(c.reward) for passenger $(c.p) has no matching layer",
        ))
        assignment_layer_mask[(c.p, c.origin, c.destination)] = RewardLayerBitset(ids[1:q])
    end

    return layer_weight, assignment_layer_mask, positive
end

"""
    create_joint_routing_assignment_pricing_data(scenario, nodes, travel_cost, candidates; kwargs...)

Build the preprocessed pricing data for one scenario's passenger free-assignment
pricing pass. `candidates` carries already-computed rewards (i.e. the caller has
already folded in the master problem's duals: `ρ_pjk = α_p - γ^O_pj - γ^D_pk - w_pjk`);
this constructor's only job is the reward-layer transformation plus grouping
opportunities by physical endpoint for fast certification.
"""
# ── pricing-data assembly (build-time, once per scenario) ───────────────────
function create_joint_routing_assignment_pricing_data(
    scenario::Int,
    nodes::Vector{Int},
    travel_cost::Dict{Tuple{Int, Int}, Float64},
    candidates::AbstractVector{PassengerAssignmentCandidate};
    route_regularization_weight::Float64,
    max_wait_time::Float64,
    repositioning_time::Float64=0.0,
    max_stops::Int=typemax(Int),
    compensated_dominance::Bool=true,
)::JointRoutingAssignmentPricingData
    layer_weight, assignment_layer_mask, positive_candidates = _build_passenger_reward_layers(candidates)
    n_layers = length(layer_weight)

    assignments_by_destination = Dict{Int, Vector{PassengerAssignmentOpportunity}}()
    assignments_by_origin = Dict{Int, Vector{PassengerAssignmentOpportunity}}()
    origin_layer_mask = Dict{Int, RewardLayerBitset}()
    destination_layer_mask = Dict{Int, RewardLayerBitset}()
    opportunities = PassengerAssignmentOpportunity[]

    for c in positive_candidates
        mask = assignment_layer_mask[(c.p, c.origin, c.destination)]
        opp = PassengerAssignmentOpportunity(c.p, c.origin, c.destination, c.ride_limit, c.reward, mask)
        push!(opportunities, opp)
        push!(get!(() -> PassengerAssignmentOpportunity[], assignments_by_destination, c.destination), opp)
        push!(get!(() -> PassengerAssignmentOpportunity[], assignments_by_origin, c.origin), opp)
        origin_layer_mask[c.origin] = union(get(origin_layer_mask, c.origin, RewardLayerBitset()), mask)
        destination_layer_mask[c.destination] = union(get(destination_layer_mask, c.destination, RewardLayerBitset()), mask)
    end

    bounded_max_stops = max_stops != typemax(Int)
    resolved_max_stops = _resolve_aggregate_od_route_pricing_max_stops(max_stops)

    return JointRoutingAssignmentPricingData(
        scenario,
        nodes,
        travel_cost,
        route_regularization_weight,
        repositioning_time,
        max_wait_time,
        resolved_max_stops,
        bounded_max_stops,
        compensated_dominance,
        n_layers,
        layer_weight,
        assignment_layer_mask,
        assignments_by_destination,
        assignments_by_origin,
        origin_layer_mask,
        destination_layer_mask,
        opportunities,
    )
end

"""
Certify passenger assignments at a newly-visited station `node`. Only
`assignments_by_destination[node]` is scanned (section 16's "group by
destination" requirement). Every opportunity whose origin is currently live and
within its ride limit contributes its full prefix mask to a single batched
union *before* diffing against `activated` -- batching (rather than folding in
one opportunity's newly-activated layers at a time) is what makes "several
different origins certify the same passenger at this destination" collapse to
"OR their prefix masks", per the design note in `exact/labels.jl`: since masks are
nested prefixes per passenger, unioning first and diffing once yields exactly
the incremental layers between the label's prior best and the best among
everything certified here, with no risk of double-crediting the same layer
under two different opportunities.
"""
# ── search-time helpers: certify / prune / age-usefulness ───────────────────
function _certify_joint_routing_assignment_layers_at_node(
    node::Int,
    station_age::Dict{Int, Float64},
    travel_time::Float64,
    activated::RewardLayerBitset,
    pricing_data::JointRoutingAssignmentPricingData,
)
    opportunities = get(pricing_data.assignments_by_destination, node, PassengerAssignmentOpportunity[])
    isempty(opportunities) && return activated, 0.0

    reachable = RewardLayerBitset()
    for opp in opportunities
        origin_age = get(station_age, opp.origin, Inf)
        origin_age + travel_time <= opp.ride_limit + 1e-9 || continue
        union!(reachable, opp.layer_mask)
    end
    new_layers = setdiff(reachable, activated)
    isempty(new_layers) && return activated, 0.0
    reward = _sum_layer_weights(pricing_data, new_layers)
    certified = union(activated, new_layers)
    return certified, reward
end

"""
Is a live clock at `station` (of age `age`, with the vehicle at `current`) still
reachable in time for *any* opportunity it could certify -- purely a ride-limit
expiry test, deliberately blind to whether those opportunities' layers are
already activated. `travel(current, dest)` is the *minimum* additional time
before reaching any destination -- detouring only adds more -- so this test is
exact, not heuristic.

A clock used to also die once every reachable opportunity's layer was already
active (nothing new left to certify), on the reasoning that discarding it cost
nothing since it represents a potential certification, not an irrevocable
passenger-pickup commitment. That entangles the clock -- a resource query,
"can this station's age still reach a ride-limit deadline" -- with
`activated_reward_layers`, which is *per-label* and differs between whatever
two labels a dominance check happens to compare: dropping the clock the moment
label `a`'s own activated set makes it look reward-dead removes it from `a`'s
`station_age` dict, so a later dominance comparison against some `b` that
still carries a live (but, from `b`'s perspective, likewise reward-exhausted)
clock at that station sees `a` as *lacking* a resource `b` has -- `a` reads as
strictly worse there even though the two are equivalent. The resource should
only expire on its own physical term (the ride limit), not on a label-local
reward-bookkeeping question; keeping it live doesn't cost anything the
dominance test doesn't already price in via `_joint_routing_assignment_compensation`.
"""
function _joint_routing_assignment_age_is_useful(
    station::Int,
    age::Float64,
    pricing_data::JointRoutingAssignmentPricingData,
    current::Int,
)::Bool
    for opp in get(pricing_data.assignments_by_origin, station, PassengerAssignmentOpportunity[])
        t_to_dest = opp.destination == current ? 0.0 :
            _joint_routing_assignment_travel(pricing_data, current, opp.destination)
        age + t_to_dest <= opp.ride_limit + 1e-9 && return true
    end
    return false
end
