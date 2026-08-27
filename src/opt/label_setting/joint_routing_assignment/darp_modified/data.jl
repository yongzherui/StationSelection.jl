"""
Builds the per-scenario pricing graph for `darp_modified/`'s branching commit-or-skip
passenger free-assignment pricer, and the eligibility/subset helpers
`extend.jl`'s extension step calls. Counterpart to `../data.jl` (`exact/`+
`station_simple/`'s reward-layer preprocessing) -- this pricer needs none of
that, since credit is assigned directly, at the moment a branch chooses to
commit (see `types.jl`'s module docstring).
"""

export create_joint_routing_assignment_darp_modified_pricing_data

function _joint_routing_assignment_darp_modified_travel(
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData, u::Int, v::Int,
)::Float64
    cost = get(pricing_data.travel_cost, (u, v), Inf)
    isfinite(cost) || throw(ArgumentError("missing finite routing cost for station arc $((u, v))"))
    return cost
end

"""
    create_joint_routing_assignment_darp_modified_pricing_data(scenario, nodes, travel_cost, candidates; kwargs...)

Build the preprocessed pricing data for one scenario's branching passenger
free-assignment pricing pass. `candidates` carries already-computed rewards
(as `create_joint_routing_assignment_pricing_data` does for `exact/`); this
constructor's only job is filtering to positive-reward candidates and
indexing them by origin/destination/passenger for fast certification.
"""
function create_joint_routing_assignment_darp_modified_pricing_data(
    scenario::Int,
    nodes::Vector{Int},
    travel_cost::Dict{Tuple{Int, Int}, Float64},
    candidates::AbstractVector{PassengerAssignmentCandidate};
    route_regularization_weight::Float64,
    max_wait_time::Float64,
    repositioning_time::Float64=0.0,
    max_stops::Int=typemax(Int),
    compensated_dominance::Bool=true,
    tol::Float64=1e-9,
)::JointRoutingAssignmentDarpModifiedPricingData
    positive = PassengerAssignmentCandidate[c for c in candidates if c.reward > tol]

    n_passengers = isempty(positive) ? 0 : maximum(c.p for c in positive)
    passenger_weight = zeros(Float64, n_passengers)
    candidates_by_origin = Dict{Int, Vector{Int}}()
    candidates_by_destination = Dict{Int, Vector{Int}}()
    candidates_by_passenger = [Int[] for _ in 1:n_passengers]

    for (idx, c) in enumerate(positive)
        passenger_weight[c.p] = max(passenger_weight[c.p], c.reward)
        push!(get!(() -> Int[], candidates_by_origin, c.origin), idx)
        push!(get!(() -> Int[], candidates_by_destination, c.destination), idx)
        push!(candidates_by_passenger[c.p], idx)
    end

    bounded_max_stops = max_stops != typemax(Int)
    resolved_max_stops = _resolve_aggregate_od_route_pricing_max_stops(max_stops)

    return JointRoutingAssignmentDarpModifiedPricingData(
        scenario, nodes, travel_cost, route_regularization_weight, repositioning_time,
        max_wait_time, resolved_max_stops, bounded_max_stops, compensated_dominance,
        n_passengers, passenger_weight, positive,
        candidates_by_origin, candidates_by_destination, candidates_by_passenger,
    )
end

"""
    JointRoutingAssignmentDarpModifiedEligibility

One not-yet-served passenger's best currently-live candidate ending at the
node being visited: `(p, origin, reward)`. Deliberately *not* committed here
-- see `_joint_routing_assignment_darp_modified_eligible_at_node`'s docstring and
`types.jl`'s module docstring for why commitment is a branch, not an
automatic consequence of reachability.
"""
const JointRoutingAssignmentDarpModifiedEligibility = Tuple{Int, Int, Float64}

"""
Every not-yet-served passenger whose assignment *could* be newly certified by
a visit to `node` right now: origin's (aged) pickup clock still beats its
ride limit. Returns the *option*, not a commitment -- see `types.jl`'s module
docstring for why forcing commitment the instant a candidate becomes
reachable is unsound (it can make strictly-better assignment sets physically
unrepresentable), and `_joint_routing_assignment_darp_modified_commit_subsets` below
for how this feeds the branch.

Among several simultaneously-live candidates for the *same* passenger ending
here (e.g. two different live origins), only the best is returned -- unlike
*whether* to commit at all, *which* of several options tied for "available
right now" to take is never worth branching on: they differ only in reward at
an otherwise-identical decision point, so the worse one is trivially
dominated and would only bloat the branch count for free. Ties broken by
smaller origin id for determinism, mirroring `exact/accept.jl`'s replay
tie-break.
"""
function _joint_routing_assignment_darp_modified_eligible_at_node(
    node::Int,
    station_age::Dict{Int, Float64},
    travel_time::Float64,
    served::Dict{Int, Tuple{Int, Int}},
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData,
)::Vector{JointRoutingAssignmentDarpModifiedEligibility}
    best_here = Dict{Int, Tuple{Int, Float64}}()  # p -> (origin, reward)
    for idx in get(pricing_data.candidates_by_destination, node, Int[])
        c = pricing_data.candidates[idx]
        haskey(served, c.p) && continue
        origin_age = get(station_age, c.origin, Inf)
        origin_age + travel_time <= c.ride_limit + 1e-9 || continue
        current = get(best_here, c.p, nothing)
        if isnothing(current) || c.reward > current[2] + 1e-9 ||
                (abs(c.reward - current[2]) <= 1e-9 && c.origin < current[1])
            best_here[c.p] = (c.origin, c.reward)
        end
    end
    return JointRoutingAssignmentDarpModifiedEligibility[(p, origin, r) for (p, (origin, r)) in best_here]
end

"""
Every subset of `eligible` worth branching into -- `2^length(eligible)` of
them, including the empty subset (commit nobody, everyone stays open for a
possibly-better opportunity later) and the full subset (the old, unsound
always-commit behavior, now just one branch among many). Each of `eligible`'s
passengers has an independent commit-or-skip choice, so this is exactly that
choice's cross product; enumerating it as explicit subsets (rather than
threading a chain of per-passenger binary sub-decisions through the search)
is what lets each subset become a single self-contained action for the
one-action-in/one-child-out `_pricing_extend_label` hook contract
(`label_setting/types.jl`) without changing the shared engine. See
`_joint_routing_assignment_darp_modified_candidate_next_nodes` (`extend.jl`), which
turns each subset into one action, and `types.jl`'s module docstring for why
this branching is what makes this pricer's optimum provably equal to
`exact/`'s.
"""
function _joint_routing_assignment_darp_modified_commit_subsets(
    eligible::Vector{JointRoutingAssignmentDarpModifiedEligibility},
)::Vector{Vector{JointRoutingAssignmentDarpModifiedEligibility}}
    k = length(eligible)
    subsets = Vector{Vector{JointRoutingAssignmentDarpModifiedEligibility}}(undef, 2^k)
    @inbounds for mask in 0:(2^k - 1)
        subsets[mask + 1] = [eligible[i] for i in 1:k if (mask >> (i - 1)) & 1 == 1]
    end
    return subsets
end

"""
Is a live clock at `station` still able to certify some not-yet-served
passenger? Mirrors `route_covering/data.jl`'s
`_prune_irrelevant_route_covering_station_ages`'s per-station usefulness
test, over candidates instead of pairs.
"""
function _joint_routing_assignment_darp_modified_age_is_useful(
    station::Int,
    age::Float64,
    served::Dict{Int, Tuple{Int, Int}},
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData,
    current::Int,
)::Bool
    for idx in get(pricing_data.candidates_by_origin, station, Int[])
        c = pricing_data.candidates[idx]
        haskey(served, c.p) && continue
        t_to_dest = c.destination == current ? 0.0 :
            _joint_routing_assignment_darp_modified_travel(pricing_data, current, c.destination)
        age + t_to_dest <= c.ride_limit + 1e-9 || continue
        return true
    end
    return false
end
