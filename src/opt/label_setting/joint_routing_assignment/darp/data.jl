"""
Builds the per-scenario pricing graph for `darp/`'s first-commit passenger
free-assignment pricer, and the certify/prune helpers `labels.jl`'s extension
step calls. Counterpart to `../data.jl` (`exact/`+`station_simple/`'s
reward-layer preprocessing) -- this pricer needs none of that, since credit
is assigned once, directly, at the moment of first commitment (see
`types.jl`'s module docstring).
"""

export create_joint_routing_assignment_darp_pricing_data

function _joint_routing_assignment_darp_travel(
    pricing_data::JointRoutingAssignmentDarpPricingData, u::Int, v::Int,
)::Float64
    cost = get(pricing_data.travel_cost, (u, v), Inf)
    isfinite(cost) || throw(ArgumentError("missing finite routing cost for station arc $((u, v))"))
    return cost
end

"""
    create_joint_routing_assignment_darp_pricing_data(scenario, nodes, travel_cost, candidates; kwargs...)

Build the preprocessed pricing data for one scenario's first-commit passenger
free-assignment pricing pass. `candidates` carries already-computed rewards
(as `create_joint_routing_assignment_pricing_data` does for `exact/`); this
constructor's only job is filtering to positive-reward candidates and
indexing them by origin/destination/passenger for fast certification.
"""
function create_joint_routing_assignment_darp_pricing_data(
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
)::JointRoutingAssignmentDarpPricingData
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

    return JointRoutingAssignmentDarpPricingData(
        scenario, nodes, travel_cost, route_regularization_weight, repositioning_time,
        max_wait_time, resolved_max_stops, bounded_max_stops, compensated_dominance,
        n_passengers, passenger_weight, positive,
        candidates_by_origin, candidates_by_destination, candidates_by_passenger,
    )
end

"""
Certify every not-yet-served passenger whose assignment ends at `node` and
whose origin's (aged) pickup clock still beats its ride limit -- first-commit,
not running-max (see `types.jl`'s module docstring): among everything
simultaneously certifiable for the same passenger right now, the best of
*those* wins (there is no "wait for a later, better dropoff" -- once
committed, a passenger's assignment cannot change). Ties broken by smaller
origin id for determinism, mirroring `exact/exact.jl`'s replay tie-break.

Returns `(certified::Dict{Int,Tuple{Int,Int}}, reward::Float64)`, `certified`
being `served` with any newly-committed passengers added.
"""
function _certify_joint_routing_assignment_darp_assignments_at_node(
    node::Int,
    station_age::Dict{Int, Float64},
    travel_time::Float64,
    served::Dict{Int, Tuple{Int, Int}},
    pricing_data::JointRoutingAssignmentDarpPricingData,
)
    # p -> (origin, reward), among this node's feasible not-yet-served candidates.
    best_here = Dict{Int, Tuple{Int, Float64}}()
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
    isempty(best_here) && return served, 0.0

    certified = copy(served)
    reward = 0.0
    for (p, (origin, r)) in best_here
        certified[p] = (origin, node)
        reward += r
    end
    return certified, reward
end

"""
Is a live clock at `station` still able to certify some not-yet-served
passenger? Mirrors `route_covering/data.jl`'s
`_prune_irrelevant_route_covering_station_ages`'s per-station usefulness
test, over candidates instead of pairs.
"""
function _joint_routing_assignment_darp_age_is_useful(
    station::Int,
    age::Float64,
    served::Dict{Int, Tuple{Int, Int}},
    pricing_data::JointRoutingAssignmentDarpPricingData,
    current::Int,
)::Bool
    for idx in get(pricing_data.candidates_by_origin, station, Int[])
        c = pricing_data.candidates[idx]
        haskey(served, c.p) && continue
        t_to_dest = c.destination == current ? 0.0 :
            _joint_routing_assignment_darp_travel(pricing_data, current, c.destination)
        age + t_to_dest <= c.ride_limit + 1e-9 || continue
        return true
    end
    return false
end
