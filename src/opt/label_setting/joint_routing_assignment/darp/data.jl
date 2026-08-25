"""
Builds the per-scenario pricing graph for `darp/`'s onboard-bitset pricer,
and the board-option/subset-enumeration helpers `labels.jl`'s extension step
calls. Counterpart to `../data.jl` (`exact/`+`station_simple/`'s reward-layer
preprocessing) and `../darp_modified/data.jl` (`darp_modified/`'s eligibility/
commit-subset helpers) -- this pricer's boarding decision is a genuinely
different combinatorial shape from either (see `types.jl`'s module
docstring), so it gets its own.
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

Build the preprocessed pricing data for one scenario's onboard-bitset
passenger free-assignment pricing pass. `candidates` carries already-computed
rewards (as `create_joint_routing_assignment_pricing_data` does for `exact/`);
this constructor filters to positive-reward candidates, indexes them by
origin/destination/passenger for fast lookup, and additionally builds the
dense `candidate_index` (triple -> array position) the bitset-based dominance
and remaining-reward bound need.
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
    tol::Float64=1e-9,
)::JointRoutingAssignmentDarpPricingData
    positive = PassengerAssignmentCandidate[c for c in candidates if c.reward > tol]

    n_passengers = isempty(positive) ? 0 : maximum(c.p for c in positive)
    passenger_weight = zeros(Float64, n_passengers)
    candidate_index = Dict{Tuple{Int, Int, Int}, Int}()
    candidates_by_origin = Dict{Int, Vector{Int}}()
    candidates_by_destination = Dict{Int, Vector{Int}}()
    candidates_by_passenger = [Int[] for _ in 1:n_passengers]

    for (idx, c) in enumerate(positive)
        triple = (c.p, c.origin, c.destination)
        haskey(candidate_index, triple) && throw(ArgumentError(
            "duplicate passenger assignment candidate $triple in DARP pricing data",
        ))
        passenger_weight[c.p] = max(passenger_weight[c.p], c.reward)
        candidate_index[triple] = idx
        push!(get!(() -> Int[], candidates_by_origin, c.origin), idx)
        push!(get!(() -> Int[], candidates_by_destination, c.destination), idx)
        push!(candidates_by_passenger[c.p], idx)
    end

    bounded_max_stops = max_stops != typemax(Int)
    resolved_max_stops = _resolve_aggregate_od_route_pricing_max_stops(max_stops)

    return JointRoutingAssignmentDarpPricingData(
        scenario, nodes, travel_cost, route_regularization_weight, repositioning_time,
        max_wait_time, resolved_max_stops, bounded_max_stops,
        n_passengers, passenger_weight, positive, candidate_index,
        candidates_by_origin, candidates_by_destination, candidates_by_passenger,
    )
end

"""
Every not-yet-resolved (not onboard, not served) passenger with a candidate
whose origin is `node`, grouped by passenger, restricted to candidates whose
direct travel time to their destination doesn't already exceed their own ride
limit (a dead-on-arrival option is never worth offering -- boarding it could
never possibly pay off, the same admissibility reasoning
`_joint_routing_assignment_darp_candidate_next_nodes`'s hard-infeasibility
filter uses). `resolved` is `p => true` for every passenger already onboard
or served, precomputed once per extension rather than re-scanned per
candidate.
"""
function _joint_routing_assignment_darp_board_options(
    node::Int,
    label_time::Float64,
    resolved::Set{Int},
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Dict{Int, Vector{Tuple{Int, Float64}}}
    options = Dict{Int, Vector{Tuple{Int, Float64}}}()
    label_time <= pricing_data.max_wait_time + 1e-9 || return options
    for idx in get(pricing_data.candidates_by_origin, node, Int[])
        c = pricing_data.candidates[idx]
        c.p in resolved && continue
        _joint_routing_assignment_darp_travel(pricing_data, node, c.destination) <= c.ride_limit + 1e-9 || continue
        push!(get!(() -> Tuple{Int, Float64}[], options, c.p), (c.destination, c.reward))
    end
    return options
end

"""
Every valid board selection at `node`: for each passenger with options here,
either skip them or commit to exactly one of their candidate destinations
(elementarity -- never more than one). Returns each selection as the flat
list of `(p, node, destination, reward)` triples actually boarded; the empty
`options` case returns the single empty selection, so a node with nothing to
decide degenerates to one action, same as every other pricer's "boring node"
case.
"""
function _joint_routing_assignment_darp_board_subsets(
    node::Int,
    options::Dict{Int, Vector{Tuple{Int, Float64}}},
)::Vector{Vector{Tuple{Int, Int, Int, Float64}}}
    isempty(options) && return [Tuple{Int, Int, Int, Float64}[]]

    passengers = sort!(collect(keys(options)))
    choice_lists = [vcat(Any[nothing], options[p]) for p in passengers]

    selections = Vector{Tuple{Int, Int, Int, Float64}}[]
    for combo in Iterators.product(choice_lists...)
        selection = Tuple{Int, Int, Int, Float64}[]
        for (i, choice) in enumerate(combo)
            choice === nothing && continue
            k, r = choice
            push!(selection, (passengers[i], node, k, r))
        end
        push!(selections, selection)
    end
    return selections
end
