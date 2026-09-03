"""
Route replay for the relaxed-cluster pricer: recompute a finished cluster
route's reduced cost from scratch, independently of the label that produced it.

This is the relaxed twin of `../exact/accept.jl`'s
`_joint_routing_assignment_column_from_route`, minus everything to do with
columns -- there is no column to build here (`types.jl` explains why), so no
concrete `(p, j, k)` assignments are recovered and no signature is formed. What
remains is the invariant that file also checks: a label's `reduced_cost` must
equal what a from-scratch replay of its own route computes, or the incremental
reward-layer accounting has drifted from a direct passenger-by-passenger sum.

`../exact/accept.jl` gets that check for free because every accepted label is
replayed anyway. Nothing replays a relaxed label in production, so the check
lives here and is exercised by the tests plus `certify.jl`'s optional
`verify_replay` mode.
"""

"""
    _relaxed_cluster_route_reduced_cost(route, data) -> (reduced_cost, tau, reward)

Replay a cluster `route` from `t = 0` and return its reduced cost, physical
travel time, and total reward, computed directly from per-passenger maxima --
no reward layers, no dominance history, no label state.

The reward model mirrors the search exactly: a visit to a cluster inside the
pickup window opens a clock there; a later cluster certifies `(p, C, D)` when
that clock survives `R_bar`; a visit inside the window also banks the cluster's
intra-cluster credit; and each passenger banks only its single best certified
reward.
"""
function _relaxed_cluster_route_reduced_cost(
    route::Vector{Int}, data::RelaxedClusterPricingData,
)
    pricing_data = data.inner
    best_reward = Dict{Int, Float64}()
    bank!(p, reward) = (best_reward[p] = max(get(best_reward, p, 0.0), reward))

    # Intra-cluster credit is keyed by passenger the same way, so it competes with the
    # inter-cluster ones for that passenger's single best reward rather than adding to it.
    # `data.intra_layer_mask` records the same credit as layer prefixes, which is what the
    # search consumes; here the raw `(p, reward)` pairs are wanted instead, so they are
    # re-collected off `opportunities` once rather than rescanned at every stop.
    intra_by_node = Dict{Int, Vector{Tuple{Int, Float64}}}()
    for opp in pricing_data.opportunities
        opp.origin == opp.destination || continue
        push!(get!(() -> Tuple{Int, Float64}[], intra_by_node, opp.origin), (opp.p, opp.reward))
    end
    intra_at!(node) = for (p, reward) in get(intra_by_node, node, Tuple{Int, Float64}[])
        bank!(p, reward)
    end

    isempty(route) && return 0.0, 0.0, 0.0
    pickup_time = Dict{Int, Float64}(route[1] => 0.0)
    elapsed_time = 0.0
    intra_at!(route[1])

    for idx in 2:length(route)
        previous, next_node = route[idx - 1], route[idx]
        elapsed_time += _joint_routing_assignment_travel(pricing_data, previous, next_node)
        for opp in get(pricing_data.assignments_by_destination, next_node,
                       PassengerAssignmentOpportunity[])
            age = elapsed_time - get(pickup_time, opp.origin, -Inf)
            age <= opp.ride_limit + 1e-9 || continue
            bank!(opp.p, opp.reward)
        end
        if elapsed_time <= pricing_data.max_wait_time + 1e-9
            pickup_time[next_node] = elapsed_time
            intra_at!(next_node)
        end
    end

    tau = length(route) < 2 ? 0.0 : sum(
        _joint_routing_assignment_travel(pricing_data, route[i], route[i + 1])
        for i in 1:(length(route) - 1)
    )
    reward = sum(values(best_reward); init=0.0)
    reduced_cost =
        pricing_data.route_regularization_weight * (tau + pricing_data.repositioning_time) - reward
    return reduced_cost, tau, reward
end
