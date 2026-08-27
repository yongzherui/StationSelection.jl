"""
The admissible remaining-reward bound behind frontier priority and pop-time
pruning (`_pricing_label_priority`, wired in `hooks.jl`): `_run_label_setting`
(`engine.jl`) stops extending a label no completion of it could ever improve
on. `ctx` is untyped (reads only `pricing_data`/`node_index`/`travel_matrix`
off it), so this file has no textual load-order dependency on `context.jl` --
unlike `../exact/prune.jl`'s twin, which needs the context struct's type in
its own signature.
"""

# ── remaining-reward bound (drives frontier priority + pop-time pruning) ────
"""
Admissible bound on the additional reward still reachable from `label`: the
summed `passenger_weight` upper bound of every not-yet-served passenger who
still has some live-or-refreshable candidate reaching them. Same two-source
structure (live origins / refreshable origins) as
`route_covering/exact/prune.jl`'s and
`joint_routing_assignment/exact/prune.jl`'s twins, generalized from "per pair"
to "per passenger, scanning that passenger's own candidates" via
`candidates_by_passenger` (`data.jl`) -- since a passenger can have several
candidate pairs with different origins/destinations/ride limits, unlike
`route_covering`'s one-weight-per-pair simplicity.
"""
function _joint_routing_assignment_darp_modified_remaining_reward_bound(
    label::JointRoutingAssignmentDarpModifiedPricingLabel,
    label_bs::JointRoutingAssignmentDarpModifiedLabelBitsets,
    ctx,
)::Float64
    pricing_data = ctx.pricing_data
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    current_idx = ctx.node_index[label.current]
    ub = 0.0
    @inbounds for p in 1:pricing_data.n_passengers
        pricing_data.passenger_weight[p] > 0 || continue    # no reward, no contribution to the bound
        p in label_bs.served_bits && continue                # already certified, already counted in reduced_cost
        reachable = false
        for idx in pricing_data.candidates_by_passenger[p]
            c = pricing_data.candidates[idx]
            origin_idx = ctx.node_index[c.origin]
            pos = searchsortedfirst(label_bs.age_idx, Int32(origin_idx))
            origin_age = pos <= length(label_bs.age_idx) && label_bs.age_idx[pos] == origin_idx ?
                label_bs.age_val[pos] : Inf
            dest_idx = ctx.node_index[c.destination]
            can_claim_current = isfinite(origin_age) &&
                origin_age + ctx.travel_matrix[current_idx, dest_idx] <= c.ride_limit + 1e-9
            can_refresh = !past_pickup_cutoff &&
                label.time + ctx.travel_matrix[current_idx, origin_idx] <= pricing_data.max_wait_time + 1e-9
            if can_claim_current || can_refresh
                reachable = true
                break
            end
        end
        reachable && (ub += pricing_data.passenger_weight[p])
    end
    return label.reduced_cost - ub
end
