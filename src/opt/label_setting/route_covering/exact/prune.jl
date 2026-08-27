"""
The admissible remaining-reward bound behind frontier priority and pop-time
pruning (`_pricing_label_priority`, wired in `hooks.jl`): `_run_label_setting`
(`engine.jl`) uses `label.reduced_cost - bound` both to order the frontier and,
at pop time, to stop extending a label no completion of it could ever improve
on. Needs `RouteCoveringSearchContext` (`context.jl`) -- unlike Joint's twin,
this pricer's precomputed indices live directly on the context struct rather
than a separate index type, so the bound takes `ctx` itself rather than a
handful of individual fields -- so this file loads after it.
"""

# Lower bound on the reduced cost of *any* descendant of `label`:
# `label.reduced_cost` minus an upper bound `ub` on every dual reward this
# route could still possibly collect (returned here as just `ub`; `hooks.jl`'s
# `_pricing_label_priority` does the subtraction). The search treats this as
# admissible (never overestimates remaining reward), so once a popped
# priority is no longer beating `reduced_cost_tol`, nothing reachable from it
# can be either -- that's what licenses `engine.jl`'s reduced-cost pruning.
#
# `ub` sums, over every not-yet-served pair with positive dual reward, that
# reward if the pair is *still reachable one way or another* -- ignoring
# whether the route could realistically detour to reach several such pairs
# at once. That slack (every reachable pair counted as if independently
# certifiable) is what makes this a bound rather than an exact remaining
# value, and is the whole point: cheap to compute, never wrong-signed.
function _route_covering_remaining_reward_bound(
    label::RouteCoveringPricingLabel, label_bs::RouteCoveringLabelBitsets, ctx::RouteCoveringSearchContext,
)::Float64
    past_pickup_cutoff = label.time > ctx.pricing_data.max_wait_time + 1e-9
    current_idx = ctx.node_index[label.current]
    ub = 0.0
    @inbounds for i in 1:ctx.n_pairs
        ctx.positive_pair_rewards[i] > 0 || continue   # no reward, no contribution to the bound
        i in label_bs.served_bits && continue          # already certified, already counted in reduced_cost
        # `label_bs.age_idx` is sorted, so this is a binary search for pair
        # i's origin among the label's currently-live pickup clocks; `Inf` if
        # that origin was never visited (or its clock has since been pruned).
        pos = searchsortedfirst(label_bs.age_idx, Int32(ctx.pair_origin_idx[i]))
        origin_age = pos <= length(label_bs.age_idx) && label_bs.age_idx[pos] == ctx.pair_origin_idx[i] ?
            label_bs.age_val[pos] : Inf
        # Two independent ways pair i could still be certified: (1) a pickup
        # clock is already live and a direct trip from here to the
        # destination still beats the ride limit, or (2) the pickup cutoff
        # hasn't passed yet, so the route could still detour to open a fresh
        # clock at the origin. Either is enough to count the reward.
        can_claim_current = isfinite(origin_age) &&
            origin_age + ctx.travel_matrix[current_idx, ctx.pair_dest_idx[i]] <= ctx.pair_ride_limit[i] + 1e-9
        can_refresh = !past_pickup_cutoff &&
            label.time + ctx.travel_matrix[current_idx, ctx.pair_origin_idx[i]] <= ctx.pricing_data.max_wait_time + 1e-9
        can_claim_current || can_refresh || continue
        ub += ctx.positive_pair_rewards[i]
    end
    return ub
end

# Unused elsewhere (no caller in src/ or test/) -- carried over unmodified
# from the pre-split labels.jl rather than dropped, since removing dead code
# was not in scope for this reorganization.
function _route_covering_label_priority(
    label::RouteCoveringPricingLabel,
    duals::RouteCoveringPricingDuals,
)::Float64
    return label.reduced_cost
end
