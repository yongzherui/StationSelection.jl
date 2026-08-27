"""
The admissible remaining-reward bound behind frontier priority and pop-time
pruning (`_pricing_label_priority`, wired in `hooks.jl`): `_run_label_setting`
(`engine.jl`) uses `label.reduced_cost - bound` both to order the frontier and,
at pop time, to stop extending a label no completion of it could ever improve
on. Needs `JointRoutingAssignmentSearchIndex`/`JointRoutingAssignmentBoundWorkspace`
(`../search_data.jl`), so this file loads after it.
"""

# ── remaining-reward bound (drives frontier priority + pop-time pruning) ────
"""
Admissible bound on the additional reward still reachable from `label`: the
summed dual reward of every not-yet-activated reward layer that could still be
certified one way or another (the two loops below). `label.reduced_cost -
bound` is therefore a lower bound on the reduced cost of every completion of
`label`, used both to order the frontier and to prune at pop time -- future
travel only ever adds nonnegative cost to reduced cost, so ignoring it here
can only make the bound *looser*, never wrong (this is exactly the paper's
`U(ell)`: a plain sum over reachable layers, with no travel discount).

Rewritten once from an `O(|opportunities|)` scan (every opportunity re-tested for
every label) to `O(#live origins' opportunities + n_nodes)`: `|opportunities| ~
P * n^2`, so the old form made per-label cost grow with `n^2` and drove measured
time to ~`n^5.5-7.6` while label counts only grew ~`n^3.4`.

That rewrite is why this is **no longer** a hot spot, and the claim that it is
"the single biggest cost in the search" -- which this docstring used to make --
has been false ever since. Profiling on 2026-07-30 put this bound at ~0.6% of
wall time against ~90% in the dominance scan (`dominate.jl`). Do not spend
effort tightening it for speed; see
`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`.

Two structurally different sources of future reward, handled separately:

  - **live origins** -- an origin with a live clock can still certify its own
    opportunities, but only those whose ride limit survives `age + travel(current, k)`.
    That test needs the actual age, so it stays per-opportunity -- but only over
    opportunities of origins that are *actually live*, which pruning keeps small.

  - **refreshable origins** -- if still inside the pickup window, any origin
    reachable before the cutoff could be visited to open a fresh clock. The
    condition depends only on the *origin*, so the whole origin's union mask
    (`origin_union_mask`) can be checked at once rather than per-opportunity.

`workspace.layer_scratch` dedups across both sources (and within each): a
layer reachable via several opportunities or origins is only ever counted, and
its weight only ever added, once.

A 2026-08 version of this bound additionally discounted each reachable node's
reward by the regularized travel cost of reaching it (`beta * travel(current,
x)`), taking a running maximum over nodes visited in increasing distance from
`current` -- a strictly tighter, still-admissible bound, requiring a
per-search sorted-nodes-by-distance index and a running-max walk to compute.
Removed 2026-08-19: its own docstring already measured it as **no effect** --
under 0.2% label-count movement and flat wall time on the benchmarked
instances (`scripts/bench_joint_routing_assignment_labels.jl`) -- so the extra
machinery was paying for a tightness that never showed up in practice, at the
cost of real complexity. See git history if a future duals regime (per that
docstring: converged CG duals where most `rho_pjk` are near zero and `beta *
travel` is comparable to the entire remaining reward) resurrects the need for
it.
"""
# `label`/`label_bs` are intentionally untyped: this bound is shared by the
# revisit-tolerant pricer (`JointRoutingAssignmentPricingLabel` /
# `JointRoutingAssignmentLabelBitsets`) and the elementary station-simple pricer
# (`../station_simple/types.jl`), whose label/bitset types differ but expose the same
# `current`/`time`/`activated_reward_layers` and `age_idx`/`age_val` fields the
# bound reads. Julia still specializes per concrete call site, so there is no
# dispatch or performance cost to dropping the annotations.
function _joint_routing_assignment_remaining_reward_bound(
    label,
    label_bs,
    pricing_data::JointRoutingAssignmentPricingData,
    index::JointRoutingAssignmentSearchIndex,
    workspace::JointRoutingAssignmentBoundWorkspace,
)::Float64
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    current_idx = index.node_index[label.current]
    activated = label.activated_reward_layers
    layer_weight = pricing_data.layer_weight

    empty!(workspace.layer_scratch)
    total = 0.0

    # live origins: age-dependent, so still per-opportunity -- but only theirs.
    @inbounds for t in eachindex(label_bs.age_idx)
        origin_idx = Int(label_bs.age_idx[t])
        age = label_bs.age_val[t]
        for i in index.opps_by_origin_idx[origin_idx]
            issubset(index.opp_layer_mask[i], activated) && continue
            dest_idx = index.opp_dest_idx[i]
            age + index.travel_matrix[current_idx, dest_idx] <= index.opp_ride_limit[i] + 1e-9 || continue
            for layer in index.opp_layer_mask[i]
                (layer in activated || layer in workspace.layer_scratch) && continue
                push!(workspace.layer_scratch, layer)
                total += layer_weight[layer]
            end
        end
    end

    # refreshable origins: condition depends only on the origin, so check its
    # whole union mask at once rather than per opportunity.
    if !past_pickup_cutoff
        @inbounds for origin_idx in eachindex(index.origin_union_mask)
            isempty(index.origin_union_mask[origin_idx]) && continue
            label.time + index.travel_matrix[current_idx, origin_idx] <=
                pricing_data.max_wait_time + 1e-9 || continue
            for layer in index.origin_union_mask[origin_idx]
                (layer in activated || layer in workspace.layer_scratch) && continue
                push!(workspace.layer_scratch, layer)
                total += layer_weight[layer]
            end
        end
    end

    return total
end
