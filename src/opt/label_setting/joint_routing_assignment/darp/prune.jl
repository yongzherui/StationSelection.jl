"""
The admissible remaining-reward bound behind frontier priority and pop-time
pruning (`_pricing_label_priority`, wired in `hooks.jl`): `_run_label_setting`
(`engine.jl`) stops extending a label no completion of it could ever improve
on. Takes `pricing_data` directly (unlike the other two pricers' twins, this
one needs no precomputed index off the context struct), so this file has no
load-order dependency on `context.jl`.
"""

# ── remaining-reward bound (drives frontier priority + pop-time pruning) ────
"""
Admissible bound on the additional reward still reachable from `label`.
Onboard commitments are excluded because their reward was credited at pickup;
while still within the pickup window, the bound adds the `passenger_weight`
upper bound of every not-yet-resolved passenger. Looser than
`darp_modified/`'s twin (no
reachability check for not-yet-resolved passengers beyond the pickup-window
gate) -- acceptable here since this pricer's whole point is measuring the
cost of a less-clever representation, not chasing bound tightness.
"""
function _joint_routing_assignment_darp_remaining_reward_bound(
    label::JointRoutingAssignmentDarpPricingLabel,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Float64
    ub = 0.0
    if label.time <= pricing_data.max_wait_time + 1e-9
        resolved = _joint_routing_assignment_darp_resolved_passengers(label)
        for p in 1:pricing_data.n_passengers
            p in resolved && continue
            ub += pricing_data.passenger_weight[p]
        end
    end
    return label.reduced_cost - ub
end
