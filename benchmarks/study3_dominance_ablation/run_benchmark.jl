"""
`run_benchmark.jl <job_line>` -- one Study 3 job: build one instance's
`JointRoutingAssignmentPricingData`, run `exact/`'s standalone driver twice
(`compensated_dominance=true` then `false`), write both `stats` payloads plus a
reduced-cost-agreement check to this job's output row under
`../../experiments/<date>_study3_dominance_ablation/` (see `../README.md`).

BLOCKED on `exact/`'s missing standalone driver -- see `README.md`'s Prerequisite
section (same blocker as Study 2). Once it exists:

```julia
pricing_data_compensated = create_joint_routing_assignment_pricing_data(...; compensated_dominance=true)
pricing_data_plain       = create_joint_routing_assignment_pricing_data(...; compensated_dominance=false)
columns_c, exhausted_c, stats_c = joint_routing_assignment_pricing_by_label_setting(
    pricing_data_compensated, existing_columns; profile=true,
)
columns_p, exhausted_p, stats_p = joint_routing_assignment_pricing_by_label_setting(
    pricing_data_plain, existing_columns; profile=true,
)
```

Output row: `{instance, compensated_dominance (true/false), labels_generated,
labels_rejected_by_dominance, labels_removed_by_dominance, max_frontier_size,
max_live_labels, t_queue_sec, t_candidates_sec, t_extension_sec, t_dominance_sec,
min_reduced_cost, wall_sec}` per arm, so `analyze.jl` can pair rows by instance and
confirm `min_reduced_cost` agreement.

TODO: not implemented -- blocked (see above). Once `exact/`'s standalone driver exists,
port `scripts/modes/dominance_audit.jl`'s structure (stale, but its renamed
current-API counterparts are all confirmed to exist -- see `README.md`).
"""

# TODO: implement (blocked on exact/'s standalone driver -- see README.md).
