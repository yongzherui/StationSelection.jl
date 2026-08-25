"""
`run_benchmark.jl <job_line>` -- one Study 2 job: build one scenario's
`JointRoutingAssignmentPricingData`, run `exact/`'s and `darp/`'s standalone pricer
drivers against it, and write both `stats` payloads plus a reduced-cost-agreement check
to this job's output row under
`../../experiments/<date>_study2_passenger_max_ablation/` (see `../README.md`).

BLOCKED on `exact/`'s missing standalone driver -- see `README.md`'s Prerequisite
section. `darp/`'s side of this comparison can be exercised today:

```julia
pricing_data = create_joint_routing_assignment_pricing_data(...)   # src/opt/label_setting/joint_routing_assignment/data.jl
darp_data    = JointRoutingAssignmentDarpPricingData(...)          # src/.../darp/types.jl -- separate struct, same shape
columns_darp, exhausted_darp, stats_darp = joint_routing_assignment_pricing_by_darp_label_setting(
    darp_data, existing_columns; profile=true,
)
# exact/'s counterpart, once built:
columns_exact, exhausted_exact, stats_exact = joint_routing_assignment_pricing_by_label_setting(
    pricing_data, existing_columns; profile=true,
)
```

Output row: `{instance, driver ("exact"/"darp"), labels_generated,
labels_rejected_by_dominance, labels_removed_by_dominance, max_frontier_size,
max_live_labels, t_queue_sec, t_candidates_sec, t_extension_sec, t_dominance_sec,
min_reduced_cost, wall_sec}` per driver, so `analyze.jl` can pair rows by instance.

TODO: not implemented -- blocked (see above). Once `exact/`'s standalone driver exists,
implement: build `pricing_data`/`darp_data` for a chosen instance, run both, write rows.
"""

# TODO: implement (blocked on exact/'s standalone driver -- see README.md).
