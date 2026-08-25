# Study 2 -- Passenger-wise-maximum ablation

## Objective

Compare `exact/`'s running-max reward encoding (each passenger contributes the best
`(j,k)` certified anywhere on the route, regardless of order) against `darp/`'s
first-commit explicit assignment (each passenger contributes whichever `(j,k)` is first
certified, in route-visitation order) -- the two pricers already implement both arms of
this comparison on the same `JointRoutingAssignmentPricingData`. See
`../../src/opt/label_setting/joint_routing_assignment/darp/types.jl`'s module docstring
for the precise difference and the soundness argument for `darp/`'s dominance rule.

## Prerequisite -- blocks this study

**`exact/`'s standalone comparison driver does not exist.** `darp/darp.jl` has one
(`joint_routing_assignment_pricing_by_darp_label_setting`, ~lines 164-222 --
"comparison/benchmark entrypoint, not wired into the CG hub"), built for exactly this
purpose. `exact/exact.jl` has no counterpart -- it ends at `_pricing_verify_column`.
Before `run_benchmark.jl` can be filled in, add `exact/`'s standalone driver, mirroring
`darp/darp.jl`'s block. (Note: current `src/` docstrings in `exact/exact.jl`/`labels.jl`
already reference a `scripts/bench_joint_routing_assignment_labels.jl` that doesn't
exist either -- that script is the stale `scripts/bench_passenger_free_assignment_labels.jl`
awaiting a rename/port; not this study's job to fix, but worth knowing it's the same gap
noticed from a different angle.)

## Method

Once the prerequisite lands: build one scenario's `JointRoutingAssignmentPricingData`,
call both drivers with `profile=true` against the identical input, compare.

## Metrics

Both drivers already return `(columns, exhausted, stats)` where `stats` is
`_run_label_setting`'s profiling `NamedTuple` (`labels_generated`,
`labels_rejected_by_dominance`, `labels_removed_by_dominance`, `max_frontier_size`,
`max_live_labels`, `t_queue_sec`/`t_candidates_sec`/`t_extension_sec`/`t_dominance_sec`)
-- no new instrumentation needed. Plus a correctness check: minimum reduced cost across
`columns` should agree between the two drivers (they optimize different reward-crediting
rules, so exact numeric agreement isn't guaranteed -- this study's real question is
whether it holds in practice, and by how much it diverges when it doesn't).

## Files

- `run_benchmark.jl` -- one instance's comparison: build `pricing_data`, call both
  drivers, write both `stats` + the reduced-cost comparison.
- `analyze.jl` -- aggregate across instances/sizes.
- `submit_benchmark.sh` -- SLURM array submission.
