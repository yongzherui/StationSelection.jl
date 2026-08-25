# Study 3 -- Compensated dominance ablation

## Objective

Compare `compensated_dominance=true` (the "catch-up" rule -- see
`../../src/opt/label_setting/README.md`'s "Compensated dominance" section for the full
mechanism) against `compensated_dominance=false` (plain subset dominance) on `exact/`'s
pricer, holding everything else fixed. Two arms only -- no "no dominance at all" arm
(dropped: dominance is necessary, not itself in question).

Prior work already established the *mechanism* and a per-search effect size
(`notes/2026-08-01_pfa_label_setting_algorithm_reference.md` §5: "the single biggest win
in this pricer, 2.5-3.9x," `max_live` roughly halves) but explicitly left the
*end-to-end* question open ("which side wins for CG is not yet run"). That's this
study.

## Prerequisite -- blocks this study

Same as Study 2: **`exact/`'s standalone comparison driver does not exist yet.** Both
arms of this ablation run through `exact/` (just with `compensated_dominance` flipped),
so this study is blocked on the same fix as Study 2 -- see that study's README for the
exact location (`darp/darp.jl`'s standalone-driver block as the template,
`exact/exact.jl` currently ending at `_pricing_verify_column`).

`scripts/modes/dominance_audit.jl` (stale, pre-rename) is the closest port target once
the driver exists -- every function it calls
(`create_passenger_free_assignment_pricing_data`,
`passenger_free_assignment_dominance_rejections`,
`passenger_free_assignment_pricing_by_label_setting`, `PassengerFreeAssignmentRouteColumn`)
has a confirmed current-API replacement (`create_joint_routing_assignment_pricing_data`,
`joint_routing_assignment_dominance_rejections`
(`src/opt/label_setting/joint_routing_assignment/exact/labels.jl`),
`JointRoutingAssignmentRouteColumn`, and the missing standalone driver above). The
`dominance_census=true` kwarg it uses still exists on the current exact search context.

## Method

Once the prerequisite lands: same instance, same `JointRoutingAssignmentPricingData`,
call `exact/`'s driver twice with `compensated_dominance=true` then `false`,
`profile=true` both times.

## Metrics

Both runs' `stats` `NamedTuple` (labels generated, labels rejected/removed by
dominance, pricing time breakdown -- same fields as Study 2). Plus: minimum reduced cost
must match between the two arms (compensated dominance only ever *adds* valid
dominations, so it must not change the pricing optimum -- this is the correctness check,
not just a metric).

## Files

- `run_benchmark.jl` -- one instance's two-arm comparison.
- `analyze.jl` -- aggregate across instances/sizes.
- `submit_benchmark.sh` -- SLURM array submission.
