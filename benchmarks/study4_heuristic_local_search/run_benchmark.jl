"""
`run_benchmark.jl <job_line>` -- placeholder. Study 4's heuristic (provisional method
name `local_search` -- see `README.md`) doesn't exist in `src/` yet: no pricer, no
`pricing_mode=:local_search` (or whatever it ends up called) value on
`AggregateODRouteJointRoutingAssignmentFormulation`, alongside today's `:exact`/`:darp`.

Once it exists, this should follow the same shape as
`../study2_passenger_max_ablation/run_benchmark.jl`/`../study3_dominance_ablation/run_benchmark.jl`:
build a `JointRoutingAssignmentPricingData`, call the heuristic's standalone driver
(mirroring `darp/driver.jl`'s "comparison/benchmark entrypoint, not wired into the CG hub"
shape) alongside `exact/`'s and/or `darp/`'s, under a wall-clock budget, and record time
to first improving column, total solve time, optimality gap vs. the exact pricer, and
whether the instance was solvable at all -- see `README.md`'s "Success criteria."

TODO: not implemented -- blocked on the heuristic's design and implementation, which is
out of scope for this scaffolding pass.
"""

# TODO: implement, once the heuristic pricer itself exists in src/.
