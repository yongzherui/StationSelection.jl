# Pool trajectory — n10_p8_s3_seed42_ms4

Zhuzhou n=10, p=8, **s=3**, seed 42, k=5, max_stops=4. Two arms
(`:column_generation`, `:column_generation_activated`) driven through a mirror of the real
Benders loop, with the pool dumped after every iteration for every scenario.

`incumbent` is the master's station set at that iteration — the point the cut is anchored at.
(Earlier notes called this an "anchor"; it is just an incumbent.)

## trajectory_summary.csv — one row per (arm, iteration, scenario)

| field | meaning |
| --- | --- |
| `lower_bound` | the master's `objective_bound`, monotone |
| `subproblem_cost` | that scenario's exact second-stage cost at this incumbent |
| `pool_size`, `added_this_iteration` | columns held, and how many were new this iteration |
| `n_unbuilt_0/1/2/3plus` | pool columns by how many DISTINCT unbuilt stations they assign at |
| `all_unbuilt_assign` | every assignment station unbuilt |
| `route_all_unbuilt` | every VISITED station unbuilt — routes living wholly outside the built set |

A column with `n_unbuilt_assign > 0` has its `theta` pinned to 0 at that incumbent, so it
cannot improve the objective — it is in the pool purely for its dual constraint.

## trajectory_columns.csv — one row per (arm, iteration, scenario, column)

The same classification per column, plus `route`, `assignments` (`p:j:k`), `tau`, and
`is_new_this_iteration`. Note the pool accumulates across iterations by design, so a column
appears once per iteration from the point it enters; filter on `is_new_this_iteration` to get
arrival times instead of holdings.

