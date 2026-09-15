# The joint-routing CG pricer omits the demand factor on walking cost

2026-09-14. **Open bug, not fixed.** Found while running the parent repo's
2026-09-14 routing-vs-clustering study, which had to fall back to `solver="direct"` for its
routing arm because of this. Verified by reading the three call sites; not yet reproduced
under a test.

## The discrepancy

The master demand-weights walking cost in both places it appears. The pricer does not.

| site | walking term |
| --- | --- |
| `constraints/aggregate_od_route/joint_routing_assignment/routing_and_assignment.jl:38` | `walk += demand * od_pair_walking_cost(data, o, d, (j, k))` |
| `objectives/aggregate_od_route/joint_routing_assignment/assembly.jl:28` | `cost = walk_cost_weight * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)` |
| `label_setting/joint_routing_assignment/pricing_round.jl:62` | `walk_cost_weight * od_pair_walking_cost(data, o, d, pair)` |

`demand` is `mapping.Q_s[s][p]` in both master sites. In the pricer, `Q_s[scenario][p]`
appears only as the `> 0` gate on `pricing_round.jl:51` — never as a factor.

So the pricer under-charges walking by exactly the demand multiplier. That inflates

```
rho_pjk = alpha_p - gamma^O_pj - gamma^D_pk - w * walk(o, d, (j,k))
```

which makes the pricer propose candidates the master then prices worse than the pricer
believed. Pricer and master agree **only when every demand group has demand 1.**

## Why the suite is green

The synthetic benchmarks give every group demand 1, so the two expressions coincide there
and no existing test can distinguish them. Real Zhuzhou data has repeated OD pairs and
`Q_s > 1`, which is where it bites. **A fix needs a real instance with repeated OD pairs as
its gate** — adding a test on the synthetic generators would pass both before and after and
prove nothing.

## The tell

Both pricing modes fail with **identical numbers**. That points at the shared candidate
reward rather than at either mode's search, which is what localised it to
`pricing_round.jl` rather than to `:relaxed_cluster` or `:exact`.

## Why this is not a one-line fix

The obvious change — multiply the pricer's walking term by `Q_s[scenario][p]` — may well be
right, but **which side is authoritative has to be settled first.** The master
demand-weights walking in both places, while the coverage dual `alpha_p` is per-GROUP, not
per-passenger. Those two conventions have to be reconciled deliberately; making one line
match the other without deciding what `alpha_p` is denominated in risks trading a visible
disagreement for a silent one.

This is the class of change where being wrong produces a plausible-looking wrong optimum
rather than an error, so it wants the dual-feasibility check that
`benchmarks/diagnostics/benders_activated_completion_audit.jl` already does for the Benders
completion — `min_c rc(c) >= 0` measured directly over an enumerated universe — run on an
instance with `Q_s > 1`, rather than an objective-value comparison.

## Consequence while it stands

`AggregateODRouteJointRoutingAssignmentFormulation` under `CGSolver` cannot be trusted on
data with repeated OD pairs. Use `DirectMIPSolver` there, or restrict to instances where
every demand group has demand 1. See [[2026-09-10_locally_pareto_optimal_activated_cut]]
for the Benders-side pricing path, which shares this pricer.
