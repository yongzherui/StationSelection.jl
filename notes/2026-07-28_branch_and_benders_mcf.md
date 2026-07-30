# Branch-and-Benders with a multicommodity-flow routing bound

## Master

`BranchAndBendersSolver` solves one Gurobi branch-and-bound tree. The master keeps station
openings `y` binary, builds the existing exact nearest-open walking-cost structure, and carries
one full routing-LP recourse variable `theta[s]` per scenario. The existing multicommodity flow
network is embedded permanently through

```text
theta[s] >= C_MCF[s](y, f)
```

and the objective is

```text
C_walk(y) + route_regularization_weight * sum(theta[s])
```

The MCF cost is not a separate additive objective term, so it is not double counted.

## Callback and global cuts

The `MOI.LazyConstraintCallback` runs only at integer callback solutions. `BendersY` uses a stable
tuple of open station indices as its exact-oracle cache key. `BendersYZ` uses the complete immutable
`(y,z)` state, including every endpoint-chain selector, so ties or formulation degeneracy cannot
reuse a cut generated for another assignment. A new first-stage state is assigned and solved first
by the existing fixed-assignment column-generation driver, then by the existing repricing loop
until an exhaustive pricing pass certifies the full route universe.

With `decomposition=BendersY()`, the certified standard cut is affine in `y`. With the default
`decomposition=BendersYZ()`, the callback also reads and verifies the deterministic nearest-open
endpoint selectors and the cut is affine in `z`. Although the oracle is called only at binary
`y`, both inequalities are ordinary LP-dual inequalities and therefore remain globally valid at
fractional nodes after Gurobi adds them to the tree.

Every cut is checked for tightness at its generating point. Repeated station sets reuse the
cached exact values and cut but resubmit the lazy row whenever the current callback solution
violates it.

## Bounds and termination

Every exact oracle evaluation gives the certified feasible value

```text
C_walk(y) + route_regularization_weight * sum(Q_LP[s](y)).
```

The best such value is the reported upper bound and final objective. Gurobi's branch-and-bound
`objective_bound` is the global lower bound. An `OPTIMAL` return is accepted only when these agree
within the configured tightness tolerance. Pricing, repricing, or cut-certification failure is
fatal; an uncertified candidate is never accepted.

The default global stopping tolerance is a 1% relative MIP gap. Set `mip_gap=0.0` in the
Branch-and-Benders solver configuration when an exact optimality proof is required. This setting
does not relax lazy-cut violation, pricing, or dual-feasibility tolerances.

Runtime logging distinguishes master preparation (`pre_optimize_seconds`), the single Gurobi
solve including synchronous callback work (`master_optimize_seconds`), and the full solver-entry
time through certification (`run_opt_seconds`). Oracle, priming-CG, and repricing times are nested
inside the master solve; they must not be added to it. Experiment-level wall time surrounds the
entire `run_opt` call but intentionally excludes instance generation, optimizer-environment
creation, Julia startup, and compilation.

With a log directory configured, every integer callback candidate is also recorded in
`aggregate_od_route_branch_benders_callbacks.csv`. Each row includes elapsed solve time, the open
station indices, cache status, exact candidate and best certified upper bounds, violated recourse
blocks, cumulative cuts and cache counters, oracle timing, and shared route-pool size. The same
information is emitted incrementally to the live log, and the CSV is written before the final
bound-consistency assertion so a failed certification still leaves a diagnostic trace.
The trace also records Gurobi's node count and current lower bound at every integer-solution event,
along with the relative gap between that lower bound and the best exactly certified upper bound.
Gurobi's complete native branch-and-bound table is saved separately as
`aggregate_od_route_branch_benders_gurobi.log` with a one-second display interval.

## Restrictions

The first implementation supports nearest-open `AggregateODRouteModel`, scenario multicut,
`BendersY` and `BendersYZ`, and certified standard cuts with exhaustive repricing. Gurobi is
forced to one thread because the callback synchronously solves separate Gurobi subproblem models
and mutates shared cache/column-pool state. Restricted-MW cuts, fractional separation, SingleCut,
no-good cuts, and routing-IP recourse are not supported.
