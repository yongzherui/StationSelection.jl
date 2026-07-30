# Branch-and-Price on y — design sketch (not implemented)

*2026-07-27*

## Status: design sketch only, discussed and shelved. No code written, no plan to implement
unless priorities change. Recorded here so the reasoning isn't lost.

## Motivation

Came out of the zhuzhou p16 scaling study (`scripts/zhuzhou_p16_scaling_route100x*.jl`,
`experiments/zhuzhou_p16_scaling_route100x*`). Two things were observed there:

1. BendersYZ's master MIP bloats badly as outer iterations accumulate optimality cuts (one per
   scenario per iteration under `cut_mode=MultiCut()` — see the n=30 runtime-breakdown analysis
   earlier in this investigation: master solve time was 88-95% of total wall time and grew ~14x
   over a run as the cut count grew into the thousands).
2. Plain (non-Benders) column generation with y unfixed (`cg_ms4`/`cg_ms5`,
   `experiments/zhuzhou_p16_cg_ms45_singlescenario/`) was tested as a possible fast heuristic
   alternative and performed badly: 26-98% worse objective than Direct's true optimum across
   n=20..60, a very weak self-reported LP bound (17-380% "integrality gap"), and station
   selections only ~64% (Jaccard) overlapping with Direct's true optimum. Root cause: pricing
   routes with y completely unfixed is a much harder combinatorial search than Benders' inner CG
   (which fixes y to one exact vector first).

This raised the question: instead of Benders' dual-cut abstraction (a single scalar `theta` cut
per y_hat/scenario), could we inject the actual CG-generated *columns* directly into the master
and branch there — i.e. proper branch-and-price? Sketched below, but not pursued further because
(a) it's a substantially bigger build than Benders, and (b) the CG-heuristic data above already
shows that pricing with y not fully fixed produces weak columns/bounds, which is exactly the
sub-step this design leans on hardest (per-node pricing at partially-fixed y).

## Core idea: branch purely on y

The one thing that makes this tractable to sketch simply: the master already has, for every
route `r` through station `j`, a linking constraint `theta_r <= y_j` (or the tighter
`x <= z[j], x <= z[k]` form under `tight_constraints`). Fixing `y_j = 0` at a branch node
therefore *automatically* zeroes every column through `j` for free — no separate per-node column
pruning/bookkeeping needed. This is what makes "branch on y only" simple: no Ryan-Foster / arc
branching on route variables is needed the way it usually is in VRP branch-and-price, because we
never branch on anything but the station-open variables. One consequence: a single **global**
column pool (grown monotonically, exactly like Benders' `shared_pool` in `benders/y.jl`) can be
shared across the whole tree — columns through closed stations are just inert in that node's LP,
not literally removed.

## Node state

```
Node:
    fixed_open   :: Set{Int}   # y_j forced = 1
    fixed_closed :: Set{Int}   # y_j forced = 0
    lp_bound     :: Float64    # set once this node has been priced to exhaustion
```

Free stations = `all \ (fixed_open ∪ fixed_closed)`. Cheap feasibility pre-check before ever
solving an LP: infeasible if `l - |fixed_open| < 0` or `l - |fixed_open| > |free|`.

## Global state

- `shared_pool` — union of every column ever priced anywhere in the tree.
- `incumbent`, `UB` — best integer-feasible solution found so far.
- `queue` — open nodes in a priority queue ordered by `lp_bound` ascending (best-first).
- `global_LB` = bound of whatever's at the head of `queue` at any point -- gives a live
  optimality gap `(UB - global_LB)/UB`, the same shape as the `outer_gap` already logged for
  Benders.

## Algorithm

```
push root(fixed_open=empty, fixed_closed=empty) onto queue with lp_bound = -inf

while queue not empty:
    node = pop node with smallest lp_bound
    if node.lp_bound >= UB - tol: continue        # fathom: bound

    # --- price this node to exhaustion ---
    loop:
        solve LP relaxation of master (y bounds per node's fixing, theta >= 0, using shared_pool)
        if infeasible: mark node infeasible; break outer   # fathom: infeasibility
        duals = extract duals (coverage, linking, sum(y)=l)
        new_cols = price(duals; allowed_stations = all \ node.fixed_closed)
        if new_cols empty: break                    # pricing exhausted -> bound is valid
        shared_pool = shared_pool union new_cols
        if current LP objective >= UB - tol: break   # early exit -- can't beat incumbent even
                                                       # before full pricing exhaustion

    node.lp_bound = LP objective
    if node.lp_bound >= UB - tol: continue            # fathom: bound (re-check post-pricing)

    y_hat = LP relaxation's y values
    j_star = most_fractional(y_hat, excluding already-fixed vars)

    if j_star is nothing:                             # y integral at this node
        ip_result = solve_fixed_y_final_ip(y_hat, shared_pool)  # reuse existing machinery
        if ip_result.objective < UB: UB, incumbent = ip_result.objective, ip_result
        continue                                      # fathom: integral, resolved
    else:
        push child(fixed_closed += {j_star})
        push child(fixed_open   += {j_star})
```

## Pruning rules

1. **Bound fathoming** -- `lp_bound >= UB`, checked before pricing, mid-pricing (cheap early
   exit), and after pricing settles.
2. **Infeasibility fathoming** -- free-slot precheck, plus LP infeasibility during pricing.
3. **Integral fathoming** -- once `y` comes out integral, resolve via a final IP over the pool
   (see reuse note below) rather than branching further.

## Reuse from the existing codebase

- Pricing/labeling search: same station-age/station-simple labeling already used for CG, just
  parameterized with `allowed_stations = all \ fixed_closed` instead of a single exact `y_hat`.
- Integral-`y` leaves: `solve_fixed_y_final_ip` is exactly `_solve_fixed_route_covering_by_cg` +
  final IP, already implemented in `benders/y.jl`/`benders/yz.jl` -- a B&P leaf node *is* a
  Benders y-fixed subproblem.
- Column pool growth/dedup: same pattern as `shared_pool` in `benders/y.jl`.

## Open design choices (unresolved, since this wasn't pursued)

- **Node ordering**: best-first (as sketched) gives the tightest live bound and likely fewest
  nodes to prove optimality, but needs the whole frontier in memory and prices every popped node
  fully (expensive -- pricing is already the dominant cost we saw in Benders' master, and here it
  would happen at *every* node, not once per outer iteration). DFS-with-best-bound-tiebreak is
  cheaper per step but gives a much looser live gap.
- **Branching variable**: "most fractional" is the simplest rule; a cheap upgrade is to prefer
  the `y_j` touched by the most pool columns, since that's likely to prune more columns/bound
  faster.
- **Per-node pricing cost** is the central scalability risk -- every node re-runs pricing to
  exhaustion (or until it can prove it can't beat UB). Would need a `max_reprice_rounds`-style cap
  per node, mirroring Benders' existing cap on subproblem repricing.

## Why shelved

The CG-heuristic data (`experiments/zhuzhou_p16_cg_ms45_singlescenario/`) already shows that
pricing with y not fully fixed produces weak columns and a weak LP bound relative to Direct's
true optimum. Branch-and-price nodes near the root have *most* of y still free, so the same
weak-pricing problem would show up there too -- the tree would likely need many nodes (down to
near-complete y fixing) before pricing becomes reliable, at a per-node cost that's already the
bottleneck in the simpler Benders master. Concluded this is a bigger build than Benders for an
uncertain payoff, and not worth pursuing right now.
