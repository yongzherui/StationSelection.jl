# Study 6 CG livelock: stale-`tau` columns can never enter the master

**Date:** 2026-08-25
**Status:** root-caused, reproduced, **FIXED and verified** 2026-08-25
**Affects:** `AggregateODRouteBaseFormulation` under `CGSolver` (Study 6's `cg_exact` arm)
**only**. `AggregateODRouteJointRoutingAssignmentFormulation` is **immune** — it already
has the guard Base is missing (see "Joint gets this right" below), so Studies 1-3 and 5,
which all run Joint, are unaffected.

## Symptom

5 of Study 6's 30 `cg_exact` runs stop at `cg_iterations == 1000` (the `max_iterations`
cap) and report `status="incomplete"`. The iteration counts are perfectly bimodal —
**24 runs finish in 2 iterations, one in 3, five hit 1000, nothing in between** — which
rules out ordinary tailing-off.

Affected cells: `n10 seed44` (31.9 s), `n15 seed47` (242.8 s), `n15 seed48` (227.6 s),
`n20 seed45` (1717.1 s), `n20 seed50` (1405.7 s).

## Reproduction

`benchmarks/diagnostics/study6_tailoff_repro.jl` (whole run) and
`study6_tailoff_instrumented.jl` (re-implements the CG loop and records what
`add_columns!` *returns*, which the real loop discards). Cheapest reproducer is
`n=10 seed=44`, ~35 s:

```
iter |    master_obj |  priced | ADDED | theta_pool | priced column
   1 |    52377.9013 |    5002 |  5002 |  142->5144 | ...
   2 |    20910.6106 |       1 |     0 | 5144->5144 | [(1,3),(3,9),(1,9),(3,4),(1,4)]@s3
   3 |    20910.6106 |       1 |     0 | 5144->5144 | [(1,3),(3,9),(1,9),(3,4),(1,4)]@s3
 ... identical through iteration 1000 ...
```

The same column is re-priced every iteration, `add_columns!` adds **0**, the θ pool is
frozen at 5144, and the master objective has spread `0.0` across iterations 2..1000.

## Root cause: two inconsistent notions of "already have this column"

`_aggregate_od_route_column_signature` is the **OD-pair set only** — `tau` is not part of
it (`label_setting/route_covering/exact/labels.jl:365`):

```julia
_aggregate_od_route_column_signature(pairs) = Tuple(sort!(collect(pairs)))
```

The **pricer** treats a same-signature column with a strictly better `tau` as novel and
worth returning (`label_setting/round.jl:203-206, 233-234`):

```julia
best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
...
candidate.tau < get(best_pool_tau, candidate.signature, Inf) - 1e-9 || return false
```

The **master** dedups on the signature alone and throws the better `tau` away
(`constraints/aggregate_od_route/base/route_activation.jl:89-103`): the signature maps to
the already-registered `column_id` (which keeps its *old, worse* `tau`), `(column_id, s)`
is already in `theta`, so the column is `:skipped`.

So the pricer says "here is a strictly cheaper route over the same OD pairs", the master
says "I already have that signature", the pool never changes, the duals never change, and
the pricer finds the same column again — forever.

This matters for the objective because `tau` *is* the cost
(`constraints/aggregate_od_route/core.jl:15`):

```julia
coefficient = route_regularization_weight * (column.tau + repositioning_time)
```

A column pinned at a stale, worse `tau` makes the master pay more than it should.

## Secondary defect: the loop cannot detect this

`solvers/cg_solver.jl:172` calls `add_columns!` and **discards its return value**, and
`cumulative_columns_added` is incremented by `length(new_columns)` (what pricing
*returned*) rather than what was actually added. The loop breaks only on
`isempty(new_columns)`, so a non-empty-but-fully-skipped round spins to `max_iterations`.

Note that adding `added == 0 && break` alone would be **wrong as a convergence
certificate** — the pricer genuinely found an improving column, so the run has *not*
converged. Such a break must leave `converged=false`, or the stale-`tau` cause must be
fixed so the situation cannot arise.

## Suspected link to the Study 6 objective discrepancy

`zhuzhou_n15_p16_s3_seed45_ms4` reports CG **33824.81** vs enumeration **33756.98** — a
0.2 % gap (67.8 absolute) with *both* arms claiming certified/optimal. CG is the worse
(higher-cost) answer, which is exactly what being stuck with stale, worse-`tau` columns
would produce. That run terminated normally in 2 iterations, so if this is the same bug
it degrades answers **silently**, without any iteration-cap tell. The other three Study 6
mismatches are ~1e-7 relative (Gurobi MIP-gap noise) and unrelated. **Unverified.**

## Joint gets this right; Base is the odd one out

`add_joint_routing_assignment_column!`
(`constraints/aggregate_od_route/joint_routing_assignment/routing_and_assignment.jl:76-79`)
makes the skip **conditional on `tau`**:

```julia
existing_id = get(signatures, signature, nothing)
if !isnothing(existing_id)
    columns[existing_id].tau <= column.tau + 1e-9 && return theta[existing_id], :skipped
end
```

A strictly-better-`tau` column is *not* skipped: Joint mints a new `column.id`, adds a new
`theta`, and repoints `signatures[signature]` at it. Its own docstring says the skip
happens only when an identical signature is already pooled "at no greater `tau`". Base's
line (`base/route_activation.jl:103`) drops that clause entirely:

```julia
haskey(theta, (column_id, s)) && return theta[(column_id, s)], :skipped
```

So this is a missing guard in Base, not a design disagreement.

### On the passenger index

The two signatures differ in content, and both are correct for their formulation:

- **Joint** keys on `column.assignments`, a sorted tuple of **`(p, j, k)` triples** — the
  passenger/demand-group index `p` **is** part of the signature
  (`label_setting/joint_routing_assignment/exact/labels.jl:589`). It has to be: Joint's
  θ columns carry the OD assignment directly, so two columns over the same stations
  serving different passengers are genuinely different columns.
- **Base** keys on `column.od_pairs`, a sorted tuple of **`(j, k)` station pairs**, with no
  passenger. That is right for Base, because assignment is a *separate* variable there
  (`x[s,p,j,k]`, decoupled from routing) — a Base θ column means only "a route visiting
  these station pairs", and carries no passenger to key on.

So the Base bug is **not** a missing passenger in the signature; it is the missing `tau`
comparison at the skip.

## Fix (not yet implemented)

1. **Mirror Joint** in `add_aggregate_od_route_base_column!`: when the incoming column's
   `tau` is strictly better than the registered one's, mint a new `column_id` (updating
   `signatures[signature]`) instead of reusing the stale id and skipping. This is the
   proven shape already running in the Joint path.
2. **Guard the loop** regardless (`solvers/cg_solver.jl:172`): capture `add_columns!`'s
   return, break on 0 with `converged=false`, and count `cumulative_columns_added` from
   it rather than from `length(new_columns)`. Necessary for robustness — it turns any
   future livelock into an honest non-convergence instead of 1000 wasted iterations — but
   not sufficient on its own.

Do both, then re-run Study 6's five cells and re-check `n15_seed45`'s objective against
enumeration.


## Fix applied and verified (2026-08-25)

1. `constraints/aggregate_od_route/base/route_activation.jl` — a rediscovered signature
   whose `tau` is strictly better now falls through to mint a fresh `column_id` and
   repoint `signatures`, instead of reusing the stale id. Mirrors Joint.
2. `solvers/cg_solver.jl` — `add_columns!`'s return is captured as a new
   `columns_accepted` iteration-log field, `cumulative_columns_added` now accumulates it
   rather than `length(new_columns)`, and the loop breaks when zero columns are accepted,
   leaving `converged=false`.

Verified on the `n=10 seed=44` reproducer:

| | before | after |
| --- | --- | --- |
| iterations | 1000 | **2** |
| wall | 31.9 s | **9.3 s** |
| converged / exhausted | false / false | **true / true** |
| master objective | 20910.6106 | **20834.2906** |

The objective improved by **76.32 (0.37 %)**, confirming that the stale-`tau` column was
inflating the cost — i.e. the bug was degrading answers, not merely wasting iterations.
That is direct support for the `n15_seed45` discrepancy hypothesis above, though that
specific cell has not been re-checked yet.

Package tests: **1182 passed, 0 failed, 13 errored**. All 13 errors are stale tests
calling pricer names removed by commit `d337054` (the `route_covering`/
`joint_routing_assignment` rename); they fail at test-file load, before any solver code
runs, and are unrelated to this fix. Adding the missing `Printf` entry to
`test/Project.toml` (a pre-existing gap — `scripts/generate_zhuzhou_instance.jl` uses it)
unblocked 216 previously-skipped tests, 966 -> 1182.

## Study 6 re-run

Study 6 was simultaneously re-scoped to its intended comparison — Joint+CG vs Base+Direct
— so it no longer exercises Base+CG at all. Pre-correction runs archived at
`benchmarks/experiments/2026-08-25_study6_basecg_SUPERSEDED/`. Re-run submitted as array
`21250418` (60 jobs, 2 h).
