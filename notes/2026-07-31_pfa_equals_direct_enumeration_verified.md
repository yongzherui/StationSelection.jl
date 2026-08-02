# Passenger free-assignment CG **equals** direct route-enumeration — VERIFIED

**Date:** 2026-07-31
**Job:** 19399481 (`sbatch_cg_vs_direct_grid.sh`, grid over 5 instance sizes)
**Status:** ✅ CONFIRMED — all cases PASS, no pool-completeness gap.

## The claim we checked

Passenger free-assignment (PFA) column generation and a **direct solve through
route enumeration** yield the **same optimum** — and not merely the same LP
bound, but the same *integer* optimum of the full problem.

This matters because CG certifies the LP bound, while its final MIP is solved
only over the **pool of columns CG happened to generate**. In principle that pool
could miss an integer optimum that the complete column set contains. This note
records that we explicitly closed that gap.

## What "direct solve through route enumeration" means here

`scripts/diag_passenger_cg_vs_direct_full_milp.jl` builds the **complete** column
set — every distinct `(assignment set A, cheapest τ over routes certifying all of
A)`, deduped by the master's column signature — and solves that MILP directly.
The complete set is finite and small because the master only ever distinguishes a
column through its assignment set and its τ (the coverage/linking rows depend only
on which `(p,j,k)` a column carries), so for a given assignment set only the
cheapest route survives. That makes the "everything" MILP enumerable on small
instances.

## Checks performed per case

1. `z_direct <= z_cg_mip + tol` — direct model can only match or beat CG's pool
   MIP (a violation would mean the direct model is wrong).
2. **`z_direct == z_cg_mip`** — the real target: CG's pool contained an integer
   optimum. A strict gap here is the failure mode this script exists to detect.
3. Certified `lp_bound <= z_direct` — the certified LP bound is valid against the
   true integer optimum.
4. `lp_bound == complete-set LP` — the certified bound equals the LP over the
   complete column set.

## Results — every case PASSED

| n / pairs / max_stops | complete-set columns | z_direct = z_cg_mip = lp_bound |
| --------------------- | -------------------- | ------------------------------ |
| 6 / 4 / 3             | 116                  | 1076.945007                    |
| 8 / 4 / 3             | 116                  | 1076.945007                    |
| 10 / 4 / 3            | 266                  | 1062.747841                    |
| **10 / 6 / 4**        | **9432**             | **1147.750256**                |
| 12 / 4 / 3            | 256                  | 603.274988                     |

In **every** case `lp_bound == complete-set LP == z_cg_mip == z_direct`, so the
CG pool was integer-complete and full-problem optimality was in fact provable
without enumeration; the direct solve confirms it. The strongest single case is
**n=10 / 6 pairs / max_stops=4**: 9432 distinct columns in the complete set, and
CG's much smaller generated pool still reached the identical integer optimum.

## How this fits with the other equivalence check (two levels, both closed)

This is the **integer / full-problem** half of the equivalence. The **pricing /
LP** half was verified separately:

- `scripts/diag_passenger_free_assignment_vs_direct.jl` — the PFA label search,
  run to genuine exhaustion, finds the **same minimum-reduced-cost route** as a
  brute-force enumeration of every physical route allowed by the same
  `max_stops` / `max_visits_per_node` limits (brute force starts from every
  station, so the oracle's start-node restriction is also under test). Matched at
  n=10/12 (see `notes/2026-07-30_pfa_scaling_and_cg_throughput_experiments.md`).

Together:

| Level              | Script                                        | Question answered                                            | Verdict |
| ------------------ | --------------------------------------------- | ------------------------------------------------------------ | ------- |
| Pricing oracle (LP)| `diag_passenger_free_assignment_vs_direct.jl` | Does the label search find the true min-rc route?            | ✅ n=10/12 |
| Full problem (IP)  | `diag_passenger_cg_vs_direct_full_milp.jl`    | Does CG's pool contain the true integer optimum vs complete set? | ✅ 5-case grid |

Regression coverage also exists at the LP level:
`test/opt/test_passenger_dual_selection.jl` (LP over the fully enumerated column
set equals ordinary CG's LP bound and the selector's bound) and
`test/opt/test_passenger_free_assignment_pricing.jl` (hand-computed replay unit
tests for the shared route-scoring function).

## Scope / caveat

The empirical proof is necessarily bounded to instance sizes where the complete
column set is enumerable (here up to n=12 / 6 pairs on the synthetic Zhuzhou
instances). The equivalence is a structural property expected to hold generally;
what this note certifies is that it was **checked and held exactly** wherever
brute-force enumeration was tractable.

## Reproduce

```bash
# single case: n_stations n_pairs max_stops  (defaults 6 4 3)
sbatch scripts/sbatch_diag_passenger_cg_vs_direct_full_milp.sh 10 6 4
```

The 5-case grid used here was `sbatch_cg_vs_direct_grid.sh` (loops the cases in
the table above in one job).
