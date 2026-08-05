# Zhuzhou common-OD MCF incremental comparison

## Experiment

This note reports the completed n=10 smoke/benchmark grid for adding an MCF lower bound and
fractional MCF cuts to the restricted-MW BendersYZ methods. Every method uses the same synthetic
Zhuzhou instances, `p in {16,32}`, seeds `{42,123,999}`, one or three scenarios, five selected
stations, routes of at most five stops, `NearestOpenPolicy(:big_m_nearest)`, no feasibility cuts,
and a 16 GB memory limit.

Here `q` denotes the number of demand scenarios contained in one optimization instance. Thus
`q=1` is a single request scenario, while `q=3` is a three-scenario stochastic/multiscenario
instance. Under multicut Benders, `q=3` has three recourse blocks and can receive one routing cut
per scenario in an outer iteration. It does **not** mean three independent repetitions; the three
seeds are the independent benchmark repetitions.

All result rows are therefore indexed by `(n,p,q)` and average the three seeds. A pooled `12/12`
statement at `n=10` means four separate `(p,q)` cells, each with `3/3` success; it is not a
12-seed cell.

The five matched configurations are:

1. classical outer-loop BendersYZ restricted-MW, without MCF;
2. classical BendersYZ with a permanent common-OD MCF lower bound in the master;
3. Branch-and-Benders restricted-MW, without MCF;
4. Branch-and-Benders with the permanent common-OD MCF lower bound; and
5. Branch-and-Benders with the common-OD bound plus scenario-specific MCF MW user cuts at
   non-root fractional `(y,z)` callback points.

## Current coverage map

The experiment families should not be conflated. The coverage available as of 2026-08-05 is:

| Experiment family | n=10 | n=15 | n=20 | n=25 | n=30 | n=35 | n=40 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Original classical Benders/direct/cut scaling | run | run | run | run | run | run | run |
| Branch-and-Benders, restricted MW, no MCF | run | run | run | — | — | — | — |
| Classical Benders + common-OD MCF | **run** | not launched | not launched | — | not planned | — | — |
| B&B + common-OD MCF | **run** | not launched | not launched | — | not planned | — | — |
| B&B + common OD + fractional scenario-MCF cuts | **run** | not launched | not launched | — | not planned | — | — |

Thus the MCF comparison in this note is strictly an n=10 result. The n=15 and n=20 MCF rows have
been generated in the job manifests but have not been submitted. No n=30 MCF jobs were generated
or launched. Existing n=15/n=30 rows belong to the older no-MCF/direct scaling experiments and
cannot answer whether MCF improves scaling at those sizes.

The natural next coverage step is n=15, then n=20 if n=15 remains objective-consistent and within
the memory/time limits. Jumping directly to n=30 would skip the sizes where the no-MCF methods
first developed objective disagreement and severe convergence failures.

Solver-reported wall time is used below. It excludes Julia startup and copying the Julia depot,
which are present in Slurm elapsed time but are not part of the optimization algorithm.

## Completion, correctness, and memory

- All 36 new MCF jobs completed successfully: 12 classical-common-OD runs and 24
  Branch-and-Benders runs.
- All five methods agree with the classical no-MCF objective on all 12 instances to within
  `1.46e-11` absolute error. Thus the n=10 MCF variants pass the objective cross-check.
- Peak RSS was about 10.7 GiB, below the 16 GB allocation.
- The common-OD set contains all ODs in one-scenario cases. In the three-scenario cases it averages
  7 common ODs for p=16 and 19 for p=32.

### Relation to the previous 16 GB OOM cases

The absence of OOMs here applies only to the completed n=10 MCF grid. The earlier full max-stops-5
Zhuzhou scaling experiment had 44 Slurm OOM terminations under the same 16 GB cap:

| n | previous 16 GB OOMs | total tasks in previous grid |
|---:|---:|---:|
| 10 | 0 | 84 |
| 15 | 1 | 84 |
| 20 | 2 | 84 |
| 25 | 7 | 24 |
| 30 | 11 | 24 |
| 35 | 11 | 24 |
| 40 | 12 | 24 |
| **Total** | **44** | **348** |

Those failures were strongly method- and size-dependent rather than evidence that every Benders
run needs more than 16 GB. Direct route enumeration was the main memory-limited method: 23 of its
24 n=35/40 tasks exceeded 16 GB, and every n=30, q=3 Direct task exceeded 16 GB. Classical
BendersYZ restricted-MW generally stayed below the cap and more often failed through wall-time or
the 500-iteration limit. The current MCF result therefore supports only the narrower statement
that n=10 common-OD and fractional-MCF formulations fit in 16 GB (observed peak 10.7 GiB). It does
not establish that n=15/20 MCF runs, especially q=3, will avoid the earlier scaling OOM boundary.

## Runtime and cut results

Each row is the mean over the three seeds. `Speedup` is reported separately below because the
geometric mean of paired ratios is preferable to the ratio of arithmetic-mean wall times.

| Method | p | scenarios | wall (s) | exact cuts | callbacks | exact y evaluations | MCF separations | MCF cuts | MCF time (s) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Classical baseline | 16 | 1 | 19.2 | 29.3 | — | — | 0 | 0 | 0.0 |
| Classical + common OD | 16 | 1 | 23.2 | 27.0 | — | — | 0 | 0 | 0.0 |
| B&B baseline | 16 | 1 | 26.7 | 29.7 | 35.0 | 29.7 | 0 | 0 | 0.0 |
| B&B + common OD | 16 | 1 | 31.8 | 26.3 | 29.7 | 26.3 | 0 | 0 | 0.0 |
| B&B + common OD + fractional MCF | 16 | 1 | 33.9 | 26.3 | 29.7 | 26.3 | 8 | 0 | 2.6 |
| Classical baseline | 16 | 3 | 25.4 | 81.7 | — | — | 0 | 0 | 0.0 |
| Classical + common OD | 16 | 3 | 28.7 | 82.0 | — | — | 0 | 0 | 0.0 |
| B&B baseline | 16 | 3 | 34.3 | 80.7 | 32.3 | 27.7 | 0 | 0 | 0.0 |
| B&B + common OD | 16 | 3 | 44.5 | 87.3 | 35.0 | 31.3 | 0 | 0 | 0.0 |
| B&B + common OD + fractional MCF | 16 | 3 | 42.9 | 74.3 | 29.7 | 26.0 | 8 | 17.0 | 4.8 |
| Classical baseline | 32 | 1 | 26.3 | 29.3 | — | — | 0 | 0 | 0.0 |
| Classical + common OD | 32 | 1 | 43.8 | 30.7 | — | — | 0 | 0 | 0.0 |
| B&B baseline | 32 | 1 | 34.9 | 29.0 | 32.0 | 29.0 | 0 | 0 | 0.0 |
| B&B + common OD | 32 | 1 | 36.9 | 31.7 | 36.0 | 31.7 | 0 | 0 | 0.0 |
| B&B + common OD + fractional MCF | 32 | 1 | 42.5 | 32.0 | 36.0 | 32.0 | 8 | 0 | 4.2 |
| Classical baseline | 32 | 3 | 30.2 | 85.0 | — | — | 0 | 0 | 0.0 |
| Classical + common OD | 32 | 3 | 39.3 | 89.0 | — | — | 0 | 0 | 0.0 |
| B&B baseline | 32 | 3 | 55.6 | 91.0 | 35.0 | 30.3 | 0 | 0 | 0.0 |
| B&B + common OD | 32 | 3 | 54.6 | 82.3 | 35.3 | 28.3 | 0 | 0 | 0.0 |
| B&B + common OD + fractional MCF | 32 | 3 | 64.2 | 86.7 | 35.3 | 29.7 | 8 | 9.7 | 10.6 |

Across all 12 instances:

| Method | mean wall (s) | mean exact cuts | total fractional MCF cuts | total MCF-cut time (s) |
|---|---:|---:|---:|---:|
| Classical baseline | 25.3 | 56.3 | 0 | 0.0 |
| Classical + common OD | 33.8 | 57.2 | 0 | 0.0 |
| B&B baseline | 37.9 | 57.6 | 0 | 0.0 |
| B&B + common OD | 41.9 | 56.9 | 0 | 0.0 |
| B&B + common OD + fractional MCF | 45.9 | 54.8 | 80 | 66.6 |

## Paired incremental effects

Here `speedup = old wall / new wall`; a value below one means the added feature is slower.

| Increment | geometric-mean speedup | improved instances |
|---|---:|---:|
| Classical baseline -> common-OD bound | 0.776x | 1/12 |
| B&B baseline -> common-OD bound | 0.891x | 3/12 |
| B&B common OD -> add fractional MCF cuts | 0.915x | 5/12 |
| B&B baseline -> common OD + fractional MCF | 0.815x | 1/12 |

The fractional separator was called eight times in every enabled run. It generated no violated
cuts in any single-scenario case, while the three-scenario cases generated 80 cuts in total. For
p=16 and three scenarios, those cuts reduced mean exact cuts from 87.3 to 74.3 and exact-y
evaluations from 31.3 to 26.0, producing a small mean wall-time improvement over common OD alone
(44.5 to 42.9 seconds). For p=32, three scenarios, the MCF cuts did not reduce the work enough to
pay for their separation cost; wall time rose from 54.6 to 64.2 seconds.

## Conclusion

At n=10, both MCF mechanisms are objective-correct but do not yet provide an overall runtime
improvement. The common-OD formulation adds enough master overhead to slow both classical and
Branch-and-Benders on average. Fractional scenario-MCF cuts show the intended reduction in exact
routing work for p=16 multi-scenario instances, but the eight unconditional separation attempts
are wasted in every single-scenario case and their cost outweighs the benefit at p=32.

Before launching n=15/20, the most defensible incremental refinement is to disable fractional MCF
separation for single-scenario problems and consider a violation/priority gate rather than always
using all eight separation rounds. If the existing configuration is kept unchanged for scaling,
the expectation should be treated as a test of whether the stronger bound amortizes only at larger
n—not as an improvement already demonstrated at n=10.

## Reproducibility

- Branch-and-Benders MCF array: `19652993`
- Classical common-OD array: `19653180`
- New Branch-and-Benders results: `experiments/zhuzhou_branch_benders_mcf_increment_ms5/`
- New classical results: `experiments/zhuzhou_classical_benders_common_od_ms5/`
- Baselines: `experiments/zhuzhou_benders_cut_scaling_ms5/` and
  `experiments/zhuzhou_branch_benders_yz_mw_no_mcf_ms5/`
- Aggregation: `scripts/analyze_zhuzhou_mcf_increment_n10.jl`

## Completed classical scaling extension: n=15 and n=20

The classical common-OD array `19668469` has now drained. All twelve `n=15`
tasks and all six `n=20, p=16` tasks completed. All six `n=20, p=32` tasks hit
the three-hour limit without producing final result rows. There were no OOM
failures; observed peak memory remained below the 16 GB request (maximum about
`11.29 GiB`).

Across the 30 matched completed baseline/common-OD cases (`n=10`, `n=15`, and
`n=20,p=16`), common-OD MCF closed the final outer gap in the same `22/30` cases
as baseline. It was faster in only `3/30`, with geometric-mean speedup `0.627x`
(about `1.60x` slower overall), and matched the baseline incumbent in `28/30`.

| n | p | scenarios | baseline wall (s) | common-OD wall (s) | speedup | baseline iterations | common-OD iterations | baseline cuts | common-OD cuts | max common-OD gap |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 | 16 | 1 | 19.2 | 23.2 | 0.844x | 30.3 | 28.0 | 29.3 | 27.0 | 0.000% |
| 10 | 16 | 3 | 25.4 | 28.7 | 0.894x | 28.7 | 28.7 | 81.7 | 82.0 | 0.000% |
| 10 | 32 | 1 | 26.3 | 43.8 | 0.623x | 30.3 | 31.7 | 29.3 | 30.7 | 0.000% |
| 10 | 32 | 3 | 30.2 | 39.3 | 0.771x | 29.3 | 30.7 | 85.0 | 89.0 | 0.000% |
| 15 | 16 | 1 | 80.7 | 199.1 | 0.434x | 101.0 | 77.7 | 100.0 | 76.7 | 0.000% |
| 15 | 16 | 3 | 200.3 | 289.8 | 0.712x | 106.7 | 109.3 | 314.3 | 321.0 | 1.696% |
| 15 | 32 | 1 | 816.1 | 3,966.9 | 0.224x | 257.3 | 267.3 | 256.3 | 266.3 | 4.858% |
| 15 | 32 | 3 | 1,724.0 | 3,770.9 | 0.445x | 244.0 | 243.0 | 728.3 | 725.0 | 5.935% |
| 20 | 16 | 1 | 480.7 | 715.7 | 0.872x | 217.3 | 111.3 | 216.3 | 110.3 | 0.567% |
| 20 | 16 | 3 | 3,383.1 | 3,844.9 | 0.961x | 270.7 | 253.7 | 798.0 | 751.7 | 0.692% |

`Speedup = baseline wall / common-OD wall`; values below one favor the baseline.
The bound substantially reduced iterations for `n=20,p=16,q=1`, but its larger
master made wall time worse. It did not rescue `n=20,p=32` within three hours.

The two incumbent differences occur in non-converged `n=15,p=32` cases. For
`seed123,q=1`, common-OD finds `26,619.9372`, exactly matching Direct enumeration
and improving the baseline incumbent `27,046.1601`. For `seed42,q=3`, common-OD
returns `84,856.9771`, which is `7.2` above the baseline/Direct incumbent; neither
run closed its gap, so this is not a changed mathematical optimum.

Scaling aggregation: `scripts/analyze_zhuzhou_classical_common_od_vs_baseline.jl`.
