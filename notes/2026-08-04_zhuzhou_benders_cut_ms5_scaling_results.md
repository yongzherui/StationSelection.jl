# Synthetic Zhuzhou `max_stops=5` Benders-cut scaling results

Date: 2026-08-04

## Question and experiment

This experiment compares exhaustive Direct route enumeration against classical
Benders decomposition for synthetic Zhuzhou instances. The common model uses
`NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`, `l=ceil(n/2)`, and
routes of at most five stops. No lifted walking objective, route-weight
schedule, lifted routing lower bound, direct-enumeration guide, or other
auxiliary lower bound/cut is enabled. Endpoint coverage is imposed eagerly in
the master; all recorded feasibility-cut counts are zero.

Axes:

- `n = 10, 15, 20, 25, 30, 35, 40`
- `p = 16, 32`
- seeds `42, 123, 999`
- scenario counts `q = 1, 3`
- for `n <= 20`: Direct plus BendersY/BendersYZ with standard+repricing,
  zero completion, and restricted fixed-pi MW cuts
- for `n > 20`: Direct plus BendersYZ restricted MW only

There are 348 tasks. Tasks used 16 GB. The wall limit was 90 minutes for
`n <= 20` and three hours for `n > 20`. Slurm arrays were `19606758`,
`19607567`, and `19607568`.

## What was recorded

Every Benders iteration log contains the current master lower bound and best
incumbent objective (upper bound), absolute and relative outer gaps, cuts added
that iteration, cumulative optimality and feasibility cuts, master/CG/
subproblem time, inner-CG iterations, generated pool size, the selected-`y`
signature, and MW/repricing diagnostics. Thus the saved files support full
bound-versus-iteration and cut-versus-iteration plots, including trajectories
for jobs later killed by Slurm.

## Overall completion

| `n` | Tasks | Returned `ok` result | Hit Benders 500-iteration cap | Slurm timeout | 16 GB OOM |
|---:|---:|---:|---:|---:|---:|
| 10 | 84 | 83 | 0 | 1 | 0 |
| 15 | 84 | 58 | 13 | 12 | 1 |
| 20 | 84 | 23 | 22 | 37 | 2 |
| 25 | 24 | 5 | 3 | 9 | 7 |
| 30 | 24 | 2 | 2 | 9 | 11 |
| 35 | 24 | 0 | 3 | 10 | 11 |
| 40 | 24 | 0 | 2 | 10 | 12 |
| **Total** | **348** | **171** | **45** | **88** | **44** |

The scheduler marked 216 tasks completed, but 45 of those wrote an error row
because Benders reached `max_iterations=500`. Treating those as successful
would materially overstate coverage; the table separates them.

The failure modes are method-specific. Direct enumeration is primarily
memory-limited: none of the Direct tasks at `n=35,40` succeeded (23/24 exceeded
16 GB and the remaining task timed out), and every three-scenario Direct task
at `n=30` exceeded 16 GB. BendersYZ MW stays well below that memory level in
most runs but increasingly exhausts the wall/iteration budget. No
three-scenario BendersYZ MW case completed successfully beyond `n=25`, and the
sole scheduler-completed `n=25,q=3` run hit the 500-iteration cap.

## Small-instance cut comparison

The following are medians over successful returned runs. Times are solver-call
wall times; Slurm elapsed time is larger because each task prepares a local
Julia depot. Multi-scenario cut totals are expected to be roughly three times
the iteration count because the experiment uses multicut, one recourse block
per scenario.

| `n,q` | Method | Successful | Median time (s) | Median iterations | Median optimality cuts |
|---|---|---:|---:|---:|---:|
| 10,1 | BendersYZ MW | 6/6 | 21 | 31.5 | 30.5 |
| 10,1 | BendersYZ zero | 6/6 | 24 | 33.5 | 32.5 |
| 10,1 | BendersYZ standard+reprice | 6/6 | 28 | 32.0 | 31.0 |
| 10,1 | BendersY MW | 6/6 | 23 | 33.5 | 32.5 |
| 10,1 | BendersY zero | 6/6 | 25 | 38.0 | 37.0 |
| 10,1 | BendersY standard+reprice | 6/6 | 38 | 32.5 | 31.5 |
| 10,3 | BendersYZ MW | 6/6 | 29 | 28.0 | 81.0 |
| 10,3 | BendersYZ zero | 6/6 | 30 | 30.5 | 88.5 |
| 10,3 | BendersYZ standard+reprice | 6/6 | 67 | 30.5 | 87.0 |
| 10,3 | BendersY MW | 6/6 | 31 | 33.5 | 95.0 |
| 10,3 | BendersY zero | 6/6 | 32 | 37.0 | 108.0 |
| 10,3 | BendersY standard+reprice | 5/6 | 128 | 33.0 | 93.0 |
| 15,1 | BendersYZ MW | 6/6 | 286 | 145.0 | 144.0 |
| 15,1 | BendersYZ zero | 6/6 | 459 | 208.5 | 207.5 |
| 15,1 | BendersYZ standard+reprice | 6/6 | 702 | 174.5 | 173.5 |
| 15,3 | BendersYZ MW | 6/6 | 823 | 175.5 | 521.5 |
| 15,3 | BendersYZ zero | 6/6 | 1,443 | 235.0 | 698.5 |
| 15,3 | BendersYZ standard+reprice | 3/6 | 807 | 122.0 | 360.0 |

Restricted MW with BendersYZ is the most consistently fast Benders
configuration at `n=10,15`. Zero completion usually takes more iterations and
cuts. Standard+repricing sometimes uses a similar number of cuts but is much
slower, especially for three scenarios, because certification repeatedly
invokes pricing. BendersY deteriorates sooner: at `n=15`, half of its
single-scenario MW/zero runs hit 500 iterations, and its three-scenario MW and
zero variants returned only 3/6 and 2/6 successful results respectively.

## Direct versus BendersYZ MW scaling

Medians below include only successful results and therefore become
increasingly survivor-biased at larger `n`.

| `n,q` | Direct success | Direct median (s) | YZ-MW success | YZ-MW median (s) | YZ-MW median cuts |
|---|---:|---:|---:|---:|---:|
| 10,1 | 6/6 | 12 | 6/6 | 21 | 30.5 |
| 10,3 | 6/6 | 22 | 6/6 | 29 | 81.0 |
| 15,1 | 6/6 | 80 | 6/6 | 286 | 144.0 |
| 15,3 | 5/6 | 458 | 6/6 | 823 | 521.5 |
| 20,1 | 6/6 | 605 | 3/6 | 398 | 175.0 |
| 20,3 | 1/6 | 2,567 | 3/6 | 3,390 | 766.0 |
| 25,1 | 4/6 | 2,105 | 1/6 | 1,117 | 210.0 |
| 25,3 | 0/6 | -- | 0/6 | -- | -- |
| 30,1 | 1/6 | 3,507 | 1/6 | 3,850 | 412.0 |
| 30,3 | 0/6 | -- | 0/6 | -- | -- |
| 35,1 | 0/6 | -- | 0/6 | -- | -- |
| 35,3 | 0/6 | -- | 0/6 | -- | -- |
| 40,1 | 0/6 | -- | 0/6 | -- | -- |
| 40,3 | 0/6 | -- | 0/6 | -- | -- |

Direct is decisively faster through `n=15`, but its route universe becomes a
memory problem. At `n=20`, Direct remains reliable for one scenario whereas
BendersYZ MW completes only half the cases; with three scenarios both methods
are already unreliable. The few successful `n>=25` timings should not be read
as population medians—the censoring rate is too high.

## Bound closure and objective agreement

All `n=10` methods agree with Direct to floating-point precision, and every
successful Benders run closes its recorded outer gap. Across all sizes,
however, only 115 of 136 returned-`ok` Benders results have
`final_outer_gap <= 1e-4`. Seven returned results have a gap above 3%, with the
largest 6.36%. These are concentrated in `n=15,p=32` BendersYZ cases. The
solver currently warns but returns an incumbent when no violated LP cut remains
even if the LP/IP outer gap is not closed, so `status=ok` is not by itself a
proof of global bound closure.

Where both a successful Direct result and Benders result exist, objectives
almost always agree to floating-point precision. Two exceptions deserve
attention:

- `n=15,p=32,seed=123,q=1`, BendersYZ MW is 1.60% above Direct, while
  BendersYZ standard and zero completion match Direct but retain a 4.86%
  recorded outer gap.
- one paired `n=20,q=3` BendersYZ-zero result differs from Direct by 0.0108%.

The first case confirms that unclosed outer-gap warnings must not be discarded
in downstream analysis. For certified comparisons, filter to tightly closed
gaps (for example `final_outer_gap <= 1e-4`) or validate against Direct where
available.

## Conclusions

1. BendersYZ restricted MW is the best of the tested decomposition/cut
   combinations on `n=10,15`: it generally uses the fewest cuts and least
   Benders time.
2. The scaling bottlenecks differ: Direct exhausts 16 GB while BendersYZ MW
   exhausts time or the 500-iteration cap.
3. Three scenarios substantially increase cut work and push the practical
   limit down: the benchmark is already heavily censored at `n=20,q=3`.
4. The current 16 GB / 90-minute / three-hour budget does not yield a complete
   benchmark beyond `n=20`. The failures are themselves useful scaling data,
   but runtime averages over only successful large cases are misleading.
5. Future plots should show timeout/OOM censoring explicitly and plot the saved
   lower/upper-bound trajectories, not only final runtime and cut totals.

Raw results are under
`experiments/zhuzhou_benders_cut_scaling_ms5/results/`; per-iteration traces are
under `experiments/zhuzhou_benders_cut_scaling_ms5/iters/`.
