# Branch-and-Benders versus direct BendersYZ (restricted MW, no MCF)

## Scope

This note compares the callback-based `BranchAndBendersSolver` with the classical
outer-loop `BendersYZ` implementation on the synthetic Zhuzhou instances for
`n = 10, 15, 20`, `p = 16, 32`, one and three scenarios, and seeds 42, 123, and 999.
Both methods use restricted MW fixed-`pi` cuts, routes of at most five stops, the
`NearestOpenPolicy`, no feasibility cuts, and no MCF or other custom lower bound.
Each Slurm task had 16 GB RAM. The comparison uses solver-reported wall time rather
than Slurm elapsed time.

## Completion and resource use

- Branch-and-Benders completed 30 of 36 tasks. All six `n=20, p=32` tasks reached
  the three-hour Slurm limit and produced no final result row.
- The other 30 tasks exited cleanly. Their maximum observed RSS was 10.31 GB, so
  the 16 GB memory budget was adequate.
- Classical Benders has matching result rows for those same 30 tasks. Its
  `n=20, p=32` grid was also unsuccessful/incomplete, so neither method has a
  usable matched comparison for that group.

## Objective agreement

The most important result is that the methods are **not yet equivalent**:

- 22 of 30 completed matched cases agree within `1e-6` absolute objective value.
- All 12 `n=10` cases agree.
- Eight cases at `n=15` or `n=20` disagree; the largest absolute difference is
  5,066.37 and the largest relative difference is 6.36%.
- Classical Benders has a final outer gap at most `1e-6` in 22 of 30 rows;
  Branch-and-Benders reports a gap at most `1e-6` in 27 of 30 rows. Therefore,
  the `OPTIMAL` termination label alone is not a sufficient correctness check.

Direct enumeration was available for six of the eight disagreements. It agrees
with the classical Benders incumbent (to numerical tolerance) in five cases. In
the remaining case (`n15_p32_s123_q1`), direct enumeration returns 26,619.94,
between classical Benders (27,046.16) and Branch-and-Benders (25,326.78). This is
strong evidence that the callback method's current “exact” candidate evaluation
or certification is underestimating routing cost on some larger instances. Its
lower objective should not be interpreted as an improvement.

| Instance | Classical Benders | Branch-and-Benders | Direct enumeration | Classical gap | B&B gap |
|---|---:|---:|---:|---:|---:|
| n15 p16 seed123 q3 | 54,866.29 | 53,935.67 | 54,866.29 | 1.696% | 0.000% |
| n15 p32 seed123 q1 | 27,046.16 | 25,326.78 | 26,619.94 | 6.357% | 0.000% |
| n15 p32 seed42 q3 | 84,849.78 | 84,514.08 | 84,849.78 | 0.396% | 0.001% |
| n15 p32 seed999 q3 | 81,834.43 | 77,772.15 | 81,834.43 | 4.964% | 0.000% |
| n20 p16 seed123 q3 | 54,473.90 | 54,119.72 | 54,473.90 | 0.650% | 0.000% |
| n20 p16 seed999 q1 | 14,709.94 | 14,626.50 | 14,709.94 | 0.567% | 0.000% |
| n15 p32 seed123 q3 | 85,360.70 | 80,294.33 | unavailable | 5.935% | 0.000% |
| n20 p16 seed42 q3 | 54,350.84 | 53,975.00 | unavailable | 0.692% | 0.000% |

## Wall time and cuts

The table below averages the three seeds. `Speedup = Benders wall time / B&B wall
time`; values below one mean Branch-and-Benders is slower. Groups containing
objective disagreements remain displayed but must not be treated as fair
performance comparisons.

| n | p | scenarios | Benders mean (s) | B&B mean (s) | geom. speedup | Benders cuts | B&B cuts | max objective difference |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 | 16 | 1 | 19.2 | 26.7 | 0.733x | 29.3 | 29.7 | 0.00 |
| 10 | 16 | 3 | 25.4 | 34.3 | 0.747x | 81.7 | 80.7 | 0.00 |
| 10 | 32 | 1 | 26.3 | 34.9 | 0.756x | 29.3 | 29.0 | 0.00 |
| 10 | 32 | 3 | 30.2 | 55.6 | 0.551x | 85.0 | 91.0 | 0.00 |
| 15 | 16 | 1 | 80.7 | 151.3 | 0.533x | 100.0 | 131.0 | 0.00 |
| 15 | 16 | 3 | 200.3 | 621.5 | 0.347x | 314.3 | 427.3 | 930.62 |
| 15 | 32 | 1 | 816.1 | 1,212.1 | 0.696x | 256.3 | 367.7 | 1,719.38 |
| 15 | 32 | 3 | 1,724.0 | 3,683.6 | 0.459x | 728.3 | 1,001.7 | 5,066.37 |
| 20 | 16 | 1 | 480.7 | 401.5 | 1.128x | 216.3 | 226.7 | 83.44 |
| 20 | 16 | 3 | 3,383.1 | 2,499.6 | 1.344x | 798.0 | 962.0 | 375.84 |
| 20 | 32 | 1/3 | no matched completed grid | timeout | — | — | — | — |

Across all 30 completed pairs, Branch-and-Benders is faster in 4 cases and has a
geometric-mean speedup of 0.678x (equivalently, it takes about 1.48x as long).
Restricting the calculation to the 22 objective-matching cases, it is faster in
2 cases and its geometric-mean speedup is 0.681x (about 1.47x as long). The apparent
advantage in the `n=20, p=16` averages is partly associated with nonmatching
objectives and should not be used as evidence of an algorithmic speedup.

## Conclusion

This incremental Branch-and-Benders configuration is not an improvement over
classical BendersYZ yet. It is slower on most validated cases, submits more cuts
as size grows, times out for every `n=20, p=32` case, and—most importantly—returns
an objective inconsistent with classical Benders and direct enumeration on some
larger cases. The next step should be a correctness audit of callback candidate
evaluation and lower-bound certification, beginning with `n15_p16_s123_q3`, before
using Branch-and-Benders runtime numbers in the main scaling comparison.

## Reproducibility

- Branch-and-Benders Slurm arrays: `19643356`, `19643357`
- Classical results: `experiments/zhuzhou_benders_cut_scaling_ms5/results/`
- Branch-and-Benders results: `experiments/zhuzhou_branch_benders_yz_mw_no_mcf_ms5/results/`
- Aggregation script: `scripts/analyze_branch_benders_vs_benders.jl`
