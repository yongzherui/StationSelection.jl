# Classical BendersYZ lifted-walking comparison

## Scope

This experiment isolates `lifted_walking_objective=true` in classical outer-loop BendersYZ.
The comparison uses the same restricted-MW cuts, max-stops 5, `big_m_nearest` linking, p=16/32,
q=1/3, and seeds 42/123/999 as the existing baseline. No MCF lower bound, route-weight schedule,
direct-enumeration guide, or other optional strengthening is enabled. The new solver ceiling is
2,000 Benders iterations; none of these n=10/15 lifted runs approached that ceiling.

Lifted walking moves the exact walking-cost expression into the master. Routing cuts and the inner
subproblem then operate only on routing cost, in unweighted units, while the master applies the
route regularization weight.

Slurm arrays `19668958` (n=10) and `19668959` (n=15) completed all 24 tasks successfully under the
16 GB memory cap.

## Overall result

- 21 of 24 lifted runs reproduce the old baseline objective within `1e-6` absolute tolerance.
- All 12 n=10 objectives agree to floating-point precision.
- Three n=15,p=32 objectives differ; these are listed separately below.
- The baseline and lifted formulations each close the recorded outer gap in 19 of 24 cases.
- Lifted walking is faster in 5 of 24 paired cases.
- Paired geometric-mean speedup is `0.834x`, meaning lifted walking takes about `1.20x` as long
  overall on this n=10/15 grid.
- It modestly reduces iterations/cuts in several p=32 groups, but the larger master and repeated
  exact walking structure make total wall time higher in most cases.

## Mean results over three seeds

`Speedup = baseline wall / lifted wall`; values below one mean lifted walking is slower.

| n | p | q | baseline wall (s) | lifted wall (s) | geom. speedup | baseline iterations | lifted iterations | baseline cuts | lifted cuts | largest lifted gap |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 | 16 | 1 | 19.2 | 21.7 | 0.918x | 30.3 | 29.0 | 29.3 | 28.0 | 0.000% |
| 10 | 16 | 3 | 25.4 | 29.8 | 0.852x | 28.7 | 29.7 | 81.7 | 83.3 | 0.000% |
| 10 | 32 | 1 | 26.3 | 25.2 | 1.052x | 30.3 | 29.0 | 29.3 | 28.0 | 0.000% |
| 10 | 32 | 3 | 30.2 | 31.3 | 0.990x | 29.3 | 28.0 | 85.0 | 81.0 | 0.000% |
| 15 | 16 | 1 | 80.7 | 119.5 | 0.673x | 101.0 | 99.7 | 100.0 | 98.7 | 0.000% |
| 15 | 16 | 3 | 200.3 | 300.2 | 0.667x | 106.7 | 108.0 | 314.3 | 316.7 | 1.696% |
| 15 | 32 | 1 | 816.1 | 978.0 | 0.799x | 257.3 | 250.3 | 256.3 | 249.3 | 4.858% |
| 15 | 32 | 3 | 1,724.0 | 2,120.2 | 0.797x | 244.0 | 234.3 | 728.3 | 698.7 | 5.935% |

## Objective changes

| Instance | baseline objective | lifted objective | difference | baseline gap | lifted gap | direct-enumeration interpretation |
|---|---:|---:|---:|---:|---:|---|
| n15 p32 seed123 q1 | 27,046.1601 | 26,619.9372 | -426.2228 | 6.357% | 4.858% | Lifted matches Direct exactly; baseline was high |
| n15 p32 seed123 q3 | 85,360.7023 | 85,360.7008 | -0.0015 | 5.935% | 5.935% | No completed Direct reference |
| n15 p32 seed42 q3 | 84,849.7771 | 84,864.0171 | +14.2400 | 0.396% | 0.412% | Baseline matches Direct; lifted is high |

These changes occur only in cases whose outer Benders gap is not closed. They should not be read
as different mathematical objectives: lifted walking is an algebraic reformulation intended to
preserve the optimum. Instead, moving walking cost into the master changes the sequence of station
sets explored before the LP-cut stopping condition fires. The direct checks show that this can
either correct or worsen the returned incumbent when the outer gap remains open.

## Conclusion

Lifted walking is objective-consistent and well behaved on n=10, but it is not an observed runtime
improvement on this n=10/15 grid. Its main benefit is structural: walking cost is represented
exactly in the master, and one previously incorrect n=15 incumbent is corrected. It does not solve
the routing LP/IP gap or the premature classical Benders stopping problem, and it produces one
direct-checked n=15 incumbent that is slightly worse than the old baseline.

It is reasonable to keep lifted walking as the default formulation for BendersY/YZ because it
models the separable walking term in the natural place. Performance and correctness reports must
still require recorded outer-gap closure or an independent Direct comparison; the inner
`OPTIMAL` status remains insufficient.

## Reproducibility

- Lifted results: `experiments/zhuzhou_classical_benders_lifted_walking_ms5/`
- Baseline: `experiments/zhuzhou_benders_cut_scaling_ms5/`
- Aggregation: `scripts/analyze_zhuzhou_lifted_walking_vs_baseline.jl`
