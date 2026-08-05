# Adaptive station-cluster certification: end-to-end results

## What was established

The adaptive station-cluster relaxation produced a valid nonnegative pricing
certificate for every scenario tested at the final passenger-CG dual solution.
Across n=10--40, every cluster-enabled run terminated with
`optimality_proven`, no lower-bound assertion failed, no refinement decrease was
logged, and exact fallback was not required.  The paired exact-certification
runs had identical CG iteration and column counts.

The first performance crossover appeared at n=40.  Summed over three seeds,
exact certification took 145.1 seconds while cluster certification took 109.0
seconds.  Total CG time was 3156.9 versus 2156.5 seconds, although the latter
comparison includes substantial early-pricing runtime variability and should
not all be attributed to clustering.

At n=25,30,35 the cluster layer remained slower.  Certification typically
required 65--75% of the physical station count at n=25--40, versus 83--100% at
n<=20.  A hard 50--60% cap would therefore have certified only 6/36 individual
large-instance scenarios and none of the complete three-scenario runs.

## Activation caveat

Clustering is not invoked during ordinary successful harvesting iterations.  It
runs after an early-return pricing pass adds zero columns, immediately before
the existing exhaustive certification pass.  In the reported experiments this
happened once, at the tail of every run.

There is nevertheless no explicit objective-stagnation window.  A zero-column
pass caused by timeout, weak harvesting, or duplicate rediscovery can activate
clustering before genuine convergence.  Correctness remains intact because a
negative/unresolved cluster result falls back to exact pricing, but time can be
wasted.  A production activation policy should log and optionally require recent
RMP objective stagnation, recent reduced-cost magnitude, and iterations since a
meaningful improvement.  It should also retain per-scenario certificates and
exact-price only unresolved scenarios.

## Interpretation

These experiments establish LP pricing exhaustion, not by themselves equality
of the final restricted-pool integer solution with a monolithic direct optimum.
The next validation is therefore an independent max-stops-3 comparison against
`DirectSolver` on identical instances and objective conventions.

## Max-stops-3 comparison with DirectSolver

Twenty-one cases (n=10,15,...,40; three seeds; three scenarios jointly) compared
cluster-certified passenger CG with a monolithic aggregate route model using
explicit free-assignment `x` variables and a fully enumerated max-stops-3 route
pool.  Demand-unity and no-slack guards made the objective conventions
comparable.

The integer objective agreed to numerical tolerance in 20/21 cases.  The one
exception was n=15, seed 314: DirectSolver found 55109.594368 while the final CG
pool MIP found 55309.594368, a 200-unit (0.3629%) gap.  Its cluster-certified LP
bound, 53157.786873, remained valid.  This demonstrates the important boundary:
cluster certification proves LP pricing exhaustion, but does not prove that the
finite generated column pool contains an integer-optimal solution.

Exact station sets matched in 5/21 cases.  Most objective-equal cases selected a
different station set, consistent with alternative integer optima; station-set
identity is therefore a stronger and generally inappropriate quality criterion
unless uniqueness is established.  Every clustered scenario again certified
nonnegativity.

For max_stops=3 DirectSolver was faster throughout n<=40.  Mean direct versus CG
times in seconds were: n10 3.5/5.9, n15 4.0/6.4, n20 4.4/7.1, n25 5.7/9.9,
n30 8.2/12.1, n35 9.7/19.8, and n40 12.4/28.7.  With only three stops, complete
route enumeration grows cubically and remains cheaper than repeated CG solves;
the motivation for CG and clustered certification is the larger-stop or
uncapped route universe where direct enumeration no longer scales.

## Secondary finding: candidate over-generation

This is an implementation-tuning result, not a priority result for the current
week's presentation.

At n=20, max_stops=10, unlimited station visits, and three seeds, three column
harvesting policies were compared under exact final pricing certification:

| candidates harvested | columns added | mean CG iterations | mean time (s) | mean labels | mean final pool |
|---:|---:|---:|---:|---:|---:|
| 20 | 20 | 40.0 | 153.8 | 17.41M | 1,284 |
| 100 | 20 | 41.0 | 184.3 | 21.76M | 1,365 |
| 100 | 100 | 29.7 | 153.0 | 16.85M | 2,516 |

Harvesting 100 candidates but retaining only the best 20 was not useful: it
slightly increased iterations and increased mean runtime by about 20% relative
to 20/20.  Adding all 100 candidates reduced iterations for every seed (52 to
32, 38 to 28, and 30 to 29), for a 26% mean iteration reduction.  Its mean
runtime was effectively tied with 20/20 because the larger restricted master
offset the saved pricing iterations, and its final column pool was nearly twice
as large.

All variants certified LP optimality in one round and reached the same
seed-matched LP objectives.  The 100/100 policy is therefore a plausible
larger-instance tuning option, where avoiding expensive late pricing iterations
may matter more, but the n=20 evidence does not establish a runtime improvement.
