# Study 11 -- the CG master's LP is integral at convergence, fractional in one transient episode

**Question.** At every CG iteration, is the restricted master's LP solution close to binary?
We know the final polytope is tight; what about the intermediate iterations?

**Answer.** The LP goes fractional in **28 of 250 iterations (11.2%)**, across 9 of 15 runs.
Every occurrence is a single **contiguous episode** in the first half of the run, and every
run is fully integral from then to the end. All 9 certified runs finish at an integrality
gap of exactly 0.

Instance: Zhuzhou n=30, p=16, s=3, k=15, `max_stops=10`, seeds 42-51. Pricers
`:relaxed_cluster` (K=18, the arm Study 9 certified 10/10) and `:exact` (control).
Experiment `benchmarks/experiments/2026-09-09_n30_study11_master_lp_integrality`,
results `benchmarks/results/2026-09-09_n30_study11_master_lp_integrality`.

## How often, per run

Do not read this study off a pooled average. The pooled 11.2% hides that half the runs
never go fractional at all and that those which do, do so in one burst.

| arm | seed | iters | frac | % | which iterations | span of run | clean tail | max gap |
| --- | ---: | ---: | ---: | ---: | --- | --- | ---: | ---: |
| relaxed_k60 | 42 | 23 | 0 | 0.0% | -- | -- | 23 | 0.0000% |
| relaxed_k60 | 43 | 19 | 1 | 5.3% | 3 | 16% | 16 | 0.4443% |
| relaxed_k60 | 44 | 9 | 2 | 22.2% | 2-3 | 22-33% | 6 | 6.0260% |
| relaxed_k60 | 45 | 20 | 6 | 30.0% | 4-9 | 20-45% | 11 | 3.9773% |
| relaxed_k60 | 46 | 25 | 6 | 24.0% | 9-14 | 36-56% | 11 | 2.8706% |
| relaxed_k60 | 47 | 11 | 0 | 0.0% | -- | -- | 11 | 0.0000% |
| relaxed_k60 | 48 | 16 | 3 | 18.8% | 3-5 | 19-31% | 11 | **7.6688%** |
| relaxed_k60 | 49 | 21 | 2 | 9.5% | 4-5 | 19-24% | 16 | 3.9070% |
| relaxed_k60 | 50 | 9 | 0 | 0.0% | -- | -- | 9 | 0.0000% |
| relaxed_k60 | 51 | 13 | 0 | 0.0% | -- | -- | 13 | 0.0000% |
| exact | 42 | 17 | 0 | 0.0% | -- | -- | 17 | 0.0000% |
| exact | 43 | 11 | 1 | 9.1% | 3 | 27% | 8 | 0.4468% |
| exact | 44 | 12 | 2 | 16.7% | 2-3 | 17-25% | 9 | 5.2444% |
| exact | 45 | 14 | 0 | 0.0% | -- | -- | 14 | 0.0000% |
| exact | 46 | 30 | 5 | 16.7% | 10-14 | 33-47% | 16 | 2.8068% |

Four regularities hold without exception across all 15 runs:

1. **Contiguous.** Every fractional stretch is one unbroken block. No run alternates.
2. **Never first.** The earliest is iteration 2; iteration 1 is always integral.
3. **Over early.** The last fractional iteration falls between 16% and 56% of the run.
4. **Clean tail.** Every run ends with 6 to 16 consecutive fully-integral iterations.

## Should you care?

**If you run CG to certification: no.** All 9 certified runs end at gap 0.00000% (residuals
are +/-1e-14 float noise) and the final LP solution is integral in `y`, `theta` and
`x_walk` simultaneously. The certified objectives match Study 9's to the digit on all 9
seeds, so the probe does not perturb the solve.

**If you stop CG early, yes, and by up to 7.7%.** An anytime consumer that halts mid-run
and reads the master LP can land inside an episode: it would get a fractional station
selection (up to 2 of 30 `y` at exactly 0.5) and an LP value up to 7.67% below the best
integer solution over the same pool. The episodes sit at 16-56% of the run, which is
exactly where a budget-stopped run tends to stop.

## Mechanism: pool incompleteness, not a loose polytope

The gap is *not* evidence that the formulation's relaxation is weak. Seed 48 shows the
mechanism in isolation (`iterations/n30_seed48_relaxed_k60.csv`):

| it | columns | LP | IP | gap | theta frac/support |
| ---: | ---: | ---: | ---: | ---: | --- |
| 2 | 2750 | 39316.99 | 39316.99 | 0.000% | 0/10 |
| 3 | 4473 | 38683.37 | 38778.82 | 0.246% | 5/11 |
| 4 | 5280 | 37347.16 | 38778.82 | 3.692% | 9/12 |
| 5 | 6217 | 35804.95 | 38778.82 | **7.669%** | 11/14 |
| 6 | 6472 | 32184.93 | 32184.93 | 0.000% | 0/5 |
| 7 | 6486 | 32184.93 | 32184.93 | 0.000% | 0/5 |

Through iterations 3-5 the LP descends 38683 -> 35804 while the IP is **pinned** at
38778.82: the LP is fractionally blending columns into a cheap solution the pool cannot yet
express integrally. One column arrives at iteration 6 and both collapse onto 32184.93,
where they stay for 11 more iterations. The episode is the pool catching up with the LP,
and it closes the moment the right column is priced.

Corroborating: active `theta` support *shrinks* as the run integralizes (8.1 -> 5.7 -> 4.5
columns, early/mid/late thirds), so this is not the LP spreading over an ever-larger set.

## It is not the pricer, and it is not `max_stops`

**Not the pricer.** The `:exact` control reproduces the pattern -- fractional early/mid,
26/26 integral in its late third. The master LP is identical across pricers; only the pool
differs. So this is a property of the formulation and of CG dynamics, not of
`:relaxed_cluster`.

**Not `max_stops`.** The `n30_maxstops.tsv` arm re-ran seeds 42/48 at `max_stops` 10 vs 14
(`benchmarks/experiments/2026-09-09_maxstops_study11_master_lp_integrality`):

| seed | max_stops | longest route in pool | at cap | objective | integral iters | max gap |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 42 | 10 | 6 | 0 | 27763.5689 | 23/23 | 0.0000% |
| 42 | 14 | 6 | 0 | 27763.5689 | 23/23 | 0.0000% |
| 48 | 10 | 10 | 5 | 32184.9289 | 13/16 | 7.6688% |
| 48 | 14 | 10 | 0 | 32184.9289 | 13/16 | 7.6688% |

Seed 48 has 5 columns sitting exactly on the cap at `max_stops=10`, which *looks* binding.
It is not: given 4 more stops the pricer still produces nothing longer than 10, and the
objective, iteration count, integrality pattern and max gap are bit-identical. Ten is the
natural route length on that instance. **Columns at a cap only mean the cap binds if
removing it changes something** -- here it changes nothing.

## Shape of the fractionality

- `y` fractionates, up to 2 of 30 stations at once (seeds 45, 46, 48, 49), under the
  equality `sum(y) == k`.
- `x_walk` essentially never does (mean 0.14 fractional in the early third, 0 after).
- `worst_dist` is repeatedly **exactly 0.500000**, and once exactly 3/7. Half-integral
  values point at symmetric ties in the instance rather than generic fractional drift.
  Not chased further here.

## Caveats

- **Seed 51 was censored** (`total_budget`, 13 iterations, 0 fractional). Study 9 certified
  it in 5195 s with a 21600 s budget; this study gave 14400 s minus the probe's own MIP
  time. Its 0% is over a truncated pool.
- **All five `exact` controls hit budget** (`total_budget`, or `pricing_inconclusive` for
  seed 43). Their late-third integrality is therefore over an incomplete pool -- suggestive
  of pricer-independence, not proof of it. Only the 9 certified `relaxed_k60` runs support
  the "gap is exactly 0 at convergence" claim.
- **Seed 45 at `max_stops=14` timed out** at the 5 h wall; the `max_stops` conclusion rests
  on seeds 42 and 48.
- The per-iteration MIP snapshots consume CG budget (they run inside the loop), so Study 11
  runtimes are **not** comparable with Study 9's. `ip_snapshot_sec` records the cost; all
  250 snapshots solved to proven optimality in <= 1.55 s each, so no gap here is an
  artifact of a truncated MIP.
- n=30, one instance family (Zhuzhou), 10 seeds. Whether the episode grows with `n` is
  unmeasured.

## Reproduce

```bash
cd benchmarks/study11_master_lp_integrality
julia --startup-file=no generate_jobs.jl
STUDY11_RUN_ID=<id> sbatch --array=1-10 submit_benchmark.sh n30.tsv
STUDY11_RUN_ID=<id> sbatch --array=1-5  submit_benchmark.sh n30_exact_control.tsv
STUDY11_RUN_ID=<id> sbatch --array=1-6  submit_benchmark.sh n30_maxstops.tsv
julia --startup-file=no --project=../.. analyze.jl ../experiments/<id>_study11_master_lp_integrality
```
