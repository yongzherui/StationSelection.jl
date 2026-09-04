# Study 9 — Relaxed-cluster certification: does it fire, and at which K?

## The question

CG's cost is not finding columns, it is **proving there are no more**. Study 5's iteration
logs put 99.86% of in-loop time in pricing, of which the two-tier escalation to the
certifying round is a measured 8.2% pure re-pricing tax; at n=30 the certifying share
reaches 77.7% and seven runs quit with up to 72% of their budget unused. More budget was
measured to be the wrong lever.

`CGSolver(certification_pricing_mode = :relaxed_cluster)` is a different lever: before
each real pricing round, ask a **relaxation** of the pricing problem whether an improving
column can still exist.

- Stations are partitioned into `K` clusters (k-medoids on travel time, computed **once**
  at build time).
- Travel between clusters is charged the *minimum* member-to-member time (then a metric
  closure); a passenger's reward for a cluster pair is the *maximum* over its real station
  pairs; a passenger whose whole trip fits inside one cluster is credited on arrival with
  no arc at all.
- So every real route maps to a cluster route that is no slower and no less rewarded:
  `min relaxed reduced cost <= min real reduced cost`.

An exhausted relaxed search that finds nothing below `-reduced_cost_tol` therefore
certifies that **no real improving column exists**, over the full revisit-tolerant route
universe — without ever running the exhaustive search. A failure proves nothing and costs
only the early exit it took to find one improving relaxed route.

Full argument: `src/opt/label_setting/joint_routing_assignment/relaxed_cluster/types.jl`.

## What is measured

`K` is the only thing swept, and the partition it produces is what sets the relaxation's
tightness:

- small `K` — the relaxed search is trivially cheap and hopelessly loose (it will find
  improving "routes" that correspond to nothing real, every round);
- `K = n` — the relaxation coincides with the exact pricing graph, so it certifies exactly
  when exhaustive pricing does, and costs exactly as much. This is the sanity end, not a
  useful operating point.

The useful `K`, if one exists, is in between. Arms: `baseline` (feature off) plus
`K ∈ {3, 6, 9, 12, 15}` on `n = 15` stations.

Three outcomes per arm, in the order `analyze.jl` reports them:

1. **Correctness gate** — every arm must reach the same certified optimum as baseline on
   the same cell. The relaxation only decides *when to stop pricing*; a moved objective is
   a bug, not a result.
2. **Certification rate** — the share of cells where the relaxation actually ended the
   solve, per `K`, followed by the failure mode of every attempt that did not. A `K` that
   never fires is dead weight regardless of its timings, and the failure-mode split says
   whether that arm's partition was too coarse or whether the certification budget was the
   binding constraint for every arm.
3. **Cost vs. saving** — paired wall-clock speedup against baseline, alongside
   `certification_sec` (what all attempts cost) and `failed_certification_sec` (pure
   overhead — the attempts that proved nothing). An arm can certify often and still lose
   on wall if its attempts are expensive; both numbers are reported rather than a net.

The mechanism check is the last block: a working arm should show **fewer two-tier
`certifying_rounds`** than its baseline, because that is precisely the round it replaces.

## Grid

One size, one difficulty, five seeds — this is a **probe**, not a measurement study. The
question is whether the relaxation ever fires; 30 jobs answer that as well as 180 would,
in minutes rather than hours.

| | |
| --- | --- |
| stations | 15 (so `k = 8`) |
| scenarios | 3 |
| OD pairs/scenario | 16 |
| seeds | 42–46 |
| max_stops | 10 |
| budgets | 120 s regular / 600 s certifying / 1800 s total |
| certification budget | 60 s per attempt |
| threads | 3, `parallel_scenario_pricing = true`, Gurobi pinned to 1 |
| recovery | `recover_integer_solution = true` |

6 arms (`n_clusters ∈ {0, 3, 6, 9, 12, 15}`) × 5 seeds = **30 jobs**, 45 min walltime each.

Budgets are sized off **Study 3's measured walls on this exact instance size** (n=15,
p=16, ms=10, s=1: 7–154 s, median ~26 s, every cell certified), not inherited from Study
7's n=20 grid. Tripling the scenarios does not triple the wall under
`parallel_scenario_pricing` — a round's wall is the max over scenarios, not the sum — but
it does add CG iterations, so the 1800 s total budget leaves roughly an order of magnitude
of headroom over that median. If cells come back `budget_exhausted`, that budget, not the
grid, is what to raise.

`n_scenarios = 3` is deliberately **not** cut to 1 even though it is the largest remaining
cost multiplier: a certification round must certify *every* scenario before CG may stop,
so a single-scenario study would measure a degenerate case of the thing being tested.

### Why `K = 15` is an arm, not a duplicate of the baseline

`K = 0` is the baseline: the feature is off and CG stops the old way (pricing exhausts, or
the two-tier escalation to the certifying round proves it). `K = 15 = n` is the **identity
partition**, where the relaxation coincides with the exact pricing graph arc for arc — so
it is "run the exact pricer as a cheap early-exit probe, then run it again for real". A
different run, and strictly slower than baseline by construction.

It earns its 5 jobs as the **ceiling control**, because it separates the two ways a `K` can
fail:

| observation | conclusion |
| --- | --- |
| `K=15` certifies, `K=6` does not | `K=6`'s partition is too coarse; some finer partition may work |
| even `K=15` does not certify | the 60 s certification budget is the binding constraint; no partition would have worked |

Without it, a sweep where nothing certifies cannot be read at all. It also doubles as the
worst-case overhead measurement. Per run, `certification_refuted_rounds` vs
`certification_inconclusive_rounds` records which of the two failure modes actually
occurred.

The arms are **not** a monotone ladder, and shouldn't be read as one. Tightness improves
under partition *refinement*, but two independent k-medoids runs at different `K` need not be
nested, so a larger `K` is not guaranteed to be tighter. The only ordering that always
holds is that `K = n` is refined by nothing and is therefore the tightest available —
which is precisely why the ceiling control is `K = n` rather than "the largest `K` that
certified".

## Running

```bash
cd benchmarks/study9_relaxed_cluster_certification
mkdir -p slurm_logs
julia --project=../.. generate_jobs.jl
sbatch --array=1-30 submit_benchmark.sh
# cheap bracketing pass -- baseline + the K=n ceiling control, which bounds what the
# middle arms can possibly achieve:
# sbatch --array=1-5,26-30 submit_benchmark.sh
julia --project=../.. analyze.jl
```

Arms are contiguous 5-job ranges in `jobs.tsv` order (`baseline`, `K=3`, `6`, `9`, `12`,
`15`), so any single arm can be submitted or re-submitted alone.

## Outputs

`analyze.jl` writes to `benchmarks/results/<run>/`:

| file | contents |
| --- | --- |
| `certification_rate_by_k.csv` | cells, certifications, rate and realized cluster sizes per `K` |
| `certification_failure_modes.csv` | attempts per `K`, split into *refuted* (too loose) vs *inconclusive* (out of budget) |
| `speedup_by_k.csv` | paired wall-clock speedup, certification cost, wasted cost, certifying rounds |
| `paired_rows.csv` | every arm row joined to its baseline, for ad hoc questions |
