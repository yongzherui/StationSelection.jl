# Study 11 -- master LP integrality at intermediate CG iterations

**Question: at every CG iteration, is the restricted master's LP solution close to binary
-- or only at the end?**

Everything measured about `:relaxed_cluster` so far concerns the *pricing* problem: a
certificate says no improving column remains. That is a statement about the column pool,
not about the master's own relaxation. This study asks the separate question the pricing
work never touches -- how integral is the LP point the loop is actually solving, iteration
by iteration -- and it asks it at `n=30`, the largest cell Study 9 certified on **10/10**
seeds.

Two things are being measured, and they are not the same thing:

1. **Are the variables binary?** Distance from each master variable to the nearer of
   `{0,1}`, per family, at every iteration.
2. **Is the relaxation tight?** The same restricted pool re-solved as a MIP, giving the
   master's integrality gap at that iteration. A solution can be *fractional* and still
   cost exactly what the best integer point over the same pool costs, so (1) cannot answer
   (2) and (2) cannot answer (1).

## The three variable families

`AggregateODRouteJointRoutingAssignmentFormulation`'s LP master has exactly three, and they
are not symmetric:

| variable | bounds in the LP master | held near {0,1} by |
| --- | --- | --- |
| `y[j]` | `0 <= y <= 1` | `sum(y) == k` (equality), so exactly `k` units of mass are distributed over `n` stations |
| `x_walk[(s,p)]` | `0 <= x_walk <= 1` | the coverage row for its own demand group |
| `theta[c]` | `theta >= 0`, **no upper bound** | only the coverage rows `sum(theta) + x_walk >= 1`; a value above 1 is possible in principle and is counted separately (`theta_n_above_one`) rather than folded into "distance to 1" |

`theta`'s missing upper bound is not an oversight to fix -- it is how
`add_joint_routing_assignment_column!` creates the variable, and cost is what pushes it
down. It is called out here because it makes "distance to binary" a slightly different
quantity for `theta` than for the other two.

## Where the snapshot is taken

`CGSolver.dual_callback`, which fires immediately after each master `optimize!` and
**before** that iteration's pricing round. That is the only point in the loop where the LP
solution is on the model and pricing has not yet added variables on top of it; reading the
same values from `iteration_callback` would read a solution `add_columns!` has since
invalidated.

The per-iteration MIP uses `integer_recovery_build` -- the same rebuild `CGSolver` performs
once at the end under `recover_integer_solution`. It constructs a **fresh** model seeded
with exactly the pool the live master holds and reads no primal values off it, so it cannot
disturb the CG run, and its numbers are directly comparable to the run's own final
`lp_objective` vs `objective_value`.

## Wall-clock caveat -- do NOT compare runtimes with Study 9

The MIP snapshots run *inside* the CG loop, so their wall time comes out of
`total_time_limit_sec`. `total_limit_sec` is therefore set to 14400 s here against Study
9's 21600 s for the same cell, and every result row carries `ip_snapshot_sec` so the
probe's own cost is separable. A Study 11 `runtime_sec` is not a Study 9 `runtime_sec`. Set
`ip_every=0` in a job row for clean timings; the integrality columns are unaffected either
way.

## Arms

| arm | pricer | K | why |
| --- | --- | --- | --- |
| `relaxed_k60` | `:relaxed_cluster` | 0.6n = 18 | the question as asked |
| `exact` | `:exact` | -- | control |

The control matters because **the master LP is identical across pricers** -- only the
column pool differs. Without it, a near-integral LP cannot be attributed: it could be a
property of the formulation, or an artefact of the pool `:relaxed_cluster` happens to
feed it. `n=30` is beyond `exact`'s certification frontier (it certifies only at `s=1`), so
the control runs are expected to stop on budget. That costs nothing: the question is about
the iterations they *do* reach, not their final objective.

## Tables

| table | jobs | what |
| --- | --- | --- |
| `smoke.tsv` | 2 | n=8 p=4 s=3, both arms, 600 s -- exercises the callback and the MIP rebuild |
| `n30.tsv` | 10 | n=30 p=16 s=3 K=18, seeds 42-51, 14400 s, snapshot every iteration |
| `n30_exact_control.tsv` | 5 | same instances, `:exact` pricer, seeds 42-46 |

```bash
cd benchmarks/study11_master_lp_integrality
julia --startup-file=no generate_jobs.jl
STUDY11_RUN_ID=_smoke        sbatch --array=1-2  --time=00:40:00 submit_benchmark.sh smoke.tsv
STUDY11_RUN_ID=2026-09-09_n30 sbatch --array=1-10 submit_benchmark.sh n30.tsv
STUDY11_RUN_ID=2026-09-09_n30 sbatch --array=1-5  submit_benchmark.sh n30_exact_control.tsv
julia --startup-file=no --project=../.. analyze.jl \
    ../experiments/2026-09-09_n30_study11_master_lp_integrality
```

**Smoke first, always.**

## Why n=30, and why every iteration

Study 9's `n30.tsv` `relaxed_k60` runs (experiment `2026-09-08_rc_scale_v3`) certified on
all ten seeds in **9-25 CG iterations**, wall 155-5195 s. That is a small enough iteration
count that "the intermediate iterations" is a bounded, fully observable set rather than a
prefix of a censored run, and it is what makes a MIP snapshot at *every* iteration
affordable. Larger `n` would trade that completeness for a truncated trajectory.

## Output

Per job, under `experiments/<run>_study11_master_lp_integrality/`:

- `n<N>_seed<S>_<arm>.csv` -- one summary row.
- `iterations/n<N>_seed<S>_<arm>.csv` -- one row per CG iteration: the distance-to-{0,1}
  statistics for all three families, the per-scenario `theta` support and mass, and the MIP
  snapshot.
- `y_values/n<N>_seed<S>_<arm>.csv` -- every `y[j]` value at every iteration, long format.
  `y` is the decision the study is *about* (which stations get built), and `n=30` values
  per iteration is small enough to keep in full rather than summarise away.
- `<stem>.progress.csv` -- one-row snapshot of an in-flight run, removed on completion.
  Deliberately does not match the pattern `analyze.jl` globs.

`analyze.jl` writes `case_results.csv`, `per_third.csv` and `ip_gap_trajectory.csv` into
`results/<run>_study11_master_lp_integrality/`. Thirds are by **iteration index**, not wall
time: a relaxed-cluster run spends wildly uneven time per iteration (early rounds exhaust
in milliseconds, late ones burn the full pricing budget), so a time split would put almost
every iteration in the first bucket and answer a different question.

## Baselines to compare against

- Study 9, `2026-09-08_rc_scale_v3`, n=30 `relaxed_k60`: 10/10 OPTIMAL by certification,
  9-25 iterations, objectives 26200-37561.
- Study 1 (`study1_formulation_lp_ip_gap`) measured **final** LP/IP gaps for this
  formulation. Those are the endpoint this study's trajectories should land on; a
  disagreement at the last iteration means the probe is wrong, not the LP.
