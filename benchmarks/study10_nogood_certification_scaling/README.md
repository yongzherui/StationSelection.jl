# Study 10 — Does no-good-cut certification scale to n=20, 25, 30?

**Status: complete (60/60 jobs, run `2026-09-04`).** Results in
`benchmarks/results/2026-09-04_study10_nogood_certification_scaling/`.

> **These numbers predate column harvesting and are a BASELINE, not current behaviour.**
> This study is what motivated it: finding 96% of certification attempts refuted, each
> throwing away a completed exact pricing search. `nogood_certify.jl` now returns the
> improving columns those searches found, and `CGSolver` takes them as the iteration's
> pricing result and skips the regular round (see that file's "Harvesting" section).
> Every timing below — `certification_sec`, `failed_certification_sec`, and every speedup —
> is therefore measured against a version that discarded them. The certification *rates*
> and the K/n conclusions are unaffected: harvesting changes where columns come from, not
> whether the relaxation certifies. Re-run `--array=41-60` to re-measure the n=25 timings.

## The question

Study 9 asked whether the relaxed-cluster relaxation certifies at all, at n=15, in the
*one-shot* mode (`certification_pricing_mode = :relaxed_cluster`). The answer was a clean
no — 0/31 at every K < n — for a structural reason: at a converged master the exact minimum
reduced cost is exactly 0 (complementary slackness on `theta >= 0`), so certifying one-shot
needs the relaxation tight to within `reduced_cost_tol`, and its slack is 10²–10³.

`:relaxed_cluster_nogood` closes that gap. When the relaxation names an improving *cluster*
route, the loop takes that route's cluster support `T`, searches `stations(T)` exhaustively
with the real pricer, and — if nothing improving is there — adds the cut *"every route must
visit a cluster outside T"* and asks again. Cuts are only ever placed on supports an
exhaustive search proved barren, so the certificate still covers the **full**
revisit-tolerant route universe.

It certifies at n=15. This study asks whether it still does at n=20/25/30, and what it
costs — the measured CG certification frontier on this instance family puts n=30/s=3 past
the point where baseline CG can certify at all.

## Grid

n ∈ {20, 25, 30} × K/n ∈ {0.4, 0.6, 0.8} + baseline, p=16, s=3, ms=10, seeds 42–46.
5 seeds × 4 arms × 3 sizes = **60 jobs**.

`K` is swept as a *fraction of n* so arms mean the same thing at every size. Budgets:
4 h/600 s (n=20), 5 h/900 s (n=25), 6 h/1200 s (n=30) for total CG / per certification
attempt. No `K = n` ceiling control and no one-shot arm — see `generate_jobs.jl` for why
both are subsumed by the no-good trace.

**n=25 was added after the fact**, once n=20 came back showing baseline `certifying_rounds
= 0` on every cell — meaning there was no expensive two-tier escalation for certification
to replace, so every n=20 arm was overhead by construction and the feature's actual claim
went untested. It is *appended* as jobs 41–60 rather than inserted in size order, because
the n=20/n=30 arrays were already live on a preemptable partition where a requeued task
re-reads `jobs.tsv`; inserting would have handed running n=30 tasks n=25 rows.

## Findings

### 1. Correctness: clean

**45/45** arm/baseline pairs that both finished agree on the objective to 1e-6 relative.
Certification decides only *when pricing stops*; it never moved an answer.

### 2. The cuts are the entire mechanism

`certified_at_round_1 = 0` across **all ~1130 attempts, at every size and every K.** Not one
scenario ever certified on its first relaxed search — i.e. without placing a cut. Since a
round-1 certification is exactly what the one-shot `:relaxed_cluster` mode does, this
reproduces Study 9's 0/31 at three further sizes and shows the no-good cuts are not an
optimization on top of a working relaxation: they *are* what makes it work.

### 3. Certification rate — and where the ceiling is

| n | K/n=0.4 | K/n=0.6 | K/n=0.8 | baseline |
| --- | --- | --- | --- | --- |
| 20 | 4/5 | **5/5** | **5/5** | 5/5 |
| 25 | **0/5** | 4/5 | 4/5 | 4/5 |
| 30 | 0/5 | 1/5 | 1/5 | **0/5** |

- **n=20/25 in the 0.6–0.8 band: reliable.** The single n=25 miss is seed42, a cell no arm
  *and no baseline* could certify.
- **n=30 is mostly past everything.** Four of five cells defeat baseline and arms alike.
- **The one n=30 cell that is solvable is an `arm only` win** (seed44): baseline returned
  FEASIBLE after 18,015 s and four certifying escalations, while K=18 and K=24 returned a
  proven full-universe optimum in 14,375 s and 16,585 s. That is a capability baseline does
  not have at any budget — but it is one cell of five, not a general n=30 result.

### 4. K/n = 0.4 is refuted, and is worse than useless

It is the only arm that ever missed at n=20, went 0/5 at n=25, and 0/5 at n=30. At n=25 it
also runs **3–4× slower than baseline** (0.22×–0.76×) while still reaching OPTIMAL by
ordinary pricing exhaustion.

The failure is a cost spiral, visible in the trace: a coarse partition makes each cut
round's exhaustive subset search huge (median |S| = 18 of 25 stations, vs 6 at K/n=0.8), so
the decisive final attempt times out (1 `inconclusive` in every single run), CG gets no
certificate, the wasted wall starves the pricing rounds, and the run then needs **more**
two-tier escalations than baseline did — median 5.0 against baseline's 1.0.

**Use K/n ∈ [0.6, 0.8].** Do not go coarser to "save time"; it costs time.

### 5. The mechanism, confirmed at n=25

`certifying_rounds`, median per cell — the two-tier escalation certification is meant to
replace:

| n | baseline | K/n=0.6 | K/n=0.8 |
| --- | --- | --- | --- |
| 20 | 0.0 | 0.0 | 0.0 |
| 25 | **1.0** | **0.0** | **0.0** |

n=20 never escalates, so nothing there could be replaced — which is exactly why n=25 was
worth adding. At n=25 the escalation fires for baseline and the working arms eliminate it.

### 6. Speedup scales with instance difficulty

Paired wall-clock, cells where both finished:

| n | K/n=0.4 | K/n=0.6 | K/n=0.8 |
| --- | --- | --- | --- |
| 20 | 1.02× | **1.17×** | 0.84× |
| 25 | 0.31× | 1.13× | **1.28×** |

Medians understate the picture and should not be quoted alone. Per cell, speedup orders
almost perfectly by how long the *baseline* took — rank correlation between baseline wall
and speedup is **positive in all 9 size×K groups** (ρ = +0.40 to +1.00; ρ = +1.00 at
n=20/K=16). At n=20 the arms run 0.68×–0.76× on the cheapest cell and 1.75×–2.28× on the
most expensive.

The mechanism explains it: certification replaces the final exhaustive pricing round, which
dominates a hard cell's solve and is already cheap on an easy one. **The feature pays
exactly where cost is a problem** — which is the property that matters, and the reason the
n=30 result is worth having despite its 2/10 rate.

## Reading the outputs

| file | contents |
| --- | --- |
| `certification_outcome_by_size_and_k.csv` | the cross-tab above, incl. `arm_certified_only` |
| `nogood_trace_by_size_and_k.csv` | rounds, cuts/attempt, subset sizes, `certified_at_round_1` |
| `speedup_by_size_and_k.csv` | paired wall-clock, certification cost, wasted cost |
| `paired_rows.csv` | every arm row joined to its baseline |

Cut counts are meaningful only **per attempt**: `cluster_sets` starts empty on every
(CG iteration × scenario) call and is discarded on return, because a cut is only valid at
the duals it was derived under (see `relaxed_cluster/nogood_certify.jl`). A summed cut count
measures how many times the loop ran, not how deep any one loop went — that is `max_rounds`,
which never exceeded 17 against the 32 cap anywhere in this study.

## Running

```bash
cd benchmarks/study10_nogood_certification_scaling
mkdir -p slurm_logs
julia --project=../.. generate_jobs.jl          # prints the --array range per size
export STUDY10_RUN_DATE=$(date +%F)             # MUST be set in the submitting shell
STUDY10_RUN_DATE=$STUDY10_RUN_DATE sbatch --array=1-20 --time=04:45:00 --mem=24G submit_benchmark.sh
STUDY10_RUN_DATE=$STUDY10_RUN_DATE sbatch --array=21-40 submit_benchmark.sh
STUDY10_RUN_DATE=$STUDY10_RUN_DATE sbatch --array=41-60 --time=05:45:00 --mem=28G submit_benchmark.sh
julia --project=../.. analyze.jl
```

`STUDY10_RUN_DATE` pins every task to one run directory. Without it each task stamps its
own date and a multi-hour array spanning midnight splits across two directories, of which
`analyze.jl` would silently read one.

## Open questions this study did not settle

- **Is the useful K/n a constant?** 0.6 was best at n=20, 0.8 at n=25. Both are inside the
  working band, and 4–5 cells per arm cannot separate them.
- **Would a bigger `certification_time_limit_sec` rescue n=30?** Probably not the four hard
  cells: baseline there fails at `pricing_inconclusive` with up to 69% of its *total* budget
  unused, so the binding constraint is not wall. But n=30 arm failures include 2
  `inconclusive` attempts at K=24, so a larger per-attempt budget is the cheapest thing to
  try first.
- **Should the no-good round be threaded?** It walks scenarios serially (`nogood_certify.jl`
  — an adaptive shared deadline plus early exit on the first refutation) while pricing runs
  them concurrently, so a certification round pays the *sum* where a pricing round pays the
  *max*. At n=20 this was measured to cost ~1–4% of wall, because ~92% of attempts are
  refuted and a refuted attempt stops at its first scenario. Worth revisiting only if the
  refuted share drops at larger n.
