# Where the CG solve budget actually goes (Study 5, 2026-08-30 two-arm run)

Companion to `2026-08-30_compute_budgets_of_record.md`. That note is authoritative for
**what the limits were and which one bound each cell**. This note is authoritative for
**how the granted budget was consumed inside the solves** — measured, not assumed.

Source: the 220 per-iteration logs in
`benchmarks/experiments/2026-08-30_study5_scaling_exact_cg/iterations/*.csv`
(columns `master_sec`, `pricing_sec`, `add_columns_sec`, `pricing_limit_sec`,
`certifying_pricing`). Every number below is a sum over all 220 runs of that run of
record. Nothing here is modelled or extrapolated.

Last verified: 2026-09-01.

## 1. The loop is pricing. Everything else is rounding error.

| Phase | Total across 220 runs | Share of in-loop time |
| --- | ---: | ---: |
| Master LP solve | 497 s | **0.03 %** |
| Pricing (label search) | 1,471,234 s | **99.86 %** |
| `add_columns!` | 1,571 s | 0.11 % |
| **Total in-loop** | **1,473,303 s (409.3 h)** | |

The master is free: 497 seconds total across every iteration of every run. Any budget or
performance discussion that is not about the label search is measuring noise. This
independently reproduces the earlier Zhuzhou frontier finding ("pricing-bound, master LP
~0 s") on a different grid.

## 2. The two-tier escalation duplicates work, and it is measurable

`CGSolver` prices at `pricing_time_limit_sec` (300 s here). If that round returns **empty
without exhausting**, it re-prices **the same duals** at `certifying_pricing_time_limit_sec`
(3600 s) — see `_cg_pricing_exhausted` and the escalation block in
`src/opt/solvers/cg_solver.jl`. Because the escalated round restarts the identical search,
the regular attempt that preceded it is discarded work.

The regular attempt necessarily ran its **full** cap: the escalation condition is "empty
AND not exhausted", which for a label search means it hit its deadline. So the duplicated
cost per escalation is the regular cap, 300 s.

| Measure | Value |
| --- | ---: |
| Certifying rounds | 402 of 5,897 rounds (**6.8 %** of rounds) |
| ...but consuming | 501,607 s = 139 h = **34 % of all pricing time** |
| Regular rounds hitting their 300 s cap | 1,794 of 5,495 (**32.6 %**) |
| **Duplicated label search** | **120,600 s = 33.5 h = 8.2 % of all pricing** |

33.5 h is **5.6 full six-hour job budgets** spent re-running searches that had already
been started. 72 of 220 runs (33 %) pay this tax at all; among those it averages
**11.7 %** of the run's own pricing time, worst case **41.1 %**
(`scenarios_12_n20_p16_s12_ms10_seed46`: 20 certifying rounds, 6000 s duplicated).

## 3. The tax lands exactly where the study is censored

| Substudy | Axis | Pricing | Certifying share | Dup-waste | Cert rounds |
| --- | ---: | ---: | ---: | ---: | ---: |
| stations | 10 | 0.0 h | 0.0 % | — | 0 |
| stations | 20 | 8.3 h | 2.7 % | 0.2 h | 3 |
| stations | **30** | 89.2 h | **77.7 %** | 6.5 h | 78 |
| passengers | 8 | 0.4 h | 0.0 % | — | 0 |
| passengers | 16 | 7.3 h | 7.7 % | 0.4 h | 5 |
| passengers | 24 | 61.9 h | 24.3 % | 5.2 h | 62 |
| passengers | 32 | 111.5 h | 14.8 % | 4.5 h | 54 |
| scenarios | 3 | 8.8 h | 7.8 % | 0.3 h | 4 |
| scenarios | 6 | 18.9 h | 4.0 % | 0.4 h | 5 |
| scenarios | **9** | 41.5 h | **31.8 %** | 6.4 h | 77 |
| scenarios | **12** | 60.7 h | **38.1 %** | 9.5 h | 114 |

The signal is the trend down each substudy's column: `stations` runs 0 % -> 2.7 % ->
77.7 %; `scenarios` runs 7.8 % -> 4.0 % -> 31.8 % -> 38.1 %. The tier structure costs
nothing on easy cells and dominates on hard ones.

At `stations=30`, **77.7 % of pricing time goes into certifying rounds** — the solve is
barely doing column generation, it is mostly failing to prove termination.

## 4. This is the same root cause as the early stops

Seven Study 5 runs quit with 6.9–72.1 % of their total budget **unused**, all of them
`stations` axis=30, all `cg_stop_reason="pricing_inconclusive"`. Five show a single label
search pinned at exactly 3600.0 s — the certifying cap.

| job | arm | seed | wall | unused | iters | cert rnds | longest search |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 26 | serial | 47 | 6035 s | 15565 s (72.1 %) | 11 | 1 | 1200.0 s |
| 52 | parallel | 43 | 6613 s | 14987 s (69.4 %) | 10 | 1 | **3600.0 s** |
| 56 | parallel | 47 | 6912 s | 14688 s (68.0 %) | 11 | 1 | **3600.0 s** |
| 58 | parallel | 49 | 7513 s | 14087 s (65.2 %) | 13 | 1 | **3600.0 s** |
| 57 | parallel | 48 | 10813 s | 10787 s (49.9 %) | 12 | 2 | **3600.0 s** |
| 53 | parallel | 44 | 18012 s | 3588 s (16.6 %) | 12 | 4 | **3600.0 s** |
| 23 | serial | 44 | 20112 s | 1488 s (6.9 %) | 19 | 4 | 2307.0 s |

A certifying round burns its full hour, returns nothing, cannot prove exhaustion, and the
loop stops as inconclusive with hours of `total_time_limit_sec` untouched.

## 5. Iteration count is NOT what scales

Certified rows only, `stations` substudy — iterations to certification **fall** as n grows
while per-iteration cost explodes:

| n | certified | mean iters | range | mean cols | mean sec/iter | mean wall |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 20/20 | 27.0 | 10–58 | 1,587 | 0.8 s | 16 s |
| 20 | 20/20 | 18.3 | 8–26 | 18,464 | 90.3 s | 1,514 s |
| 30 | 2/20 | 9.0 | 9–9 | 1,422 | 423.8 s | 3,814 s |

~3x fewer iterations, ~500x more time per iteration. **Caveat:** the n=30 row is a single
seed (50) — the only one that certified in either arm — so it is survivorship-biased and
its 1,422 columns are *below* n=10's average. Do not read "n=30 certifies in 9 iterations"
off it. The 18 censored n=30 rows reached a mean of **16.6** iterations (range 10–34)
without certifying, i.e. already at or past n=20's certification average.

Side observation validating the arm comparison: at n=10, serial and parallel produce
**identical** iteration counts on every seed — parallelism changes the wall, not the
algorithm. They diverge only at n>=20, where time-truncated pricing rounds find different
columns.

## 6. Three separable remedies — do not conflate them

**Status: undecided.** Nothing below is a decision. This section records what each
remedy would actually change, what it would cost, and where the reasoning is still
unmeasured, so the choice can be made later on evidence rather than re-derived.

### 6.0 What the escalation does today

`optimize_model(::BuildResult, ::CGSolver)` in `src/opt/solvers/cg_solver.jl:240-262`:

```julia
pricing_limit = min(pricing_time_limit_sec, remaining_budget())      # 300 s
new_columns = price_columns(...; time_limit_sec=pricing_limit)
if empty && !_cg_pricing_exhausted(m)
    certifying_limit = min(certifying_pricing_time_limit_sec, remaining_budget())  # 3600 s
    new_columns = price_columns(...; time_limit_sec=certifying_limit)              # SAME duals
end
```

The second call is a fresh `_run_label_setting`. Every piece of search state —
`frontier`, `live_labels`, `labels_by_state`, `best_by_signature` — is a function local
(`src/opt/label_setting/engine.jl:37-42`). When the deadline fires the loop simply
`break`s (`engine.jl:149`) and all of it is discarded. The escalated call re-seeds from
`_pricing_initial_labels` and deterministically re-walks the identical first 300 s before
it reaches new ground. That is the 33.5 h in section 2.

### 6.1 Remedy 1 — make pricing resumable (cheap, no algorithmic risk)

Hoist the per-scenario search state out of `_run_label_setting` into an object that
survives the return, and on escalation re-enter the engine with the surviving frontier.
Search order, dominance tests, and returned columns are unaffected — it is bookkeeping.

**Resuming is not merely faster, it is deeper.** Today the certifying round explores
3600 s of tree. Resumed with `time_limit = certifying_limit` (not `certifying_limit −
300`), it explores 300 + 3600. So the reachable benefit is not only the ~8 % of compute;
it is also a marginally higher certification rate, free. Both versions are valid pricers,
so the certified optimum, where one is reached, is identical either way.

**A cheaper variant is probably the right first move.** *Skip the regular tier once a run
has escalated* is a boolean on the loop — no state persistence, no memory cost — and it
captures nearly the whole 33.5 h, precisely because escalation is sticky (a cell that
escalates once escalates up to 20 times, section 2). Full resumability costs real
engineering and real RAM: it means holding live frontiers for every scenario across
rounds, and the frontier is already what OOM'd at n=40 on 24 G (see the Zhuzhou frontier
note). Take the boolean first; resumability earns its keep for the reason in 6.4, not for
the 8 %.

**The unmeasured part is larger than "how much of the 300 s would be saved."** The two
arms discard different amounts:

- **Parallel:** every scenario gets the *whole* round limit
  (`src/opt/label_setting/round.jl:134`), so a round's wall is the max over scenarios, not
  the sum. The discarded work is `n_scenarios × 300 s` of CPU but only 300 s of wall.
- **Serial:** the limit is divided and re-divided as scenarios exhaust
  (`round.jl:186-196`). Section 4's table shows this directly — job 26's longest search is
  1200.0 s = 3600/3, and job 23's 2307 s is consistent with two earlier scenarios leaving
  slack (1200 + 93 + 2307 = 3600).

So the 8.2 % in section 2 is a **CPU-time** figure, not a wall-time one, and the wall-time
saving differs by arm. Anything claiming a wall speedup from this remedy has to be
measured per arm.

### 6.2 Remedy 2 — uncap the certifying round, let it burn the total budget

Mechanically: `certifying_limit = remaining_budget()` instead of
`min(certifying_pricing_time_limit_sec, remaining_budget())`.

**What it buys is measurement honesty, not answers.** Right now the `stations=30` column
mixes runs that spent 6 h with runs that quit at 1.7 h, and that difference is an artifact
of the tier structure, not of the instances. Seven runs quit with 6.9–72.1 % unused, five
pinned at *exactly* 3600.0 s — the cap signing its own work (section 4). Any cross-seed or
serial-vs-parallel comparison at n=30 is therefore comparing unequal spend. Uncapping
makes every censored cell spend the same 6 h, which makes the row defensible rather than
contaminated.

**It is unlikely to buy certifications.** The escalating round already burned a full hour
and exhausted nothing, and label-search cost is super-linear in explored depth (the
max_stops 3→7 sweep). 6x the clock is far less than 6x the frontier. Expect a couple of
marginal seeds, not a regime change.

**The costs are real and asymmetric.**

- It converts cheap failures into expensive ones: those seven runs stop billing ~2 h and
  start billing 6 h each for the same non-answer, on a shared quota.
- It makes the budget non-preemptible — one dual vector's proof attempt swallows every
  iteration that would otherwise have followed. The incumbent survives (the final master
  re-solve at the bottom of the loop handles that), so what is lost is opportunity, not
  the answer.
- It interacts with the arms rather than being neutral to them: in serial, handing the
  round the whole remaining budget still gives each scenario only `budget / n_scenarios`,
  while parallel gives each scenario the full amount. Uncapping *widens* the arm gap.

### 6.3 Remedy 3 — the real n=30 blocker

One label search cannot exhaust in an hour. That is a pricer question (the dominance-scan
bottleneck), not a budget question. No budget tuning fixes it, and neither 6.1 nor 6.2
pretends to: 8 % and 6x are both small numbers against a search that needs orders of
magnitude.

### 6.4 What 6.1 and 6.2 jointly point at

The tier exists to stop a cheap timeout from masquerading as an optimality proof — a good
reason. But its trigger is "empty AND not exhausted", which on a hard instance is the
state the solve is in essentially always. The design assumes the short round *nearly*
proved it and one more push finishes the job; that assumption holds at n=10 (0 %
certifying) and inverts at n=30 (77.7 %, section 3). The structure misfires exactly where
the study is censored.

Read that way, **remedy 2 is the degenerate limit of remedy 1**. With a resumable pricer
there need not be two tiers at all: one monotone deadline that the loop extends as it
decides the run is worth continuing, no discarded work at any extension, and no cliff at
3600 s. That is the version of remedy 1 worth building if any is, and it subsumes remedy 2
rather than competing with it. Still undecided — recorded here as the shape of the option,
not as a plan.

## 7. Not yet done

- `analyze.jl` does not emit unused-budget. `censored_cells.csv` labels these rows
  `cause=pricing_inconclusive` and names the certifying limit, but does not quantify how
  much budget went unused, so section 4 is not reproducible from the results directory
  alone. Adding an `unused_budget_sec` column would fix that.
- The `certifying_pricing` split in section 1/3 is likewise computable only from
  `iterations/*.csv`, which is untracked (raw experiment data). Only this note carries it.
- The 8.2 % duplicated-search figure (section 2) is **CPU time**. The wall-time saving a
  resumable pricer would deliver is not measured and differs by arm (see 6.1): parallel
  discards `n_scenarios × 300 s` of CPU per escalation but only 300 s of wall, serial
  discards a re-divided slice. Needed before either remedy in section 6 is chosen.
