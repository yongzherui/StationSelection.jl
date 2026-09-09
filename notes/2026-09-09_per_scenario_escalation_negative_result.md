# Per-scenario escalation and a 600 s pricing round: NEGATIVE at n=40

Status 2026-09-09. Companion to `2026-09-09_n40_certification_frontier_5_of_10.md`, whose
cap table, exhaustion cliff and bottom line this run's analysis also corrected in place.

**Headline: 5/10 in every arm, on the identical seed set {43, 44, 46, 48, 49}. No change
against the published baseline, and three of the five failing seeds reached a WORSE LP
bound.** Run `2026-09-09_n40_reach5_...`, SLURM array 22389096, config `n40_reach5.tsv`,
K2=24 / K1=16 / cap=16 / g=3, 7200 s budget, 10 seeds x 2 arms.

## What was tried

Two changes, deliberately split into separate arms so they could be attributed:

- **`p300`** -- per-scenario escalation, otherwise identical to the baseline. Isolates the
  code fix.
- **`p600`** -- the same fix plus `pricing_time_limit_sec` 300 -> 600 s, which doubles the
  derived station-search slice from 150 s to 300 s.

The code fix: escalation used to be decided round-wide AND only when the round produced no
columns at all, so `cg_solver.jl`'s harvest-and-continue path skipped past it entirely. One
productive scenario therefore masked a permanently stuck one indefinitely. It now escalates
exactly the scenarios that came back `:inconclusive`, via
`RelaxedClusterCertificationResult.inconclusive_scenarios` and `cg_certification_round`'s
`only_scenarios`.

## Result

| seed | baseline | p300 | p600 |
| --- | --- | --- | --- |
| 42 | fail 7077 s 32it | fail 7219 s 18it | fail 7215 s 24it |
| 43 | **CERT 2610 s** 20it | **CERT 6142 s** 20it | **CERT 3259 s** 20it |
| 44 | **CERT 177 s** | **CERT 173 s** | **CERT 150 s** |
| 45 | fail 7211 s 12it | fail 7217 s 9it | (last job) |
| 46 | **CERT 1966 s** | **CERT 785 s** | **CERT 582 s** |
| 47 | fail 7210 s 38it | fail 7210 s 19it | fail 7207 s 19it |
| 48 | **CERT 533 s** | **CERT 483 s** | **CERT 233 s** |
| 49 | **CERT 205 s** | **CERT 211 s** | **CERT 176 s** |
| 50 | fail 7026 s 16it | fail **5426 s** 10it | fail 7017 s 12it |
| 51 | fail 6845 s 26it | fail 7213 s 11it | fail 7212 s 11it |

LP bound reached on the seeds that failed (lower is closer):

| seed | baseline | p300 | delta |
| --- | --- | --- | --- |
| 42 | 31604.13 | 32289.46 | **+685** |
| 45 | 34955.92 | 35698.27 | **+742** |
| 47 | 32043.01 | 32037.95 | -5.06 |
| 50 | 30271.39 | 30271.39 | 0 |
| 51 | 31587.79 | 32337.78 | **+750** |

## What DID happen (mechanism confirmed, outcome unchanged)

The fix does what it was designed to do, and seed 47 is the clean demonstration. Baseline:
**0 escalated attempts in 38 iterations**, scenario 1 replaying a bit-identical 262 s search
24 consecutive times, master frozen at 32043.0090 from iteration 19 to 38. With the fix: 3
escalations, the replay stops, **19 iterations instead of 38**, and the LP passes the frozen
value with FEWER columns (3373 vs 3424).

It is also cheaper where it was wasteful. Escalated attempts fell 48 -> 26 (p300) -> 14
(p600), because 29 of the baseline's 48 were re-runs of scenarios that already had a verdict.
That is where seed 46's 1966 s -> 582 s and seed 48's 533 s -> 233 s come from.

**But none of it certifies anything new.** Seed 47's gain is -5.06 on ~32040, i.e. 0.016% --
noise at the scale that matters. An earlier reading of this run called it "breaking the
freeze" and treated it as progress; it is a mechanism confirmation, not progress.

## Regressions, kept in

1. **An escalation that costs 3824 s and returns NOTHING.** Seed 50 p300 iteration 10:
   `inconclusive_escalated`, 3824 s, **0 columns accepted** -- 53% of that run's entire wall
   for no verdict and no column. This is the `inconclusive -> inconclusive` tail (4/19 in
   reach3) landing at full price.

   RETRACTED, and worth keeping as a lesson in reading a symptom instead of the code: an
   earlier draft called this "`pricing_inconclusive` fires early and abandons budget... a
   plain bug", on the grounds that seed 50 p300 stopped at 5426 s with 1774 s unspent and the
   fix was to CONTINUE when the round still harvested columns. Both halves are wrong. That
   round harvested nothing, so the continue-condition would not have fired; and stopping was
   correct, because another attempt at unchanged duals is a replay. Checked across all nine
   uncertified runs, seed 50 p300 is the ONLY one that stopped with budget to spare -- every
   other stop consumed 7207-7219 s of 7200. And it reached the IDENTICAL LP bound
   (30271.3882) as the baseline in 5426 s against 7026 s, so it is 1600 s better, not a
   regression at all. The real waste is the 3824 s escalation itself, not the stop that
   followed it. `inconclusive_escalated` is also not per se wasteful: seed 47 iterations 16
   and 17 came back inconclusive_escalated and still harvested 52 and 16 columns.
2. **Seed 51 went backwards**: LP 32337.78 vs 31587.79, 11 iterations vs 26. Escalations ate
   the budget without paying for themselves. Seeds 42 and 45 likewise, by +685 and +742.
3. **Seed 43 certified 2.4x slower** under p300 (6142 s vs 2610 s), with 7 escalations
   against the baseline's 3 -- the recurrence cost, as predicted before the run. p600 avoided
   it (3259 s, and **0 inconclusive rounds, 0 escalations** -- the 600 s round removed
   inconclusiveness on that seed entirely), but was still slower than the baseline.

## Claims of mine this run refuted

- **"The 150 s slice cap is the binding constraint, not the pricer."** Argued from the
  reach3 dumps: 86 of 96 blocking rounds died at that cap, and 15-station searches exhaust
  0% at 150 s against 86% above 200 s. All true, and it does not matter. Both ways of
  removing the cap -- lifting it (p600) and routing around it (escalation) -- work
  mechanically and neither moves certification. The `2026-09-09` note's original conclusion,
  that the pricer's cost sets the frontier, survives this run better than my correction to
  it did. What my correction got right is narrower: the specific exhaustion-cliff TABLE was
  confounded by the slice, so it could not be used as evidence for the claim it was cited
  for.
- **"Escalation recurrence will be affordable because resolution costs less than the cap."**
  Median resolving escalation is 632 s and the budget is 7200 s, so a run gets ~10. Seeds
  42/45/51 show that is enough to consume the budget and not enough to change a verdict.

## Bookmarked: guarantee dual movement in EVERY scenario

The idea this run leaves behind, and the reason the negative result is not the end of the
thread. Escalating a stuck scenario was the right diagnosis of the symptom (dual replay) but
the wrong lever, because it resolves ONE scenario ONCE and the next iteration re-poses the
same question.

**Proposal: no scenario is allowed to end an iteration without contributing a new route.**
Rather than letting each scenario terminate on its own verdict -- certified, refuted, or
inconclusive -- and accepting whatever the round happens to harvest, require every scenario
to hand back at least one new column each iteration. A scenario that certifies has genuinely
nothing to give and is exempt; the case that matters is the inconclusive one, which today
returns nothing and therefore leaves its own duals untouched, which is precisely what makes
the next iteration a replay of this one.

Why this is the right shape:

- The replay is not caused by the search being too short. It is caused by the search
  returning NOTHING, so the master's solution for that scenario does not change, so the duals
  do not change, so the next search is bit-identical. MEASURED at n=40 seed 47: scenario 1's
  macro `relaxed_rc` was -2000.6 at iterations 9, 13, 30 and 38 -- identical to the decimal
  across 29 iterations.
- Escalation attacks this only by making the search succeed, which costs 632 s a time and
  works 79% of the time. Forcing a column out attacks it directly and cheaply: ANY new column
  in that scenario perturbs its duals and guarantees the next iteration asks a different
  question.
- It also repairs the early-quit above for free: a round in which every scenario contributed
  is never a round with nothing to add, so `pricing_inconclusive` stops being reachable while
  work remains.

Open design questions, none answered yet:

- Where does the forced column come from when the exact search exhausts its slice without
  finding an improving one? Candidates: the best non-improving label the truncated search
  already has in hand (cheap, but a non-improving column may be rejected by the master or be
  a no-op on the duals); a route harvested from the relaxed cluster layer and repaired to a
  real one; or a deliberately perturbed variant of an existing column in that scenario.
- Does adding a non-improving column actually move the duals, or does the master keep the
  same basis? This is the crux and it is testable offline against the captured dual vectors
  (`STUDY9_EXPORT_DUALS=1` / `PS_DUALS` is built and still unused; `duals/` is empty).
- Soundness is not at risk -- extra columns can only tighten the master and never invalidate
  a certificate, which is established only by an exhausted relaxation. The risk is pool
  bloat and churn, not correctness.

## Keep regardless

- Per-scenario escalation itself. It is correct, it deletes the 29/48 wasted re-runs, and it
  is what made the replay visible. It just is not sufficient.
- Bound the escalation so a single attempt cannot eat half a run for nothing (regression 1).
  A cap relative to REMAINING budget rather than `certifying_pricing_time_limit_sec`, or the
  no-re-buy guard at unchanged duals, would both have stopped seed 50's 3824 s attempt. NOT
  the "continue instead of stopping" fix an earlier draft proposed -- see regression 1.
- Do NOT keep tuning cap/K1/g. Three runs (reach3, reach4, reach5) have now moved parameters
  and plumbing without moving 5/10.
