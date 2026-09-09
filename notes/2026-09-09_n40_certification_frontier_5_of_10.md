# n=40 certification: 4/10 -> 5/10 at one third the budget

Status 2026-09-09. Supersedes the parameter recommendations in
`2026-09-08_two_tier_certification_n40_status.md`. Everything below is measured, with the
disconfirming results kept in.

## Result

| configuration | budget | certified |
| --- | --- | --- |
| published single-tier `relaxed_k60` | 21600 s | 4/10 -- 44, 46, 48, 49 |
| published two-tier `twotier_k60m14` | 21600 s | 4/10 -- 44, 46, 48, 49 |
| single-tier control | 7200 s | **3/10** -- 44, 48, 49 |
| `twotier_k60m14c20g3` (K1=14, cap=20, g=3) | 7200 s | **5/10** -- **43**, 44, 46, 48, 49 |
| `twotier_k60m16c16g3` (K1=16, cap=16, g=3) | 7200 s | **5/10** -- **43**, 44, 46, 48, 49 |

**Seed 43 is new**: no configuration at any budget up to 21600 s had certified it. Two
independent configurations do now (2610 s and 7016 s), agreeing on the objective
33284.23313708192 at `full_route_universe` scope. Four earlier runs -- the 21600 s
`relaxed_k60`, `twotier_k60m14` and `twotier_k80m16`, plus the cap=20 arm here -- had
already driven the LP to that exact value without being able to prove it. The pool was
converged; only the certificate was missing.

The 6/10 target was NOT reached. Seeds 42, 45, 47, 50, 51 remain uncertified by anything,
and at cap=16 all five now exhaust 95-100% of their budget before failing, so they are no
longer budget-limited.

Speed on shared seeds (fastest observed): seed 44 87 s (was 96 s two-tier / 6150 s
single-tier), seed 46 1291 s (was 9376 s / 19524 s), seed 48 78 s, seed 49 155 s.

## The parameter that was never swept

`CGPricingConfig.relaxed_cluster_aligned_subset_max` defaulted to **15** through every
measurement in the previous note, and it gates whether a barrenness proof can reach the
macro layer at all -- which is the only layer that can certify. It is now the 19th job
column in Study 9 (`run_benchmark.jl` accepts 17, 18 or 19 fields).

It trades against K1 through

    |aligned stations| ~ (macro cells the support touches) x (n / K1)

so a COARSE macro layer needs a LARGER budget to align the same support. This is the
opposite of what the round-cost diagnostics recommend, and it is why their advice was wrong
(below).

### The cap has a two-sided optimum -- but NOT because of cut supply

Seed 43, at fixed K2=24 / K1=16 / g=3. All three arms ran exactly 63 attempts.

CORRECTED 2026-09-09. An earlier version of this table reported SUMMED macro cuts (342 /
390 / 345) and read cap=16's 390 as the mechanism. That is a round-count artefact and the
reading was wrong. Cuts are built per round, the cut sets are local to an attempt
(`two_tier.jl` initialises `macro_cuts`/`meso_cuts` empty every attempt), and a sum over
attempts therefore measures mostly how many rounds ran. **Report cut yield per round.**

| cap | outcome | macro rnds | **macro cuts/round** | **meso cuts/search** | **alignSkip/search** | **unexh/search** |
| --- | --- | --- | --- | --- | --- | --- |
| 13 | failed, 4954 s | 273 | 1.253 | 0.769 | 0.205 | 0.061 |
| **16** | **CERTIFIED, 2610 s** | 303 | **1.287** | 0.733 | 0.027 | 0.090 |
| 20 | failed, 2586 s | 268 | **1.287** | 0.684 | 0.000 | 0.142 |

Per round the three caps are indistinguishable on cut yield: cap=16 and cap=20 are
IDENTICAL at 1.287 macro cuts/round, and cap=13 is 2.6% below them while having the HIGHEST
meso yield. Per attempt they are indistinguishable too -- 44% zero-cut attempts, median 2,
in all three arms. cap=16 did not cut more productively; it got 303 macro rounds in against
273 and 268.

What the cap really controls, and both gradients are monotone and real:

- **alignment refusals** fall with the cap (0.205 -> 0.027 -> 0.000 per station search), the
  too-small half;
- **unexhausted searches** rise with it (0.061 -> 0.090 -> 0.142), the too-large half, and
  only an EXHAUSTED search licenses a cut.

But neither gradient shows up as a cut-yield difference, so "too small starves the macro
layer" -- the previous reading -- is NOT supported per round.

### What actually separates them: cost concentration

| cap | blocking rounds | per search | cost of those rounds | sec/macro round | wall |
| --- | --- | --- | --- | --- | --- |
| 13 | 2 | 0.009 | **2001 s** | 24.9 | 4954 |
| **16** | 2 | 0.010 | **53 s** | **13.8** | **2610** |
| 20 | 2 | 0.010 | **278 s** | 15.5 | 2586 |

A *blocking* round is one that is unexhausted AND found nothing improving: no cut, no
column, nothing learned. All three caps produce **exactly two** of them, at an identical
rate. What differs is what those two rounds COST -- cap=13 burns 2001 s on them, 40% of its
entire run.

So cap=16 wins because its rounds are CHEAP (13.8 s per macro round against 24.9), so more
of them fit inside the budget -- not because each one yields more. cap=16 with K1=16 lands
the priced supports at a median of 12 stations, immediately below the exhaustion cliff,
which is what keeps the cost down. **Recommend cap=16, K1=16, g=3 at n=40** -- the
recommendation survives; only the explanation changed.

## The exhaustion cliff (1230 station searches, K2=24)

| stations | n | exhausted | median sec |
| --- | --- | --- | --- |
| 7-11 | 737 | 100% | 0.06-0.83 |
| 12 | 102 | 99% | 2.02 |
| 13 | 163 | 89% | 3.40 |
| **14** | 68 | **50%** | **161.92** |
| 15-16 | 111 | 36-40% | 150-180 |
| 17 | 29 | 3% | 150 |
| 18-20 | 20 | **0%** | 150 |

`<=12` stations: 838/839 exhausted. `>12`: 222/391.

**CORRECTED 2026-09-09: this table is CONFOUNDED by the slice each search was granted, and
the original reading of it -- "a property of the exact pricer, not of the cut machinery, and
what sets the frontier" -- is not supported.** The station search receives
`min(headroom, 0.5 x attempt_budget)`, which is only **150 s** inside an ordinary 300 s
attempt. Of 1150 station rounds at cap=16: 976 ran at the 30 s base slice, 142 at ~150 s,
and only **32** ever reached the escalated tier. 97% of unexhausted rounds ran to >=0.95 of
their slice, i.e. were CUT OFF rather than finishing. Splitting the same rounds by the slice
they were given:

| stations | slice ~150 s | slice >200 s |
| --- | --- | --- |
| 14 | 100% exh | 86% exh |
| 15 | **0% exh** (n=43, median 163 s = the cap) | **86% exh** (median 517 s) |
| 16 | 47% exh (median 180 s) | 73% exh |
| 17 | **0% exh** (median 180 s) | **50% exh** (median 525 s) |
| 18 | -- | 67% exh (n=3) |

Searches that DO exhaust at the big slice cost 57-974 s, median ~250 s -- routinely 2-6.5x
the 150 s cap they normally get. So the "median 150-180 s at 14+ stations" above is the CAP
SATURATING, not the search's cost, and the 0% at 18-20 is 20 rounds that were never given
time to finish. The pricer is expensive, but this table does not establish it as the
frontier; what it establishes is that the pricer is expensive *relative to a 150 s slice*.

## What predicts certifiability (and what does not)

At cap=16, `two_tier_station_unexhausted` separates the seeds with no overlap:

- certified: 44 (0), 49 (1), 48 (4), 46 (6), 43 (20)
- failed: 50 (39), 51 (52), 45 (57), 42 (60), 47 (83)

Macro-cut count does NOT separate them -- seed 48 certified with 95 cuts while seed 47
failed with 103. An earlier draft of this claim (cut yield as the discriminator) was wrong.
Unexhausted searches are the right quantity because each one is a support whose barrenness
can never be established, so its proof is unavailable at any budget.

`two_tier_align_skipped` was the discriminator BEFORE the fix, perfectly and with no
overlap (certified <=7%, failed >=53% across 7 runs), which is what identified the cap as
the problem in the first place.

## Three bugs fixed

1. **Alignment starvation.** At K1=12 with the default cap, alignment was refused on 105 of
   111 station searches and the run produced **ZERO** macro cuts, so it could not certify
   however much budget remained -- it quit having used 6%. Fixed by the cap, plus
   `_two_tier_aligned_support` now SHRINKING the guide prefix (all `g`, then `g-1`, ...)
   until the aligned set fits, rather than abandoning the macro cut. Sound: soundness never
   depended on which guides produced the support, only on cutting exclusively on a support
   whose search exhausted.

2. **The inner slice schedule ran backwards.** The meso sweep took `0.5 * remaining`, so
   slices fell 142 s / 56 s / 13 s while each round carries one MORE meso cut and is
   therefore harder. Only 3 inner rounds fit in a 300 s attempt and 6 in a 3600 s one, so
   `RELAXED_CLUSTER_TWO_TIER_MAX_INNER_ROUNDS = 8` never bound and the previous note's
   "65 -> 8" change did nothing. Now `25 s * 1.7^(k-1)`, rising with the cut load. A
   truncated sweep also used to return `:meso_unexhausted` and kill the whole attempt over
   a scheduling artefact; it now retries once on the real remaining budget.

3. **Escalation caps in absolute rather than budget-relative units.** The station-search
   escalation was `4 x 30 s`, which clamps identically in a 300 s ordinary attempt and an
   1800 s escalated one -- so the escalated tier's extra 1500 s was unreachable by the very
   search that provoked the escalation. Seeds 43 and 50 stopped with 64% and 70% of budget
   unspent for this reason. Now a SHARE (half) of the attempt's own budget. With the
   escalated attempt limit also raised 1800 -> 3600 s, runs now use 95-100% of budget.

**The `g=1` recommendation in the previous note is wrong and actively harmful.** Cleanest
evidence, seed 49 at fixed K1=12 and fixed budget: g=1 gives `align_skipped=49`,
`guides_used=0.64` and FAILS; g=3 gives 25, 1.85 and CERTIFIES -- identical LP objective
28148.510 in both. At g=1 there is no prefix left to shrink.

## Why the diagnostics recommended the wrong values

`two_tier_param_scan.jl` and `relaxed_cluster_two_tier_guide.jl` both rank configurations
by predicted ROUND COST. Certification is gated on macro-cut SUPPLY, and the two objectives
point in opposite directions: g=1 and K1=12 are cheap per round and starve the macro layer.
Both of their recommendations (g=1, K1=12) were measured harmful here.

This is a SECOND, independent reason to distrust them beyond the late-duals problem the
previous note identified, and capturing late duals does not fix it. Any future scan must
rank on cut yield, not seconds. (Late-dual capture via `STUDY9_EXPORT_DUALS=1` / `PS_DUALS`
remains built and still unused; `duals/` is empty.)

Also fixed: `generate_jobs.jl` hard-coded `k1 = round(0.6 * k2)`, giving 11 at K2=18 and 19
at K2=32, both outside the measured 12-16 band. K1 is an absolute node count, not a
fraction of K2. Now clamped.

## Instrumentation added

The previous note's per-tier counters existed but were summed to run level, so no question
about a specific iteration could be asked. Now:

- `iteration` on every per-attempt stat row (threaded through six signatures), so attempts
  can be ordered in time.
- per-round `sec` / `slice` / `exhausted` in the trace -- `sec` vs `slice` distinguishes a
  round that finished from one its cap cut off. The cliff table above needs exactly this.
- `attempts/<stem>.csv` and `rounds/<stem>.csv` dumps (neither matching the result glob).
- `two_tier_meso_escalations`, `two_tier_station_escalations`,
  `two_tier_guides_used_mean`, `aligned_subset_max` on the result row.
- `two_tier_exit_reasons` now actually lands. It was fully coded before but no run had ever
  produced it (`run_benchmark.jl` changed after every in-flight task had started).

## The honest bottom line

The mechanism is safe -- every seed certified by two configurations agrees on the objective,
all at `full_route_universe` scope -- and it is now 5/10 at 7200 s against a published 4/10
at 21600 s, and 5/10 against a matched-budget control's 3/10.

But 5/10 is one seed, not the 6-8/10 hoped for, and the remaining five seeds fail with
95-100% of budget consumed.

**CORRECTED 2026-09-09.** This section originally concluded "the frontier at n=40 is set by
the exact pricer's inability to exhaustively search a 13+ station subset, not by cut
plumbing, budget, or scheduling." The per-round data does not support that. The pricer
cannot exhaust a 13+ station subset *in 150 s*, which is the derived slice an ordinary
300 s attempt grants it; given the escalated slice it exhausts 75% of the time (24/32),
including an 18-station subset in 401 s. And of the 96 rounds that are genuinely blocking
(unexhausted AND nothing improving, so no cut and no column), **86 died at that ~150 s cap**
and only 3 at the escalated tier. It is a scheduling bound at least as much as a pricer
bound. The three bugs were real and worth fixing; they bought a 3x budget reduction and one
seed.

Getting further needs a cheaper exhaustive subset search, not another parameter. Candidates,
in the order I would try them: reduce the aligned-support size structurally (a nested THIRD
tier, so rounding up costs fewer stations); or accept partial proofs (a cut valid only over
the routes a truncated search did cover -- unsound in the current form, see the
`|route ∩ T| <= |T|-1` discussion in `two_tier.jl`, so this needs new theory); or attack the
pricer's cost directly, which is the `project_pfa_label_search_dominance_bottleneck` thread.

## Open

- `RELAXED_CLUSTER_TWO_TIER_MAX_INNER_ROUNDS = 8` is still dead weight under the new
  schedule; it should be re-derived or removed rather than left as a constant nothing hits.
- The last-ditch escalation is still config-shaped, not code-shaped: when the solver is
  about to stop on `pricing_inconclusive` it should spend the ENTIRE remaining budget on one
  final attempt, since the alternative is stopping. Raising `certifying_limit_sec` to 3600
  approximates this but wastes budget on non-terminal escalations.
- `certify.jl`'s single-tier loop still has untagged `:inconclusive` exits.
- The old `twotier_k80m16` arm (K2=32) finished 3/10 with all 10 rows. Its seed 46 ran
  21508 s of a 21600 s budget over 59 iterations and still failed, where K2=24/K1=16/cap=16
  certifies that seed in 1966 s -- an 11x gap on a seed the coarse arm never closed at all.
  It is also the only run where the alignment DOWNGRADE path fired heavily
  (`align_downgraded=40`, against 0-8 everywhere else), i.e. alignment kept being applied
  and then timing out, which is the cap-too-large half of the two-sided optimum.
