# A certification round exits as soon as one station search finds a column, and its cuts reset every attempt

> **Naming note (added 2026-09-10).** The outcome this note calls `:refuted` is now
> `:negative_rc_column_found`, and the metadata counter `cg_certification_refuted_rounds` is
> now `cg_certification_negative_rc_column_rounds`. The rename is because "refuted" read as
> a failure and was repeatedly misread as one: the relaxed-cluster mode **prices first and
> certifies second**, so an attempt that finds an improving real column is that iteration's
> pricing round doing its job, not a failed certification. Read every "refuted" below as
> "priced a column, so CG iterates again".

Two findings, both about where the 300 s pricing round's effort actually goes:

1. **The loop barely iterates.** Median 1 macro round and 1 station search per attempt, and
   82% of attempts finish in under 50 s of their 300 s budget -- because the first station
   search that refutes returns immediately.
2. **Cut sets are re-initialised empty every attempt**, so every summed cut count in prior
   analysis measures how many attempts ran, not how much cutting power accumulated.


Status 2026-09-09. Measured from the `attempts/` and `rounds/` dumps of
`2026-09-09_n40_reach3_...` (n=40, cap=16, 690 attempts, 1150 station searches, 10 seeds).
Reconstructed by segmenting each attempt on `round_idx` resets and `tier=='macro'` rows;
the reconstruction reproduces the attempts CSV exactly (690 = 690).

Written up because it answers a question none of the run-level counters could: the two-tier
loop is macro (outer) over meso (inner) over station searches, and every statistic we had
summed those together. Nobody could say how many of each actually fit in a round.

## The answer: about one of each

Ordinary attempts (budget 300 s), n=642:

| | median | distribution |
| --- | --- | --- |
| macro rounds per attempt | **1** | 67% do exactly 1; max 21 |
| station searches per attempt | **1** | 67% do exactly 1, 21% do 2; max 10 |
| station searches per macro round | **1** | 41% get 0, 46% get 1, 9.5% get 2, 3.3% get >=3 |
| attempt wall | **7.8 s** | p90 193 s |

**The 300 s is mostly unspent: 82% of ordinary attempts finish in under 50 s**, and only 5.8%
use more than 250 s. The inner meso loop essentially never iterates.

`RELAXED_CLUSTER_TWO_TIER_MAX_INNER_ROUNDS = 8` is therefore dead weight for a second,
independent reason: across all 690 attempts the maximum station searches in one macro round
is **7, once**, and the median is 1. The 09-09 frontier note attributes its deadness to the
old slice schedule; that is not the main reason.

## Why: refute-and-return, not budget

Split by outcome:

| outcome | n | med macro | med station | med sec | % using >250 s |
| --- | --- | --- | --- | --- | --- |
| refuted | 356 (55%) | 1 | 1 | **3.4 s** | 0% |
| certified | 188 (29%) | 4 | 2 | 8.9 s | 0% |
| inconclusive | 98 (15%) | 1 | 1 | **195 s** | 38% |

`subset_rc < -tol && return _result(:refuted)` -- **the first station search that finds an
improving column ends the entire attempt.** That is 55% of attempts, exiting in ~3 s. The meso
loop can only iterate when a station search comes back BARREN, which is the minority path. So
the loop is not burning its budget on deep search; it does one search, refutes, and returns to
the master.

Only the inconclusive 15% actually consume the round, which is where the 150 s derived station
slice binds. Both facts are consistent: the cap matters in exactly the attempts that decide
certification, and nowhere else.

## The structural consequence: certification lives in the OUTER loop

| attempt | macro rounds | station searches | result |
| --- | --- | --- | --- |
| seed 47 it30 sc1 | **1** | 3 | stuck, 262 s |
| seed 47 it38 sc1 | **1** | 2 | stuck, 35 s |
| seed 43 it19 sc1 | **13** | 4 | certified, 63 s |
| seed 43 it20 sc3 (escalated) | **16** | 8 | certified, 406 s |

Advancing the macro loop requires the meso sweep to exhaust, which requires its station
searches to keep proving supports barren. A stuck attempt never leaves macro round 1 and
earns 0 macro cuts; a certifying one runs 13-16 macro rounds, driving the relaxed bound from
-1143 to +47.7. **The macro layer is what certifies, and the failure mode is never reaching
it.**

## Cut sets are LOCAL to an attempt, which invalidates every summed cut count

`two_tier.jl` initialises `macro_cuts` and `meso_cuts` empty at the top of every attempt.
Nothing carries across. Per attempt:

| outcome | macro cuts built | meso cuts |
| --- | --- | --- |
| refuted | median **0** (max 8) | median 0 |
| certified | median 4 (max 28) | median 2 |
| inconclusive | median **0** (max 27) | median 0 |

So seed 43's "390 macro cuts" is a **sum over 63 attempts** -- 6.2 per attempt, median 0 --
not 390 accumulated cuts. This is why macro-cut count fails to discriminate certifiable seeds
(48 certified with 95, 47 failed with 103): it mostly counts how many attempts ran. It is also
why the 64-bit cut mask and `RELAXED_CLUSTER_MAX_CUT_ROUNDS = 65` never bind -- every attempt
restarts from an empty cut set against duals that have barely moved.

**Report cut yield per round.** The same error made the 09-09 note's cap table read backwards
(corrected in place there).

## Two candidate levers this suggests

Neither measured, both cheap to state:

1. **Persist cut sets across attempts.** A macro cut says "no route confined to these cells",
   which is a statement about route geometry against a partition fixed at build time -- it is
   not obviously dual-dependent. If sound, rediscovering it 63 times is pure waste. Needs a
   soundness argument before anything else.
2. **Do not return on first refutation.** Harvest the column and continue the meso loop so the
   attempt actually spends its 300 s building cuts. Costs staleness in the harvested columns.

## Caveat

All of this is n=40, cap=16, one arm, from one run's dumps. The refute-and-return structure is
code, so it generalises; the specific distributions are not known to.
