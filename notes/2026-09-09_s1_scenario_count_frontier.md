# The scenario count, not the station count, was the binding axis

Status 2026-09-09, COMPLETE (40/40 cells, Study 10, array 22409240 + 9 salvaged cells from
22405869). Companion to `2026-09-09_n40_certification_frontier_5_of_10.md` and
`2026-09-09_per_scenario_escalation_negative_result.md`, and it partly supersedes the reading
in both.

## Result

Every Study 9 measurement was at s=3 -- all 490 job rows it ever wrote -- so `s` had never
been varied. Varying it moves the frontier more than any parameter tried in three prior runs.

| | s=3 (Study 9) | s=1 (Study 10) |
| --- | --- | --- |
| n=40 single-tier | 4/10 @ 21600 s -- 44, 46, 48, 49 | **6/10 @ 7200 s** -- 43, 44, 48, 49, **50**, **51** |
| n=40 two-tier | 5/10 @ 7200 s -- 43, 44, 46, 48, 49 | **7/10** -- +**50**, **51** |
| n=50 single-tier | 0/4 @ 21600 s | 1/10 -- 48 |
| n=50 two-tier | never run | **5/10** -- 43, 45, 48, 50, 51 |

**n=50 went from 0/4 to 5/10.** Seeds 50 and 51 at n=40 certified in both arms having never
certified at s=3 at any budget up to 21600 s. Three prior runs (reach3/4/5) moved cap, K1, g,
the escalation plumbing and the pricing budget without ever exceeding 5/10 at n=40; s=1
reaches 7/10 and opens a size class that was closed.

Safety holds: 7 instances certified by both arms, all agreeing on the objective exactly; 0
errors; every result `full_route_universe`.

## Two-tier is dominant at s=1, which s=3 never showed

Paired on identical instances where both arms certified:

| n | seed | single | two-tier | speedup |
| --- | --- | --- | --- | --- |
| 40 | 44 | 3320.7 s | 56.2 s | **59.1x** |
| 40 | 51 | 1572.2 s | 63.8 s | 24.6x |
| 40 | 43 | 1670.5 s | 100.8 s | 16.6x |
| 40 | 50 | 314.3 s | 28.8 s | 10.9x |
| 40 | 48 | 341.9 s | 40.0 s | 8.5x |
| 40 | 49 | 168.1 s | 30.5 s | 5.5x |
| 50 | 48 | 4459.1 s | 2684.3 s | 1.7x |

Median **10.9x**, two-tier wins every pair. At s=3 the two-tier machinery bought 5/10 against
4/10 and its value was arguable; at s=1 it is not. Note the single n=50 pair is only 1.7x, so
the advantage may compress with size -- one data point.

Time to certify: n=40 median 168 s (two-tier median ~57 s, min 29 s), n=50 median 2004 s.
Roughly a 12x jump for ten more stations.

## The LP is integral, and always was

`recover_integer_solution=true`, so `objective_value` is a real MIP solve over the final pool
and `lp_objective_value` is the CG master bound. **Max absolute gap 1.8e-12 across 19
certified s=1 runs**, and 0.000% across 125 certified s=3 runs (n=8-40) too. Integer recovery
costs 0.08-0.20 s. So the s=1 win is entirely certifiability and speed; there was no bound
quality left to win.

Scope: this is Zhuzhou at p=16, seeds 42-51. Other demand structures are untested, and the
grid family has behaved differently from Zhuzhou before.

## Disconfirming results, kept in

**s=1 is not uniformly easier.** n=40 seed 46 certified under single-tier at s=3 and does NOT
at s=1. `y` is shared across scenarios, so at s=1 station selection is driven by one
scenario's demand alone: different master, ~1/3 the columns, different optimum (objectives
~8.6-10.9k vs ~28-33k). This is a frontier shift, not the same instances made easier, and
individual cells can go either way.

**Seeds 45 and 47 still fail at n=40 two-tier**, exactly as at s=3. There is residual
per-scenario hardness; it just was not what blocked most seeds.

**Seed 45 fails at n=40 but certifies at n=50.** Changing `n` changes the station set, so
per-seed difficulty is not monotone in size.

**The s=1 failures are not budget-limited.** All 21 stop on `pricing_inconclusive` with a
median 736 s and up to 4758 s (66%) of budget unspent. That is a different failure mode from
s=3 at n=40, where failures consumed 95-100%. Worth testing whether relaxing the
two-consecutive-inconclusive stop rescues any of them before calling them out of reach.

## What this says about the prior two notes

The `4/10 -> 5/10` note concluded the frontier is set by the exact pricer's cost. This run
does not overturn that for a fixed `s`, but it shows the pricer was being asked a question
three times over, and that the conjunction -- every scenario must certify in the SAME
iteration -- was carrying more of the difficulty than the pricer's per-search cost.

The per-scenario-escalation note's negative result stands and is now better explained: that
change removed a real defect (a productive scenario masking a stuck one) but could not help
much, because with three scenarios the round still had to win three simultaneous proofs.

## Claims of mine this session refuted, and by what

- **"The failing seeds contain individually-uncertifiable scenarios, so s=1 will not help."**
  Argued from reach5's per-scenario dumps (seed 42: zero certifications across all three
  scenarios in 59 attempts; 8 of 10 scenario-slots C=0). Seeds 50 and 51 then certified at
  s=1 in 314 s and 64 s. The per-scenario census was real but did not license the prediction.
- **"The 150 s slice cap, not the pricer, is the binding constraint."** Both ways of removing
  it -- lifting it (600 s pricing round) and routing around it (per-scenario escalation) --
  work mechanically and neither moved certification.
- **"The LP/IP gap risk is live, cf. the 21.6% hub-route case."** That measurement is from
  `CompatibilitySetAssignmentModel`, which no longer exists in the package, with an `l`
  activation stage the current formulations do not have, at p=8 and seed 123 -- and its note
  reports identical LP/IP figures for two different instance sizes, so the figures are
  unreliable. The mechanism is worth watching; the number was not evidence about current work.
- **"Cap=16 wins by producing more macro cuts (390 vs 342/345)."** Those are sums over
  attempts, and cut sets are local to an attempt. Per round the three caps are
  indistinguishable (1.253/1.287/1.287). Corrected in place in the 09-09 note.

## Open

- Why does the conjunction cost so much? Round success being roughly the product of three
  per-scenario successes predicts the direction but has not been checked quantitatively
  against the per-scenario rates.
- Does s=2 sit between? Nothing measured between 1 and 3.
- The 21 `pricing_inconclusive` stops leaving up to 66% of budget unused.
- n=50 single-tier is 1/10 against two-tier's 5/10 -- the largest arm gap seen anywhere, and
  worth a look on its own.
