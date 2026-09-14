# The completion can be computed one unbuilt station at a time

2026-09-10. Job 22493919, `benchmarks/diagnostics/benders_pool_and_cut_anatomy.jl`. Zhuzhou
n=10 p=8 s=1 seed 42, k=5, `max_stops=4`, 16,320 enumerated columns, 4 anchors spread over
the Q ranking. Raw export in
`benchmarks/diagnostics/results/pool_anatomy/n10_p8_s1_seed42_ms4/` (schema in its README).

Follow-on from `2026-09-10_activated_dual_completion_verified_and_why_weak.md`, which
established that the closed-form completion charges ~8x the true dual's coefficient mass.
This asks what the pricer's columns actually differ by, and finds a construction.

## 1. A correction: the enumerated pool is NOT the column universe

Asserted repeatedly earlier today, including in the previous note and in
`subproblem.jl`/`CLAUDE.md`: "at `max_stops=4` the enumerated pool IS the universe". **False.**
386 columns in the two arms' pools are absent from the enumeration:

- **329 are strict SUBSETS** of an enumerated column on the same route. The enumerator lists
  the assignment sets a route can certify; the pricer emits the sub-assignment that is optimal
  at the current duals, keeping only triples whose net contribution is negative. Those are
  separate columns with separate dual constraints.
- **57 are rotations** of revisiting cycles -- pooled `9>3>9` where the enumeration holds
  `3>9>3`. Same physical cycle, same `tau`, same certifiable triples.
- **0 supersets, 0 incomparable.** So the two agree about what a route can certify; they
  differ only on which sub-assignments get listed.

Consequence for the earlier audit: its `min rc` over the enumerated pool was a minimum over a
strict SUBSET of the dual constraints, i.e. an upper bound on the true minimum, so "full-universe
dual feasible" did not follow from it as stated.

**Re-tested properly, and the conclusion survives.** The reduced-cost-minimising column on a
route keeps exactly the triples with negative net contribution, one per group, so the true
minimum is closed-form and needs no subset enumeration:

    min over the TRUE universe = min over routes R of
        [ beta*(tau_R + rho) + sum_p min(0, min over p's triples on R of net) ]
    net(p,j,k) = w*demand_p*walk(o,d,(j,k)) - (alpha_p - gammaO_pj - gammaD_pk)

Measured over every route x every sub-assignment, both arms, 4 anchors: worst
**-3.87e-12** (plain) and **-1.82e-12** (activated). The 57 rotations were checked directly
(worst rc -4.55e-12) and are covered by their enumerated twin anyway. `route_min_rc.csv`.

The soundness argument never rested on enumeration -- it rests on pricing exhaustion -- so
what needed fixing was the measurement's claim to be exhaustive, not the claim it measured.

## 2. `need_j` reproduces full CG's duals exactly

For an unbuilt `j`, take every column whose unbuilt assignment stations are exactly `{j}` and
maximise `excess_over_built = sum alpha - sum(gamma at built) - f_c`. That is what `j` must be
charged if the whole requirement lands on it. Against what plain CG's duals actually carry:

| anchor | station | `need_j` | plain CG | closed form | over-charge |
| --- | --- | --- | --- | --- | --- |
| 1,2,3,7,9 | 4 | 0.0 | 0.0 | 5210.3 | infinite |
| 1,2,3,7,9 | 6 | 0.0 | 0.0 | 5199.0 | infinite |
| 2,3,4,7,8 | 1 | 0.0 | 0.0 | 13041.7 | infinite |
| 2,3,4,7,8 | 6 | 0.0 | 0.0 | 6028.3 | infinite |
| 2,3,4,7,8 | 9 | 829.35 | 829.35 | 5998.4 | 7.2x |
| 1,2,3,7,10 | 4 | 1375.12 | 1392.37 | 9581.9 | 6.9x |
| 1,2,3,7,10 | 6 | 406.04 | 406.04 | 9570.6 | 23.6x |
| 1,2,3,7,10 | 9 | 2204.47 | 2204.47 | 9540.6 | 4.3x |
| 1,6,7,8,10 | 2 | 650.55 | 660.63 | 11073.4 | 16.8x |
| 1,6,7,8,10 | 3 | 1877.44 | 1877.44 | 16339.0 | 8.7x |
| 1,6,7,8,10 | 4 | 1619.63 | 1619.63 | 11053.3 | 6.8x |
| 1,6,7,8,10 | 9 | 2448.98 | 2448.98 | 11012.0 | 4.5x |

Exact agreement in 10 of 12. **Plain CG's `gamma` is not merely smaller -- it is tight against
the binding column.** The two discrepancies (1375 vs 1392, 650.6 vs 660.6) are exactly the
cells where columns serve 2+ passengers at that station: those constrain a SUM of `gamma_pj`
rather than one coordinate, so a per-station number is a lower bound there. The caveat fires
precisely where predicted, which is a check on the method rather than a defect.

Stations 5, 8, 10 have `n_cols_only_j = 0` -- no column can assign only at them -- so both
`need_j` and the closed form are 0. Those are the "not chargeable" stations, 2-3 of 5 unbuilt
per anchor.

## 3. The per-station values compose EMPIRICALLY -- but nothing certifies most of the universe

The obvious objection: a column touching two unbuilt stations is constrained by neither
per-station search. Tested directly against the export -- set `gamma_j = need_j` and check
`sum_{j in unbuilt(c)} need_j >= excess_over_built(c)` for every column with 2+ unbuilt
stations:

| anchor | multi-unbuilt columns | violations | worst shortfall |
| --- | --- | --- | --- |
| 1,2,3,7,9 | 8,288 | **0** | 0.0 |
| 2,3,4,7,8 | 11,918 | **0** | 0.0 |
| 1,2,3,7,10 | 15,012 | **0** | 0.0 |
| 1,6,7,8,10 | 15,868 | **0** | 0.0 |

**The proof sketch does NOT close, and the gap is most of the universe.** Corrected the same
day, after trying to write it out.

The sketch was: shortcut `c` past `j'` to get `c_j`, whose only unbuilt station is `j`, so
`excess(c_j) <= need_j`; bound the dropped `j'` part by `need_j'` symmetrically. It fails in
two places.

1. **The decomposition does not add up.** With `A_S` the assignments of `c` having both
   stations built, `excess(c)` counts the `A_S` part ONCE while `sum_j excess(c_j)` counts it
   `t` times. That over-count helps, but the cost terms do not decompose to match --
   `sum_j f_{c_j}` against `f_c` runs the other way -- so the inequality does not follow.

2. **An assignment with BOTH ends unbuilt is in no `c_j` at all**, so no per-station run
   certifies it. MEASURED, and it is not a corner case: columns containing such an
   assignment are **37% / 56% / 76% / 85%** of the universe at the four incumbents
   (`1|2|3|7|9`, `2|3|4|7|8`, `1|2|3|7|10`, `1|6|7|8|10`).

So the 0/51,086 result is real but is evidence without an argument over 37-85% of the
universe. The plausible reason it holds -- a route serving `j -> j'` between two
out-of-the-way stations pays heavy driving, so its excess is small while it draws on TWO
budgets -- was constructed after seeing the result and is not a proof.

**Do not build on this.** The failure mode is an invalid cut, i.e. pruning the true optimum
and reporting a wrong answer as OPTIMAL, which is the same class as the elementary-pricer
cut-validity bug (70-76% objective error, `notes/2026-07-27`).

A sound version costs the saving. A column has at most `max_stops` distinct stations, so it
touches at most `max_stops` unbuilt ones; pricing `S union T` for every `T` of that size
certifies everything, which is `C(20,4) = 4845` searches at n=40 against one search over 40.
Restricting `T` to the CHARGEABLE stations would fix that, but
`2026-09-10_activated_dual_completion_verified_and_why_weak.md` measured the chargeable set
to be the whole instance at n >= 20.

**All three attempts fail on one fact: columns do not decompose by unbuilt station, because
most touch several.** That is a property of the instances, not of the algorithm.

## 4. What the binding columns look like, and the construction

`best_route` across the 12 station-anchor cells:

    4>3>9>7   6>3>9>7   1>4>7   6>3>4>7   9>3>9>7   4>3>4>7
    6>3>2>7   9>3>9>7   6>1>2>7  6>3>7   4>1>4>7   9>1>9>7

Two structural facts:

- **11 of 12 START at the unbuilt station**, then run through built ones. The unbuilt station
  appears as the pickup for the groups that can only be served there (`4:4:3|7:4:3`), and the
  built stations carry the rest.
- **4 of 12 REVISIT the unbuilt station** -- `9>3>9>7`, `4>3>4>7`, `4>1>4>7`, `9>1>9>7`. So a
  per-station search MUST be revisit-tolerant. `:station_simple` searches elementary routes
  only, so it would report a too-low `need_j` and produce an INVALID cut. This is the
  cut-strength analogue of `project_non_elementary_optimal_routes_exist`.

And the selectivity: `n_cols_only_j` runs 37 to 3,781, so thousands of candidates per station
and exactly one binding.

**The construction.** For each unbuilt `j`, exhaustively price over `S union {j}` -- `k+1`
stations -- and read `gamma_j` off it. That is `(n-k)` searches of `k+1` stations instead of
one search of `n`, and they are embarrassingly parallel. At n=40/k=20 that is 20 searches over
21 stations against one over 40, and the measured pricing frontier puts 21 comfortably inside
what exhausts while 40 is past it (`project_pfa_cg_scaling_frontier_zhuzhou`: n<=20 all
scenarios, n=25 to <=5, n=30 only s=1, n=40 OOM at 24G).

**This does NOT rescue the localisation idea** (an earlier draft of this note claimed it
did). That refutation was of
covering the whole chargeable set `F` in ONE search set, which fails because `F` is the whole
instance at n>=20. Doing it one station at a time never needs `F` to be small -- `|T| = 1`,
repeated.

## Open before this is worth building

1. **Cost.** 20 searches over 21 stations vs 1 over 40 is the bet, and neither side is
   measured. A 21-station exhaustion could still be expensive x20.
2. **Composition.** Section 3 is 4 anchors at n=10. Needs the same check at n=15/20, and
   ideally the proof sketch closed.
3. **Per-`(p,j)` rather than per-station.** `need_j` is an aggregate; a real implementation
   reads the per-`(p,j)` duals off each restricted solve. The two multi-passenger cells above
   are where that distinction bites.
