# Certifiability is not predictable from the instance: 20 measures, all negative

Status 2026-09-09. Companion to `2026-09-09_n40_certification_frontier_5_of_10.md`, which
owns the parameter recommendations and the frontier result. This note owns one question that
note does not ask: **can you tell, before solving, whether an n=40 instance will certify?**

Answer: no, by 20 measurements. The only quantity known to separate the seeds is
solve-derived (`two_tier_station_unexhausted`, in the companion note), and nothing about the
instance itself predicts it.

**Scope warning.** The n=40 anytime run this note analyses (`2026-09-08_n40_anytime_...`,
array 22325389, 1800 s budget, K1=14 / cap=default 15 / g=5) PREDATES the three bug fixes in
the companion note. Its certification counts are therefore not comparable to the current
5/10, and are not offered as a frontier result. What survives the code change is the
instance-level analysis, which never depended on the pricer's plumbing, and the two
arm-level facts below.

## The generator makes a seed sweep a DEMAND sweep

`scripts/generate_zhuzhou_instance.jl` takes `top_rows = all_rows[1:n_stations]` -- the
top-N most popular stations, deterministically. The seed enters at exactly one line,
`MersenneTwister(seed + (s-1)*1000)`, feeding the per-scenario weighted OD-pair sample.

So across seeds at fixed `n_stations`: station set, coordinates, walking costs,
Floyd-Warshall routing costs and the entire k-medoids partition are IDENTICAL. Measured
confirmation: mean intra-cell dispersion at n=40 / K2=24 is 22.1 for all ten seeds 42-51, to
three significant figures.

**Consequence for experiment design: any hypothesis about geography, station layout, cluster
tightness or map structure is untestable across a seed sweep.** Those variables do not vary.
Varying them needs a different station-selection rule, or `demand_station_count` (which
already exists to decouple candidate-station count from demand endpoints).

It also means certifiability swings from 78 s to never-at-21600 s on a FIXED map. Geography
cannot be the explanation, whatever else is.

## What the 16 requests look like

Every seed is the same shape: a **star centred on station 1**, which is what Zipf-weighted
endpoint sampling produces. Per scenario, 10-13 distinct stations of 40, one hub carrying
56-75% of the 16 requests, 15-16 of 16 requests sharing an endpoint with another, and 10-15
"chains" (one request's destination is another's origin). Seeds differ in *which* peripheral
stations get drawn -- seed 48 stays on stations 1-7 plus one outlier, seed 47 reaches
stations ranked 31st and 38th -- and that difference does not track difficulty.

## The 20 measures

Difficulty scale: seconds per certification round, single-tier `relaxed_k60`, n=40, Study 9.
Spearman rank correlation, n=9. |rho| >= 0.68 is p<0.05 at that size.

Geometric, on haversine km over station coordinates (deliberately independent of the
routing-cost table the partition is built from, so a hit could not be a clustering artefact):

| measure | rho |
| --- | --- |
| all_spread (endpoint dispersion about centroid) | **+0.683** |
| dest_spread | +0.617 |
| orig_spread | +0.567 |
| diam_km | +0.467 |
| trip_mean | +0.333 |
| mid_spread | +0.300 |
| trip_sd | -0.217 |

Partition-relative, on the demand footprint: `disp_hit` -0.600, `cells_hit` +0.467,
`multi_hit` +0.100, `disp_all` 0.000 (constant across seeds, see above).

Request-graph structure, per scenario then averaged: `stations` +0.233, `dup_ends` -0.267,
`recip` -0.250, `max_out` -0.133, `hub_share` -0.067, `max_in` +0.033, `chains` +0.017.

Dynamic: relaxation slack at iteration 1, before any cuts -- `relaxed_k60` -0.067,
`twotier_k60` -0.183.

**One measure crossed p<0.05 out of 20 tested, which is exactly what chance gives.**
`all_spread`'s +0.683 sits on the threshold, its range is 1.39-1.49 (a 7% spread against a
6.5x spread in round cost, so it cannot be causal at that magnitude), and the groups
overlap: seeds 42 and 47 are both hard at 1.42, below easy seeds 49 (1.44) and 46 (1.45).

Two specific refutations worth keeping, because both looked compelling:

- **"Easy demand sits in tight or singleton cells."** Motivated by the known slack source
  (the inter-cluster charge is a minimum over member pairs, so passengers pick their own
  in-cell station freely; a singleton cell grants no such freedom). Refuted with the sign
  BACKWARDS -- certified seeds have *higher* `disp_hit`, 28.2 vs 25.9 -- and the failed
  group holds both extremes (42 at 31.2, 47 at 22.2).
- **"Easy stays in the core, hard reaches the periphery."** Visible and convincing in four
  dumped seeds. Refuted by seed 51, which fails while touching the FEWEST stations of any
  seed (10.3), at the same hub share (0.73) and with MORE chains (14.7) than the easiest
  seed. A four-sample visual pattern that does not survive nine.

- **Relaxation starting slack is uninformative**, and pointedly so: seed 48, the fastest
  certifier in the set, starts with the WORST bound of all ten (-4041 against a median
  -2298). What matters is how fast cuts close the gap, not where it opens.

## Two arm-level facts (pre-fix code, but not plumbing-dependent)

**`exact` contributes nothing at n=40.** Objective drop from iteration 1 across 10 seeds:
median **0.00%**, range 0.00-1.62%, with 8 of 10 seeds at exactly 0.00% after 1-6 iterations.
By contrast `relaxed_k60` median 4.12% (0.17-13.53%) and `twotier_k60` median 3.24%
(0.04-13.53%). This is the cleanest available statement of why exact pricing is unusable at
this size -- not "slower", but *zero progress* inside 30 minutes.

Note also how SMALL the relaxed arms' drops are: mostly 2-5%, from a starting point that is
only the seed columns. The master LP barely descends inside the budget.

**Objective drop nearly separates the seeds in the two-tier arm.** Against the correct
certifiable set {43, 44, 46, 48, 49}: two-tier certified seeds drop 3.05-13.53% and failed
seeds 0.04-3.43%, overlapping by **0.38 pp** on the single inversion seed 46 (3.05%, certified)
against seed 47 (3.43%, failed). Single-tier is much muddier, 3.27-13.53 against 0.17-5.35
(overlap 2.08 pp). Two seeds -- 48 and 49 -- show identical drops and identical final
objectives in BOTH arms (13.53% -> 27062.9, 7.85% -> 28148.5), single-tier incomplete where
two-tier certified: the pool converged and only the certificate was missing, the same
pattern the companion note reports for seed 43.

This is suggestive and no more. It is measure 21 on the same nine seeds, one arm of two, and
`drop%` is partly a function of how many iterations a run got.

## Instrumentation added

`RelaxedClusterCertificationResult.relaxed_rc_bound`, surfaced as `relaxed_rc_bound` on every
per-iteration record in `iterations/<stem>.csv`. A **valid lower bound on the minimum reduced
cost over the whole real route universe** -- the number the round already computed and then
discarded after testing it against `-tol`.

Soundness: a real route either escapes every cut, so its image escapes and its reduced cost
is at least the relaxed minimum over escaping routes; or it lies inside some cut support `T`,
and a cut is only added after `stations(T)` was searched EXHAUSTIVELY and found barren, so
that route is at least `-tol`. The bound is the smaller of the two.

It is `NaN` whenever ANY scenario came back `:inconclusive`, and that exclusion is the whole
correctness of the field: an inconclusive sweep's running minimum is the best value *seen*,
an UPPER bound on the relaxed minimum, which bounds nothing from below. Plotted as a bound it
would read as steady progress while proving nothing. The guard fired on live data (seed 49,
iteration 9, `inconclusive_escalated`). NaN poisoning is order-independent because Julia's
`min` propagates NaN -- unlike C's `fmin`, where a later conclusive scenario would have
silently erased the poison.

Worked example, n=8 smoke: `-365.96` (refuted) -> `-365.96` (refuted) -> `+649.31`
(certified). The certified iteration is exactly the one whose bound crosses zero. It also
makes the Study-5 escalation tax directly visible -- identical consecutive `relaxed_rc_bound`
with an unchanged master objective is a round that re-priced the same duals and learned
nothing.

## What I got wrong

- **The certifiable set.** Analysed as {44, 46, 48, 49} because seed 43's single-tier arm had
  never run. It is {43, 44, 46, 48, 49} (companion note). Correcting it *improved* the
  two-tier objective-drop separation from 5.93 pp overlap to 0.38 pp.
- **`cert_s/round` as a difficulty scale is partly circular.** Certification is ~99% of loop
  time, so for a run that exhausted its budget it is approximately `budget / rounds`. All five
  hard seeds ran to budget; the certified ones stopped early. Its "clean separation" of easy
  from hard substantially restates *certified vs not*. Non-circular replacement: certification
  seconds at fixed iterations 1-3, where cut state is comparable.
- **"Cheap rounds cause certification" is wrong**, and the companion note had already
  corrected this reading independently. On the non-circular measure it fails outright: seed 47
  has cheaper two-tier rounds (45 s) than three of four certifiable seeds and still fails;
  seed 50 has the cheapest single-tier rounds of any seed (96 s) and fails. Three seeds are
  censored at exactly 300.0 s (the cap), so their true cost is unmeasured. Round cost sets how
  long you wait per attempt; it does not decide the outcome. Consistent with the companion
  note's finding that diagnostics ranking on round cost recommend actively harmful parameters.
- Three directional predictions, three misses. Treat priors from this thread as uninformative.

## Bottom line

Certifiability at n=40 is a property of the DEMAND DRAW on a fixed map, and no a-priori
descriptor of that draw predicts it -- not geometric spread, not the request graph's hub or
chain structure, not the relaxation's starting slack. The only known discriminator is
solve-derived: `two_tier_station_unexhausted`, perfectly separating in the companion note,
which counts supports whose barrenness can never be established and whose proof is therefore
unavailable at any budget.

That is a coherent place to land. If the blocker is unexhausted station searches, then
difficulty is not a static feature of the instance but of which supports the cut loop happens
to need -- and the productive direction is the companion note's (make the exhaustive subset
search cheaper), not further instance profiling.

## Open

- Whether a-priori prediction becomes possible once geography actually varies. Untested and
  untestable on this generator without the station-selection change described above.
- `relaxed_rc_bound` is recorded but unused. Turning it into a valid `z_LP` lower bound
  (Farley/Lagrangian) additionally needs a bound on the number of columns in an optimal LP
  solution; the joint master carries no fleet or cardinality constraint on theta, so that needs
  a derivation. It would give a real gap at n=40 where nothing certifies -- the only pricer
  that can produce a lower bound without exhausting.
- The `exact` 0.00% result deserves confirmation post-fix, though nothing in the three fixes
  touches the exact pricer.

## Reproduce

```
sbatch benchmarks/diagnostics/run_instance_geometry.sh   # partition/footprint measures
sbatch benchmarks/diagnostics/run_od_spread.sh           # haversine spread vs round cost
sbatch benchmarks/diagnostics/run_request_level.sh       # request graph + raw 16-pair dumps
julia --project=../.. analyze_anytime.jl <outdir> 40 1200   # bound-vs-time snapshot
```
