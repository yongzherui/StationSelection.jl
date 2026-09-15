# Instance generation

Synthetic and real-instance builders. The Zhuzhou generator is what every benchmark in
`benchmarks/` is built on; the `test*` cases are small hand-built geometries with known
answers, used to pin specific behaviours rather than to measure performance.

Note on the Zhuzhou generator: **the seed varies demand only, not geography.** Stations are
the deterministic top-N by popularity, so the partition and the cost matrices are identical
across seeds and only the OD draw changes.

## Zhuzhou

{{autodocs generators/zhuzhou.jl}}

## Grid

`generators/grid.jl` is the plain lattice: every arc costs the same, so a station's
attractiveness depends only on where it sits.

{{autodocs generators/grid.jl}}

## Fielbaum toy grid

`generators/fielbaum_grid.jl` reproduces the 10x10 test network of Fielbaum, Bai &
Alonso-Mora (2021), *On-demand ridesharing with optimized pick-up and drop-off walking
locations*, Transportation Research Part C 126, 103061 (Section 3.1 and Fig. 4). Unlike
the plain grid above, its streets carry three different **vehicle speeds** -- slow
(20 km/h, bidirectional), mid-speed (30 km/h, one-way) and fast (40 km/h, bidirectional)
-- while every arc keeps the same 150 m length and walking stays at 5 km/h everywhere.
That is the point of the instance: a vehicle gains by staying on a fast street, so
detouring a passenger onto one can pay for itself, which is what makes coordinating the
vehicle and the walking passenger worth doing. A uniform-cost grid cannot express it.

Two consequences to keep in mind when using it:

* **Costs are in minutes, not metres.** Every arc is the same length, so distance carries
  no information here; only time distinguishes a fast street from a slow one. The walking
  limit is therefore a walking *time* limit -- pass `FIELBAUM_MAX_WALKING_TIME_MIN` (the
  paper's `Ω_a = 12` min) as `StationSelectionProblem.max_walking_distance`.
* **Routing costs are asymmetric.** The mid-speed streets are one-way, so the return leg
  between two stations can be strictly longer than the outbound one.
  `StationSelectionData` indexes routing costs as `[from, to]`, so this survives, but any
  analysis that assumes a symmetric cost matrix will be wrong on this instance.

The paper's text fixes the speeds, the 150 m arc length and the rule that the mid-speed
streets are the one-way ones — but *which* street is which appears only in Fig. 4.
`notes/2026-09-14_fielbaum_grid_figure_transcription.md` records how that figure was
transcribed, how to re-verify it, and the three places the paper is silent and this
generator had to interpret. Read it before touching `FIELBAUM_BASE_ROW_STREETS` or
`FIELBAUM_BASE_COL_STREETS`. The PDF itself is not in this repository; it lives in the
sibling checkout at `MicroTransitSimulator.jl/docs/references/`.

`print_fielbaum_grid_street_map` renders the layout as ASCII for checking against Fig. 4.

Note that `FIELBAUM_MAX_WAITING_TIME_MIN`, `FIELBAUM_MAX_DELAY_MIN` and
`FIELBAUM_WALK_COST_WEIGHT` are transcribed from the paper for callers to pass on; no code
in this package reads them. Only `FIELBAUM_MAX_WALKING_TIME_MIN` has a consumer here, and
only because you hand it to the problem.

{{autodocs generators/fielbaum_grid.jl}}

## Hand-built test cases

{{autodocs generators/test_cases}}
