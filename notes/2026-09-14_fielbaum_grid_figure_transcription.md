# Where the Fielbaum grid's street layout came from — 2026-09-14

`src/generators/fielbaum_grid.jl` reproduces the toy network of

> Fielbaum, A., Bai, X., & Alonso-Mora, J. (2021). *On-demand ridesharing with optimized
> pick-up and drop-off walking locations.* Transportation Research Part C 126, 103061.

**The PDF is not in this repository.** It lives in the sibling checkout at
`MicroTransitSimulator.jl/docs/references/fielbaum-2021-pudo-walking.pdf` (deliberately
untracked there — it is a copyrighted article; that directory's README says how to
re-fetch it from the DOI). Section 3.1 is on pages 13-16; Fig. 4 is the single embedded
image on page 13.

This note exists because the generator's correctness rests on a claim that cannot be
checked by reading the paper's text: that `FIELBAUM_BASE_ROW_STREETS` and
`FIELBAUM_BASE_COL_STREETS` are *exactly* what Fig. 4 draws. The text gives the speeds and
the directionality rule but never says which street is which. That is only in the figure.

## What the text does fix

Section 3.1, verbatim:

> "The base graph we use is as follows: a 10x10 grid, in which streets can be slow
> (20 km/h), mid-speed (30 km/h), or fast (40 km/h). Mid-speed streets are unidirectional,
> and the others are bidirectional. All the arcs are assumed to have the same length
> (0.15 km), so walking times are the same everywhere; walking speed is 5 [km/h]."

Fig. 4's caption fixes the colour key: "Green, grey and red arcs are fast, mid-speed, and
slow, respectively."

So: speeds, arc length, walking speed, and the class/direction pairing are all stated.
The *spatial pattern* is not.

## How the pattern was extracted

1. Page 13's embedded image (`X1.png`, 739x742 px) extracted with `pypdf`.
2. Node centres located from the non-white pixel profile: columns at x =
   11, 90, 169, 250, 330, 409, 489, 570, 649, 729 and rows at y =
   10, 90, 170, 249, 330, 410, 489, 569, 649, 730 — a clean 10x10 lattice at ~80 px pitch.
3. Each of the 9x10 horizontal and 10x9 vertical segments classified by counting olive
   (~135,134,0), red and grey pixels in a band around the segment midpoint, excluding the
   node circles. Every street came back **homogeneous along its entire length**, which is
   why one entry per row and one per column suffices.
4. One-way directions recovered from the arrowhead. Each one-way segment carries a single
   arrowhead at one end; averaging the stroke-width profile over all 9 segments of a
   street (to beat anti-aliasing noise) puts the arrowhead unambiguously at one end, and
   the arrowhead sits on the side the arrow points to.
5. Confirmed by eye on two enlarged crops — the top-left 4x4 node block (rows 1-4 x
   cols 1-4) and a bottom-middle block (rows 7-10 x cols 4-7) — checking class and arrow
   direction for each street against the automated result. No disagreements.

## The result

| | fast (40, bidirectional) | slow (20, bidirectional) | mid (30, one-way) |
| --- | --- | --- | --- |
| rows (top = row 1) | 2, 6, 9 | 3, 5, 7 | 1, 8 eastbound; 4, 10 westbound |
| columns (left = col 1) | 3, 6, 9 | 1, 8, 10 | 2, 5 northbound; 4, 7 southbound |

Three fast, three slow and four mid-speed streets on each axis. The pattern is *not*
symmetric under 180-degree rotation, and the row and column patterns are not the same —
both are genuine properties of the figure, not transcription slips.

`print_fielbaum_grid_street_map` renders this back as ASCII for eyeballing against the
figure:

```
o-M>o-M>o …   row 1   mid, one-way east
S   ^   F …   cols 1,2,3 = slow, mid-north, fast
o=F=o=F=o …   row 2   fast
```

## Independent cross-checks

Counting arcs from the table above and comparing against what the generator builds (all in
`test/utils/test_fielbaum_grid_generator.jl`):

- 288 directed arcs = 2 axes x (6 bidirectional streets x 9 segments x 2 + 4 one-way x 9).
- By class: 108 fast, 108 slow, 72 mid — matching 3 fast + 3 slow streets per axis at 9
  segments x 2 directions, and 4 one-way streets per axis at 9 segments.
- Arc times take exactly three values, 0.225 / 0.300 / 0.450 min, i.e. 150 m at 40 / 30 /
  20 km/h.
- Max walking time 32.4 min = 18 hops x 150 m at 5 km/h, so walking spans the full grid.

## What is interpretation, not transcription

Three places the paper is silent and the generator had to choose. All are kwarg-overridable
and flagged in the docstrings; none affect the base network.

1. **Non-uniform spacing profile.** The paper says only that spacing "increases from 75[m]
   at the center of the network to 300[m] at the edges". The generator interpolates
   linearly in a gap's distance from the middle gap, symmetrically. Consequence worth
   knowing: mean spacing becomes 200 m, so the non-uniform network is *larger* overall than
   the 150 m base network, not a rearrangement of it.
2. **Concentrated demand collisions.** The paper says each origin and destination moves "in
   the diagonal towards the center ... with a probability of 0.5" but not what happens when
   that collapses an OD pair onto one node. The generator keeps the original endpoints for
   such a draw.
3. **Grid sizes other than 10x10.** The paper defines only 10x10. Other sizes tile the
   pattern cyclically — an extension of this package, not something the paper states.

## Two traps for callers

- **Costs are in minutes, not metres.** Every arc is the same length, so distance carries
  no information on this instance; only time separates a fast street from a slow one. That
  makes `StationSelectionProblem.max_walking_distance` a walking *time* limit here — pass
  `FIELBAUM_MAX_WALKING_TIME_MIN` (the paper's `Ω_a` = 12 min).
- **Routing costs are asymmetric.** The one-way mid-speed streets mean `t(i,j) != t(j,i)`;
  the worst case on the base grid is 1.43 min. `StationSelectionData` indexes routing costs
  as `[from, to]` so this survives, but any analysis assuming a symmetric cost matrix is
  wrong on this instance. This is the first instance in the package with that property —
  the Zhuzhou and plain-grid generators are both symmetric.

## Exported constants that nothing here consumes

`FIELBAUM_MAX_WAITING_TIME_MIN` (`Ω_w` = 5), `FIELBAUM_MAX_DELAY_MIN` (`Ω_d` = 10) and
`FIELBAUM_WALK_COST_WEIGHT` (`p_a = 2 p_v`) are transcribed from the paper for callers to
use, but **no code in StationSelection.jl reads them**. `Ω_w` and `Ω_d` are dispatch-time
bounds and belong to MicroTransitSimulator.jl's dynamic PUDO path;
`FIELBAUM_WALK_COST_WEIGHT` is the value to pass as an `AggregateODRoute*` formulation's
`walk_cost_weight`, not a default it picks up. Only `FIELBAUM_MAX_WALKING_TIME_MIN` has a
consumer here, and only because the caller hands it to the problem.
