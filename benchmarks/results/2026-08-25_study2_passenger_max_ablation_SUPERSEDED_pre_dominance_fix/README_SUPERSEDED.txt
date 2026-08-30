SUPERSEDED 2026-08-29 -- pre-dominance-fix results. Do not use for any result.

These runs predate the dominance-soundness corrections and CG fixes landed
2026-08-26..29 on StationSelection.jl (branch pfa-two-stop-seeding):

  f644a7c  Fix joint routing-assignment station-clock dominance unsoundness
  b38d46b  Fix route_covering station-clock dominance unsoundness
  6f8db0b  Fix CG livelock from stale-tau duplicate columns
  0bb2d43  Fix scenario-blind column dedup in Joint CG (infeasible-master bug)
  de5d56b  Widen pricing budget 900->1800s; re-scope Studies 2/3/5/6

The station-clock dominance rule used here was UNSOUND: it could discard labels
that were not in fact dominated. Numbers produced by the exact pricer in these
runs are therefore suspect on correctness, not merely on timing. The corrected
rule is provably weaker (more labels and more CG iterations for the same
instance), so the replacement runs are NOT expected to reproduce these timings.

Study 2 note: the replacement run also changes the grid, n {15,20,25} -> {10,15,20},
and raises the pricing cap 900 -> 1800s. The n=25 cells have no counterpart in the
new run; the n=10 cells have no counterpart here.

Superseded by: benchmarks/experiments/2026-08-29_study2_passenger_max_ablation/
               benchmarks/results/2026-08-29_study2_passenger_max_ablation/

Nothing here was deleted; this directory is the evidence trail for the pre-fix
state. Note that *.csv and experiments/ are gitignored, so these files exist
only on this filesystem.
