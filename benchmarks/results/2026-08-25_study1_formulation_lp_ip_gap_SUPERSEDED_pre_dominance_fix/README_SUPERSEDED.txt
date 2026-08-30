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

Study 1 note: its config was NOT re-scoped by de5d56b -- the replacement run uses
the identical grid, 900s pricing cap and 30min walltime, so its cells remain
directly comparable to these apart from the pricer correction itself.

Superseded by: benchmarks/experiments/2026-08-29_study1_formulation_lp_ip_gap/
               benchmarks/results/2026-08-29_study1_formulation_lp_ip_gap/

Nothing here was deleted; this directory is the evidence trail for the pre-fix
state. Note that *.csv and experiments/ are gitignored, so these files exist
only on this filesystem.
