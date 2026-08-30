SUPERSEDED 2026-08-30 -- single-arm (serial-only) scaling run.

This is the 6 h-total-budget Study 5 run: 120 single-threaded tasks, arrays 21589496 /
21589497 / 21589498, walltime 06:30:00. Outcome: 78 certified, 29 budget-bound,
2 inconclusive, 11 killed at the walltime by the budget-clamp defect.

Superseded because Study 5 was rewritten in place as a TWO-ARM study (serial vs parallel
scenario pricing) so the scaling axes can be read against thread count. This run is the
serial arm's predecessor, not a discarded result -- its numbers remain valid for what they
measured, and it is the evidence base for:

  - the n>=30 certification frontier (1 of 8 correctly-bounded n=30 tasks certified);
  - the finding that certifying rounds consumed 62-84% of the budget at n>=30;
  - the budget-clamp defect that cost 11 rows.

IT IS NOT COMPARABLE TO THE REWRITE. It ran under PER-SCENARIO pricing budgets
(300 s regular / 3600 s certifying per scenario, so up to n_scenarios x that per round).
The rewrite uses a PER-ROUND budget of 300 s divided equally across scenarios -- at s=3
that is roughly 3x less search per round, and the gap widens with s.

Full settings, outcomes and caveats:
  ../../notes/2026-08-30_compute_budgets_of_record.md
