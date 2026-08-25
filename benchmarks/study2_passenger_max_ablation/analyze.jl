"""
`analyze.jl` -- aggregates Study 2's per-instance, per-driver result rows (written by
`run_benchmark.jl`) into a curated two-tier summary under
`../../results/<date>_study2_passenger_max_ablation/`, following the convention in
`results/theta_rho_comparison_2026-08-04/`:

- `case_comparison.csv` -- one row per (instance, driver): all `stats` fields plus
  `min_reduced_cost`, `wall_sec`.
- `variant_summary.csv` -- one row per driver, aggregated over instances: `driver,
  n_cases, mean_labels_generated, mean_wall_sec, max_abs_reduced_cost_delta` (paired
  against the other driver's `min_reduced_cost` on the same instance).
- `slides_results.tex` -- via `../lib/latex_rows.jl`.

TODO: not implemented -- depends on `run_benchmark.jl`'s exact output row shape, which
is itself blocked (see `README.md`).
"""

# TODO: implement.
