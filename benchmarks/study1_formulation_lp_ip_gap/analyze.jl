"""
`analyze.jl` -- aggregates Study 1's per-job result rows (written by `run_benchmark.jl`
under `../../experiments/<date>_study1_formulation_lp_ip_gap/`) into a curated
two-tier summary under `../../results/<date>_study1_formulation_lp_ip_gap/`, following
the convention in `results/theta_rho_comparison_2026-08-04/`:

- `case_comparison.csv` -- one row per (instance, formulation, setting) cell:
  `instance, formulation, setting, z_lp, z_ip, gap, termination_status, runtime_sec`.
- `variant_summary.csv` -- one row per (formulation, setting), aggregated over
  instances: `formulation, setting, n_cases, mean_gap, max_gap, all_optimal`.
- `slides_results.tex` -- one `\newcommand` row macro per `case_comparison.csv` row
  (or per `variant_summary.csv` row, whichever the manuscript table wants), via
  `../lib/latex_rows.jl`'s `write_latex_rows`.

Column-naming kept consistent with `../../scripts/analyze_method_compare.jl`'s
vocabulary (`termination_status`, `runtime_sec`) even though the underlying schema is
new, per `../README.md`.

TODO: not implemented -- depends on `run_benchmark.jl`'s exact output row shape (should
land in step with it, not before).
"""

# TODO: implement.
