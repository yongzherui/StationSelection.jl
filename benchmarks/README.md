# `benchmarks/`

Solver-internals benchmark studies backing the "Result Placeholder" frames in
`../../VBS-Location-Manuscripts/Presentation/main.tex`. Each study calls `run_opt`/the
label-setting pricers directly and reports bounds, timings, and label counts — none of
them touch the simulator, so none of them use the top-level project's
`experiments/`/`scripts/create_study.jl` selection→simulation pipeline (that pipeline is
untouched, and still targets a different, older model API — see its own `CLAUDE.md`).

Benchmark code and output now live together under this directory. Generated directories
are dated when a study is actually run:

- `experiments/<date>_studyN_<name>/` — raw per-run CSVs (gitignored).
- `results/<date>_studyN_<name>/` — curated case-level + variant-summary CSVs
  (git-tracked), plus a `slides_results.tex` of `\newcommand{\RowName}{col & col & ...}`
  macros ready to `\input` into the manuscript, following
  `../experiments/2026-07-15_restricted_pricing_report/slides_results.tex`'s convention.
- `notes/<date>_studyN_<name>_results.md` — benchmark write-ups.

Instance generation reuses `../scripts/generate_zhuzhou_instance.jl`'s
`generate_zhuzhou_data` (current, unmodified) via relative `include`, and each
`submit_benchmark.sh` sources `../scripts/lib/slurm_modules.sh` /
`slurm_array_task_env.sh` (also current, unmodified) rather than duplicating them.

End-to-end CG studies share `lib/cg_benchmark.jl` for Zhuzhou problem loading,
baseline parameters, solver construction, output locations, and standard CG and
label-search metrics. Per-search label statistics are retained in
`OptResult.metadata["cg_pricing_stats"]`, including across integer recovery.

## Status

| # | Directory | Objective | Status |
| - | --- | --- | --- |
| 1 | `study1_formulation_lp_ip_gap/` | Base vs. Joint and operating-condition LP/IP gaps | implemented — four sub-studies, 110 Zhuzhou jobs |
| 2 | `study2_passenger_max_ablation/` | `exact` running-max pricing vs. explicit DARP-style pricing | implemented — 60-job Zhuzhou grid, n ∈ {15, 20, 25} at p=8 |
| 3 | `study3_dominance_ablation/` | `compensated_dominance` true vs. false | implemented — 60-job grid, n ∈ {15, 20, 25} at p=16 |
| 4 | `study4_heuristic_local_search/` | Heuristic pricing frontier (provisional name `local_search`) | placeholder — no design yet, see study README |
| 5 | `study5_scaling_vs_enumeration/` | Runtime vs. `\|P\|`/`\|J\|`/`\|S\|`: CG pricing, `darp/`, raw enumeration | scaffolded, TODO — blocked on a new `enumerate_joint_routing_assignment_columns` (see study README) |

Studies 1–3 are executable end to end. Studies 4–5 remain placeholders or scaffolds as
described above. `submit_benchmark.sh` files are pure SLURM plumbing.
