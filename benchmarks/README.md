# `benchmarks/`

Solver-internals benchmark studies backing the "Result Placeholder" frames in
`../../VBS-Location-Manuscripts/Presentation/main.tex`. Each study calls `run_opt`/the
label-setting pricers directly and reports bounds, timings, and label counts — none of
them touch the simulator, so none of them use the top-level project's
`experiments/`/`scripts/create_study.jl` selection→simulation pipeline (that pipeline is
untouched, and still targets a different, older model API — see its own `CLAUDE.md`).

This directory holds **code only**. Output flows into the package-root directories that
already hold this kind of pricing-benchmark work, dated when a study is actually run:

- `../experiments/<date>_studyN_<name>/` — raw per-run CSVs (gitignored).
- `../results/<date>_studyN_<name>/` — curated case-level + variant-summary CSVs
  (git-tracked), plus a `slides_results.tex` of `\newcommand{\RowName}{col & col & ...}`
  macros ready to `\input` into the manuscript, following
  `../experiments/2026-07-15_restricted_pricing_report/slides_results.tex`'s convention.
- `../notes/<date>_studyN_<name>_results.md` — write-up, same convention as every other
  file in `../notes/`.

Instance generation reuses `../scripts/generate_zhuzhou_instance.jl`'s
`generate_zhuzhou_data` (current, unmodified) via relative `include`, and each
`submit_benchmark.sh` sources `../scripts/lib/slurm_modules.sh` /
`slurm_array_task_env.sh` (also current, unmodified) rather than duplicating them.

## Status

| # | Directory | Objective | Status |
| - | --- | --- | --- |
| 1 | `study1_formulation_lp_ip_gap/` | Base vs. Joint LP/IP gap across operating settings | scaffolded, TODO |
| 2 | `study2_passenger_max_ablation/` | `exact/`'s running-max reward vs. `darp/`'s explicit first-commit assignment | scaffolded, TODO — blocked on `exact/`'s missing standalone driver (see study README) |
| 3 | `study3_dominance_ablation/` | `compensated_dominance` true vs. false | scaffolded, TODO — same blocker as Study 2 |
| 4 | — | Heuristic pricing frontier | shelved, no directory |
| 5 | `study5_scaling_vs_enumeration/` | Runtime vs. `\|P\|`/`\|J\|`/`\|S\|`: CG pricing, `darp/`, raw enumeration | scaffolded, TODO — blocked on a new `enumerate_joint_routing_assignment_columns` (see study README) |

Every `run_benchmark.jl`/`generate_jobs.jl`/`analyze.jl` in each study directory is a
stub: module docstring naming the study's I/O contract and the exact `src/` function(s)
it will call, `# TODO:` body, no solve logic yet. `submit_benchmark.sh` files are
complete — they're pure SLURM plumbing, nothing study-specific to fill in.
