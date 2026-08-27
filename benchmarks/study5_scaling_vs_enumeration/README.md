# Study 5 — Exact column-generation scalability

## Objective

Measure how the Joint formulation's exact, dual-certified column-generation solver
scales independently with candidate stations, passengers, and scenarios.

## Grid

One dimension is swept at a time using the following fixed settings:

- stations: `n = 10, 20, 30, 40`, holding `p=16`, `s=3`;
- passengers/OD pairs: `p = 8, 16, 24, 32`, holding `n=20`, `s=3`;
- scenarios: `s = 3, 6, 9, 12`, holding `n=20`, `p=16` per scenario;
- `max_stops`: 10 throughout;
- seeds: 42–51.

These are three separately submitted sub-studies with one exact-CG job per configured
instance. Each table contains 40 jobs, for 120 jobs total.

Every task is explicitly single-threaded: SLURM allocates one CPU, Julia uses one thread
(so scenarios are priced serially), and Gurobi's `Threads` parameter is set to one.

Each job has a 900-second per-scenario pricing limit. The SLURM task limit is one hour
and memory is 16 GB. Jobs killed by the scheduler remain visible as `missing_result` in
the analyzer output and should be classified with `sacct`.

## Running

From the repository root:

```bash
julia --project=. benchmarks/study5_scaling_vs_enumeration/generate_jobs.jl
cd benchmarks/study5_scaling_vs_enumeration
mkdir -p slurm_logs
sbatch --job-name=study5_stations --array=1-40 submit_benchmark.sh stations
sbatch --job-name=study5_passengers --array=1-40 submit_benchmark.sh passengers
sbatch --job-name=study5_scenarios --array=1-40 submit_benchmark.sh scenarios
```

`STUDY5_DATA_DIR` and `STUDY5_OUTPUT_DIR` override the shared defaults. Raw results go to
`benchmarks/experiments/YYYY-MM-DD_study5_scaling_exact_cg/`.

After the array finishes:

```bash
cd ../..
julia --project=. benchmarks/study5_scaling_vs_enumeration/analyze.jl \
    benchmarks/experiments/YYYY-MM-DD_study5_scaling_exact_cg
```

The analyzer writes `case_comparison.csv`, `variant_summary.csv`,
`missing_results.csv`, and `slides_results.tex` under `benchmarks/results/`.
