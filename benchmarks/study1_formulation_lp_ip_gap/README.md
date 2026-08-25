# Study 1 — Formulation and operating-condition LP/IP gaps

## Question

How does the formulation and its operating constraints change the gap between the
column-generation LP bound and the integer solution recovered over the generated
column pool?

Both `AggregateODRouteBaseFormulation` and
`AggregateODRouteJointRoutingAssignmentFormulation` use `CGSolver` with exact pricing
and `recover_integer_solution=true`. Each job records its own LP objective and recovered
IP objective. The minimization gap is `(z_ip - z_lp) / abs(z_ip)`.

The recovered IP is optimal over the columns found by exhaustive LP pricing. It is a
feasible upper bound, but it is not claimed to be the globally optimal IP over every
possible column.

## Four comparisons

1. Base versus Joint at `max_stops=10`, `max_wait_time=900`, and
   `detour_factor=2.0`.
2. Joint with `max_stops` equal to 3, 5, and 7.
3. Joint with `max_wait_time` equal to 600, 900, and 1200 seconds.
4. Joint with `detour_factor` equal to 1.5, 2.0, and 2.5.

Parameters not varied in a comparison retain the Study 2 baseline. These are four
independent sub-studies—not one 120-task array. Each sub-study has its own job table,
SLURM submission, and comparison CSV. Across all four submissions there are 110 jobs.

## Shared instance and solver design

The instance grid matches Study 2: Zhuzhou top 10 stations, 8 OD pairs, one scenario,
and seeds 42–51. All cells use `k=5`, a 600-second walking cap, route/walk weights
10.0/0.1, 20-second repositioning, exact pricing, and a 900-second per-scenario pricing
limit. Every row of the four `config/*_jobs.tsv` tables runs in a separate Julia process.

`runtime_sec` is copied from `OptResult.runtime_sec`. It includes the CG loop and
integer-recovery solve and is retained as a secondary result.

## Certification

All configured jobs must be present. `analyze.jl` refuses to produce summaries unless
every job has an optimal LP, exhaustive/converged pricing, a recorded LP objective, and
an optimal recovered IP solve. Raw per-job CSVs remain available if certification fails.

## Running

```bash
julia --project=. benchmarks/study1_formulation_lp_ip_gap/generate_jobs.jl
mkdir -p benchmarks/study1_formulation_lp_ip_gap/slurm_logs

# Sub-study 1: Base versus Joint (20 jobs)
sbatch --job-name=study1_formulation --array=1-20 \
    benchmarks/study1_formulation_lp_ip_gap/submit_benchmark.sh formulation

# Sub-study 2: max_stops (30 jobs)
sbatch --job-name=study1_max_stops --array=1-30 \
    benchmarks/study1_formulation_lp_ip_gap/submit_benchmark.sh max_stops

# Sub-study 3: max_wait_time (30 jobs)
sbatch --job-name=study1_max_wait --array=1-30 \
    benchmarks/study1_formulation_lp_ip_gap/submit_benchmark.sh max_wait_time

# Sub-study 4: detour_factor (30 jobs)
sbatch --job-name=study1_detour --array=1-30 \
    benchmarks/study1_formulation_lp_ip_gap/submit_benchmark.sh detour_factor
```

`STUDY1_DATA_DIR` and `STUDY1_OUTPUT_DIR` override the defaults. Aggregate with:

```bash
julia --project=. benchmarks/study1_formulation_lp_ip_gap/analyze.jl \
    experiments/YYYY-MM-DD_study1_formulation_lp_ip_gap
```

Outputs are `case_results.csv`, `variant_summary.csv`, four `comparison_*.csv` files,
and `slides_results.tex`.
