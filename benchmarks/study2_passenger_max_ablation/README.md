# Study 2 — DARP-style versus specialized exact pricing

## Question

How much does the explicit DARP-style label representation cost relative to
the specialized exact running-maximum representation for the uncapacitated
joint-routing-assignment problem?

- `pricing_mode=:darp` explicitly boards `(p,j,k)` commitments, carries an
  onboard/open-request set and ride clocks, and must discharge those requests.
- `pricing_mode=:exact` certifies assignment opportunities from route visits
  and retains each passenger's best reward through reward layers.

Both are exact for the same master problem when pricing exhausts. This is a
representation/search comparison, not a capacity ablation: both modes are
uncapacitated.

## Independent-job design

Every `(generated instance, pricing_mode)` pair is a separate row in
`config/jobs.tsv` and therefore a separate SLURM job and Julia process. Exact
and DARP never run sequentially in one process. This prevents one mode from
benefiting from the other's JIT compilation, caches, or memory state.

The initial baseline grid in `generate_jobs.jl` is:

- Zhuzhou top 10 stations;
- 8 OD pairs, one scenario;
- seeds 42–51;
- `max_stops=10`;
- modes `exact` and `darp`;
- 900-second limit for each scenario's label search.

This produces 20 jobs. Edit the constants in `generate_jobs.jl` to expand the
grid after the baseline is established.

Both modes use the production `CGSolver`, normal two-stop seed columns, and
the same formulation parameters: `k=ceil(n/2)`, 600-second walking cap,
route/walk weights 10.0/0.1, 20-second repositioning, 900-second pickup
horizon, and detour factor 2.0.

## Runtime definition

The benchmark's `runtime_sec` is **only** `OptResult.runtime_sec` returned by:

```julia
result = run_opt(problem, formulation, solver)
runtime_sec = result.runtime_sec
```

It does not use `@elapsed`, `time()`, scheduler duration, data-generation
time, or model-build time. Under `CGSolver`, this measures the repeated RMP
optimization, pricing, and column-addition loop implemented by `run_opt`.

## Running

Generate and submit the grid:

```bash
julia --project=. benchmarks/study2_passenger_max_ablation/generate_jobs.jl
mkdir -p benchmarks/study2_passenger_max_ablation/slurm_logs
sbatch --array=1-20 benchmarks/study2_passenger_max_ablation/submit_benchmark.sh
```

`STUDY2_DATA_DIR` and `STUDY2_OUTPUT_DIR` override the default data and raw
output locations. Aggregate completed jobs with:

```bash
julia --project=. benchmarks/study2_passenger_max_ablation/analyze.jl \
    experiments/YYYY-MM-DD_study2_passenger_max_ablation
```

## Outputs and interpretation

Each job writes one CSV containing its mode, instance parameters,
termination status, objective, `run_opt` runtime, CG iterations, convergence,
pricing exhaustion, column count, and seed-column count.

`analyze.jl` produces `case_results.csv`, `paired_comparison.csv`,
`variant_summary.csv`, and `slides_results.tex`.

Only rows with both `cg_converged=true` and `cg_pricing_exhausted=true` count
as certified. A master may have `termination_status=OPTIMAL` even when a
pricing search times out; such a result is deliberately not treated as a
pricing certificate or objective-equivalence observation.
