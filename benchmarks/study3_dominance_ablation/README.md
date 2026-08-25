# Study 3 — Compensated-dominance ablation

## Question

How much does compensated dominance reduce the end-to-end work of exact Joint Routing
Assignment column generation relative to plain subset dominance?

The only experimental switch is
`AggregateODRouteJointRoutingAssignmentFormulation(compensated_dominance=...)`:

- `true`: the compensated/catch-up dominance rule;
- `false`: plain subset dominance.

Both arms use `pricing_mode=:exact` and solve the same master problem. Certified paired
objectives must agree; `analyze.jl` raises an error if they do not.

## Larger instance grid

The grid deliberately scales beyond Studies 1 and 2 so compensation has room to matter:

- Zhuzhou top 20 stations;
- 16 OD pairs and one scenario;
- seeds 42–51;
- `max_stops=10`;
- 900-second per-scenario pricing limit.

Every `(instance, dominance mode)` pair is a separate Julia process, producing 20 jobs.
The remaining settings match Study 2: `k=10`, 600-second walking cap, route/walk
weights 10.0/0.1, 20-second repositioning, 900-second pickup horizon, and detour factor
2.0.

## Metrics

The headline paired metrics are runtime, total labels generated, and peak live labels.
The raw rows also retain termination and pricing certification, CG iterations, columns,
dominance rejection/removal counts, maximum frontier size, and objective value.

Label statistics are accumulated across every pricing search in the full CG run. Runtime
is `OptResult.runtime_sec`; it is not an outer process timer. Summaries use certified
runs (`cg_converged=true` and `cg_pricing_exhausted=true`) while retaining every raw row.

## Running

```bash
julia --project=. benchmarks/study3_dominance_ablation/generate_jobs.jl
mkdir -p benchmarks/study3_dominance_ablation/slurm_logs
sbatch --array=1-20 benchmarks/study3_dominance_ablation/submit_benchmark.sh
```

`STUDY3_DATA_DIR` and `STUDY3_OUTPUT_DIR` override the defaults. Aggregate with:

```bash
julia --project=. benchmarks/study3_dominance_ablation/analyze.jl \
    experiments/YYYY-MM-DD_study3_dominance_ablation
```

Outputs are `case_results.csv`, `paired_comparison.csv`, `variant_summary.csv`, and
`slides_results.tex`.
