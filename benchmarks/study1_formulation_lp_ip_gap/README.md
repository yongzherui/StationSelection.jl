# Study 1 — Formulation and operating-condition LP/IP gaps

## Question

How does the formulation and its operating constraints change the gap between the LP
bound and the integer solution?

Comparisons 2-4 vary a single Joint parameter and use `CGSolver` with exact pricing and
`recover_integer_solution=true`, exactly as before: `z_lp` is CG's dual-certified LP
bound (`cg_lp_objective_value`), and `z_ip` is the MIP re-solved over exactly the column
pool CG discovered -- a feasible upper bound, not claimed to be globally optimal over
every possible column.

Comparison 1 (Base versus Joint) instead solves **both formulations through the same
exhaustive-enumeration + `DirectMIPSolver` strategy**, not `CGSolver`: `z_ip` is the
exhaustive-pool MIP solved directly; `z_lp` is the true LP relaxation of that same model,
solved by relaxing it in place (`JuMP.relax_integrality`) and re-optimizing -- no CG duals
or CG-restricted-recovery heuristic on either side.

- **Base** (`AggregateODRouteBaseFormulation`) enumerates its whole column universe up
  front (`enumerate_aggregate_od_route_columns`), which throws rather than return a
  truncated pool if it can't finish, so any result at all is exact by construction.
- **Joint** (`AggregateODRouteJointRoutingAssignmentFormulation`) has no separate
  assignment variable to decouple from `θ` the way Base's `x` does, so a genuinely
  exhaustive pool needs more than Base's route enumeration alone
  (`enumerate_joint_routing_assignment_columns`,
  `label_setting/joint_routing_assignment/exact/enumeration.jl`): it reuses Base's own
  physical-route DFS verbatim (the two formulations' ride-limit rule is identical per
  `(j, k)` pair, so the route universe is provably the same one -- a route achieving the
  same certifications with fewer stations is already its own separate entry in that
  shared pool, since the DFS seeds a label at every relevant node and visits every
  prefix length, not just one start/one length), then for each route takes the maximal,
  elementarity-preserving cartesian product over every certified passenger's own
  certified `(j, k)` options -- always claiming every certifiable passenger (never a
  reason to omit one under `>= 1` coverage), branching only when a single passenger is
  certified through more than one pair on the same route (never crediting the same
  passenger twice in one column, since one column is one physical trip). Measured on
  this study's instance at `max_stops=4`: 16,320 columns, cross-validated exact against
  Base's own independently-computed global optimum at `max_stops=2` and `max_stops=4`.

Running Base through `CGSolver` (as this comparison did previously) would compare
Joint's native mechanism against Base forced through Joint's -- not apples to apples.
Giving both formulations the same true direct-solve LP/IP pair is the fairer read, at
the cost of capping `max_stops` low enough that exhaustive enumeration stays tractable
for both (see below).

## Four comparisons

1. Base versus Joint at `max_stops=4` (not the Study 2 baseline of 10 -- see above),
   `max_wait_time=900`, and `detour_factor=2.0`.
2. Joint with `max_stops` equal to 3, 5, and 7.
3. Joint with `max_wait_time` equal to 600, 900, and 1200 seconds.
4. Joint with `detour_factor` equal to 1.5, 2.0, and 2.5.

Parameters not varied in a comparison retain the Study 2 baseline, except comparison 1's
`max_stops` (see above). These are four independent sub-studies—not one 120-task array.
Each sub-study has its own job table, SLURM submission, and comparison CSV. Across all
four submissions there are 110 jobs.

## Shared instance and solver design

The instance grid matches Study 2: Zhuzhou top 10 stations, 8 OD pairs, one scenario,
and seeds 42–51. All cells use `k=5`, a 600-second walking cap, route/walk weights
10.0/0.1, 20-second repositioning, exact pricing, and a 900-second per-scenario pricing
limit. Every row of the four `config/*_jobs.tsv` tables runs in a separate Julia process.

`runtime_sec` is `OptResult.runtime_sec` and is retained as a secondary result. For
Joint in comparisons 2-4 it is the CG loop plus the integer-recovery solve. For
comparison 1's rows (both formulations) it is instead the direct-MIP solve plus the
LP-relaxation solve combined (column enumeration itself runs once per solve, so its
cost is paid twice) -- comparison 1's runtimes and comparisons 2-4's runtimes are not
comparable to each other, only within their own comparison.

## Certification

All configured jobs must be present. `analyze.jl` refuses to produce summaries unless
every job has an optimal LP, exhaustive/converged pricing, a recorded LP objective, and
an optimal recovered IP solve. Comparison 1's rows have no CG loop to converge, so they
report `cg_converged=true`/`cg_pricing_exhausted=true` unconditionally -- an OPTIMAL
termination status on both the direct MIP and its LP relaxation already certifies
exactness for those rows (both enumerators throw rather than silently truncate the
pool), so the same certification gate still applies uniformly. Raw per-job CSVs remain
available if certification fails.

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
