# Free-assignment CG versus Direct at `max_stops=5`

Date: 2026-08-05

## Purpose

This benchmark tests the new passenger-level free-assignment column-generation
scheme against an independent exhaustive-route Direct solve.  It uses the same
instance sizes and route/resource settings as the 2026-08-04 Benders scaling
study so runtime scaling can be discussed alongside it.

The comparison with Benders is not an objective-equivalence comparison:
Benders used nearest-open assignment, whereas this benchmark uses free
passenger assignment.  Direct and CG within this benchmark do solve the same
free-assignment model and therefore provide the objective-quality check.

## Configuration

- `n = 10, 15, 20`
- `p = 16, 32`
- seeds `42, 123, 999`
- scenario counts `q = 1, 3`
- `l = ceil(n/2)`
- `max_stops = 5`
- unrestricted `max_visits_per_node`
- maximum wait 900 seconds
- detour factor 2.0
- route regularization weight 10.0
- walking-cost weight 0.1
- repositioning time 20.0
- **16 GB memory limit per task**
- 90-minute wall limit per task

The primary new-CG benchmark harvests 100 candidates per scenario, globally
retains up to 100 per iteration, uses one Julia thread, and certifies with the
exact physical pricer.  Adaptive cluster certification and reduced-cost
recomputation checks are disabled.  Direct exhaustively enumerates the physical
route universe and then solves the monolithic free-assignment model with
explicit station-selection, passenger-assignment, and route variables.

## Passenger free-assignment column reformulation

The monolithic Direct model contains physical passenger assignment variables
`x[p,j,k]` in addition to station variables `y[j]` and route-use variables.  The
passenger column-generation model applies a Dantzig--Wolfe reformulation: each
route column `r` contains both a physical route and its passenger assignments.
Consequently the ordinary `x[p,j,k]` variables disappear from the generated
master.  Its core decisions are:

- `y[j]`: whether physical station `j` is selected;
- `lambda[r]` (called `theta[r]` in code): whether passenger-assignment route
  column `r` is used.

For a column `r`, let `a[r,p]` indicate that it serves passenger `p`, and let
`aO[r,p,j]` and `aD[r,p,k]` identify its pickup and dropoff stations.  The core
master is

```text
min  sum_r c_r lambda_r

s.t. sum_r a[r,p] lambda_r                         >= 1       for each p
     sum_r aO[r,p,j] lambda_r                      <= y_j     for each p,j
     sum_r aD[r,p,k] lambda_r                      <= y_k     for each p,k
     sum_j y_j                                      = l
     y binary, lambda >= 0 (binary in the final restricted-master MIP).
```

Here `c_r` includes route/repositioning cost and the walking cost of the
passenger assignments embedded in the column.  Thus the station-pair assignment
choice is priced jointly with routing rather than retained as a separate master
variable block.

Two small auxiliary blocks are required in the implemented exact formulation:
`v[p]` keeps an initially empty restricted master feasible with a dominated
unserved penalty, and `x_same[p,j]` represents the special no-vehicle case where
pickup and dropoff use the same station.  Therefore “only y and lambda” is an
accurate description of the generated routing/assignment core, but the literal
implementation is `(y, lambda, v, x_same)`.  Ordinary `x[p,j,k]` variables for
`j != k` are fully absorbed into `lambda` columns.

The reduced cost is

```text
rc_r = beta * (tau_r + repositioning)
       - sum_p (alpha_p - gammaO[p,j(r,p)] - gammaD[p,k(r,p)] - walk[p,j,k]),
```

which is the joint passenger-assignment-and-routing problem solved by the label
setting pricer.

## Multi-scenario parallelism in this benchmark

The code path is called with `parallel_scenarios=true` (its default), but the
primary Slurm rerun explicitly sets `JULIA_NUM_THREADS=1` for comparability with
the Benders study.  The earlier diagnostic wrapper also effectively used one
Julia thread because it did not set `JULIA_NUM_THREADS` or `--threads`.
The implementation activates threaded scenario pricing only when both
`parallel_scenarios=true` and `Threads.nthreads() > 1`; consequently the reported
multi-scenario CG timings used **serial scenario pricing**.

Log lines such as `Threads : 30` are emitted by Gurobi and do not establish
Julia pricing parallelism.  Future parallel benchmarks must explicitly use, for
example, `JULIA_NUM_THREADS=$SLURM_CPUS_PER_TASK` (the jobs request four CPUs) or
`julia --threads=$SLURM_CPUS_PER_TASK`, and must log `Threads.nthreads()` in each
result.  The current timings must not be labeled as the parallelized variation.

## Validation criteria

For every matched CG/Direct case, record:

1. termination status and censoring reason;
2. CG LP-certification status;
3. CG and Direct integer objectives and their relative gap;
4. selected station supports, recognizing that equal objectives may have
   alternative optimal supports;
5. runtime, CG iterations/rounds, generated columns, labels, and Direct route
   count.

## Memory and censoring protocol

Memory is part of the experimental result, especially for DirectSolver.  Every
task requests and is limited to **16 GB**.  Final reporting must include Slurm
`MaxRSS` by method and instance size, plus counts of jobs that exceeded the
memory allocation.  A Direct task killed for OOM is a censored scalability
observation, not evidence of infeasibility or an optimization failure.

Completion outcomes must be separated into at least:

- successful solver return;
- Slurm out-of-memory termination;
- 90-minute Slurm timeout;
- internal enumeration/solver time limit;
- experiment-harness or other error.

Runtime summaries must use successful runs only and show success counts beside
their medians or means.  Increasingly censored DirectSolver results must not be
presented as representative runtime averages.  The 16 GB allocation must be
stated alongside every DirectSolver scalability comparison.

## Execution status

The first submitted array (`19653310`) failed before solving because a
standalone Julia docstring was incorrectly attached to a `using` statement in
the experiment harness.  It produced no optimization results and must not be
included in analysis.  The harness was corrected and load-validated; replacement
array `19667765` contains the 72 intended tasks.

## Final completion and certification

Primary core-CG array `19672983` completed all 36 CG tasks. Direct results are
reused from replacement array `19667765`, which produced 30 successful Direct
results from 36 tasks. There are three seeds per `(n,p,q)` cell, so `p=16` and
`p=32` must be reported separately rather than pooled into 6-run cells:

| `n` | `p` | CG q=1 | CG q=3 | Direct q=1 | Direct q=3 |
|---:|---:|---:|---:|---:|---:|
| 10 | 16 | 3/3 | 3/3 | 3/3 | 3/3 |
| 10 | 32 | 3/3 | 3/3 | 3/3 | 3/3 |
| 15 | 16 | 3/3 | 3/3 | 3/3 | 3/3 |
| 15 | 32 | 3/3 | 3/3 | 3/3 | 3/3 |
| 20 | 16 | 3/3 | 3/3 | 3/3 | 0/3 |
| 20 | 32 | 3/3 | 3/3 | 3/3 | 0/3 |
| **Total** |  | **18/18** | **18/18** | **18/18** | **12/18** |

Every core-CG run terminated `OPTIMAL`, proved its LP bound using exact physical
pricing, and required one CG round. Thus the limited generation phase did not
prematurely stall and restart in this matrix.

The six missing Direct results are exactly the three seeds for each
`(n=20,p=16,q=3)` and `(n=20,p=32,q=3)` cell; all were `OUT_OF_MEMORY` under
the stated 16 GB limit. There were no replacement-array timeouts or
solver/harness errors.

Median CG iteration counts were 34 at n=10, 36 at n=15, and 49 at n=20.
The success counts above are per three-seed cell, not pooled across `p`.

## CG versus Direct objective validation

There are 30 cases in which both methods returned an optimum.  All 30 agree to
relative tolerance `1e-7`; the largest relative discrepancy is
`3.66e-8` (an absolute difference of `5.69e-4` on an objective near 15,559),
well inside the configured MIP tolerance.  At n=10 all 12 pairs agree to about
`1e-11` absolute error.

Exact selected-station support matched in 16/30 paired cases: 12/12 at n=10,
4/12 at n=15, and 0/6 among the completed n=20 Direct pairs.  Objective
agreement is the relevant validation criterion; different supports can occur
under alternative or near-alternative integer optima, and support identity is
not required for formulation equivalence.

These results validate that the CG formulation and exhaustive-route monolithic
Direct formulation optimize the same free-assignment objective wherever Direct
fits in memory.  They also retain the usual CG distinction: certified LP pricing
exhaustion validates the relaxation bound, while agreement with Direct checks
that the generated pool contained a globally optimal integer solution in these
30 cases.

## Runtime comparison

Times below are task-internal wall times in seconds. Values are arithmetic means
over the three seeds for each `(n,p,q)` cell. Direct means use only successful
tasks; the success count exposes censoring.

| `n,p,q` | CG success | CG mean (s) | Direct success | Direct mean (s) | Direct / CG |
|---:|---:|---:|---:|---:|---:|
| 10,16,1 | 3/3 | 9.9 | 3/3 | 9.5 | 1.0x |
| 10,16,3 | 3/3 | 10.9 | 3/3 | 19.0 | 1.7x |
| 15,16,1 | 3/3 | 8.4 | 3/3 | 32.5 | 4.0x |
| 15,16,3 | 3/3 | 15.7 | 3/3 | 330.2 | 21.0x |
| 20,16,1 | 3/3 | 16.2 | 3/3 | 195.9 | 12.1x |
| 20,16,3 | 3/3 | 49.9 | 0/3 | -- (all OOM) | -- |
| 10,32,1 | 3/3 | 14.3 | 3/3 | 10.9 | 0.8x |
| 10,32,3 | 3/3 | 18.3 | 3/3 | 28.5 | 1.6x |
| 15,32,1 | 3/3 | 19.3 | 3/3 | 70.7 | 3.7x |
| 15,32,3 | 3/3 | 71.3 | 3/3 | 467.9 | 6.6x |
| 20,32,1 | 3/3 | 120.3 | 3/3 | 448.7 | 3.7x |
| 20,32,3 | 3/3 | 484.8 | 0/3 | -- (all OOM) | -- |

CG becomes increasingly favorable as scenarios and station count grow. Direct
is slightly faster in the smallest cells, while CG's advantage reaches 21x for
`n=15,p=16,q=3`. At `n=20,q=3`, CG completes both passenger-count regimes;
Direct has 0/3 successful runs for either `p=16` or `p=32`.

### CG pricing work

The `cg_iterations` field is the number of internal pricing iterations in the
core serial run. These are arithmetic means over the three seeds; `cg_rounds`
was 1 in every cell.

| `n,p,q` | CG iterations (mean) | CG columns (mean) |
|---|---:|---:|
| 10,16,1 | 14.3 | -- |
| 10,16,3 | 22.0 | -- |
| 15,16,1 | 15.0 | -- |
| 15,16,3 | 30.7 | -- |
| 20,16,1 | 24.7 | -- |
| 20,16,3 | 30.3 | -- |
| 10,32,1 | 50.7 | -- |
| 10,32,3 | 63.3 | -- |
| 15,32,1 | 45.7 | -- |
| 15,32,3 | 85.3 | -- |
| 20,32,1 | 68.3 | -- |
| 20,32,3 | 89.7 | -- |

The column counts are available per task in the CSV outputs, but the current
comparison reports iterations as the primary CG work measure.

## Memory results under the 16 GB limit

Slurm `MaxRSS` confirms different scaling behavior:

- CG remains around 0.8--1.6 GB throughout n=10--20; its largest observed RSS
  is approximately 1.6 GB.
- successful n=10 Direct tasks remain below about 1.7 GB;
- n=15 Direct ranges from roughly 1.2 GB to 15.6 GB, already approaching the
  allocation on the hardest cases;
- successful n=20,q=1 Direct tasks use roughly 3.6--8.6 GB;
- every n=20,q=3 Direct task reaches approximately 16.0 GB and is OOM-killed.

The 16 GB allowance is therefore essential context: Direct's missing n=20,q=3
runtime is memory censoring, not evidence of infeasibility, whereas CG completes
all corresponding cases with about one tenth of the allocation.

## Context against the nearest-open Benders study

The table below places free-assignment CG means beside the best-performing
nearest-open Benders configuration from the prior note, BendersYZ restricted MW.
The data axes, route/resource limits, and 16 GB / 90-minute task limits match,
but the assignment policies do not; these figures establish a system-level
scaling comparison, not a controlled same-model algorithm comparison.

| `n,p,q` | Core free CG mean (s) | BendersYZ MW success | BendersYZ MW mean (s) | Benders / CG |
|---:|---:|---:|---:|---:|
| 10,16,1 | 9.9 | 3/3 | 19.2 | 1.9x |
| 10,16,3 | 10.9 | 3/3 | 25.4 | 2.3x |
| 15,16,1 | 8.4 | 3/3 | 80.7 | 9.6x |
| 15,16,3 | 15.7 | 3/3 | 200.3 | 12.8x |
| 20,16,1 | 16.2 | 3/3 | 480.7 | 29.6x |
| 20,16,3 | 49.9 | 3/3 | 3,383.0 | 67.8x |
| 10,32,1 | 14.3 | 3/3 | 26.3 | 1.8x |
| 10,32,3 | 18.3 | 3/3 | 30.2 | 1.7x |
| 15,32,1 | 19.3 | 3/3 | 816.1 | 42.4x |
| 15,32,3 | 71.3 | 3/3 | 1,724.0 | 24.2x |
| 20,32,1 | 120.3 | 0/3 | -- | -- |
| 20,32,3 | 484.8 | 0/3 | -- | -- |

Free-assignment CG completes all 36 cases. It is faster in every Benders cell
where Benders returns, and it completes all six `n=20,p=32` cases while Benders
completes none of them. The assignment-policy caveat remains essential.
Because nearest-open assignment imposes different structure, the defensible
claim is:

> Under matched instance sizes, max-stops/resource settings, and compute limits,
> the new free-assignment CG implementation scales substantially better than the
> current nearest-open Benders implementation; within the free-assignment model,
> it matches exhaustive Direct objectives wherever Direct completes.

It is not valid to claim that the two methods solve the identical optimization
problem or to compare their objective values directly.

## Conclusions

1. New CG returned a certified optimum in 36/36 cases and needed one CG round
   throughout the matrix; every `(n,p,q)` cell is 3/3.
2. CG and Direct objectives agree within `3.66e-8` relative error in every one
   of the 30 completed Direct pairs.
3. Direct route enumeration becomes memory-limited: all three seeds in each
   `n=20,p=16,q=3` and `n=20,p=32,q=3` cell OOM at 16 GB, while CG completes.
4. CG's mean runtime advantage varies by passenger count; it reaches 21x for
   `n=15,p=16,q=3` and completes all six `n=20,p=32` cases.
5. Against the prior nearest-open Benders benchmark, free CG completes all
   36 cases and is faster in every Benders cell with a returned result, with
   the assignment-policy caveat stated above.

## p=32 comparison with BendersYZ restricted MW

The core serial-CG array already includes all 18 p=32 cases requested here
(n=10,15,20; seeds 42/123/999; q=1,3). The p=32-only arithmetic means are:

| `n,q` | Core free CG mean (s) | BendersYZ MW p=32 success | BendersYZ MW mean (s) | Benders / CG |
|---:|---:|---:|---:|---:|
| 10,1 | 14.3 | 3/3 | 26.3 | 1.8x |
| 10,3 | 18.3 | 3/3 | 30.2 | 1.7x |
| 15,1 | 19.3 | 3/3 | 816.1 | 42.4x |
| 15,3 | 71.3 | 3/3 | 1,724.0 | 24.2x |
| 20,1 | 120.3 | 0/3 | -- | -- |
| 20,3 | 484.8 | 0/3 | -- | -- |

At p=32, core CG completes all 18 cases.  BendersYZ MW completes all n=10 and
n=15 cases but none of the n=20 cases in the prior 16 GB/90-minute benchmark.
The p=32 comparison therefore strengthens the scaling picture, while retaining
the same assignment-policy caveat: Benders uses nearest-open assignment and CG
uses free passenger assignment.

## Core serial-CG rerun for the Benders comparison

The primary tables above now use CG-only array `19672983`, which reran the same
36 cases for the Benders comparison
with exactly one Julia thread and the following core configuration:

- `use_adaptive_cluster_certification=false`;
- `verify_reduced_costs=false`;
- `parallel_scenarios=true`, but `JULIA_NUM_THREADS=1`, hence serial execution;
- `station_simple_warm_start=false`;
- `seed_two_stop_routes=true`;
- 100 candidates harvested and up to 100 columns added per iteration.

Outputs are isolated under `results_core_serial/`; the earlier diagnostic
results remain available for audit. The primary results are arithmetic means
over three seeds for each `(n,p,q)` cell. The earlier pooled six-run medians
mixed `p=16` and `p=32` and should not be used for p-specific comparisons.
Adaptive clustering and reduced-cost verification were disabled in this core
benchmark.
