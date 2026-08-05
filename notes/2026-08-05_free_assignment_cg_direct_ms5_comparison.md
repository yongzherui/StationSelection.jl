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

New CG harvests 100 candidates per scenario, globally retains up to 100 per
iteration, and uses adaptive cluster certification with exact physical-pricing
fallback.  Direct exhaustively enumerates the physical route universe and then
solves the monolithic free-assignment model with explicit station-selection,
passenger-assignment, and route variables.

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

Replacement array `19667765` completed.  It produced 66 result files from 72
tasks:

| `n` | CG optimal/certified | Direct optimal | Direct OOM | Other failures |
|---:|---:|---:|---:|---:|
| 10 | 12/12 | 12/12 | 0 | 0 |
| 15 | 12/12 | 12/12 | 0 | 0 |
| 20 | 12/12 | 6/12 | 6 | 0 |
| **Total** | **36/36** | **30/36** | **6** | **0** |

Every CG run terminated `OPTIMAL`, proved its LP bound, and required one CG
round.  Thus the limited generation phase did not prematurely stall and restart
in this matrix.  The six missing Direct results are exactly the six
`n=20,q=3` cases; Slurm classified every one as `OUT_OF_MEMORY` under the stated
16 GB limit.  There were no replacement-array timeouts or solver/harness errors.

Median CG iteration counts were 34 at n=10, 36 at n=15, and 49 at n=20.

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

Times below are task-internal wall times in seconds.  Direct medians use only
successful tasks; the success count is shown to expose censoring.

| `n,q` | CG success | CG median | Direct success | Direct median | Direct / CG |
|---:|---:|---:|---:|---:|---:|
| 10,1 | 6/6 | 9.9 | 6/6 | 10.5 | 1.1x |
| 10,3 | 6/6 | 12.1 | 6/6 | 24.6 | 2.0x |
| 15,1 | 6/6 | 13.5 | 6/6 | 52.0 | 3.9x |
| 15,3 | 6/6 | 36.5 | 6/6 | 371.9 | 10.2x |
| 20,1 | 6/6 | 51.4 | 6/6 | 260.2 | 5.1x |
| 20,3 | 6/6 | 206.9 | 0/6 | -- (all OOM) | -- |

CG becomes increasingly favorable as scenarios and station count grow.  It is
roughly tied with Direct at n=10,q=1, twice as fast at n=10,q=3, 4--10x faster
at n=15, and about 5x faster for n=20,q=1.  At n=20,q=3 only CG completes.

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

The table below places free-assignment CG medians beside the best-performing
nearest-open Benders configuration from the prior note, BendersYZ restricted MW.
The data axes, route/resource limits, and 16 GB / 90-minute task limits match,
but the assignment policies do not; these figures establish a system-level
scaling comparison, not a controlled same-model algorithm comparison.

| `n,q` | Free CG median (s) | BendersYZ MW success | BendersYZ MW median (s) | Benders / CG |
|---:|---:|---:|---:|---:|
| 10,1 | 9.9 | 6/6 | 21 | 2.1x |
| 10,3 | 12.1 | 6/6 | 29 | 2.4x |
| 15,1 | 13.5 | 6/6 | 286 | 21.2x |
| 15,3 | 36.5 | 6/6 | 823 | 22.5x |
| 20,1 | 51.4 | 3/6 | 398 | 7.7x |
| 20,3 | 206.9 | 3/6 | 3,390 | 16.4x |

Free-assignment CG is faster in every cell and completes all 36 cases.  The
largest median differences occur at n=15, where it is about 21--23x faster.
At n=20 it is about 8--16x faster while Benders completes only half the cases.
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
   throughout the matrix.
2. CG and Direct objectives agree within `3.66e-8` relative error in every one
   of the 30 completed Direct pairs.
3. Direct route enumeration becomes memory-limited: all six n=20,q=3 tasks OOM
   at 16 GB, while CG remains below about 1.6 GB and completes.
4. CG's median runtime advantage over Direct grows from near parity at n=10,q=1
   to 10x at n=15,q=3; n=20,q=3 has no successful Direct comparator.
5. Against the prior nearest-open Benders benchmark, free CG is 2--23x faster
   by median and more reliable, with the assignment-policy caveat stated above.
