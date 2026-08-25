# Study 1 -- Formulation LP/IP gap

## Objective

Quantify how much the Joint reformulation (assignment embedded in route columns)
tightens the LP relaxation bound relative to Base (decoupled assignment + route
coverage), and find the operating regimes where that tightening breaks down. This is
purely a **bound-quality** comparison -- no runtime/label/dominance metrics here (those
are Studies 2/3/5).

## Method

Same instance (or a small, fixed set of representative instances, generated via
`../../scripts/generate_zhuzhou_instance.jl`'s `generate_zhuzhou_data`), solved four
ways per instance:

| Formulation | Solver | Setting |
| --- | --- | --- |
| `AggregateODRouteBaseFormulation` | `DirectMIPSolver` | baseline |
| `AggregateODRouteJointRoutingAssignmentFormulation` | `CGSolver` | baseline |
| `AggregateODRouteJointRoutingAssignmentFormulation` | `CGSolver` | short `max_wait_time` |
| `AggregateODRouteJointRoutingAssignmentFormulation` | `CGSolver` | tight `detour_factor` |
| `AggregateODRouteJointRoutingAssignmentFormulation` | `CGSolver` | `max_stops=3` |

Base gets $z^{LP}$ by relaxing its up-front enumerated pool. Joint has no up-front
enumeration -- its $z^{LP}$ must come from the CG-converged master LP, which is only a
valid bound if pricing ran to exhaustion. **`CGSolver` must be constructed with
`recover_integer_solution=true`**, the only way `OptResult.metadata["cg_lp_objective_value"]`
gets populated (see package `benchmarks/README.md` / the top-level plan notes) --
without it there's no separate LP bound to read off, only the final integer objective.

## Metrics

Per cell: $z^{LP}$, $z^{IP}$, gap $=(z^{IP}-z^{LP})/z^{IP}$, `termination_status`,
solve/build runtime (from `OptResult`/its `metadata`).

## Prerequisites

None -- both formulations, both solvers, and `recover_integer_solution` all already
exist in `src/`. This study can be filled in first.

## Sequencing note

Depends on Study 2's pricer being correct (Joint's $z^{LP}$ is only meaningful if CG
pricing is exhaustive/correct) -- run after Study 2, not before, even though it has no
code prerequisite.

## Files

- `generate_jobs.jl` -- expands the (formulation, setting) grid above into
  `config/jobs.tsv`.
- `run_benchmark.jl` -- one job's body: build/reuse the fixed instance, `run_opt`, write
  one result row.
- `analyze.jl` -- `config/jobs.tsv` results -> case CSV + variant-summary CSV +
  `slides_results.tex` (via `../lib/latex_rows.jl`).
- `submit_benchmark.sh` -- SLURM array submission.
