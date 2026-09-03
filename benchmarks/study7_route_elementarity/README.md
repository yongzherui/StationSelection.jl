# Study 7 — Are optimal route columns elementary?

**Question.** In a *certified* joint routing+assignment CG optimum, do the selected route
columns visit each station at most once?

The exact pricer's labels are revisit-tolerant — `JointRoutingAssignmentPricingLabel`
(`label_setting/joint_routing_assignment/types.jl`) is deliberately not the elementary
label that `station_simple/` uses — so a route that returns to a station it has already
visited is representable, priceable, and can enter the master. Whether the optimum ever
*wants* one is a separate, unmeasured question. It matters because an affirmative answer
("optima are essentially always elementary") is the precondition for the cheaper elementary
pricer being safe to use as more than a warm start, and non-elementary optima would explain
part of the LP–MIP structure recorded in earlier studies.

## What this study is, and what it is not

It is not a timing study. Nothing here compares solvers, arms, or dominance rules; wall
clock is recorded only to confirm the budget was adequate. The artefact is the *content* of
the optimal solution, which no previous study preserved: `result.solution` is `nothing` for
every formulation in the package, and the routes lived only inside `result.model`'s
`joint_routing_assignment_{theta,columns}` dicts until the process exited. Study 7 is the
first study to call `export_variables`.

## Design

| Setting | Value | Why |
| --- | --- | --- |
| `n_stations` | 20 | Every cell must certify. n=20/s=3 certified 40/40 at p≤16 and 8/10 at p=24 in Study 5. |
| `n_pairs` | 8, 16, 24 | The axis. More passengers per scenario ⇒ longer routes ⇒ revisiting has a chance to pay. p=32 excluded: 1/10 certified in Study 5. |
| `n_scenarios` | 3 | Study 5's baseline; keeps every cell inside the certified frontier. |
| `seed` | 42–51 | 10 instances per regime, 30 jobs total. |
| `max_stops` | 10 | Against 20 stations, an elementary route is always available at this cap, so a non-elementary optimum is a genuine preference and not a cap artefact. |
| CG budget | 300 s/round, 3600 s certifying, 14400 s total | The 4 h total is measured, not guessed: all 8 p=24 cells that certified in Study 5 did so within 12,099 s, and the 2 that failed were still running at 21,600 s. |
| `recover_integer_solution` | `true` | Load-bearing. Without it `result.model` is the LP master and every exported `theta` is fractional — there is no "selected route" to ask about. |
| pricing | parallel, Gurobi 1 thread | Study 5's faster arm; chosen only for certifications per wall-hour. |

**Uncertified cells are never pooled into the headline.** A budget-stopped run's columns
come from a restricted master pricing never exhausted: the selection is a valid upper
bound, but asking whether *the optimum* revisits is not a question its columns can answer.
`analyze.jl` reports them as a separate row.

## Files

```
generate_jobs.jl      -> config/jobs.tsv   (30 rows)
submit_benchmark.sh   sbatch --array=1-30 submit_benchmark.sh
run_benchmark.jl      one job: solve, write metrics row, export routes
analyze.jl            aggregate + answer the question
```

Outputs land in `benchmarks/experiments/<date>_study7_route_elementarity/`:

- `job_NNNN.csv` — one metrics row per job (certification status lives here)
- `routes/job_NNNN/variable_exports/` — the artefact:
  - `route_activations.csv` — one row per selected θ, with `route_station_ids` (the decoded
    stop sequence), `n_stops`, `n_distinct_stations`, and `is_elementary`
  - `route_assignments.csv` — one row per (column, passenger): `p`, origin/dest, demand,
    pickup/dropoff station, and the boarding/alighting stop positions **as route replay
    certified them** (`column.metadata["assignment_positions"]`). On a revisiting route the
    certifying dropoff is the earliest ride-limit-feasible index and its pickup is the most
    recent prior visit to the origin, so these cannot be re-derived from the station pair
    after the fact; `0` marks a column built by a path that does not record them.
  - `walk_only_assignments.csv` — the `x_walk` complement, so all demand is accounted for
  - `station_selection.csv` — `y`
  - `variable_export_metadata.json` — includes the two self-checks below

## Reading the result

`analyze.jl` reports elementarity twice. The headline over all selected columns is
*optimistic by construction*: a 2-stop column cannot revisit, so it is elementary
trivially. The `multi-stop (≥3)` rate restates it over the columns that could have
revisited — the only ones where elementarity is a choice the optimum made. Prefer that
number.

Two self-checks are recomputed from the exported rows alone and must hold before any of
this means anything:

- `coverage_shortfall_max` — over every `(s, p)`, `max(0, 1 − (Σ θ over covering columns +
  x_walk))`. Coverage is a `>=` constraint, so a shortfall is a violation while an excess is
  merely paid for; `coverage_max` reports the latter without flagging it.
- `objective_residual` — `Σ θ·column_cost + Σ x_walk·walk_cost` against
  `result.objective_value`.

Either one drifting means the exporter has misread an index convention (`p` vs station
array index vs station id), and the elementarity numbers are meaningless.
