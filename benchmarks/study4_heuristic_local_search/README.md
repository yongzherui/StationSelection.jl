# Study 4 -- Heuristic pricing frontier (`local_search`)

## Status

Placeholder. The deck's own frame for this ("Potential Heuristic: Push the Scalability
Frontier") is explicit that it's a concept, not a spec -- restrict-search -> heuristic
pricing -> exact fallback, with the actual design (candidate-set restriction, reward
thresholds, beam width, warm starts, exact-pricing frequency) still to be chosen. This
scaffold exists so the study has a number and a directory, matching Studies 1/2/3/5 --
it is not scheduled to be filled in yet.

`local_search` below is a placeholder method name for whatever the eventual heuristic
pricer turns out to be, not a commitment to local search specifically -- swap it (and
the directory name) once the actual design lands.

## Objective (as scoped by the deck, subject to change)

A fast restricted search finds strong negative-reduced-cost columns before invoking
`exact/`'s (or `darp/`'s) full pricing, the same way `AggregateODRouteJointRoutingAssignmentFormulation`
already dispatches between pricing strategies via its `pricing_mode` keyword
(`:exact`/`:darp` today) -- a `:local_search` (or whatever it ends up named) value would
be a third option there, not a separate code path outside the existing pricer hierarchy.

## Design choices to evaluate (from the deck, unresolved)

Candidate-set restriction, reward thresholds, beam width, warm starts, exact-pricing
frequency (how often the heuristic hands off to `exact/`/`darp/` for a certified
column).

## Success criteria (from the deck)

Time to first improving column, total solve time, optimality gap, largest solvable
instance -- likely measured the same way as Study 5's scaling sweep, once this exists.

## Files

- `run_benchmark.jl`, `generate_jobs.jl`, `analyze.jl` -- placeholder stubs only,
  intentionally thinner than Studies 1/2/3/5's (no I/O contract to sketch yet, since
  there's no pricer to call).
- `submit_benchmark.sh` -- same SLURM plumbing shape as the other studies, so it's ready
  once there's something to submit.
