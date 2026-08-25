# Study 5 -- Scalability vs. raw enumeration

## Objective

Runtime vs. `|P|` (passengers), `|J|` (candidate stations), `|S|` (scenarios), swept
independently, for three methods: `exact/`'s CG pricing, `darp/`'s pricing, and raw
brute-force enumeration. The point is showing exactly where enumeration stops scaling
and pricing keeps going -- not just that CG "is faster."

## Prior findings to extend, not re-derive

- `notes/2026-08-05_free_assignment_cg_direct_ms5_comparison.md` already has real
  n=10/15/20 CG-vs-Direct timing at `max_stops=5`: CG up to 21x faster, Direct OOMs at
  n=20/q=3 (16GB limit), CG stays at 0.8-1.6GB throughout. **This study's sweep should
  extend past n=20** (Direct already can't finish there) rather than re-measure n<=20.
- `notes/2026-08-04_adaptive_station_cluster_certification_results.md` found the
  opposite ordering at `max_stops=3`: **Direct is faster than CG all the way to n=40.**
  The crossover is stop-count-dependent, not purely size-dependent -- **this study's
  sweep must vary `max_stops` explicitly** to map where that crossover sits, not assume
  one fixed `max_stops` throughout. The same note flags a 0.36% integer-pool gap at
  n=15 -- worth carrying an integrity check (does the CG-generated pool's integer
  optimum match Direct's, when both finish) alongside the timing comparison.
- `notes/2026-07-31_pfa_equals_direct_enumeration_verified.md` is the correctness
  precondition (small instances, complete-column-set equivalence) but has no timing
  data by itself.

All of the above used the now-removed `AggregateODRouteModel`/`DirectSolver` -- these
are target/sanity-check numbers to reproduce and extend against the current API, not
scripts to resurrect.

## Prerequisite -- blocks the enumeration arm

**`enumerate_joint_routing_assignment_columns` does not exist.** Port it from
`../../src/opt/label_setting/route_covering/exact/enumeration.jl`'s
`enumerate_aggregate_od_route_columns` pattern: plain bounded DFS over the same label
transitions the pricer uses, dominance and reduced-cost pruning both off (uniform
positive rewards so nothing gets pruned), keep the cheapest column per served-signature.
This is a straightforward port of an existing, working pattern (see that file's own
module docstring for the exact rationale), not a new design -- it belongs in
`src/opt/label_setting/joint_routing_assignment/exact/`, not in this `benchmarks/`
directory. The CG and `darp/` arms of this study don't depend on it and can be built
first.

## Method

For each of `|P|`, `|J|`, `|S|` swept independently (holding the other two fixed, plus a
`max_stops` axis per the crossover above): build an instance via
`../../scripts/generate_zhuzhou_instance.jl`'s `generate_zhuzhou_data(data_dir,
n_stations, n_pairs; n_scenarios, ...)`, run all three methods under a wall-clock time
limit, record runtime + completion status (exhausted / timed out / OOM-killed) per cell.

## Metrics

Per (method, `|P|`, `|J|`, `|S|`, `max_stops`) cell: wall-clock runtime, memory (if
measurable -- see the OOM-censoring bookkeeping convention in the 2026-08-05 note above),
completion status, and (where more than one method completes) objective/optimum
agreement.

## Files

- `run_benchmark.jl` -- one cell: build instance, run the three methods, write one row
  per method.
- `analyze.jl` -- aggregate across cells, including the OOM/timeout censoring
  convention.
- `submit_benchmark.sh` -- SLURM array submission.
