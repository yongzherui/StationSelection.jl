# Study 4 — exact versus warm-start pricers

Compares three `AggregateODRouteJointRoutingAssignmentFormulation` CG strategies on ten
paired n=20, p=16, s=3 instances (seeds 42–51):

1. pure `:exact`;
2. `warm_start_mode=:station_simple` followed by `:exact`;
3. `warm_start_mode=:cluster_guide` followed by `:exact`.

All arms use the same master, seed pool, pricing clocks, and scenario
parallelism. The supplied SLURM script requests three CPUs and starts Julia with three
threads; three scenarios can therefore price concurrently. Gurobi is pinned to one
thread so this is the only parallelism.

Generate the 30 jobs with `julia generate_jobs.jl`, then submit with
`sbatch --array=1-30 submit_benchmark.sh`. Each TSV row is one independent SLURM task,
Julia process, and node. Summary CSVs report the actual pricing thread IDs
and counts by column source. `columns/` contains one row per generated column, including
source mode, route length, revisits, assignments, travel cost, and generation reduced cost.

## Fixed parameters

- n=20 stations, p=16 OD pairs, s=3 scenarios;
- seeds 42–51, max stops 10;
- three Julia threads and parallel scenario pricing;
- one Gurobi thread;
- 300 s regular pricing, 3600 s certifying pricing, 14400 s total CG budget;
- integer recovery enabled;
- cluster guide K=9 and five guide routes.

K=9 and five routes are starting values for the comparison, not claimed optima. A later
tuning sweep should be separate from this paired method comparison.
