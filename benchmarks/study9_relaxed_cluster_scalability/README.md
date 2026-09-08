# Study 9 — full relaxed-cluster scalability

This study validates and then advances the complete `:relaxed_cluster` pipeline on
Zhuzhou instances (`p=16`, `s=3`, `max_stops=10`). Every task is one Julia process on one
node, with three Julia threads for parallel scenario pricing and one Gurobi thread.

The validation table covers `n=20,25`, ten matched seeds, and three arms:

- `exact`: the full exact pricer and its ordinary two-tier certification.
- `relaxed_k60`: current `:relaxed_cluster` with fixed `K/n=0.6`.
- `relaxed_k80`: current `:relaxed_cluster` with fixed `K/n=0.8`.

All unconfirmed features are explicitly disabled: `relaxed_cluster_max_count=nothing`,
`relaxed_cluster_barren_cache=false`, and `relaxed_cluster_cut_management=false`. The
confirmed no-good certification, harvested subset columns, parallel scenario pass, and
five-route cluster guidance remain part of `:relaxed_cluster` itself.

The six-hour frontier probe queues anchor sizes `n=30,40,50,60,70,80,84`. Each size runs
both confirmed fixed-K settings on ten seeds (20 tasks) with the same 21,600-second CG
budget. The empirical frontier is the largest size at which at least one setting certifies
**more than 90%** of instances. With ten seeds this deliberately means 10/10; 9/10 is the
first failed frontier. Intermediate `n=35,45,...` tables remain available to localize the
boundary afterward without changing the algorithm or budget.

Generate and validate:

```bash
julia --project=../.. generate_jobs.jl
bash -n submit_benchmark.sh
```

Submit validation from this directory:

```bash
mkdir -p slurm_logs
STUDY9_RUN_ID=2026-09-08_rc_scale sbatch --array=1-60 submit_benchmark.sh validation.tsv
julia --project=../.. check_gate.jl ../../benchmarks/experiments/2026-09-08_rc_scale_study9_relaxed_cluster_scalability validation
```

After that gate passes, submit only the next frontier (example for `n=30`):

```bash
STUDY9_RUN_ID=2026-09-08_rc_scale sbatch --array=1-20 submit_benchmark.sh n30.tsv
julia --project=../.. check_gate.jl ../../benchmarks/experiments/2026-09-08_rc_scale_study9_relaxed_cluster_scalability 30
```

Increase Slurm `--time` and `--mem` at later frontiers without changing the job-table
algorithmic budgets. Result rows record both limits, all certification outcomes, cut and
cache activity, refinement counts, thread IDs, iterations, labels, and columns.

Aggregate every completed stage with:

```bash
julia --project=../.. analyze.jl ../../benchmarks/experiments/2026-09-08_rc_scale_study9_relaxed_cluster_scalability
```

The analyzer writes all rows, matched validation pairs, and a frontier summary under
`benchmarks/results/`, while printing median runtime, paired speedup, CG iterations,
certification time, cut counts, and realized refined cluster counts.
