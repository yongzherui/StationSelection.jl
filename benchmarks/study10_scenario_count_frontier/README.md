# Study 10 -- scenario-count frontier

**Question: does the relaxed-cluster certification frontier scale in `n`, or in `n x s`?**

Study 9 measured everything at s=3 -- all 490 job rows it ever wrote -- so the scenario
count is completely unmeasured as a difficulty axis. Yet a round certifies only when EVERY
scenario certifies in the SAME iteration, which makes `s` a plausible first-order driver.

## The two hypotheses, and they disagree

**CONJUNCTION.** Round success is roughly the product of three per-scenario successes.
Removing the conjunction (s=1) should move the frontier a long way, and n=50 -- which is
0/4 at s=3 -- should come back into reach.

**PER-SCENARIO.** The blocking lives inside individual scenarios and the conjunction is
incidental. Evidence for this, from reach5's per-scenario dumps at n=40: the five failing
seeds contained scenarios that never certified even ONCE. Seed 42 recorded zero
certifications across all three scenarios in 59 attempts, and 8 of the 10 scenario-slots
across the failing seeds were C=0.

One of these is wrong. That is the study.

## Arms

| arm | K2 | K1 | guides | cap |
| --- | --- | --- | --- | --- |
| `relaxed_k60` (single-tier) | 0.6n | -- | 5 | -- |
| `twotier_k60m16c16g3` | 0.6n | 16 | 3 | 16 |

K2 follows Study 9's 0.6n rule (n=40 -> 24, n=50 -> 30). K1=16 is an ABSOLUTE node count
from the measured 12-16 band, valid against both K2 values. cap=16 / g=3 are the
recommendation in `notes/2026-09-09_n40_certification_frontier_5_of_10.md`.
`aligned_subset_max` is inert for the single-tier arm and stays at its default 15 there
rather than being dressed up as a swept parameter.

## Tables

| table | jobs | what |
| --- | --- | --- |
| `smoke_s1.tsv` | 2 | n=20 s=1, both arms, 900 s |
| `s1_frontier.tsv` | 40 | n=40/50 x 2 arms x seeds 42-51, s=1, 7200 s |
| `s3_control.tsv` | 20 | n=50 x 2 arms x seeds 42-51, s=3, 7200 s |

```bash
cd benchmarks/study10_scenario_count_frontier
julia --startup-file=no generate_jobs.jl
STUDY10_RUN_ID=_smoke_s1 sbatch --array=1-2  --time=00:40:00 submit_benchmark.sh smoke_s1.tsv
STUDY10_RUN_ID=2026-09-09_s1 sbatch --array=1-40 --time=02:30:00 submit_benchmark.sh s1_frontier.tsv
julia --startup-file=no --project=../.. analyze.jl ../experiments/2026-09-09_s1_study10_scenario_count_frontier
```

**Smoke first, always.** s=1 has never run through this harness, and it takes a different
code path: the concurrent scenario loop in `certify.jl` is gated on
`length(scenarios) > 1`, so s=1 uses the serial branch Study 9 never exercised.

**Wall time.** 2.5 h for a 7200 s budget. MEASURED on Study 9's reach3 array (job 22356068,
same budget): worst elapsed 2:05:21, i.e. ~5.3 min of depot-copy + startup overhead at 16
concurrent tasks. Do not inherit the 6.5 h default that belongs to the 21600 s arms.

## Baselines to compare against (all from Study 9, s=3)

| | budget | certified |
| --- | --- | --- |
| n=40 `relaxed_k60` | 21600 s | 4/10 -- 44, 46, 48, 49 |
| n=40 `twotier_...c16g3` | 7200 s | 5/10 -- 43, 44, 46, 48, 49 |
| n=50 `relaxed_k60` | 21600 s | **0/4**, all `pricing_inconclusive`, quitting at 5.6-15.6k s |
| n=50 two-tier | -- | never run |

The s=1 arms get 7200 s where the n=40 single-tier baseline got 21600 s, so a win there is
a win at one third the budget; the two-tier comparison at n=40 is budget-matched.

## What NOT to expect

**s=1 is not the same problem made smaller.** `y` (station build) is shared across
scenarios, so at s=1 the station selection is driven by one scenario's demand alone:
different master, ~1/3 the columns, different optimum. This locates the frontier as a
function of `s`; it does not measure "the same instance, easier".

**Do not expect a 3x per-round speedup.** Scenarios already price CONCURRENTLY on 3
threads, so a round's wall is the MAX over scenarios, not the sum. Keep the hard scenario
and you save nothing per round. The gain, if any, is the conjunction and the smaller
master. If s=1 IS much faster per round, that is itself a finding -- it would mean the
serial/parallel accounting or master size matters more than currently believed.

## Provenance of the number 10

`study10_nogood_certification_scaling` previously held this slot and was deleted in commit
`bdd8d3a`; its findings survive in
`notes/2026-09-06_relaxed_cluster_harvesting_refinement_and_cuts.md`, which still refers to
it by that path. Different question, same number -- renumber if that ambiguity bites.
