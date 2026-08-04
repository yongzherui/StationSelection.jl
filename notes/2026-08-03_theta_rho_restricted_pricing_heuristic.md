# Theta/rho restricted-pricing heuristic

## Evidence from the n=15 exhaustive-pricing census

The four `n=15`, `l=8`, `max_stops=4` runs exhausted label-setting at every
iteration. Their y-support trajectories were byte-for-byte identical to the
earlier 10,000-candidate runs, so the observed support movement was not caused
by early return.

The theta snapshots show that raw y-support churn is often degenerate:

- only 9/20 support-shift events had any positive-theta route using an entering
  station;
- only 4/20 had a theta >= 0.99 route using an entering station;
- 5/20 shifts had essentially zero theta movement;
- final solutions used 4--8 endpoint stations, leaving 0--4 of the nominal
  eight open stations as unused fillers.

Station lower-bound reduced cost did not rank entrants: every later-entering
station had zero previous reduced cost. It remains useful as a safe elimination
test, not as an ordering inside the degenerate zero-RC set.

Define theta endpoint load

    L_j = sum_r theta_r * 1{j is a pickup/drop-off endpoint of route r}.

Top-k theta-load subsets contained the unrestricted best route in 20.8%, 59.7%,
and 84.7% of snapshots for k=4, 6, and 8. Top-eight raw-rho stations contained
the whole best route only 5.6% of the time. Theta therefore identifies the
stable core; rho is better used to select exploration stations outside it.

Anchoring on the current top-l support was particularly strong: the best route
needed zero outside stations in 53/72 snapshots, one in 16/72, and two in 3/72.
Current support plus the highest-total-rho outsider recovered the exact best
route in 95.8% of snapshots; two outsiders recovered all 72.

## Heuristic under test

At each solved RMP:

1. Build a core from endpoints of theta >= 0.99 routes, ranked by `L_j` if the
   set is too large.
2. Fill remaining core slots by descending `L_j`.
3. Rank stations outside the core by total incident positive rho.
4. Restrict early label-setting to the core plus `m` rho-ranked outsiders.
5. Keep certification unrestricted. A certified result therefore retains the
   original optimality meaning even when the heuristic misses useful columns.

The first grid tests core sizes 4 and 6 with one or two outsiders. Metrics must
include wall/pricing time, labels, iterations, restricted misses recovered by
certification, final LP/MIP objective, and certification status. A useful
heuristic should preserve the certified objective while reducing labels or
pricing time; an uncertified faster result is reported only as a heuristic
solution, never as equivalent optimality.

## Termination semantics

The theta/rho restriction applies only to the early-return pricing phase. It
does not decide that column generation is finished.

For each outer CG round:

1. Solve the RMP and rebuild the theta core plus rho-ranked outsiders.
2. Run restricted early pricing on that station set.
3. If at least one novel improving column is added, re-solve the RMP and repeat.
4. If restricted pricing adds no column, run an unrestricted exhaustive
   certification pass over all stations.
5. If certification finds improving columns, add them, start another outer
   round, and resume restricted pricing.
6. Terminate with `cg_stop_reason = optimality_proven` only when unrestricted
   certification returns no improving column and reports that its search was
   exhausted.

Timeout, iteration-limit, missing-primal-solution, and non-exhausted-search
paths retain their existing non-certified stop reasons. In particular, a
restricted stall is never reported as proof.

This termination certifies the full LP relaxation, not the final integer
optimum over all possible columns. The final MIP is solved over the accumulated
pool. The core-6+2 counterexample (equal certified LP bound but MIP objective
28.19 worse) demonstrates why `LP-certified` and `integer-pool-equivalent` must
be reported separately. Integer-pool enrichment is a post-CG requirement if
equivalent MIP optimality is required.

## Interpretation rules

- Ignore a y-support shift with no theta-load change; it is likely filler churn.
- Treat positive-theta and near-one route endpoint sets as the effective support.
- Use rho for exploration, not to rebuild the whole support from scratch.
- Retain periodic/unrestricted certification as the recovery mechanism.

## First online comparison (20-column early return)

Four matched cases compared unrestricted pricing with core-4+1 outsider,
core-6+1, and core-6+2. Every run reported `optimality_proven`, and every
heuristic reproduced the unrestricted final LP bound. Durable internal timing
(rather than Slurm elapsed, which was dominated by startup noise) gives:

| variant | total time (s) | pricing time (s) | labels | CG iterations | final pool |
|---|---:|---:|---:|---:|---:|
| unrestricted | 9.53 | 6.36 | 564,367 | 23.5 | 705.8 |
| core 4 + 1 | 10.18 | 5.04 | 393,099 | 51.0 | 607.3 |
| core 6 + 1 | 8.49 | 4.61 | 249,773 | 46.5 | 671.5 |
| core 6 + 2 | 8.40 | 4.81 | 176,262 | 28.5 | 648.0 |

Core-4 was too restrictive: its 30% label reduction was lost to repeated
certification recovery and extra RMP cycles, making it 7% slower overall.
Core-6+1 was 11% faster and cut labels by 56%. Core-6+2 was 12% faster and cut
labels by 69%, making it the best tested restricted configuration despite its
slower final convergence in solve-count terms.

Most importantly, LP certification is insufficient for equivalence of the
final integer solve over the generated pool. Core-6+2 reproduced all four LP
bounds but returned MIP objective 46750.84 rather than the unrestricted
46722.65 for `n15_sc3_s42`, a 28.19 (0.060%) loss. The missing column need not
have negative reduced cost at the terminal LP dual and therefore is invisible
to ordinary LP certification.

Consequences for the next design:

- distinguish `LP-certified` from `integer-pool-equivalent`;
- add an integer-pool enrichment pass (for example, exhaustive columns on the
  final open support and one/two-station neighborhoods, or route-pool crossover
  from incumbent supports) before comparing MIP objectives;
- avoid overly small fixed cores; adaptively expand after a restricted miss;
- benchmark internal pricing time and labels separately from Slurm elapsed time.

### Objective trajectory and support response

The restricted searches were effective at coarse objective improvement but slow
in the tail. Averaged across four cases, the solve index reaching 90% of the
total LP improvement was 9.5 for unrestricted pricing, 10.2 for core-4+1, and
9.0 for both core-6 variants. Reaching 99% took 17.8, 47.8, 30.0, and 27.2
solves respectively. Thus core-6+2 preserves early progress but delays final
convergence.

Restricted early pricing itself produced 95.0% of total objective improvement
for core-4+1, 98.5% for core-6+1, and 99.3% for core-6+2. Unrestricted
certification supplies only the last small fraction, but repeated recovery and
RMP solves dominate the tail cost. Adaptive expansion should therefore remember
stations/routes found by recovery and target this tail rather than replacing
the successful early restricted phase.

Raw y churn did not become more meaningful. Roughly 45--50% of support-shift
events in every variant had an entering station used by a positive-theta route.
Core-4+1 and core-6+1 nevertheless ended with exactly the same active-route
endpoint station sets as the unrestricted baseline in all four cases. The
correct progress diagnostic is theta-loaded/effective support convergence, not
the number of raw top-l y changes.

## Durable output and recovery

Each diagnostic case now writes `*_iterations.csv` and `*_run_summary.csv` in
addition to the y, theta, route, rho, and subset tables. The summary contains
the full heuristic configuration, LP/MIP termination and objectives, internal
wall/pricing/LP/certification time, labels, iterations, rounds, pool size, and
early-versus-certification route counts.

The comparison is reproducible without Slurm accounting or console parsing:

```bash
julia --project=. scripts/analyze_theta_rho_heuristic_compare.jl \
  results/theta_rho_comparison \
  baseline=tmp_ss_bench/pfa_n15_theta_rho_baseline20_2026-08-03 \
  c4o1=tmp_ss_bench/pfa_n15_theta_rho_c4o1_2026-08-03 \
  c6o1=tmp_ss_bench/pfa_n15_theta_rho_c6o1_2026-08-03 \
  c6o2=tmp_ss_bench/pfa_n15_theta_rho_c6o2_2026-08-03
```

This writes case-level and variant-level CSVs including objective milestones,
normalized gap AUC, certification improvement share, meaningful y shifts,
theta/y movement, effective-support agreement, and LP/MIP deltas.
