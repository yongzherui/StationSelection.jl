# Two-stop seeding, and what the PFA bound trajectory actually looks like

Results from 2026-07-30/31. Companion to
`2026-07-30_pfa_scaling_and_cg_throughput_experiments.md` (the n x p scaling
grid) and `2026-07-30_passenger_pricing_label_search_optimizations.md` (the
pricer itself). Raw data lives under `experiments/` (gitignored):
`2026-07-30_pfa_scaling_grid/{route_lengths,seed_ab,seed_ab_n30}/` and
`2026-07-31_pfa_scaling_grid_seeded/`.

## 1. The "upper bound" was mostly big-M, not routing cost

The per-iteration `lp_bound` in the CG logs is the restricted master's LP
objective -- for this minimization a decreasing upper bound on the LP optimum.
Read naively it looks like CG improves the bound by 39x-131x. It does not.

| cell | iter-1 LP | final | ratio | first iterate covering all pax | remaining drop |
| --- | --- | --- | --- | --- | --- |
| n10_p8 | 1487480 | 38516 | 39x | iter 3 (49869) | 22.8% |
| n10_p32 | 6527973 | 67443 | 97x | iter 20 (99681) | 32.3% |
| n15_p32 | 6705514 | 52691 | 127x | iter 25 (64196) | 17.9% |
| n20_p24 | 4539579 | 39038 | 116x | iter 11 (57589) | 32.2% |
| n30_p16 | 3654327 | 39689 | 92x | iter 24 (39689) | **0.0%** |
| n30_p24 | 5279500 | 48137 | 110x | iter 38 (50836) | 5.3% |
| n30_p32 | 7213519 | 55093 | 131x | iter 43 (56127) | 1.8% |

The pool used to start EMPTY, so until it happened to cover every passenger the
objective was dominated by `unserved_penalty * sum_p v[p]`. Measured from the
first iterate that covers everyone, genuine CG improvement is only **1.8%-50%**,
typically 20-35%. On the n=30 cells the harvest that achieves coverage also
lands essentially the final bound -- n30_p16 goes 505% -> 0.0% in one iteration.

## 2. The bound is a step function that then flatlines

Gap above each run's own final value (%), at deciles of CG wall time:

```
cell         10%    20%    30%    40%    50%    60%    70%    80%    90%   100%
n10_p32     9579   37.0   19.1   12.1   7.28   2.27   1.05   0.02   0.02    0
n15_p16     14.0   5.09   1.52   1.52   0.54   0.44   0.44   0.44   0.43    0
n20_p24     20.8   12.8   6.88   2.26   1.81   ~0     ~0     ~0     ~0      0
n30_p16    510.4  505.6  505.6    0      0      0      0      0      0      0
n30_p24     1701  864.2  861.9  861.0  861.0    0      0      0      0      0
n30_p32     1126   1120   1119   1117   1117   1.88   1.88   1.88    0      0
```

Share of CG wall time spent AFTER the last improvement worth >0.01% relative:

| cell | iters | stop | last real improvement | % of time after |
| --- | --- | --- | --- | --- |
| n30_p16 | 43 | TIME | iter 24 | **64%** (~1h55m of 3h) |
| n30_p8 | 17 | OPT | iter 11 | 44% |
| n30_p24 | 53 | TIME | iter 42 | 41% |
| n20_p24 | 80 | OPT | iter 59 | 40% |
| n20_p16 | 34 | OPT | iter 24 | 39% |

Two tail shapes. **Hard flatline** (all n=30, n20_p8/16/24): the bound stops
moving *exactly* for the last 20-45% of iterations, while each early-return
iteration still burns its full `PFA_PRICING_TIME=120` and each certification
pass its full `PFA_CERT_TIME=1800`, returning 0-7 columns. **Slow creep**
(n10_p32, n15_p24/32, n20_p32): sub-1% improvements continue to within 1-2
iterations of the end.

On the truncated cells the time is not spent improving the bound -- it is spent
failing to prove no improving column remains.

## 3. Selected routes are short, and so is the whole pool

`max_stops` was genuinely unbounded (`PFA_MAX_STOPS=0` -> `typemax(Int)`,
`truly_unbounded_stops: true`) with `max_visits_per_node=3`.

| cell | pool | selected | selected stop histogram | distinct stations | pax/col |
| --- | --- | --- | --- | --- | --- |
| n10_p8 | 246 | 3 | 3:1 4:1 6:1 | 2-5 | 6.0 |
| n10_p24 | 2442 | 7 | 2:1 3:2 4:4 | 2-4 | 8.6 |
| n10_p32 | 4515 | 6 | 4:4 5:2 | 4-5 | 13.2 |
| n15_p16 | 1245 | 3 | 5:1 6:1 7:1 | 5-6 | 12.7 |
| n20_p16 | 1330 | 3 | 4:1 6:1 7:1 | 4-7 | 12.0 |
| n30_p8 | 327 | 4 | 4:4 | -- | -- |

Only 3-7 columns are selected in total (~1-2 routes per scenario), carrying 4-13
passengers each. The *pool* is short too -- stop counts peak at 3-4 and die at 8
(exactly one 8-stop column among 4,515 at n10_p32). `beta*tau_r` grows linearly
per leg while `sum_p rho_p` saturates once the route has swept the passengers
its walk-feasible `(j,k)` can reach.

**Do NOT "optimize" this by capping `max_stops`.** A finite cap sets
`bounded_max_stops=true`, which adds `a.route_length <= b.route_length` to the
dominance test (`labels.jl:420`), making dominance *harder* to establish and the
frontier larger. The uncapped path is the fast one; short routes are already
emerging on it for free.

## 4. Two-stop seeding

`passenger_free_assignment_two_stop_seed_columns` (master.jl) seeds one column
per distinct `(scenario, j, k)` before the first LP. `seed_two_stop_routes=true`
is the library default; `PFA_SEED_TWO_STOP=0` reproduces the empty-pool runs.

Why it is exactly the right seed set: `_default_unserved_penalty` derives the
big-M from "serving ONE passenger never requires more than a direct two-stop
route `[j,k]`" -- so these are precisely the columns `v[p]` was standing in for.
And they are provably sufficient: `ride_limit[(p,j,k)] = detour_factor *
travel(j,k)`, replaying `[j,k]` ages the pickup at `j` by exactly `travel(j,k)`,
and `AggregateODRouteModel` enforces `detour_factor >= 1`, so every feasible
`(p,j,k)` is certified by its own two-stop route. One column per `(s,j,k)` not
per `(p,j,k)`: a two-stop route carries every passenger of that scenario whose
pair it certifies. The walking filter keeps the count well under `n(n-1)`
(448 at n=20, 1310 at n30_p16).

### Correctness

8 cells (n10-n30), both arms: identical certified `lp_bound` and MIP objective
(`d_lp <= 1.3e-16`), `ALL CERTIFIED CELLS AGREE`. 276/276 unit tests pass
(`test/opt/test_passenger_free_assignment_seeding.jl`), pinning coverage
(seeded assignments == all feasible `(p,j,k)`), optimum-preservation, and that
the seeded iteration-1 LP is below one `unserved_penalty`.

### Benefit is size-dependent

| cell | unseeded | seeded | note |
| --- | --- | --- | --- |
| n10_p8 | 3.3s | 0.2s | first arm absorbed JIT; not a real 16x |
| n10_p16 | 2.0s | 0.9s | |
| n15_p8 | 1.3s | 1.0s | |
| n10_p24 | 3.6s | 3.9s | slightly worse |
| n20_p8 | 10.0s | 10.3s | slightly worse |
| n15_p16 | 49.6s | 63.1s | **worse** |
| **n30_p8** | **1426.2s** | **709.8s** | **2.0x**, 14->10 iters, same optimum |
| **n30_p16** | mip 45643.16 | **mip 42750.11** | equal 1800s budget, **6.3% better** |

At n<=20 it is a wash or slightly negative -- the pool grows (221->637 at
n20_p8) so every LP carries more columns, and the coverage phase was only
seconds anyway. At n=30 it pays clearly. Mechanism is both fewer iterations
*and* cheaper ones: n30_p8 pricing went 92s/iter -> 62s/iter. Starting empty
makes the pricer maximize against `alpha_p ~ M`, where nearly every column looks
improving and the label search explores a huge attractive frontier.

For n30_p16 neither arm certified, so compare the MIP objectives (real feasible
solutions), not `lp_bound`. Reference: the full 3h unseeded run of that cell
reaches 39688.68.

**Small cells were misleading.** Concluding from the n<=20 A/B alone would have
rejected a change that halves runtime at n=30.

## 5. Status and next

- Full seeded n x p grid re-run: job 19368525 ->
  `experiments/2026-07-31_pfa_scaling_grid_seeded/`, same config as the
  2026-07-30 baseline so wall times compare cell-for-cell. **Still running.**
- Unexplored and probably the biggest remaining win on truncated cells: a
  **tail-detection stopping rule** (bail out of early-return once k consecutive
  iterations move the bound <0.01%), given section 2's 40-64% dead time.
- Next direction: a **relaxed pricer with easier termination conditions**. The
  motivation is section 2 -- certification is what fails to terminate, and it
  fails while the bound is provably not moving.
