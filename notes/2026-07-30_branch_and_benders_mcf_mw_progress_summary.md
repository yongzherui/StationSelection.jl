# Branch-and-Benders with MCF/MW cuts: progress summary

Date: 2026-07-30

## Current algorithm

The implemented `BranchAndBendersSolver` runs one single-threaded Gurobi branch-and-bound tree.
Station-opening variables remain binary. At integer callback solutions, the exact routing LP is
solved by certified column generation and a globally valid routing Benders cut is submitted lazily.
Repeated binary first-stage solutions are cached.

The master's routing variable represents the full routing recourse. A permanent MCF formulation is
used only as a lower bound on that same recourse; it is not added separately to the objective. The
tested scalable configuration embeds the Common-OD MCF relaxation permanently and separates two
additional cut families:

1. restricted-MW exact-routing cuts at binary `(y,z)` callback solutions; and
2. scenario-specific MCF MW user cuts at fractional `(y,z)` solutions below the root.

No dynamic MCF cuts are generated at the root. Gurobi's node count is checked explicitly, and root
skips are recorded as `branch_benders_projected_mcf_root_skips`. The permanent Common-OD MCF is the
only MCF contribution at the root.

## MCF MW cut construction and validation

For a fixed fractional master point, one scenario MCF primal is solved. Its explicit dual includes:

- every scalar linear constraint;
- lower and upper variable-bound rows;
- the correct sign restriction for equality, greater-than, and less-than rows; and
- the primal objective constant.

The dual is constrained to remain tight at the generating point and maximizes its value at a core
point. Its coefficients yield a global inequality in the live master `y` and `z` variables. Cut
evaluation and constraint construction share the `_MCFMWCut` representation, avoiding duplicate
coefficient algebra.

The targeted lifted-routing/Branch-and-Benders suite passed all 111 tests. The completion audit
reported zero cut tightness error and zero worst dual residual. Repricing from the completed routing
dual found no additional negative-reduced-cost columns (`n_new_columns=0`, `exhausted=true`).

## Bugs found and corrected

The failed n=15 attempts were implementation failures rather than invalid-cut certificates:

1. `objective_coefficient` was not a defined JuMP helper. Coefficients are now obtained with
   `coefficient(objective_function(model), variable)`.
2. Several completion functions required `Dict{Any,...}`. Julia's invariant `Dict` type rejected the
   concrete endpoint-chain dictionaries produced by callbacks. These interfaces now accept
   `AbstractDict`.
3. The explicit MCF dual implicitly assumed a zero objective constant. The constant is now carried
   through the dual objective and cut intercept.
4. Root MCF user-cut suppression was implicit. It is now an explicit, tested callback condition.

The first corrected n=15 attempt generated eight non-root separation rounds and 24 valid MCF MW
cuts before exposing the last invariant-`Dict` boundary. After correcting that boundary, the n=15
and n=20 runs completed successfully.

## Main computational results

All results below use the Zhuzhou instance, 16 OD pairs per scenario, three scenarios, seed 999, and
a 1% Gurobi MIP-gap target unless stated otherwise.

### n=15, max-stops 5

| Method | Runtime (s) | Nodes | Unique exact evaluations | Exact cuts | Final gap |
|---|---:|---:|---:|---:|---:|
| Full scenario MCF + restricted MW | 177.8 | 939 | 84 | 246 | 0.818% |
| Common-OD + older projected MCF cuts | 330.9 | 1,087 | 107 | 307 | 0.991% |
| Full scenario MCF + standard repriced cuts | 732.9 | 1,360 | 110 | 317 | 0.965% |
| Classical BendersYZ + restricted MW | 1,457.9 | - | 100 iterations | - | converged |

All successful methods returned objective `57481.83283904121` and stations
`[11, 22, 92, 100, 133, 138, 158, 202]`.

### n=20, Common-OD + new MCF MW

| Metric | max-stops 4 | max-stops 5 |
|---|---:|---:|
| Certified objective | 48775.98465466723 | 47994.06465466723 |
| Runtime (s) | 546.3 | 2074.8 |
| Nodes | 6,265 | 7,416 |
| Unique exact evaluations | 265 | 277 |
| Exact-routing cuts | 726 | 771 |
| Exact-oracle time (s) | 476.6 | 1962.3 |
| MCF MW cuts | 22 | 24 |
| MCF MW time (s) | 34.6 | 48.1 |
| Final gap | 0.972% | 0.979% |
| Time to eventual optimal incumbent (s) | 118 | 430 |
| Time to 5% certified gap (s) | 539 | 2034 |

For max-stops 4, the direct solve took 143.3 seconds and returned the identical objective and station
set. Branch-and-Benders found that incumbent in 118 seconds, but required another 428 seconds to
certify a 1% gap. Thus incumbent discovery is competitive; lower-bound certification and repeated
exact-oracle work dominate runtime.

For max-stops 5, the new MCF MW implementation improved on the older projected-cut Common-OD run:
runtime fell from 2615 to 2075 seconds, unique exact evaluations from 350 to 277, and exact cuts from
962 to 771. It nevertheless remained about five times slower than the full scenario-MCF restricted-
MW run (410 seconds). The Common-OD master is much smaller, but loses too much scenario-specific
lower-bound strength.

## Scaling experiments and decision

The n=30/n=40 full-MCF and Common-OD runs were stopped because their gaps remained large after
multiple hours. Full MCF produced much stronger bounds but expensive node relaxations; Common-OD
processed nodes quickly but its lower bound progressed too slowly. The stopped snapshots were:

| Formulation | Size | Runtime snapshot | Gap |
|---|---:|---:|---:|
| Full MCF | n=30 | about 3h13m | 20.6% |
| Full MCF | n=40 | about 3h23m | 28.3% |
| Common-OD MCF | n=30 | about 2h36m | 46.8% |
| Common-OD MCF | n=40 | about 1h43m after restart | 62.1% |

The immediate optimization priority is therefore not adding more MCF separation rounds. It is to
reduce exact-routing oracle calls and strengthen the scenario-specific lower bound without embedding
every scenario's full MCF network.

## Reproduction and logs

- n=15 corrected validation:
  `experiments/mcf_mw_yz_n15_v4_20260729/`
- n=20 Common-OD + MCF MW, max-stops 5:
  `experiments/mcf_mw_yz_n20_20260729/`
- n=20 Common-OD + MCF MW, max-stops 4:
  `experiments/mcf_mw_yz_n20_ms4_20260729/`
- n=20 full-MCF restricted-MW baseline:
  `experiments/branch_and_benders_yz_mw_uncapped_n20_20260729/`
- n=20 direct max-stops-4 baseline:
  `experiments/direct_n20_ms4_clean_20260729/`

Julia/Gurobi experiments must be run through Slurm, with persistent logs and outputs under the shared
home repository rather than `/tmp`. The job wrapper uses line-buffered Julia output for readable
incremental Slurm logs.
