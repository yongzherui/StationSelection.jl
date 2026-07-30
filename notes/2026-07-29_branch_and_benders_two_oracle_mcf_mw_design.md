# Branch-and-Benders with common-OD MCF and two MW cut oracles

Date: 2026-07-29

## Purpose

This note records the current Branch-and-BendersYZ algorithm, the routing lower bounds and
cut families implemented so far, the computational evidence motivating the latest design,
and the next improvements to test.

The target problem is

\[
\min_{y\in Y\cap\{0,1\}^m}
C_{\mathrm{walk}}(y,z)+\beta\sum_s Q_s(y,z),
\]

where `Q_s` is the exact column-generated routing LP for scenario `s`. The master carries one
recourse variable `theta[s]` per scenario. Every routing lower bound and Benders cut bounds the
same `theta[s]`; no MCF cost is added separately to the objective.

## Master formulation

The master contains:

- binary station-opening variables `y[j]`;
- continuous endpoint-chain variables `z`, shared by the walking model, BendersYZ cuts, and
  MCF endpoint flow requirements;
- one nonnegative `theta[s]` per scenario;
- exact nearest-open walking cost;
- a permanently embedded common-OD MCF relaxation;
- ordinary cuts known before optimization;
- dynamically generated fractional MCF user cuts;
- dynamically generated exact-routing lazy constraints.

The objective is

\[
\min\quad C_{\mathrm{walk}}(y,z)+\beta\sum_s\theta_s.
\]

The permanent common-OD constraint is

\[
\sum_{s\in S}\theta_s \ge |S|L_{\mathrm{common}}(y,z).
\]

`L_common` uses only physical OD pairs occurring in every scenario. This commodity set is a
restriction of every scenario MCF, so

\[
L_{\mathrm{common}}(y,z)\le L_{\mathrm{MCF},s}(y,z)\le Q_s(y,z)
\]

for every scenario, which proves the scaled aggregate inequality. Although earlier logs and
discussion sometimes abbreviated the expression as `L_common(y)`, its flow balance uses the
master-compatible endpoint selectors and is therefore coupled to both `y` and `z`.

## Cut family 1: fractional full-MCF MW user cuts

At selected fractional branch-and-bound solutions `(y_hat,z_hat)`, the algorithm constructs the
complete scenario-specific MCF LP with auxiliary copies fixed to the callback values:

\[
y^{\mathrm{aux}}=\hat y,\qquad z^{\mathrm{aux}}=\hat z.
\]

For scenario `s`, let its optimal value be

\[
v_s^{\mathrm{MCF}}=L_{\mathrm{MCF},s}(\hat y,\hat z).
\]

The implementation explicitly constructs the complete finite MCF dual, including variable-bound
dual rows. Let `phi_s(pi;y,z)` be the affine dual objective. Full-dual Magnanti-Wong selection is

\[
\begin{aligned}
\max_{\pi\in D_s}\quad &\phi_s(\pi;y^0,z^0)\\
\text{s.t.}\quad &\phi_s(\pi;\hat y,\hat z)=v_s^{\mathrm{MCF}},
\end{aligned}
\]

where `(y0,z0)` is a feasible running core point. The selected dual produces

\[
\theta_s\ge
\alpha^{\mathrm{MCF}}_{s,k}
+(\beta^{\mathrm{MCF}}_{s,k})^\top y
+(\gamma^{\mathrm{MCF}}_{s,k})^\top z.
\]

These inequalities are submitted through `MOI.UserCut`, because the purpose is to strengthen
fractional node relaxations. Calling them lazy constraints would not reliably accomplish that.

Required checks for every new fractional cut are:

1. the MCF primal is optimal;
2. the explicit complete dual is feasible and optimal;
3. the cut is tight at `(y_hat,z_hat)` within tolerance;
4. all coefficients are finite;
5. the cut is violated before submission.

The initial implementation caps separation at eight distinct fractional points. This cap is a
runtime safeguard, not a mathematical requirement.

## Cut family 2: binary exact-routing restricted-MW lazy cuts

At every integer callback candidate:

1. verify binary `y` and assignment-consistent endpoint-chain `z`;
2. construct an immutable cache key for the complete first-stage state;
3. reuse a cached oracle result or solve every required routing scenario;
4. run column generation to certified pricing termination;
5. perform the corrected restricted-MW dual completion;
6. verify complete routing-dual feasibility, including a final pricing audit;
7. construct scenario-specific globally valid BendersYZ cuts;
8. submit violated cuts through `MOI.LazyConstraint`;
9. accept the candidate only when all exact scenario recourse values are satisfied.

The exact-routing cut has the form

\[
\theta_s\ge
\alpha^Q_{s,k}+(\beta^Q_{s,k})^\top y+(\gamma^Q_{s,k})^\top z.
\]

The routing oracle is currently evaluated only at binary first-stage states, but its completed
dual cut is globally valid throughout the master relaxation.

The word "restricted" refers to the MW dual selection/completion construction, not to the
validity domain of the final cut.

## Core-point policy

The fractional MCF MW core begins at an observed feasible fractional master solution and is
updated as

\[
(y^0,z^0)\leftarrow0.9(y^0,z^0)+0.1(\hat y,\hat z).
\]

This preserves relaxation feasibility because it is a convex combination of feasible points.
It does not guarantee a high-quality relative-interior core; improving core construction is an
open optimization task.

## Bounds and termination

Every exactly evaluated binary solution provides the feasible upper bound

\[
UB(y)=C_{\mathrm{walk}}(y,z)+\beta\sum_sQ_s(y,z).
\]

The global lower bound is Gurobi's best bound for the common-MCF-plus-cut master. The configured
termination target is a 1% relative MIP gap. The final objective is recomputed from cached exact
routing values rather than trusting the callback values of `theta`.

## What is cached and logged

The implementation caches exact routing results by binary first-stage state. Repeated candidates
reuse their routing values and cuts, but a violated lazy constraint may be resubmitted if Gurobi
presents the candidate again.

Logs include:

- Gurobi branch-and-bound progression;
- integer callbacks and unique first-stage states;
- exact Benders cuts and cache hits;
- certified UB, Gurobi LB, and relative gap;
- oracle, priming-CG, repricing, and MW-completion time;
- fractional MCF separation count, cuts, and time;
- common-OD count and selected MCF mode;
- master dimensions and shared route-pool size.

## Computational evidence

All figures below use the Zhuzhou three-scenario instance with 16 OD pairs per scenario, seed 999,
and `ms5`. Branch-and-Benders runs target a 1% gap; Direct and historical classical Benders runs
continued to exact convergence.

### n=15

All trustworthy methods found objective `57481.83283904121` and selected stations
`[11,22,92,100,133,138,158,202]`.

| Method | Runtime (s) | Nodes | Unique y | Exact cuts | Oracle (s) |
|---|---:|---:|---:|---:|---:|
| Full three-scenario MCF + restricted MW | 177.8 | 939 | 84 | 246 | 141.1 |
| Common-OD MCF | 280.1 | 1402 | 105 | 303 | 263.5 |
| Common-OD + earlier arbitrary projected y-cuts | 330.9 | 1087 | 107 | 307 | 313.1 |
| Standard-repriced Branch-and-Benders + full MCF | 732.9 | 1360 | 110 | 317 | 668.1 |
| Single-scenario MCF | 896.7 | 1174 | many | 614 lazy cuts | about 881 callback seconds |
| Classical Benders, standard repriced, no MCF | 537.1 | 113 outer iterations | - | - | - |
| Direct enumeration | 559.4 | - | - | - | - |
| Classical Benders, standard repriced, full MCF | 893.1 | 81 outer iterations | - | - | - |

The earlier projected-y experiment generated 23 cuts in only 1.1 seconds. It reduced nodes but
changed the candidate trajectory toward harder exact routing instances, increasing oracle time.
It must remain labeled separately: it was not the new full-MW `y,z` method.

Master dimensions at n=15 were:

| MCF design | Rows | Columns | Nonzeros | Root LP (s) |
|---|---:|---:|---:|---:|
| Full MCF | 13937 | 11088 | 49577 | 0.40 |
| Single scenario | 6328 | 4366 | 21311 | 0.12 |
| Common OD | 2719 | 1118 | 7768 | 0.02 |

### n=20

Full-MCF Branch-and-Benders reached a 0.9986% gap in 410.4 seconds with objective
`47994.06465466723`. Common-OD-only reached the same objective and a 0.9726% gap in about
1258 seconds.

| Metric | Full MCF | Common OD |
|---|---:|---:|
| Nodes | 2107 | 7397 |
| Unique y | 107 | 289 |
| Cuts | 295 | 796 |
| Oracle time (s) | 252.4 | 1220.9 |
| Rows | 24236 | 4959 |
| Columns | 19558 | 2146 |
| Nonzeros | 87460 | 14575 |
| Root LP (s) | 1.69 | 0.06 |

Common OD greatly reduces model size and memory but transfers work to exact routing callbacks.
The earlier projected-y n=20 run likewise showed that projected-cut construction itself was cheap;
exact oracle evaluations and incumbent discovery dominated.

## Current implementation status

- Permanent common-OD MCF: implemented and computationally validated.
- Binary exact-routing restricted-MW BendersYZ lazy cuts: implemented and validated.
- Earlier arbitrary projected `y` MCF user cuts: implemented and benchmarked, but superseded for
  new experiments.
- Complete-dual fractional MCF MW `y,z` cuts: implemented.
- Focused deterministic validation for the new code: 108/108 tests passed on compute job 19193389.
- First real n=15 full-MW `y,z` attempt, job 19193462_1, reached the fractional separator and
  exposed an unexercised API-name error while reading the auxiliary objective coefficients
  (`objective_coefficient` versus `coefficient(objective_function(...), var)`). The solve stopped
  before submitting an MCF-MW cut; this was an implementation error, not a failed validity check.
  The API call was corrected and replacement job 19194992_1 was submitted.
- No n=20 full-MW `y,z` experiment should be launched until n=15 confirms the objective and
  exercises the fractional separation path on the real instance.

## Reporting issue discovered during experiments

Some reduced-MCF solves completed successfully but were marked failed after optimization because
an optional scenario ID was serialized as `nothing`, which CSV cannot print. The writer now emits
`missing`. This was a post-solve reporting failure and did not affect optimization or certificates
recoverable from the Gurobi logs.

## Highest-priority improvements

### 1. Progressive exact scenario evaluation

At a binary candidate, evaluate the scenario most likely to violate first. If it produces a lazy
cut, reject the candidate without solving the remaining scenarios. Solve all scenarios only when
the candidate may be accepted. This directly targets the dominant oracle time.

### 2. Root-focused MCF cut convergence

Generate scenario MCF MW cuts in a controlled root cutting-plane loop until violations or marginal
bound improvement become small. Add these as ordinary constraints before branching instead of
using the first few fractional callback points indiscriminately.

### 3. Strong incumbent or MIP start

Preload a known feasible station set from full MCF, common MCF, classical Benders, or a heuristic.
Several slow runs spent substantial time before discovering the known best station set.

### 4. Cut selection and deduplication

Rank fractional cuts by normalized violation and core-point improvement. Reject nearly parallel or
numerically negligible cuts. Log bound improvement attributable to each accepted cut.

### 5. Better core construction

Compute a relative-interior `(y0,z0)` by maximizing distance from active bounds subject to the
master relaxation. The current moving average is feasible but may remain close to a face.

### 6. Clustered common-OD MCF

Partition many scenarios into similarity clusters and impose

\[
\sum_{s\in C}\theta_s\ge |C|L_{\mathrm{common},C}(y,z).
\]

This interpolates between one weak global common block and one expensive full MCF per scenario.

### 7. Reuse more routing state

Reuse scenario restricted-master bases, pricing labels, active route subsets, completed duals, and
station-set similarity information in addition to the existing shared route-column pool.

### 8. Dynamic scenario ordering

Order exact scenario evaluations using historical violation frequency, current `theta[s]`, MCF
violation, cached solve time, and similarity to previously evaluated states.

### 9. Persistent cut pools

Store globally valid MCF and exact-routing cuts between runs of the same instance and preload them
as ordinary root constraints.

## Main conclusion

Full scenario MCF is fastest at n=15 and n=20, but its LP grows approximately quadratically with
stations and linearly with scenarios. Common-OD MCF removes most of that master size but is too weak
alone and causes excessive exact routing work. The intended hybrid is therefore:

1. a small permanent common-OD MCF;
2. full-dual scenario MCF MW `y,z` user cuts at fractional points;
3. exact-routing restricted-MW BendersYZ lazy cuts at binary points.

This preserves exactness, restores scenario-specific fractional guidance selectively, and avoids
carrying every scenario's full MCF variables through the entire branch-and-bound tree.
