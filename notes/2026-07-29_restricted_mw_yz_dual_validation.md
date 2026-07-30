# Restricted-MW YZ dual validation

Date: 2026-07-29

## Scope

This note records the investigation of `cut_derivation=:restricted_mw_fixed_pi` for
`BendersYZ`, including its attempted use in `BranchAndBendersSolver`.  The MCF residual/floor
transformation is not implicated: the full-recourse affine cut is already invalid before any
MCF expression is subtracted.

## Reproducing failure

The reproducer is the test fixture `lifted_lb_fixture()` / `lifted_lb_model()` in
`test/opt/test_aggregate_od_route_lifted_routing_lower_bound.jl`.  It is entirely synthetic and
deterministic:

- Five candidate stations with array indices and IDs `1:5`; coordinates are `(lon=i, lat=0)`.
- One scenario with two requests: `(origin=1,destination=5,time=08:00)` and
  `(origin=2,destination=4,time=08:01)` on 2024-01-01.
- Every unspecified walking cost is 100.  The finite entries used by the instance are
  `w(1,1)=0`, `w(1,2)=3`, `w(4,5)=3`, `w(5,5)=0`, `w(2,2)=0`, and `w(4,4)=0`.
- Routing cost is `c(i,j)=abs(i-j)+1` for every ordered station pair.
- Four of five stations must open (`l=4`).  Request 2 pins stations 2 and 4; station 3 is a
  decoy, while request 1 has two real pickup and dropoff choices.
- Model settings are `NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`,
  `max_walking_distance=5`, route weight 3, walking weight 0.37, repositioning time 1.5,
  `max_stops=3`, maximum wait 1000, and detour factor 2.
- Solver settings for the focused branch comparison are Gurobi with `mip_gap=0`, one callback
  thread, CG limited to 200 rounds with 20 columns/candidates per round, and a 30-second final-IP
  limit.  The full test helper appears under the `BranchAndBendersSolver certified Y/YZ callback
  modes` testset in that file.

On this fixture, standard, fully repriced BendersYZ returns 20.61.
The restricted-MW branch-and-Benders solve returns 31.11.  A generated cut evaluated at an
independently certified candidate gives

```text
cut RHS       = 10.0
exact recourse = 6.5
```

Both points have the same binary station set `(1,2,3,4)`, identical binary `z`, and maximum
`z` distance/fractionality equal to zero.  The callback cache and chain-key mapping are therefore
not the source of this counterexample.

### Scheduler-safe reproduction

Do not run Julia on the login node.  From the repository root, submit the focused test with
unbuffered output to a shared project path (the historical jobs below used the same command):

```bash
mkdir -p experiments/branch_and_benders_yz_mw_n10_clean_20260728/slurm_logs
sbatch \
  --job-name=bb_yz_mw_repro \
  --cpus-per-task=1 --mem=8G --time=00:15:00 \
  --output="$PWD/experiments/branch_and_benders_yz_mw_n10_clean_20260728/slurm_logs/bb_yz_mw_repro_%j.out" \
  --error="$PWD/experiments/branch_and_benders_yz_mw_n10_clean_20260728/slurm_logs/bb_yz_mw_repro_%j.err" \
  --wrap="cd '$PWD' && exec stdbuf -oL -eL julia --project=. --startup-file=no -e \
    'using Test, StationSelection, Gurobi, JuMP, DataFrames, Dates; \
     const MOI = JuMP.MOI; \
     include(\"test/opt/test_aggregate_od_route_lifted_routing_lower_bound.jl\")'"
```

Before the uncapped-route repair is applied, expected diagnostics include:

```text
restricted-MW objective                       31.11
standard objective                            20.61
generating/candidate station indices          (1,2,3,4)
MW cut RHS / exact routing recourse            10.0 / 6.5
max z distance / fractionality                 0.0 / 0.0
existing route column 3 dual residual          +3.5
equivalent reduced cost of existing column 3  -3.5
```

The test exits nonzero intentionally while the invalidity assertions reproduce the defect.

## Checks completed

1. The completion LP solves to `OPTIMAL`, with feasible primal and dual status.  Its objective
   and bound both equal 4.5; the largest completed dual magnitude is only 5.5.
2. The completion is exactly tight at its imposed target: `Phi(z_hat)=Q_bar=10.0`.
3. The hand-reconstructed affine constant equals the returned constant exactly.
4. All inspected `x`-variable dual inequalities have residual at most zero (worst 0.0).
5. Label-setting pricing exhausts its search and returns no *new* column.
6. Nevertheless, the reconstructed routing dual violates the dual inequality for existing route
   column 3 by 3.5.  Equivalently, that existing column has reduced cost -3.5.

The previous `no new columns` check was a false certificate.  The pricing routine intentionally
filters signatures already present in `existing_columns`; its implementation documents the
assumption that an optimal-RMP dual cannot make an existing column negative.  That assumption no
longer holds after converting the RMP coverage duals into `pi_by_request`.

## Current diagnosis

`_extract_route_covering_pi_by_request` reads the converged route-covering RMP coverage duals,
groups requests sharing the same active `(j,k,s)`, sums their dual credit, and puts all credit on
one representative request.  `_zero_extended_pi` then embeds that request-level vector into the
full YZ completion.  The resulting vector is not currently checked against every column already
in the RMP.  In the counterexample it is not route-dual feasible.

More specifically, every relaxed route-selection variable is currently declared on `[0,1]`.
An existing route at its upper bound can have a negative ordinary reduced cost whose effect is
offset by the variable's upper-bound dual.  The request-level `pi` extraction discards that bound
dual, while the hand-written completion assumes the standard uncapped set-covering dual
`sum(pi*a) <= route_cost`.  This explains why the RMP dual is solver-feasible but the extracted
`pi` alone violates existing column 3.  The pricer suppresses the same existing signature and
therefore cannot reveal this violation by returning a new route.

This invalid `pi` is then treated as certified and the completion LP only enforces the `x` dual
constraints.  Since route-column inequalities are absent from the completion LP, forcing
`Phi(z_hat)=Q_bar` can succeed even when `Q_bar` is not a valid full-dual value.  Both zero and
maximize-core completions are unsafe in this state; MW merely selects a different completion of
the same invalid routing block.

## Required certificate before accepting a completion cut

For every scenario and every existing column `r`, explicitly check

```math
\sum_{p,(j,k)\in r} \pi_{p,jk} \le c_r + \varepsilon_{dual}.
```

Then run exact pricing with the same aggregated `(j,k,s)` rewards to prove that no absent route
violates the inequality.  Both checks are required: existing-pool validation and missing-column
pricing.  Tightness at the generating point remains a separate required assertion.

## Candidate repairs to evaluate

1. Remove the primal-redundant upper bound on relaxed route variables, producing the standard
   nonnegative set-covering LP whose coverage duals must satisfy every route inequality directly.
   This is the repair currently under validation.  It is primal-equivalent because coverage RHSs
   are at most one, route coefficients are binary/nonnegative, and route costs are nonnegative:
   replacing any route value above one by one preserves feasibility and cannot increase cost.
2. Preserve the raw request-row duals without winner-take-all redistribution and validate them
   against the complete pool.
3. If request-level recovery is intrinsically ambiguous, solve a small recovery LP whose
   variables are request-level `pi`, constrained to preserve the certified aggregate pair duals,
   satisfy every existing route inequality, and reproduce the route-covering LP objective.
4. Include all existing route inequalities directly in the completion LP and retain exact
   pricing as separation for absent columns.  If pricing finds a violation, add the route row and
   re-solve completion until exhausted.

Option 1 is the simplest formulation-alignment repair; option 4 is the most defensive
correctness-first implementation.  No restricted-MW cut should enter a master until both the
existing-column and absent-column certificates pass.

## Uncapped-route repair result

The option-1 experiment removed the redundant upper bound from relaxed route variables in the
main route-covering model, dynamically added CG columns, and the Y/YZ/XY/YZH Benders routing LPs.
Slurm job `19176427` passed all 100 focused checks in 41.2 seconds.  On the previously failing
`y=(1,2,3,4)` state:

```text
Q_bar / exact recourse                    6.5 / 6.5
worst existing-route dual residual        0.0
worst x-dual residual                     0.0
completion tightness error                0.0
new negative-reduced-cost routes          0 (pricing exhausted)
restricted-MW vs standard objective       equal (20.61)
cross-candidate cut validity              passed
```

The old invalid target `Q_bar=10.0` disappeared.  This validates the upper-bound-dual diagnosis
on the deterministic reproducer.  Larger-instance regression and performance tests are still
required, and production code should retain an explicit existing-column residual assertion so a
future formulation change cannot silently recreate this failure.

## Why removing `lambda <= 1` is mathematically correct

For a fixed assignment vector `x`, the route-covering part of the recourse LP is

```math
\min_{\lambda}\ \sum_r c_r\lambda_r
\quad\text{s.t.}\quad
\sum_r a_{rq}\lambda_r\ge x_q\quad\forall q,
\qquad 0\le\lambda_r\le1.
```

Here `q` is a request/pair coverage row, `a[r,q]` is zero or one, `0<=x[q]<=1`, and every route
cost `c[r]` is nonnegative.  Suppose a feasible solution has `lambda[r]>1`.  Replace that one
value by `min(lambda[r],1)=1`.  For any row covered by route `r`, its contribution remains one,
which alone is at least the row's RHS `x[q]`; rows not covered by `r` do not change.  Feasibility
is therefore preserved.  Since `c[r]>=0`, the objective cannot increase.  Repeating this for all
routes proves that the uncapped LP

```math
\lambda_r\ge0
```

has the same optimum as the explicitly capped LP and always has an optimum inside `[0,1]`.

The difference is in the dual representation.  With only nonnegativity, every route produces the
dual constraint

```math
\sum_q a_{rq}\pi_q\le c_r.
```

These are precisely the constraints assumed by column-generation pricing and by the restricted
MW completion.  With an explicit upper bound, an additional bound dual `gamma[r]>=0` appears:

```math
\sum_q a_{rq}\pi_q-\gamma_r\le c_r.
```

Thus `pi` by itself may violate a route inequality while `(pi,gamma)` remains feasible.  That is
exactly what happened for route column 3: `gamma[3]` implicitly compensated 3.5, but the
completion copied only `pi`, discarded `gamma`, and incorrectly treated `Q_bar=10` as a full-dual
value.  Removing the redundant primal bound removes `gamma` from the dual and forces the RMP to
return a `pi` that is independently route-feasible.  The corrected `Q_bar=6.5` then matches exact
recourse.

This change applies only to LP relaxations used for CG, dual recovery, and Benders cuts.  The
final routing IP continues to use binary route-selection variables.  The proof also depends on
the present model properties `x<=1`, nonnegative coverage coefficients, and nonnegative route
costs; an explicit assertion/certificate should remain in place if any of those properties change.

## Diagnostic jobs

- `19172370`: cross-candidate global cut validity (`10.0 > 6.5`).
- `19174701`: zero-completion pricing audit; no new columns, later shown insufficient.
- `19175120`: completion algebra and initial dual residual audit.
- `19175369`: identical binary `y` and `z` mapping confirmed.
- `19175819`: maximize-core status audit and existing-column-3 violation of 3.5.
- `19176427`: uncapped-route repair; all 100 focused checks passed.
