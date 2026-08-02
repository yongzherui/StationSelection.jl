# Excluding stations from PFA pricing by reduced-cost fixing (design)

Proposal: use master-side reduced-cost / Lagrangian variable fixing on the
build variables `y` to **permanently delete stations from every future pricing
graph** during a passenger free-assignment (PFA) CG run, shrinking the label
search where it is most expensive (the converged certification tail).

Status: **OPEN, not implemented.** This note pins the theorem, the exact fixing
threshold, and the correctness guardrails so the idea can be picked up cleanly.
It also records the one seductive-but-unsound version so we don't rebuild on
sand.

Origin: the intuition that "the good negative-reduced-cost routes in one basis,
and the stations they involve, must be related to the next basis, so we should
be able to exclude stations from the next pricing step." That intuition is
correct, but only in the bound-anchored form below -- not in the
basis-to-basis form.

Background:
- `2026-08-01_pfa_label_setting_algorithm_reference.md` -- how the pricer works.
- `2026-07-31_pfa_state_space_relaxation_design.md` -- where the state lives.
- `lagrangian_relaxation.jl` -- the valid per-iteration bound this note leans on.
- `station_subset.jl` -- the *other* station-pruning attempt (contrast below).

## What is fixed vs. what moves between pricing rounds

For the master formulation see `master.jl`. The physical graph (`nodes`,
`travel_cost`, `ride_limit`) and every column's cost `beta*(tau + repo)` are
**fixed for the whole run**. The only thing that changes iteration to iteration
is the per-assignment reward the pricer carries,

    rho_pjk = alpha_p - gamma^O_pj - gamma^D_pk - w_pjk,

driven entirely by the RMP duals (`passenger_free_assignment_pricing_candidates`
rebuilds it every iteration).

A station `j` is *usable* in an improving route **iff it is the origin or
destination of some opportunity with `rho_pjk > 0`**. Reason: `travel_cost` is a
shortest-path / metric matrix (the triangle inequality is already relied on by
the age-pruning in `data.jl` and `_passenger_free_assignment_age_is_useful`), so
an optimal route never visits a station purely as a waypoint -- every visited
station either opens a pickup clock or certifies a delivery. Hence the
per-iteration station set feeding the pricer is exactly
`{ j : exists (p,k) with rho_pjk > 0 } union { k : exists (p,j) with rho_pjk > 0 }`,
which the `rho > 1e-9` filter in `passenger_free_assignment_pricing_candidates`
already computes.

So the question is precisely: **can we predict which stations will have
`rho > 0` next time and never build them into the pricing graph?**

## The unsound version (the trap)

"Station `j` carried no negative-reduced-cost route under basis `t`, so exclude
`j` from pricing at `t+1`" is **false here**, because of dual oscillation.

Between two consecutive RMP optima the duals sit on different faces -- this
master is highly degenerate, with disaggregated `(p,j)` linking rows -- so
`alpha_p` and `gamma` can swing enough to flip `rho_pjk` from `<= 0` to `> 0`. A
station absent this round can legitimately be the endpoint of an improving column
next round. That is *why* the candidate filter recomputes `rho` from scratch
every iteration rather than caching a station set.

Any pruning keyed on "used last time" can therefore cut a real improving column
and, worse, can make a search that stopped early *look* exhausted -- breaking the
`:optimality_proven` certificate contract documented at the top of
`column_generation.jl`. Do not build on this version.

## The sound version: reduced-cost fixing on `y`

The rigorous statement anchors the excluded set to a **global bound**, not to a
basis. It uses the master's own structure.

`y` couples in only through the linking rows and the budget row `sum_j y_j = l`.
Define station `j`'s **dual worth**

    Gamma_j = sum_p ( gamma^O_pj + gamma^D_pj )   >= 0

(the total dual price the linking rows pay for `j` being open, over all
passengers, as pickup and as dropoff), and let `mu` be the dual of the budget
row `sum_j y_j = l`.

`y_j` has objective coefficient 0, a `-1` in every linking row it touches, and a
`+1` in the budget row, so its LP reduced cost is

    rc(y_j) = -Gamma_j - mu.

At the LP optimum this is the expected p-median behaviour: `-mu` is a threshold,
stations with `Gamma_j > -mu` open (`y_j = 1`), stations with `Gamma_j < -mu`
close, ties go fractional. The LP effectively selects the **top-`l` stations by
`Gamma_j`**. That threshold *is* the "station basis" the original intuition was
reaching for -- but expressed as a scalar order statistic on `Gamma`, not as a
set carried between iterations.

### Fixing theorem

Let `z_LP` be a **valid** lower bound and `z_UB` an incumbent. Standard
reduced-cost fixing on a variable at its lower bound gives:

    Gamma_j  <  -mu - (z_UB - z_LP)   ==>   y_j = 0 in every improving
                                            integer solution.

Once `y_j` is fixed to 0, the linking constraint `theta_r <= y_j = 0` forbids
*any* route through `j`, so **`j` is provably deleted from every subsequent
pricing graph**, including the exhaustive certification pass. Equivalently: a
station whose dual worth falls short of the open/close threshold by more than the
current optimality gap can never be in an optimal build set.

Because the budget is a hard cardinality (`= l`), the same order-statistic
argument fixes stations *in* as well as out (a station far above the `l`-th
largest `Gamma` by more than the gap must be open). Fixing `y_j = 1` does not
shrink the pricing graph, but it tightens the master and can trigger further
`y_j = 0` fixings.

### Which bound to use

- `z_UB`: solve the pool MIP occasionally (it is already solved once at the end;
  do it periodically in the converged tail). Any feasible integer pool solution
  is a valid `z_UB`.
- `z_LP`: **must be valid.** Use either the certified LP bound (only trustworthy
  when `cg_stop_reason == :optimality_proven`) or, better for mid-run use, the
  Lagrangian bound from `lagrangian_relaxation.jl`, which is valid *every*
  iteration. **Never** use the early-return-phase LP objective as `z_LP` -- that
  pool is knowingly incomplete (see the two-phase docstring in
  `column_generation.jl`).

## Why this is a different lever than `station_subset.jl`

The station-subset B&B pricer
(`project_pfa_station_subset_bnb_bound_quality`) loses because *its subproblem's*
reward+routing LP upper bound is ~1.9-2.2x the optimum -- it is bound-quality
limited at the leaf oracle. This proposal never touches the subproblem bound. It
prunes the station set from the **master side** using the master's Lagrangian
bound, then hands the *exact* pricer a smaller graph.

Since PFA pricing cost is per-iteration *width* (scenarios x label search, ~90%
dominance-scan; see `project_pfa_label_search_dominance_bottleneck` and
`project_pfa_cg_scaling_frontier_zhuzhou`), deleting stations shrinks the live
label population directly -- attacking the actual bottleneck without depending on
a weak leaf bound.

## When it bites, and when it does nothing

It fixes nothing until `z_UB - z_LP` is small, so it is inert early in CG when the
gap is wide. That is fine: the payoff window is the **converged tail**. The bound
here is a step function that flatlines for 40-64% of the budget
(`project_pfa_cg_throughput_open_questions`), and the exhaustive certification
pass dominates late-run wall time at n=25-30
(`project_pfa_cg_scaling_frontier_zhuzhou`). In that tail `z_LP` is stable and a
pool-MIP `z_UB` is cheap, so fixing can prune the graph precisely before the most
expensive pricing work.

Open empirical question: how many stations actually clear the threshold at, say,
n=30 with a realistic tail gap? If `Gamma_j` is flat across stations (many
near-tie candidates), few get fixed and the win is small. A cheap first
experiment is to *measure* the `Gamma_j` spread and the implied fixing count at
the point CG reaches the certification pass, before wiring any pruning in.

## Correctness guardrails (bake these in)

1. Use a **valid** `z_LP` (certified LP or Lagrangian), never the early-return
   LP objective.
2. Fixing is **monotone**: once `y_j` is fixed to 0 it stays fixed; never un-fix.
   Then the final certificate legitimately means "no improving column exists over
   the reduced station set," which is sound because every deleted route was
   proven non-optimal by the bound.
3. Recompute the excluded set only from a fresh valid `(z_LP, z_UB)` pair; do not
   carry a station's exclusion forward on stale duals.
4. Sanity assertion: no station carrying an assignment in any *pooled* column
   that participates in the incumbent may be fixed out. (A correct bound cannot
   produce this, so a violation flags a sign/threshold bug -- the analogue of
   `_verify_passenger_master_reduced_cost`.)

## Suggested next step

Phase A (measure): at the iteration CG first enters the certification pass,
solve the pool MIP for `z_UB`, take the Lagrangian `z_LP`, compute
`Gamma_j = sum_p (gamma^O_pj + gamma^D_pj)` and `mu`, and log the histogram of
`Gamma_j` and the count of stations satisfying the fixing inequality. No pricing
change yet. This answers "is there anything to fix?" for a few cents of compute.

Phase B (exploit): if Phase A shows a non-trivial fixed set, delete those
stations from the pricing graph (drop their opportunities before
`create_passenger_free_assignment_pricing_data`) and re-run the certification
pass, checking the certified optimum is unchanged and measuring the label-count /
wall-time reduction. Compare against the n=25-30 tail where certification
dominates.
