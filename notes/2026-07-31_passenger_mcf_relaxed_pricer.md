# A multi-commodity-flow relaxed pricer for PFA column generation

2026-07-31. Code: `src/opt/optimize/aggregate_od_route/pricing/passenger/mcf_relaxation.jl`.
Tests: `test/opt/test_passenger_mcf_relaxation.jl`.
Diagnostic: `scripts/diag_passenger_mcf_relaxation_gap.jl`.

**Verdict up front: the formulation is valid but far too loose and far too slow
to serve as a termination certificate. Not wired into the CG loop. Keep the code
as a measured negative result and as the reference implementation if someone
wants to revisit it with integrality or stronger cuts.**

## Why we wanted it

`pricing/passenger/column_generation.jl` only claims `:optimality_proven` when
the exhaustive certification pass comes back empty *and* exhausted. That pass is
what fails to terminate: `notes/2026-07-30_pfa_scaling_and_cg_throughput_experiments.md`
has 7/20 grid cells hitting the 3h wall, one certification alone burning 6900s,
and label counts growing 43k -> 1.38M as duals converge. On an uncertified cell
`lp_bound` is not a bound at all.

But the last question CG asks is not "what is the best column?" -- it is "does
any column have negative reduced cost?". That needs only a valid **lower bound**
on the pricing optimum. Hence: solve one LP per scenario, and if

    L_s >= -reduced_cost_tol

skip the label search entirely.

## The formulation

Time grid `delta`, arc time advance `dbar(i,j) = delta*floor(d(i,j)/delta)`,
horizon `H = max_wait + max ride_limit`. States `(i, t)` restricted to those
forward-reachable from a start node; travel arcs at the **true** cost
`beta*d(i,j)`; a source arc into every `(i, 0)` at `beta*repositioning_time` and
a sink arc out of every state.

  - `f_a >= 0`: one unit of vehicle flow, source to sink.
  - `h^{j,b}_a in [0, f_a]`: a **suffix flow** per (origin station `j`, boarding
    bucket `b`), injected where the vehicle is at `j` during `b`.
  - `u_pjk in [0,1]` with `sum_{(j,k)} u_pjk <= 1` -- the per-passenger *maximum*,
    which here needs no reward layers at all.
  - `u_pjk <= sum_b sum_{t2 <= sup(b) + R_pjk} inflow of h^{j,b} at (k, t2)`.

Objective `sum_a c_a f_a - sum rho_pjk u_pjk`.

### Two things worth keeping from the derivation

**Round travel times DOWN, and no window inflation is needed.** Grid times are
`t_m = sum_{i<m} dbar <= sum_{i<m} d = T_m`, so the pickup window `t_1 <= T_1 <= W`
holds directly, and the ride limit `t_k - t_j = sum_{j<=i<k} dbar <= T_k - T_j`
holds because both endpoints are cumulative sums over the *same* arcs. Flooring
does not accumulate drift here. Rounding up would have forced a `L*delta` slack
on every deadline.

This requires `delta <= min_{i!=j} d(i,j)`: a longer step floors the shortest arc
to zero, the network stops being a DAG, and the suffix flows circulate within a
layer and manufacture reward. `config.time_step` is therefore an *upper bound*
that the builder clamps down; the clamp is measured over the **endpoint nodes
only**, since stations that certify nothing are dropped from the network.

**A suffix flow, not an origin-to-destination commodity.** One unit of flow
routed from `j` to a sink could certify only one dropoff, which would
*under*-count reward -- an over-estimated `L_s` that could wrongly certify. The
suffix (conservation everywhere except the boarding nodes, exit via sink arcs)
is simultaneously 1 at every downstream node of an integral path.

## Measurements

n=10, p=16, 3 scenarios, l=5, max_stops=5, max_visits=3, Zhuzhou seed 42.
Full CG run with exhaustive exact pricing, MCF bound computed alongside at every
iteration and scenario (48 rows, converged in 16 iterations).

### Validity: clean

| check | result |
| --- | --- |
| `false_certificates` (certified while an improving column existed) | **0 / 48** |
| `bound_violations` (`L_s > exact_rc`) | **0 / 48** |
| randomised bound-admissibility unit tests | **36 / 36 pass** |

### Tightness: nowhere near

At iteration 16 -- convergence, where the exact pricer *proves* no improving
column exists, so the true optimum is `>= 0`:

| scenario | MCF bound | true min rc |
| --- | --- | --- |
| 1 | -1717.46 | none (>= 0) |
| 2 | -1550.06 | none (>= 0) |
| 3 | -1856.87 | none (>= 0) |

Certification rate on certifiable rows: **0 / 5 (0%)**. The pre-registered bar
was >= 50%.

At iteration 1 the relative gap is 107-150% (`L_s = -3019` against an exact
`-1359`).

### Cost: the wrong direction by two orders of magnitude

| n | mean MCF sec | mean label-search sec | ratio |
| --- | --- | --- | --- |
| 10 | 13.97 | 0.07 | **200x slower** |
| 15 | hits a 60s LP limit every scenario (`reason=lp_not_optimal`, no bound at all) | 0.26-0.92 | -- |

Network sizes are modest (n=10: ~750 states, ~6.2k arcs, 8-10 commodities;
n=15: ~1.2k states, ~14k arcs) -- it is the `h <= f` coupling across commodities
that makes the LP itself slow, not the network build.

### Both knobs go the wrong way

| setting | bound (n=10, s=1, iter 1) | mcf sec |
| --- | --- | --- |
| `delta = min_travel`, 1 bucket | -3019.02 | 11.9 |
| `delta = min_travel`, 4 buckets | **-3180.10** (looser) | 69.2 |
| `delta = min_travel/2`, 1 bucket | -3001.90 (0.6% tighter) | 33.7 |
| `delta = min_travel/2`, 4 buckets | LP time limit, no bound | 122.2 |

More boarding buckets tighten each commodity's ride deadline but hand the LP one
*independent* injection per bucket, and the second effect wins. Tying total
injections at a station to the number of vehicle visits there
(`sum_b injections^{j,b} <= sum_t x_{j,t}`) removes some of that slack and is in
the shipped code, but not enough to reverse the sign. **The bound is not
monotone in the bucket count** -- the unit test asserts only validity for both,
not an ordering.

## Why it is loose, and what would actually be needed

Fractional `f`. The coupling `u_pjk <= sum_{t2} inflow of h at (k, t2)` sums
arrivals *across time layers*, so a single unit of vehicle flow split over
several paths collects several routes' worth of reward while paying one route's
travel cost. At convergence that is worth ~1500-1850 units of phantom reward
against a true optimum of 0.

This is the classic arc-flow weakness and neither knob touches it. Options that
would:

  1. **Integral `f`** -- makes it an exact arc-flow pricer, not a cheap
     certificate. Would have to beat a label search that runs in <1s at n=10.
  2. **ng-route / state-space relaxation of the label search itself** -- the
     standard "relaxed pricer" in VRP column generation. Keeps the DP and its
     speed, and relaxes only the elementarity/state dimension, so it stays in the
     regime where PFA pricing is already fast.
  3. Stronger valid inequalities on `u` (e.g. tying total certified passengers to
     the stop budget). Unexplored; would have to close a >100% gap.

Given that the tail-detection stopping rule (open question 1 in the scaling note,
"probably the biggest remaining win") is untouched and much cheaper, option 2 or
the stopping rule are better next steps than pursuing this LP.

## Status of the code

Valid, tested (86 assertions), **off by default**, and **not referenced by the CG
loop** -- `run_passenger_free_assignment_column_generation` is unchanged, so no
existing run behaves differently. `passenger_free_assignment_mcf_lower_bound`
returns `(-Inf, false, ...)` on every failure path (degenerate network, size
guard, non-optimal LP), so a missing bound can never be mistaken for a
certificate.
