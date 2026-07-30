# `lifted_routing_lower_bound` (objective+residual version): stale-shift unsoundness under non-`:standard` cuts

*2026-07-28*

## Status: residual-cut transformation corrected; real-instance re-validation mostly confirms it (2026-07-29)

The stale-snapshot implementation described below was replaced on 2026-07-28. The routing
subproblem and completion LP now remain in full-recourse units. For every full-routing cut
`alpha_k + b_k' z`, the master installs the equivalent residual row

```text
eta >= alpha_k + b_k' z - C_MCF
```

where `C_MCF` is the live master expression, not `value(C_MCF)` at the cut-generating incumbent.
The objective is `C_walk(y) + beta*C_MCF + beta*eta`, with `eta >= 0`. Neither `v_hat` nor the
completion LP's certified `Q_bar` is shifted before cut construction.

The focused `test_aggregate_od_route_lifted_routing_lower_bound.jl` suite passes all 37 checks,
including fixed-`y` verification that `C_MCF` is below the full repriced routing LP and integrated
optimum equivalence for all three cut derivations, both with and without lifted walking.

**Re-ran the exact 3 real Zhuzhou cases that broke under the old stale-snapshot version**
(`:restricted_mw_fixed_pi`, `lifted_walking_objective=true`, n=15) -- see "Re-validation" section
below. 2 of 3 now match the known-correct baseline exactly; the third improved from a genuinely
wrong station set (+1.2% objective) down to a 0.05%-off near-tie (still a different station set by
one station). `lifted_routing_lower_bound` remains opt-in/default-false regardless.

## Background

`BendersSolver(lifted_routing_lower_bound=true)` (`benders/lifted_routing_lower_bound.jl`) builds
a multicommodity arc-flow relaxation of the routing subproblem, `route_lb_expr[s]`, and adds it
directly as a master **objective** term rather than a floor constraint on `theta[s]`. `theta[s]`
is then meant to represent only the *residual*: each outer iteration reads
`route_lb_hat[s] = value(route_lb_expr[s])` off the master's current solution and subtracts it
from `v_hat` (and, for non-`:standard` cut derivations, from `qbar_for_cut`, before the two are
combined via `min`) before deriving that scenario's cut (`benders/yz.jl`'s outer loop).

An earlier version added `theta[s] >= route_lb_expr[s]` as a **static floor constraint** instead
(alongside the ordinary iteratively-added cuts, not replacing `theta`'s meaning). That version was
sound but didn't pay for itself: master solve time got up to 7.5x worse at n=15 because the
arc-flow variables sit in the master and get re-solved jointly with `y` every iteration, and the
iteration-count savings didn't offset that. The objective+residual version was built specifically
to test whether reformulating (not just relocating) the term would help -- it does substantially
reduce iteration count (24-38% on `:standard`+reprice, n=15 zhuzhou instances) and tighten the
initial gap (~99.8% -> 25-36%), a real, working signal that the bound itself is informative. But
extending it to `:restricted_mw_fixed_pi` (needed for the user's actual at-scale test target)
surfaced a genuine soundness bug.

## The bug: frozen per-cut shift vs. freshly-evaluated objective term

Every cut derived at outer iteration `k` bakes in **that iteration's** `route_lb_hat_k` as a
frozen constant (via the pre-cut shift of `v_hat`/`qbar_for_cut`). But the master's objective
re-evaluates `route_lb_expr(z)` **fresh** at whatever `z` the current solve holds. These two are
only mutually consistent at `z = z_hat_k` (the same iteration's own solution) -- once the master
moves to a different `z` in a later iteration, `route_lb_expr(z)` can differ arbitrarily from any
historical `route_lb_hat_k` baked into an already-added cut, and nothing forces those to agree.

Concretely: weak duality guarantees `Phi_k(z) <= v(z)` (a bound on the **true, unshifted**
subproblem value) for *every* `z`, regardless of what target the completion LP's
`phi_zhat_expr == target` equality aimed for -- the target only controls tightness at `z_hat_k`
specifically (`Phi_k(z_hat_k) = Q_bar_true_k - route_lb_hat_k` by construction). That does **not**
make `Phi_k(z)` a valid bound on the *residual* `v(z) - route_lb_expr(z)` at any other `z` -- only
at `z_hat_k`, where `route_lb_expr(z_hat_k)` happens to equal the `route_lb_hat_k` that was
subtracted. So `theta + route_lb_expr(z) >= v(z)` is only actually certified at each cut's own
`z_hat_k`, not globally. Once enough cuts with different frozen shifts accumulate, the master can
be misled into treating a genuinely worse `y` as if it were cheaper, and the outer loop's
`theta_hat < v_hat - tol` stopping check -- itself evaluated correctly, using the *current*
iteration's shift -- ends up satisfied at the wrong point.

The original **floor-constraint** version does not have this problem: `theta >= route_lb_expr(z)`
is a live constraint, re-evaluated at whatever `z` the master currently holds, every time -- there
is no frozen historical constant anywhere. It was sound, just slow (master-size cost, not a
correctness issue).

## Empirical reproduction (real Zhuzhou p16, n=15, `:restricted_mw_fixed_pi`, `lifted_walking_objective=true`)

3 of 6 seed/max_stops-mode combinations converged to a **worse objective and a genuinely different
station set** than the (known-correct, cross-validated against `:standard`-cut and `DirectSolver`)
baseline, while still reporting `termination_status=OPTIMAL`:

| case | baseline obj | +LB obj | baseline iters | +LB iters | baseline stations | +LB stations |
|---|---|---|---|---|---|---|
| seed=42, ms5 | 58770.67 | 59463.33 (+1.2%) | 137 | 10 | `[11,22,92,100,121,133,158,202]` | `[11,22,92,100,**117**,133,158,202]` |
| seed=123, ms4 | 59056.53 | 61772.72 (+4.6%) | 94 | 15 | `[11,40,100,117,121,133,158,202]` | `[11,40,**48,54**,100,117,158,202]` |
| seed=123, ms5 | 54866.29 | 56594.83 (+3.2%) | 100 | 10 | `[11,40,100,117,121,133,158,202]` | `[11,40,**48**,100,117,133,158,202]` |

The other 3 (seed=42/ms4, seed=999/ms4, seed=999/ms5) matched exactly. n=10 (all 6 combinations)
also matched exactly -- the bug appears to need enough outer iterations / enough distinct `z_hat`
values with meaningfully different `route_lb_expr` values for the staleness to actually bite.

Debug trace (`CS_DEBUG_LIFTED_LB=1` env var, temporary instrumentation still in
`benders/yz.jl`'s outer loop as of this note) on the seed=123/ms4 case shows `route_lb_hat`
varying every single iteration (1343, 1609, 1736, 1510, ..., 1342 across 15 iterations) --
directly confirming each of the 15 accumulated cuts carries a different frozen shift.

**Also noted in passing, likely a separate/pre-existing issue**: baseline (no `lifted_lb`,
unmodified `:restricted_mw_fixed_pi`) itself shows `lower_bound > incumbent` at its final
iteration on 2 of 6 cases (seed=42/ms5: 61262.59 > 58770.67; seed=999/ms5: 58592.07 > 57481.83) --
one of which (seed=999/ms5) still reached the correct final answer. This is the same *class* of
"bound exceeds incumbent" symptom found earlier this session on the tiny 5-station fixture under
`:zero_completion`, and confirms it's not unique to that fixture. Not investigated further here --
flagged for whoever next looks at MW-cut bound tightness on real data, since it's orthogonal to
(and predates) this note's bug.

## The fix

Replaced the pre-cut numeric shift (`v_hat -= route_lb_hat[cut_id]`, `qbar_for_cut -=
route_lb_hat[cut_id]`, both `value(...)` snapshots) with a purely symbolic one:

- `v_hat`/`qbar_for_cut` are no longer mutated at all -- they stay in full, true units exactly as
  they did before this feature existed.
- `_add_aggregate_od_route_benders_yz_optimality_cut!` (`yz_mw_cut.jl`) now takes a
  `route_lb_expr::Union{Nothing,AffExpr}` kwarg -- the *same live expression* used in the
  objective, not a number -- and subtracts it directly inside the cut's own algebra:
  `theta[cut_id] >= full_cut_expr(z) - route_lb_expr` for both the `:standard` branch and the
  restricted-completion branch (`yz_mw_cut.jl:466,470,505`). Since `route_lb_expr` is symbolic, the
  stored JuMP constraint re-evaluates it correctly against whatever `y`/`z` the master holds on
  *any* later solve -- there is no snapshot to go stale.
- The gating decision (is a new cut even needed) was updated to match: it reconstructs the full
  current bound as `theta_hat[cut_id] + route_lb_hat[cut_id]` (both read fresh this iteration) and
  compares that against the unshifted `v_hat` (`benders/yz.jl:551-552`).

This makes the construction mathematically equivalent to ordinary Benders on the combined
quantity `theta + route_lb_expr(z)` in place of a bare `theta` -- each cut is still only a
possibly-loose lower bound on the *true* `v(z)` (standard weak duality, untouched), but since
`route_lb_expr(z)` is now always evaluated live rather than frozen, accumulating cuts converges the
same way ordinary Benders does, just with a valid nonnegative head start baked in.

## Re-validation on real data (2026-07-29)

Re-ran the same 3 previously-broken cases (n=15, `:restricted_mw_fixed_pi`,
`lifted_walking_objective=true`) against the corrected code:

| case | before fix | after fix |
|---|---|---|
| seed=42, ms5 | wrong obj (+1.2%), stations `[...,121,...]` vs `[...,117,...]` | **+0.05%** (58770.67 vs 58801.99), stations differ by 1 (`121` vs `138`) |
| seed=123, ms4 | wrong obj (+4.6%), 3 stations differed | **exact match** |
| seed=123, ms5 | wrong obj (+3.2%), stations differed | **exact match** |

2 of 3 now match exactly. The third's gap shrank by >20x (1.2% -> 0.05%) and is no longer a
clearly-wrong answer -- it's in the same range as the pre-existing `:restricted_mw_fixed_pi`
baseline tolerance slack noted below (the `lower_bound > incumbent` cases), rather than the
severe staleness symptom. Not yet confirmed whether this residual 0.05% gap is that same
pre-existing tolerance issue or a smaller remaining edge case of this fix -- see "Not yet done".

## Likely fix direction (not designed in detail, per the user's request to note this first)

**Superseded by "The fix" above** -- kept below for the historical record of the reasoning that
led there.

The user's own hypothesis, worth taking seriously: **the dual completion problem itself
(`_yz_completion_lp`/`_solve_yz_completion` in `benders/yz_mw_cut.jl`) likely needs to change, not
just the target it's shifted by.** Sketch of why: the completion LP currently derives a cut valid
for `v(z)` (the true subproblem value), tight at one point. To get a cut valid for the *residual*
`v(z) - route_lb_expr(z)` *globally* (not just at `z_hat_k`), the completion LP would need to
account for how `route_lb_expr` itself varies with `z` -- e.g. by folding `route_lb_expr(z)`'s own
structure (it's a linear function of the arc-flow variables, which are themselves linked to `z`
only indirectly through `y`) into the dual-feasibility rows somehow, so the resulting affine cut
is a valid bound on the residual *as a function of z*, not just frozen-tight at one historical
point. This is not the same problem as "shift a scalar target" and needs real derivation before
attempting -- likely closer in spirit to how Benders cuts on a *sum* of two subproblems' duals
would normally be combined (jointly, not by shifting one subproblem's cut by a snapshot of the
other's current value).

An alternative, much simpler fix if the joint-derivation approach turns out to be too involved:
go back to the floor-constraint formulation (sound, already validated) and instead attack its
actual problem (master size/solve time) directly -- e.g. the "take it out of the master entirely"
nested-Benders-on-the-arc-flow-relaxation redesign already sketched earlier in this
investigation (generate a cheap subgradient cut on `theta` itself from a small, separately-solved,
`y`-fixed arc-flow LP each iteration, rather than embedding the flow network as live master
variables at all) -- that design doesn't have this staleness problem either, since it produces an
ordinary cut on `theta` representing the *true* `v(z)` (no shift/residual split involved), just
derived cheaply.

## Not yet done

- Track down whether the remaining 0.05% gap on seed=42/ms5 (post-fix) is the same pre-existing
  `:restricted_mw_fixed_pi` tolerance-slack issue noted above, or a smaller residual edge case of
  this fix specifically -- not yet isolated.
- Confirm/deny whether `:standard` cut derivation was *actually* immune to the original bug, or
  just lucky on the n=15/n=10 instances tested (no adversarial case constructed to test this
  directly) -- moot for going forward now that the fix applies uniformly to all cut derivations,
  but still an open question about the original diagnosis.
- Investigate the separate baseline `lower_bound > incumbent` observation noted above (predates
  and is independent of this bug/fix).
- Re-run the n=30 at-scale cases (the original motivation for this whole feature) now that
  correctness looks solid -- not yet done; wall time/iteration-count tradeoff at n=30 with the
  corrected fix is still unknown.
- Remove the temporary `CS_DEBUG_LIFTED_LB` debug-print instrumentation in `benders/yz.jl`'s outer
  loop once no longer needed (harmless -- gated behind an env var, off by default -- but not meant
  to be permanent).
