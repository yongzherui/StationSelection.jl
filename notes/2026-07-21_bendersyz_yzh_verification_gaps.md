# BendersYZ/BendersYZH: Three Issues Found During Implementation, and the Pattern Behind Them

*2026-07-21*

## Status: all three fixed and verified (913/913 full suite) before landing. This note exists
because the *pattern* behind them is worth remembering for the next new decomposition, not
because anything shipped broken.

## What happened

Adding `BendersYZ` (master=y,z; subproblem=x,θ) and `BendersYZH` (master=y,z,h; subproblem=θ)
alongside the existing `BendersY`/`BendersXY`, three real issues surfaced in sequence, each
caught by a different kind of check:

1. **Missing feasibility cut in `BendersYZ`'s design**, caught by design review before any code
   was written. `BendersXY`'s master has `x` linked to both `zp` and `zd` with no diagonal entry,
   so a `(y,z)` where both sides collide on the same station is structurally MIP-infeasible —
   Gurobi prunes it for free. `BendersYZ`'s master has no `x` at all, so nothing stops `zp`/`zd`
   from independently resolving to the same station. The fix was to reuse `BendersY`'s existing
   `_add_endpoint_collision_feasibility_cut!` branch verbatim — the mechanism already existed in
   the codebase, the gap was only in *not initially reusing it* for the new master shape.

2. **Tie-break epsilon numerically inert**, caught by a dedicated small fixture (two stations
   tied at equal walking cost, forced both open). Continuous `z` under `:big_m_nearest` has a
   genuine degenerate face when two open candidates tie exactly — fixed with a strictly
   increasing `tb_costs` perturbation in `_endpoint_big_m_variable!`
   (`src/opt/constraints/aggregate_od_route.jl`). First attempt used `~1e-9` relative scale,
   which is mathematically a valid strict tie-break but *smaller than Gurobi's default
   `FeasibilityTol`/`IntFeasTol` (~1e-6)* — the solver treated the "broken" tie as still
   satisfied and returned the fractional-adjacent point anyway. A test built specifically to hit
   the tied case (not just "does it converge on average") caught this immediately; raised to
   `max(1e-4, cost_scale * 1e-6)` fixed it.

3. **`BendersYZ` stale-cut premature convergence**, caught by the real-data fixture test —
   `isapprox(benders_yz.objective_value, ground_truth.objective_value)` failed with `123.9` vs.
   the true `56.3888` (`:big_m_nearest`) and `186.8` vs `56.3888` (`:endpoint_chain`), both
   reported as `OPTIMAL`. Root cause is *the same structural gap* documented in
   `notes/2026-07-15_bendersy_stale_cut_soundness.md` for `BendersY`: `_build_yz_route_subproblem_lp`
   fixes only `z`, leaving `x` free, so a column pool proven exhaustive by
   `_solve_fixed_route_covering_by_cg` for *one* nearest-open assignment is not necessarily
   complete for the LP's own, more general dual structure. Fixed the same way:
   `_solve_yz_route_subproblem_lp_with_repricing`, gated behind
   `BendersSolver(reprice_subproblem=true)` — required for a provably optimal `BendersYZ`
   result, same as `BendersY`. `BendersYZH` was checked and is *not* affected: its subproblem
   fixes `h` fully (like `BendersXY`'s `x`), so its priming CG is always exhaustive for exactly
   the LP the cut is drawn from.

## The pattern to remember

(1) and (3) are two instances of the same root cause: **a new decomposition built "by analogy"
to an existing one inherits that existing one's assumptions, and those assumptions can silently
break when the new master's variable set doesn't fully cover what the old one's did.**
`BendersXY` structurally guarantees two things almost for free — no collision (via `x`) and no
stale-cut risk (via fixing `x` fully in its subproblem) — that `BendersYZ` had to re-derive
explicitly because it deliberately has a *smaller* master (`z` only, not `x`).

Checklist for the next new Benders variant:

- **Does the subproblem fix every variable, or does something stay free?** If anything is free
  (continuous, not pinned by an equality/fixing constraint), assume the restricted-pool cut is
  unsound until proven otherwise by a real-data-scale test — don't wait for it to fail. `BendersY`
  and `BendersYZ` both have this shape; `BendersXY` and `BendersYZH` don't (their subproblems fix
  the assignment variable fully).
- **Does the master structurally block every failure mode the more detailed decomposition it's
  derived from blocked?** Compare constraint-by-constraint against the closest existing variant,
  not just against the mathematical formulation on paper.
- **Tie-break / big-M perturbations must be checked against the solver's actual numerical
  tolerance**, not just proven correct in exact arithmetic. A fixture engineered to hit the exact
  degenerate case (not a random synthetic instance that happens to avoid it) is the only way to
  catch this.
- **Small synthetic fixtures are not enough to catch stale-cut bugs.** All three of the fixtures
  used before the real-data check passed cleanly for `BendersYZ` — the bug only showed up on the
  Zhuzhou subset. A real-data-scale objective comparison against `DirectSolver` ground truth
  should be a standing requirement before trusting any new Benders variant, not an optional
  follow-up.
