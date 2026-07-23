# AggregateODRouteModel Benders: Guaranteeing Subproblem Feasibility By Construction

*2026-07-22*

## Status: implemented and verified (508/508 targeted `AggregateODRouteModel` tests). Reactive
feasibility cuts are no longer reachable under the default model configuration; the mechanism
that used to add them is kept as a defensive fallback, not deleted.

## The problem this replaces

`BendersY` and `BendersYZ` used to discover an infeasible `y_hat` only *after* proposing it:
solve the master, resolve each request's nearest-open assignment procedurally
(`_fixed_assignments_from_y`), and if some request came back infeasible, add a reactive cut
(`_add_endpoint_nearest_feasibility_cuts!`, `_add_endpoint_collision_feasibility_cut!`, or
`_add_pair_open_feasibility_cut!`, all in `benders/covering.jl`) and re-solve the master. Correct,
but wasteful — every rejected `y_hat` costs a full master + CG round trip before the cut is even
derived.

## The mechanism

Three changes together make the subproblem provably always feasible for the default model
configuration, so those reactive cuts are never needed:

1. **Default endpoint-coverage constraints in the master.**
   `_add_default_endpoint_coverage_constraints!` (`benders/covering.jl`) adds one
   `sum(y[j] for j in candidates) >= 1` row per unique physical `(endpoint, side)` touched by any
   request (aggregated across every scenario, since `y` is scenario-agnostic) directly to every
   Benders master (`y.jl`, `xy.jl` ×2, `yz.jl`, `yzh.jl`, and the standalone
   `solve_benders_yzh_master` test API in `subproblem_api.jl`). `candidates` is
   `_nearest_open_endpoint_candidates` — the same walking-distance-bounded set
   `compute_valid_jk_pairs` uses to build real `(j,k)` pairs in the first place, so "some candidate
   on each side is open" is a genuine necessary condition, not an approximation.
   `_check_aggregate_od_route_endpoint_feasibility!` runs the same constraints as a cheap
   stand-alone pre-flight MILP (`y` + coverage only) before any mapping/CG machinery is built, so
   an infeasible station budget (`l` too small to cover every endpoint) fails fast with a targeted
   message instead of surfacing deep inside the outer Benders loop.

   Gated off only when `unmet_demand_penalty !== nothing`
   (`_endpoint_coverage_applicable`) — under that "always feasible" mode an uncovered endpoint is
   a deliberate, legitimate `u=0` outcome, not an infeasibility, so forcing coverage would defeat
   the relaxation's purpose. That mode never used reactive cuts either (see point 3), so nothing
   changes for it here.

2. **`allow_same_station` is now unconditionally `true`.** `create_map`
   (`data/maps/aggregate_od_route_map.jl`) used to pass
   `allow_same_station=!isnothing(base_model.unmet_demand_penalty)` to `compute_valid_jk_pairs`; it
   now always passes `true`. A same-station pair `(j,j)` is a valid real assignment whenever `j` is
   within `max_walking_distance` of both `o` and `d` — so the classic "both sides' independently
   nearest-open station collide on the same station" case, which used to be infeasible unless
   `allow_walk_only` covered it, now always resolves to `(j,j)` instead. The Benders call sites that
   resolve assignments procedurally (`_fixed_assignments_from_y` in `y.jl`/`yz.jl`, and the
   assertion oracle `_assert_x_matches_nearest_open`) were updated to pass `allow_same_station=true`
   to match.

   **Necessary companion fix, not optional:** `compute_valid_jk_pairs` (`data/maps/clustering_od_map.jl`)
   now omits the same-station real pair specifically when `WALK_ONLY_PAIR` is *also* available for
   that OD (`allow_walk_only` and `dist(o,d) <= 2*max_walking_distance`). By the triangle
   inequality, any `j` that qualifies as a same-station candidate for `(o,d)` already implies
   `dist(o,d) <= 2*max_walking_distance`, so walk-only already covers exactly the same case at
   equal-or-lower cost. Offering *both* `(j,j)` and `WALK_ONLY_PAIR` simultaneously is not merely
   redundant — once `z`/`zp`/`zd` are forced to a deterministic 0/1 value (see point 3), both get
   the identical lower bound `>= zp+zd-1` from a *shared* forced-open station, which forces *both*
   to 1 simultaneously and breaks `sum(x)==1`. This was caught empirically (genuine `INFEASIBLE` LP
   status on the walk-only direct-walking fixture) before being understood, not derived up front.

3. **`sum(z)==1`/`sum(x)==1` stay hard-required everywhere, deliberately.** An earlier draft of
   this change relaxed `_endpoint_chain_variable!`/`_endpoint_big_m_variable!`'s `sum(z)==1` to
   `<=1` (matching the `unmet_demand_penalty` relaxation) whenever a request was walk-only-eligible,
   to avoid forcing a station open purely for bookkeeping. This was deliberately reverted: coverage
   necessity is *not* relaxed per-request based on walk-only eligibility. Every request's
   assignment must resolve to a real pair (distinct-station or same-station), full stop, given the
   model default `allow_walk_only=false`. `_endpoint_chain_variable!`/`_endpoint_big_m_variable!`
   are therefore unchanged from before this work.

## Why this guarantees feasibility (the actual argument)

`_independent_nearest_open_assignment` (`benders/covering.jl`) is the procedural resolver behind
`_fixed_assignments_from_y`, and its only path to returning `nothing` (infeasible) is
`isnothing(j_star) || isnothing(k_star)` — no open candidate at all on one side. Point 1 above
makes that unreachable: every physical endpoint any request touches has `sum(y[candidates]) >= 1`
baked into the master, so `y_hat` always has an open candidate on both sides. Given that, the
function always finds `j_star`/`k_star`; if they differ, it returns the real distinct-station pair;
if they collide, `allow_same_station=true` (point 2) means it always returns `(j_star, j_star)`
instead of falling through to `nothing`. So `_fixed_assignments_from_y` can never place a request in
its `infeasible` list, and the reactive-cut branch in `y.jl`/`yz.jl`
(`if !isempty(infeasible) && isnothing(model.unmet_demand_penalty)`) can never be entered.

## Verification

508/508 targeted `AggregateODRouteModel` tests pass (`test_aggregate_od_route_*.jl`, run via
`test/runtests.jl`'s "Model Integration" testset). Beyond passing, the claim was checked directly:
a temporary `@warn` at the reactive-cut branch's entry guard in both `y.jl` and `yz.jl`, run across
the full targeted suite, fired **zero times** — including on the fixture
(`test_aggregate_od_route_nearest_open_alignment.jl`, "BendersYZ collision fixture") specifically
engineered to force a station collision, which used to require this exact branch and whose test
assertion was updated from `feasibility_cuts_added > 0` to `== 0` to reflect the new behavior. The
ground-truth optimal station set for that fixture also changed (`{2,3}` → `{1,2}`), since the
same-station resolution at the collision station is now a real, strictly *cheaper* option instead
of being excluded as infeasible.

## What this does *not* cover (deliberately deferred)

- **`allow_walk_only=true` models are not exempted from the coverage requirement.** A request whose
  only servable option is `WALK_ONLY_PAIR` (same-station real pair deliberately excluded per point
  2's companion fix) still gets its endpoint hard-covered by the default constraint — forcing a
  station open that provides no actual service benefit for that request, since it resolves via
  direct walking either way. This was identified and discussed explicitly; the decision (for now)
  is to keep `allow_walk_only=false` as the default path this guarantee targets, and not implement
  a per-request walk-only exemption. If a future need re-opens this: the natural pre-solve is to
  detect, per physical OD, whether `real_offdiag` (distinct-station pairs) is structurally empty
  (pickup-candidate set == dropoff-candidate set == a single station) — that demand can *never* be
  served by an actual vehicle route regardless of `y`, so under `allow_walk_only=true` it could be
  exempted from coverage and priced as the fixed constant `walk(o,d)`, while under
  `allow_walk_only=false` the single candidate is a logically forced `y[j]=1` that could be
  hard-fixed rather than left as a free binary (explicitly declined for now — see the "Pre-fix
  forced y" discussion in conversation; left as a free binary, no pre-solve step).
- **The reactive-cut helpers are not deleted.** `_add_endpoint_nearest_feasibility_cuts!`,
  `_add_endpoint_collision_feasibility_cut!`, `_add_pair_open_feasibility_cut!`, and the
  `if !isempty(infeasible) && isnothing(model.unmet_demand_penalty)` guard in `y.jl`/`yz.jl` are
  untouched — they remain a defensive fallback for any configuration outside this guarantee's
  scope (chiefly `allow_walk_only=true`), where feasibility has not been proven unconditional.
- **`unmet_demand_penalty` mode was never in scope.** It already tolerates unserved demand by
  design and already skipped the reactive-cut branch entirely via the same
  `isnothing(model.unmet_demand_penalty)` guard, independent of anything in this change.

## Key functions, for future reference

- `_endpoint_coverage_applicable`, `_aggregate_od_route_endpoint_candidate_sets`,
  `_add_default_endpoint_coverage_constraints!`, `_check_aggregate_od_route_endpoint_feasibility!`
  — `benders/covering.jl`.
- `_independent_nearest_open_assignment`, `_fixed_assignments_from_y` — `benders/covering.jl` (the
  procedural resolver the feasibility argument above is stated in terms of).
- `compute_valid_jk_pairs` — `data/maps/clustering_od_map.jl` (the `allow_same_station` /
  walk-only-exclusivity fix).
- `create_map` for `AggregateODRouteModel`/`RouteCoveringProblem` — `data/maps/aggregate_od_route_map.jl`
  (where `allow_same_station=true` is now hard-coded).
