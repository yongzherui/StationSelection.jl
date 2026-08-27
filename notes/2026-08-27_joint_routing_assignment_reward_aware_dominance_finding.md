# Joint routing-assignment: reward-aware station-age dominance -- tried and rejected

Follow-up to `f644a7c` ("Fix joint routing-assignment station-clock dominance
unsoundness"), which removed a reward-coupled prune from
`_joint_routing_assignment_age_is_useful` because it made dominance unsound,
at a measured cost of ~40% more labels generated on a real Zhuzhou (n=10,
p=16, s=1) CG run. That commit left open whether a *sound* reward-aware
relaxation could recover some of that cost. This session tried two versions
and rejected both -- kept here so the idea isn't re-attempted from scratch.

## Version 1: exempt a station when the dominator (`a`) is reward-dead there

Rule: for each station `j` where `b` still has a live pickup clock, `a` is
exempt from needing a matching clock if `a` has already collected every
reward layer reachable from origin `j` (checked fresh against
`pricing_data.origin_layer_mask[j]` at comparison time, never by mutating
`station_age` -- that mutation is exactly what made the original prune
unsound).

**Sound.** Proof sketch: for a layer `ℓ` reachable from `j`, reward-dead-for-`a`
means `ℓ ∈ A_a` already. If `ℓ ∉ A_b` too, then `ℓ ∈ A_a \ A_b`, which is
exactly the term D4's compensation budget already charges `a` for. No
dependence on `a` actually being able to *reach* the resource physically --
it already owns the outcome.

Implemented as `_dominates_joint_routing_assignment_label_reward_aware`, wired
in via `JointRoutingAssignmentSearchContext(pricing_data;
reward_aware_dominance=true)`, deliberately the plain `Dict`-based label form
rather than the optimized bitset scan (correctness-first, not yet a
hot-path change). Verified against 300-trial randomized brute-force and
dominance-preservation fuzz tests: 0 failures.

**Measured on real Zhuzhou (n=10, p=16, s=1, 124 opportunities, cold-start-style
candidates via `scripts/generate_zhuzhou_instance.jl`):**

| | max_stops=6 | max_stops=unbounded |
| --- | --- | --- |
| labels_generated (default / reward-aware) | 9137 / 9073 | 11424 / 11347 |
| max_live (default / reward-aware) | 2970 / 2904 | 2989 / 2924 |
| distinct improving columns | 1910 / 1904 | 1838 / 1832 |
| `best_rc` | matched exactly, both cases | matched exactly, both cases |

Only ~0.7% fewer labels, ~2% smaller `max_live` -- far short of recovering the
~40% lost when the unsound prune was removed. Reason: the old unsound rule
pruned a reward-dead station out of *every* label's stored `station_age`
unconditionally, shrinking both sides of every comparison regardless of
pairing. The sound, one-sided version only fires when the *specific*
dominator has independently exhausted the *same* origin the dominated label
still holds live -- two different labels rarely reach that exhaustion at the
same point, so it fires rarely. The symmetric coupling was where essentially
all of the power (and the unsoundness) lived.

## Version 2: also exempt when the dominated label (`b`) is reward-dead there

Tempting next step, framed two equivalent ways: "exempt if reward-dead for
`a` OR for `b`", or "only check clocks of `b` that are still reward-*live* for
`b`" -- these are the same operation (skipping a station from the D3 loop),
just described differently.

**Unsound.** Found immediately by the existing randomized brute-force fuzz
test (seed 2026, trial 21) without needing a new test written for it.

Minimal counterexample: 5 nodes, one candidate (`origin=3, destination=5,
ride_limit=4.0, reward=1.0`), `route_regularization_weight=1.0,
max_wait_time=1.0, max_stops=3`. Seed label `a` starts directly at node 5
(`rc=0`, no useful clocks at all -- pricer seeds every opportunity endpoint).
Label `b` starts at 3, travels to 5, certifying the opportunity (`rc=1.0`,
live clock `{3: age 2.0}`). Both land at `current=5`. Station 3 is reward-dead
*for `b`* (`b` already banked the only layer reachable from it), so the
version-2 rule drops it from the D3 check entirely -- leaving nothing to stop
`a` from "dominating" `b`. `b` gets discarded. Since `a` can never open a
clock at 3 in time (`travel(5,3)=2 > max_wait_time=1.0`) and there is no other
origin in the instance, `a` can never certify anything, ever. The search
reports `best_rc = Inf`; brute force gives `1.0` (via `[3,5]`, tied with
`[3,4,5]`).

**Why the asymmetry matters, not just an edge case:** reward-dead-for-`a` on a
non-empty origin mask *implies* `a` has, at some point in its own history,
actually certified something there -- so `a`'s current `reduced_cost` already
corresponds to a real, extractable route, and the D4 comparison is between two
real achievements. Reward-dead-for-`b` says nothing about `a`'s own
capability: `a` can hold a numerically better `reduced_cost` while being
*physically incapable* of ever certifying anything (no live clock anywhere,
no way to reopen one before the pickup window closes). Its "advantage" is a
mirage that never cashes out into a real column, while `b`'s current position
already is one. The reach-containment algebra (`reward_b(sigma) <=
reward_a(sigma) + w(A_a\A_b)`) holds numerically at every step regardless --
the failure is that it says nothing about whether `a`'s side of that
inequality is ever *extractable*, and reward-dead-for-`a` is exactly the
condition that guarantees it is.

## Disposition

Both versions were removed from the codebase after this investigation
(reverted in full -- `_dominates_joint_routing_assignment_label_reward_aware`,
its `reward_aware_dominance` search-context flag, and the associated tests are
gone, not just disabled). The one-sided version was sound but not worth the
maintenance surface for a ~1-2% label-count improvement; the symmetric version
was never viable. `f644a7c`'s station-clock rule (ride-limit expiry only, no
reward coupling) remains the correct, permanent state of this pricer's
dominance test.

If this is revisited, the open direction flagged at the end of this
investigation: a condition that ties `a`'s advantage to an actual physical
capability guarantee (e.g., `a` provably able to reopen an equivalent clock
somewhere still reachable), not to which side's `activated_reward_layers`
happens to be more advanced.
