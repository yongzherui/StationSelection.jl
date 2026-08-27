# `label_setting/`

The pricing/column-enumeration engine the two `AggregateODRoute` formulations
sit on: route enumeration for `DirectMIPSolver`, label-setting pricing rounds
for `CGSolver`.

- `engine.jl` — `_run_label_setting`, the generic label-setting search loop
  (priority-queue frontier, dominance-based pruning). Knows nothing about
  routes, stations, or passengers; every piece of domain meaning is a hook
  supplied by the caller.
- `types.jl` — the shared hook contract (`AbstractPricingSearchContext`) every
  concrete pricer implements to plug into `engine.jl`.
- `round.jl` — `_run_pricing_round`, `CGSolver`'s `price_columns` hook: builds
  a context per scenario, runs the engine against it, and harvests/verifies
  the surviving labels into columns.
- `utils.jl` — reward-model-independent dominance mechanics (sparse
  station-age comparisons, bitset-weight compensation) shared by every pricer.
- `route_covering/` — the aggregate, pair-based pricer
  (`AggregateODRouteBaseFormulation`). `exact/` is the revisit-tolerant search
  actually wired into production; `station_simple/` is an elementary-route
  alternative, not currently reachable from any `build_model`.
- `joint_routing_assignment/` — the passenger free-assignment pricer
  (`AggregateODRouteJointRoutingAssignmentFormulation`). `exact/`,
  `darp_modified/`, and `darp/` are wired into production, selectable per solve
  via the formulation's `pricing_mode` field (`:exact`, the default,
  `:darp_modified`, or `:darp`) --
  `pricing_round.jl` (this directory's own, not either subdirectory's) is
  where `_pricing_build_scenario_context` branches between them. `darp/` is a
  DARP-style adaptation with explicit pickup commitment, onboard liabilities,
  pickup-time reward credit, and compulsory feasible delivery. Run to
  exhaustion it is required to reach the same optimum as `exact/`; randomized
  small-instance regression tests enforce that invariant. It exists to measure how much
  `exact/`'s reward-layer running-max trick is worth computationally against
  a search that makes the same choice explicit via branching instead.
  `station_simple/` is the same elementary-route alternative as
  `route_covering/`'s, not currently reachable from any `build_model`.

Every pricer directory below `route_covering/`/`joint_routing_assignment/`
(`route_covering/exact/`, `route_covering/station_simple/`,
`joint_routing_assignment/exact/`, `joint_routing_assignment/station_simple/`,
`joint_routing_assignment/darp_modified/`, `joint_routing_assignment/darp/`)
is split by label-setting *functionality* rather than bundled into one
`labels.jl` + one context file -- logic and wiring kept in separate files
throughout:

- `types.jl` — label/bitsets/dominance-filter structs, plus the module
  docstring describing the operational contract those types encode (what a
  route gets *credit* for). Start here for "what does a label mean".
- `seed.jl` — the depth-1 labels the search seeds its frontier from.
- `extend.jl` — candidate next-nodes + label extension: which nodes/actions a
  label may legally visit next, and what taking one produces.
- `prune.jl` — the admissible remaining-reward bound behind frontier ordering
  and pop-time pruning. Not present in `joint_routing_assignment/station_simple/`,
  which reuses `exact/prune.jl`'s bound directly (the reward model is
  identical; only the route universe differs) -- wired straight into its own
  `hooks.jl` with no forwarding file of its own.
- `dominate.jl` — state/order keys, the bitsets mirror, any reward-diff
  compensation sub-test, and the dominance predicate itself. Start here for
  "why did the search not explode" — this is measured at ~85-90% of wall
  time in every pricer that has been profiled.
- `context.jl` — just the search-context struct and its constructor. No hook
  methods, no search logic.
- `hooks.jl` — all wiring, no logic: every one-line (or thin-adapter) method
  forwarding `context.jl`'s struct into the two hook contracts every pricer
  implements -- the twelve `AbstractPricingSearchContext` hooks
  (`_pricing_initial_labels`, ..., forwarding to `seed.jl`/`extend.jl`/
  `prune.jl`/`dominate.jl`) and the four `round.jl` accept/harvest hooks
  (`_pricing_candidate_from_label`/`_pricing_pool_signature`/
  `_pricing_make_column`/`_pricing_verify_column`). Loads last in every
  directory -- it needs `context.jl`'s struct type in every method signature,
  and calls into every logic file above it.
- `accept.jl` — route replay: how a finished label's physical route becomes
  concrete per-passenger assignments. Pure logic, no `ctx`, no hook methods.
  Present only in `joint_routing_assignment/exact/` -- every other pricer's
  round-level hooks are either a trivial projection off the label's own
  fields (`route_covering/`, `darp_modified/`, `darp/`, all folded straight
  into `hooks.jl`) or reuse `exact/accept.jl`'s replay directly
  (`station_simple/`).
- `logging.jl` — off-by-default dev instrumentation (a dominance
  rejection census), split out of `dominate.jl` so the production algorithm
  and the tooling for measuring it don't share a file. Present only in
  `joint_routing_assignment/exact/`, the one pricer profiled/instrumented
  this way.
- `driver.jl` — a standalone entrypoint reproducing `round.jl`'s accept/
  dedupe/harvest logic by hand, for benchmarking one pricer against another
  outside the CG hub (no live master model/duals needed). Present only in
  `joint_routing_assignment/darp_modified/` and `joint_routing_assignment/darp/`,
  which exist specifically as comparison points against `exact/`.
- `data.jl` — pricing-data preprocessing, present only where a pricer can't
  share `../data.jl`'s (or, for `route_covering/`, the parent directory's)
  construction: `joint_routing_assignment/darp_modified/` and
  `joint_routing_assignment/darp/`, whose branching search needs its own
  eligibility/subset-enumeration helpers (see either's `types.jl` module
  docstring for why). Unaffected by this split -- it predates it and was
  already its own file.

Load order in `optimize.jl` follows each directory's real dependency graph
(not always identical across directories): `types.jl` and `logging.jl`
first (no dependencies), then `seed.jl`/`extend.jl`/`dominate.jl` (logic
files that read from `types.jl`/`data.jl` but never need the search-context
struct), then anything `prune.jl` needs precomputed
(`joint_routing_assignment/`'s shared `search_data.jl`, included between
`exact/`'s `dominate.jl` and `prune.jl`), then `prune.jl` and `context.jl` in
whichever order that directory's `prune.jl` allows (a `prune.jl` typed on the
context struct loads after `context.jl`; one that takes `pricing_data`
directly, or an untyped `ctx`, doesn't care), then `accept.jl` (pure logic,
no context dependency), then `hooks.jl` last (needs the context struct in
every method signature), then `driver.jl` last of all (needs `hooks.jl`'s
hooks). See each directory's own `types.jl` module docstring, and the
comments directly above its block in `optimize.jl`, for the specific
ordering rationale that applies there.

## Compensated dominance (a.k.a. "catch-up" dominance)

Both pricers support a `compensated_dominance` toggle
(`AggregateODRouteBaseFormulation`/`AggregateODRouteJointRoutingAssignmentFormulation`,
default `true`) on their label-setting dominance test. This section explains
what "compensated" means here, and why the mechanic is also sometimes thought
of as a "catch-up" rule -- both names describe the same transaction, just from
opposite ends.

### The plain rule, and why it needs a subset condition

For label `a` to dominate label `b`, `a` must end up no worse than `b` on
*every* possible shared future completion. Two labels certify sets of reward
(reward layers for the passenger pricer, station pairs for the route-covering
pricer) as they extend; on a shared future suffix `ζ`, the *incremental*
reward either label collects is whatever `ζ` newly certifies, minus whatever
that label has already banked.

The plain dominance rule requires `A_a ⊆ A_b` -- everything `a` has already
banked, `b` has too. That's what makes the proof work: since `b` can never be
"behind" `a` on anything already certified, `b` can never pick up strictly
more new reward from `ζ` than `a` would, so `a`'s existing cost edge over `b`
can only hold or grow. Combined with `a` being no worse positionally (same
state, no later, no worse live clocks), that's sufficient for `a` to
dominate.

### What breaks it, and what "catch-up" refers to

If `a` holds something `b` doesn't (`A_a \ A_b ≠ ∅`), the proof fails: on a
future suffix, `b` might newly certify that *same* reward -- something `a`
already used up and can't bank again -- so `b` picks up extra reward `a`
doesn't. That's `b` **catching up** to `a`. The worst case is `b` catching up
on everything `a` holds that it doesn't, worth `w(A_a \ A_b)`.

### What "compensated" refers to

`a` can still safely dominate `b` if `a`'s *existing* reduced-cost lead over
`b` is large enough to absorb that worst-case catch-up:

```
rc_a + w(A_a \ A_b) <= rc_b        ⟺        rc_b - rc_a >= w(A_a \ A_b)
                                              (a's cost lead)   (b's max catch-up)
```

Read right to left: **`a`'s banked savings must be at least as large as the
bill `b` could hand it back.** That's the compensation -- `a`'s cost
advantage isn't just compared to `b`'s in the abstract, it's spent covering
the specific risk introduced by dropping the strict subset requirement. This
is the same sense as "compensating for a weakness with a strength," or an
engineering compensator that cancels a known error term: the reduced-cost
lead is the payment, the catch-up value is the bill, and the inequality is
just "can the payment cover the bill."

Setting `compensated_dominance = false` falls back to the plain rule
(`A_a ⊆ A_b`, no payment required, no catch-up ever tolerated) -- strictly
weaker dominance (fewer labels get pruned), trading column diversity per
search for speed. Which side wins for column generation end to end is a
solve-quality question, not a pricing-speed one; see
`RouteCoveringSearchContext`/`JointRoutingAssignmentSearchContext`'s own
docstrings for the specific structs this feeds.

### Why both names, and why neither should be dropped

"Catch-up" names the *bill* -- what `b` could still recover, the thing the
rule has to defend against. "Compensated" names the *rule* -- the fact that
`a` is required to have already paid for that bill out of its own cost lead
before it's allowed to dominate despite holding extra, not-yet-shared reward.
They're two views of one transaction: `catch-up` is the risk, `compensated`
is the insurance against it. The codebase uses both, at different scopes on
purpose -- `compensated_dominance` names the toggle/rule everywhere it
appears (formulation fields, the `Compensated` type parameter on
`RouteCoveringDominanceRules`/`JointRoutingAssignmentDominanceRules`,
`_bitset_diff_weight`'s `compensated` keyword), while "catch-up" is reserved
for the specific quantity inside it (`w(A_a \ A_b)`, called out as the
"catch-up term"/"catch-up advantage"/"catch-up reward" in `label_setting/utils.jl`
and both pricers' dominance docstrings). Renaming the toggle itself to
`catch_up_dominance` would rename the rule after its own justification rather
than after what it does, and would touch every one of those call sites for no
change in behavior -- not worth it unless the current split genuinely causes
confusion in practice.
