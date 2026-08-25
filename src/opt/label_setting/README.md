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

`exact/` and `station_simple/` (both pricer families) follow the same
three-file shape: `types.jl` (label/bitsets/dominance structs), `labels.jl`
(the label-DP primitives — the file to audit for "is the search correct"),
and a context file named after the directory (`exact.jl` /
`station_simple.jl`, the `AbstractPricingSearchContext` + hooks).
`joint_routing_assignment/darp/` adds a fourth file, `data.jl`, because
unlike `exact/`'s and `station_simple/`'s shared reward-layer preprocessing
(`joint_routing_assignment/data.jl`), `darp/`'s branching commit-or-skip
search needs its own pricing-data construction (see `darp/types.jl`'s module
docstring for why); its own context file is named `darp.jl` rather than
`exact.jl`, since it has no further internal elementary-route split to
distinguish itself from.

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
