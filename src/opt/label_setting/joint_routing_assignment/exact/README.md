# Exact joint-routing-assignment label search

This directory implements the production `:exact` pricing mode for
`AggregateODRouteJointRoutingAssignmentFormulation`. Here, **exact** means that
the search may revisit stations and that passenger assignments are represented
without approximation. The search branches only on the next station in the
physical route; it does not branch on passenger pickup or drop-off decisions.

Start with this file, then read `seed.jl`, `extend.jl`, and `dominate.jl` in
that order. `engine.jl` in the parent `label_setting/` directory contains the
generic search loop.

## A label in plain language

`JointRoutingAssignmentPricingLabel` represents one partial route:

| Field | Meaning |
| --- | --- |
| `current` | Station where the partial route ends. |
| `route` | Complete station sequence visited so far; revisits are allowed. |
| `time` | Elapsed time since the synchronized route start. |
| `station_age` | Live pickup clocks: origin station to time elapsed since its most recent eligible visit. |
| `activated_reward_layers` | Compact encoding of the best reward certified so far for each passenger. |
| `tau` | Physical travel time accumulated by the route. It excludes the fixed repositioning time. |
| `reduced_cost` | Regularized route cost minus reward already certified by the label. It includes the fixed repositioning cost. |
| `route_length` | Number of station visits, used by the `max_stops` resource limit. |

`time` and `tau` are currently advanced by the same travel time. They remain
separate because `time` is the feasibility clock, whereas `tau` is the route
coefficient placed in the generated column.

## Search flow

```text
pricing_round.jl builds a context
        |
        v
label_setting/engine.jl
  1. seed initial labels                         seed.jl
  2. compute a priority lower bound              prune.jl
  3. pop the most promising live label
  4. compare it with labels at the same state    dominate.jl
  5. choose useful next stations                 extend.jl
  6. extend and insert each child                extend.jl
        |
        v
replay surviving routes into assignments         accept.jl
        |
        v
deduplicate, verify, and create columns           hooks.jl / label_setting/round.jl
```

`hooks.jl` contains adapters between the generic engine and these files. It is
useful as an index of operations, but it is not the best place to learn the
algorithm.

## What happens during extension

For a move from the current station to `next_node`, `extend.jl`:

1. adds travel time to `time` and `tau`;
2. certifies every newly reachable passenger reward layer at `next_node`;
3. subtracts that new reward from reduced cost;
4. ages existing pickup clocks and removes clocks that can no longer certify
   any assignment;
5. while still inside the pickup window, refreshes `next_node`'s clock to zero.

No passenger is committed to a concrete `(origin, destination)` pair during
the search. `accept.jl` replays a completed route and selects each passenger's
best certified assignment. This keeps intermediate labels small.

## Dominance rule

Labels are compared only when they end at the same current station. Label `a`
can discard label `b` when all of the following hold:

- `a` is no later than `b`;
- every live pickup clock needed by `b` is also live, and no older, in `a`;
- when `max_stops` is bounded, `a` has used no more stops;
- `a`'s reduced-cost advantage is large enough to cover reward that `b` could
  still earn by catching up to reward layers already held by `a`.

The last condition is the compensated-dominance inequality:

```text
reduced_cost(a) + weight(layers(a) ∖ layers(b)) <= reduced_cost(b)
```

The five-argument `_dominates_joint_routing_assignment_label` is the readable
reference implementation of this rule. `_pricing_dominates_at_state` is its
allocation-conscious implementation for the hot per-state scan. Its sparse
arrays, masks, inline filters, and type-level switches change representation
and condition order, not the rule itself.

## Representation versus model concepts

The following types are implementation aids rather than additional model
state:

- `JointRoutingAssignmentLabelBitsets` mirrors live clocks and reward layers
  in forms that are cheaper to compare repeatedly.
- `JointRoutingAssignmentDominanceFilters` copies scalar comparison fields
  inline into each per-state entry.
- `JointRoutingAssignmentDominanceRules` encodes fixed search options in type
  parameters so disabled checks compile away.
- `JointRoutingAssignmentSearchContext` bundles immutable pricing data,
  precomputed search indexes, bound scratch space, and the dominance closure.

Performance measurements and discarded optimization experiments live in
`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`; they are
background, not prerequisites for understanding the search.
