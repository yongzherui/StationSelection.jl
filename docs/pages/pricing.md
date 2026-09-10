# Pricing & label setting

The pricer is the search that finds improving columns. It is selected on
[`CGPricingConfig`](@ref) and implemented by the label-setting engine under
`opt/label_setting/`.

## Which universe a pricer searches

This is the distinction that decides what an `OPTIMAL` is worth:

- `:exact`, `:darp` and `:darp_modified` search the full revisit-tolerant route universe.
  They are **exhaustive-equivalent** — different searches of the same space.
- `:station_simple` searches elementary routes only, so it is a *restriction* of the
  universe rather than a different search of it. A revisiting column can beat its
  optimum, which is why its scope is reported separately and why the Benders oracles
  reject it outright.
- `:relaxed_cluster` and `:relaxed_cluster_two_tier` price a **relaxation** whose minimum
  reduced cost lower-bounds the real one, which is what lets them *certify*.

See [Results & solve status](@ref) for how each case is reported.

## Core engine

{{autodocs opt/label_setting !opt/label_setting/joint_routing_assignment !opt/label_setting/route_covering}}

## Route-covering pricer

The fixed-`y` shape, kept for the subproblem oracle that would reuse it.

{{autodocs opt/label_setting/route_covering}}

## The pricers themselves

Each concrete pricer for the joint formulation has its own page: [Joint routing
pricers](@ref) for the exhaustive-equivalent and elementary searches,
[Relaxed-cluster relaxation](@ref) and [Certification](@ref) for the certifying pair.
