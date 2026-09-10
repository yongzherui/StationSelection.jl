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

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/label_setting/types.jl",
    "opt/label_setting/engine.jl",
    "opt/label_setting/round.jl",
    "opt/label_setting/utils.jl",
]
```

## Exhaustive-equivalent pricers

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/label_setting/joint_routing_assignment/types.jl",
    "opt/label_setting/joint_routing_assignment/seeding.jl",
    "opt/label_setting/joint_routing_assignment/exact/types.jl",
    "opt/label_setting/joint_routing_assignment/exact/dominate.jl",
    "opt/label_setting/joint_routing_assignment/exact/extend.jl",
    "opt/label_setting/joint_routing_assignment/exact/enumeration.jl",
    "opt/label_setting/joint_routing_assignment/darp/types.jl",
    "opt/label_setting/joint_routing_assignment/darp/data.jl",
    "opt/label_setting/joint_routing_assignment/darp/driver.jl",
    "opt/label_setting/joint_routing_assignment/darp/extend.jl",
    "opt/label_setting/joint_routing_assignment/darp_modified/types.jl",
    "opt/label_setting/joint_routing_assignment/darp_modified/data.jl",
    "opt/label_setting/joint_routing_assignment/darp_modified/driver.jl",
    "opt/label_setting/joint_routing_assignment/darp_modified/extend.jl",
]
```

## Route-covering pricer

The fixed-`y` shape, kept for the subproblem oracle that would reuse it.

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/label_setting/route_covering/duals.jl",
    "opt/label_setting/route_covering/exact/types.jl",
    "opt/label_setting/route_covering/exact/enumeration.jl",
]
```

## Elementary-only pricer

```@autodocs
Modules = [StationSelection]
Pages = ["opt/label_setting/joint_routing_assignment/station_simple/types.jl"]
```

## Where the certifying relaxation is documented

`:relaxed_cluster` and `:relaxed_cluster_two_tier` are large enough to warrant their own
page -- see [Certification](@ref).
