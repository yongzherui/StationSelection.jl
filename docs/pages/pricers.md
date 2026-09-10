# Joint routing pricers

The concrete searches for `AggregateODRouteJointRoutingAssignmentFormulation`, all built on
the engine in [Pricing & label setting](@ref). Which one runs is
[`CGPricingConfig`](@ref)'s `mode`.

`:exact`, `:darp` and `:darp_modified` are **exhaustive-equivalent** -- three different
searches of the same full revisit-tolerant universe. `:station_simple` is not: it searches
elementary routes only, so it is a *restriction* of the universe, and a revisiting column
can beat its optimum. That is why its certified result is scoped separately (see [Results &
solve status](@ref)) and why the Benders oracles reject it outright.

## Shared pricing data

{{autodocs opt/label_setting/joint_routing_assignment !opt/label_setting/joint_routing_assignment/relaxed_cluster !opt/label_setting/joint_routing_assignment/station_simple !opt/label_setting/joint_routing_assignment/exact !opt/label_setting/joint_routing_assignment/darp !opt/label_setting/joint_routing_assignment/darp_modified}}

## Exact

{{autodocs opt/label_setting/joint_routing_assignment/exact}}

## DARP and DARP-modified

{{autodocs opt/label_setting/joint_routing_assignment/darp opt/label_setting/joint_routing_assignment/darp_modified}}

## Elementary-only (`:station_simple`)

{{autodocs opt/label_setting/joint_routing_assignment/station_simple}}
