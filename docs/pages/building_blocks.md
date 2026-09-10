# Model building blocks

`build_model` methods compose these; they do not live inside `formulations/` or
`problems/` themselves. Each block's docstring names which formulations use it — that
"Used by:" line is the map from a row or variable family back to the models that carry it.

Adding a variable family here means adding its export-variables counterpart too, or the
analysis output silently omits it.

## Variables

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/variables/base.jl",
    "opt/variables/assignment.jl",
    "opt/variables/routes.jl",
    "opt/variables/walk.jl",
    "opt/variables/flow.jl",
    "opt/variables/benders_cuts.jl",
]
```

## Constraints

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/constraints/base.jl",
    "opt/constraints/assignment.jl",
    "opt/constraints/flow_activation.jl",
    "opt/constraints/endpoint_feasibility.jl",
    "opt/constraints/benders_cuts.jl",
    "opt/constraints/aggregate_od_route/base/coverage.jl",
    "opt/constraints/aggregate_od_route/base/linking.jl",
    "opt/constraints/aggregate_od_route/base/route_activation.jl",
    "opt/constraints/aggregate_od_route/joint_routing_assignment/coverage.jl",
    "opt/constraints/aggregate_od_route/joint_routing_assignment/linking.jl",
    "opt/constraints/aggregate_od_route/joint_routing_assignment/routing_and_assignment.jl",
]
```

## Objectives

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/objectives/clustering_base.jl",
    "opt/objectives/clustering_od.jl",
    "opt/objectives/benders_master.jl",
    "opt/objectives/aggregate_od_route/base/assembly.jl",
    "opt/objectives/aggregate_od_route/joint_routing_assignment/assembly.jl",
]
```
