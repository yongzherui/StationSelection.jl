# DirectMIPSolver

A single `optimize!` on the fully built model. No outer loop, and no selectable pricer —
`AggregateODRouteBaseFormulation`'s column pool is enumerated exhaustively at build time.

```@autodocs
Modules = [StationSelection]
Pages = ["opt/solvers/direct_solver.jl"]
```
