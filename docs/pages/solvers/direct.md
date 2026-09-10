# DirectMIPSolver

A single `optimize!` on the fully built model. No outer loop, and no selectable pricer —
`AggregateODRouteBaseFormulation`'s column pool is enumerated exhaustively at build time.

{{autodocs opt/solvers/direct_solver.jl}}
