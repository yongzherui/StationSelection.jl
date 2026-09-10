# CGSolver

Solve the restricted master, price against its duals, add improving columns, repeat.

`CGSolver` owns the *loop* and its **budgets**. Which search finds the columns lives on
[`CGPricingConfig`](@ref), and the pricers themselves are documented under
[Pricing & label setting](@ref).

{{autodocs opt/solvers/cg/solver.jl}}

## Choosing the pricer

{{autodocs opt/solvers/cg/pricing_config.jl}}

## Loop internals

{{autodocs opt/solvers/cg !opt/solvers/cg/solver.jl !opt/solvers/cg/pricing_config.jl}}
