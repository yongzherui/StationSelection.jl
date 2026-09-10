# CGSolver

Solve the restricted master, price against its duals, add improving columns, repeat.

`CGSolver` owns the *loop* and its **budgets**. Which search finds the columns lives on
[`CGPricingConfig`](@ref), and the pricers themselves are documented under
[Pricing & label setting](@ref).

```@autodocs
Modules = [StationSelection]
Pages = ["opt/solvers/cg/solver.jl"]
```

## Choosing the pricer

```@autodocs
Modules = [StationSelection]
Pages = ["opt/solvers/cg/pricing_config.jl"]
```

## Loop internals

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/solvers/cg/state.jl",
    "opt/solvers/cg/loop.jl",
    "opt/solvers/cg/metadata.jl",
    "opt/solvers/cg/hooks.jl",
]
```
