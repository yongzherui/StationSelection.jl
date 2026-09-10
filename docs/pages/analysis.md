# Analysis & output

Helpers for reading a solved model back out: which stations were built, what each demand
group was assigned to, and where the objective's value actually went.

`export_variables.jl` is the largest single file in the package. Every variable family added
under `opt/variables/` needs a corresponding export function here, or that family is
silently missing from the exported CSVs.

## Solution analysis

```@autodocs
Modules = [StationSelection]
Pages = [
    "utils/analysis/solution_analysis.jl",
    "utils/analysis/objective_decomposition.jl",
    "utils/analysis/export_variables.jl",
]
```

## Core utilities

`OptResult`, `SolveStatus` and `BuildResult` also live in `utils/core/results.jl` but are
documented under [Results & solve status](@ref), where they belong.

```@autodocs
Modules = [StationSelection]
Pages = [
    "utils/core/coords.jl",
    "utils/core/costs.jl",
    "utils/core/export.jl",
]
```

## Scenario and candidate-station helpers

```@autodocs
Modules = [StationSelection]
Pages = [
    "utils/data/scenarios.jl",
    "utils/data/candidate_stations.jl",
]
```
