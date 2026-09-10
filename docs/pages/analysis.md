# Analysis & output

Helpers for reading a solved model back out: which stations were built, what each demand
group was assigned to, and where the objective's value actually went.

`export_variables.jl` is the largest single file in the package. Every variable family added
under `opt/variables/` needs a corresponding export function here, or that family is
silently missing from the exported CSVs.

## Solution analysis

{{autodocs utils/analysis output}}

## Core utilities

`OptResult`, `SolveStatus` and `BuildResult` also live in `utils/core/results.jl` but are
documented under [Results & solve status](@ref), where they belong.

{{autodocs utils/core !utils/core/results.jl}}

## Scenario and candidate-station helpers

{{autodocs utils/data}}
