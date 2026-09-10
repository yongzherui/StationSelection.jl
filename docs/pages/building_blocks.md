# Model building blocks

`build_model` methods compose these; they do not live inside `formulations/` or
`problems/` themselves. Each block's docstring names which formulations use it — that
"Used by:" line is the map from a row or variable family back to the models that carry it.

Adding a variable family here means adding its export-variables counterpart too, or the
analysis output silently omits it.

## Variables

{{autodocs opt/variables opt/variables.jl}}

## Constraints

{{autodocs opt/constraints opt/constraints.jl}}

## Objectives

{{autodocs opt/objectives opt/objective.jl}}
