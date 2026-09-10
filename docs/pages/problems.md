# Problems

An [`AbstractProblem`](@ref) carries the instance data plus the decision made on top of
it. It says nothing about how the decision is encoded.



## Kept but unwired

`RouteCoveringProblem` is a fixed-`y`/fixed-assignment shape that nothing builds today.
The live Benders subproblem fixes `y` but leaves assignment to `θ`, so this remains the
shape a `:column_generation` subproblem oracle would reuse rather than one anything
constructs now.

{{autodocs opt/problems}}
