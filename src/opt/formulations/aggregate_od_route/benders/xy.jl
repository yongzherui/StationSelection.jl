"""
Formulation-level counterpart to the `BendersXY` decomposition marker in
`optimize/aggregate_od_route/solver_types.jl` -- see `y.jl` in this directory for the
Problem/Formulation/Solver split rationale shared by every file here.
"""

export AggregateODRouteBendersXYFormulation

"""
    AggregateODRouteBendersXYFormulation <: AbstractFormulation

Benders master expressed over first-stage `y` and linking/assignment variables `x`
together. Always uses the standard subgradient cut -- unlike `BendersY`/`BendersYZ`, no
restricted dual-completion variant is implemented for this decomposition, so there is no
`cut_derivation` choice here. Structural counterpart to the `BendersXY` decomposition
marker.
"""
struct AggregateODRouteBendersXYFormulation <: AbstractFormulation
    cut_mode::AbstractBendersCutMode

    AggregateODRouteBendersXYFormulation(; cut_mode::AbstractBendersCutMode=MultiCut()) =
        new(cut_mode)
end
