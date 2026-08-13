"""
Formulation-level counterpart to the `BendersYZH` decomposition marker in
`optimize/aggregate_od_route/solver_types.jl` -- see `y.jl` in this directory for the
Problem/Formulation/Solver split rationale shared by every file here.
"""

export AggregateODRouteBendersYZHFormulation

"""
    AggregateODRouteBendersYZHFormulation <: AbstractFormulation

Benders master expressed over `y`, `z`, and the scenario-compressed assignment variable `h`
(one per physical OD pair, shared across every scenario it appears in); only route-covering
`θ` is left to the subproblem. `cut_derivation=:restricted_mw_fixed_pi` is invalid here --
once `h` is fixed fully there is no remaining free dual block to optimize over, so it would
coincide exactly with `:zero_completion`. No `lifted_walking_objective`/
`direct_enumeration_guide` here either -- both are `BendersY`/`BendersYZ`-only. Structural
counterpart to the `BendersYZH` decomposition marker.
"""
struct AggregateODRouteBendersYZHFormulation <: AbstractFormulation
    cut_mode::AbstractBendersCutMode
    cut_derivation::Symbol

    function AggregateODRouteBendersYZHFormulation(;
            cut_mode::AbstractBendersCutMode=MultiCut(),
            cut_derivation::Symbol=:zero_completion,
        )
        cut_derivation in (:standard, :zero_completion) || throw(ArgumentError(
            "cut_derivation must be :standard or :zero_completion for BendersYZH " *
            "(:restricted_mw_fixed_pi would coincide exactly with :zero_completion once h is fixed)"
        ))
        new(cut_mode, cut_derivation)
    end
end
