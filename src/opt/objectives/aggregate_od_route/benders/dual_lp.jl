"""
Objective composers for the Benders core-point / dual-completion LPs (`y_mw_cut.jl`,
`yz_mw_cut.jl`). Extracted with textually identical algebra -- see
`variables/aggregate_od_route/benders/dual_lp.jl`'s module docstring.
"""

"""
    set_core_point_objective!(m, delta)

Section B2's objective: maximize the shared normalization variable `delta`.
"""
function set_core_point_objective!(m::JuMP.Model, delta)
    @objective(m, Max, delta)
    return nothing
end

"""
    set_completion_objective!(m, phi_core_expr, objective_mode)

`objective_mode=:maximize_core` maximizes `Phi(core; d)` (the Magnanti-Wong-style refinement);
`:zero` uses a flat zero objective (the `:zero_completion` baseline -- any dual-feasible
completion tight at the hat point).
"""
function set_completion_objective!(m::JuMP.Model, phi_core_expr::AffExpr, objective_mode::Symbol)
    if objective_mode == :maximize_core
        @objective(m, Max, phi_core_expr)
    else
        @objective(m, Max, 0.0)
    end
    return nothing
end
