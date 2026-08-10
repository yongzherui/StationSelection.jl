"""
    add_benders_optimality_cut!(master, theta, cut_id, rhs_expr) -> ConstraintRef

Attaches one Benders optimality cut `theta[cut_id] >= rhs_expr` to a persisted master model --
the single line every cut-derivation/seeding call site used to write directly via raw
`@constraint` (`y_mw_cut.jl`'s `_add_aggregate_od_route_benders_y_optimality_cut!`,
`yz_mw_cut.jl`'s `_add_aggregate_od_route_benders_yz_optimality_cut!`, `yzh.jl`'s
`_add_aggregate_od_route_benders_yzh_optimality_cut!`, `generic_runner.jl`'s `BendersXY` cut,
`direct_enumeration_guide.jl`'s `_seed_y_cuts!`/`_seed_yz_cuts!`). Each site still builds its
own `rhs_expr` -- the algebraic form differs enough across decompositions (e.g. `BendersYZ`'s
`:standard` cut sums `rho*(chain_cache - z_hat)` while its restricted mode sums
`beta*chain_cache` against a separately-folded constant) that unifying construction risks a
floating-point summation-order change; only this final attachment step is shared.
"""
function add_benders_optimality_cut!(master::JuMP.Model, theta, cut_id::Int, rhs_expr)
    return @constraint(master, theta[cut_id] >= rhs_expr)
end
