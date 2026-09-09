"""
The Benders master objective: the sum of the cut placeholders, plus whatever first-stage
cost the decomposition has.

Generic over formulations (see `variables/benders_cuts.jl` for why).
"""

export set_benders_master_objective!

"""
    set_benders_master_objective!(m, theta; first_stage_cost=AffExpr(0.0))

`min sum(Theta) + first_stage_cost`.

`first_stage_cost` defaults to zero because the decomposition this was written for has
none: in the joint routing+assignment model every cost term (walking, route columns) is
second-stage, so the master's objective is the placeholders alone. That is what makes the
placeholders' lower bound mandatory (see `add_benders_cut_variables!`) and what makes the
first few masters degenerate -- any feasible `y` is optimal at `Theta == 0` until cuts
arrive.
"""
function set_benders_master_objective!(
        m::Model,
        theta::Dict{Int, VariableRef};
        first_stage_cost::AffExpr=AffExpr(0.0),
    )
    obj = copy(first_stage_cost)
    for group in sort!(collect(keys(theta)))
        add_to_expression!(obj, theta[group])
    end
    @objective(m, Min, obj)
    return nothing
end
