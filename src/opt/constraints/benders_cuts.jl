"""
Benders optimality cuts -- the rows that teach a master's `Theta` placeholders
(`variables/benders_cuts.jl`) what the second stage actually costs.

Generic over formulations, for the same reason the placeholder variables are: the row
shape `Theta >= constant - sum_j coefficient_j * y_j` is the shape of every LP-duality
Benders cut, whatever the subproblem was.
"""

using JuMP

export add_benders_optimality_cut!

"""
    add_benders_optimality_cut!(m, theta, constant, y, coefficients) -> ConstraintRef

Add `theta >= constant - sum_j coefficients[j] * y[j]`.

This is the standard LP-duality cut: `constant` is the subproblem dual objective's
`y`-independent part and `coefficients[j]` is the (sign-flipped) dual weight on
`y_j`, so the row evaluates to the subproblem's optimal value at the `y` it was derived
at and underestimates it everywhere else. Validity comes from the subproblem's dual
feasible region not depending on `y` -- see
`optimize/aggregate_od_route/benders/subproblem.jl` for the derivation in this
decomposition's own terms, including why no feasibility cuts are needed.

`coefficients` is sparse (a `Dict{Int, Float64}`): a station appearing in no linking row
of this scenario has a zero coefficient and is simply absent, which for a cut over a few
hundred stations of which a handful matter is most of the row.
"""
function add_benders_optimality_cut!(
        m::Model,
        theta::VariableRef,
        constant::Float64,
        y::Vector{VariableRef},
        coefficients::Dict{Int, Float64},
    )::ConstraintRef
    rhs = AffExpr(constant)
    for (j, coefficient) in coefficients
        coefficient == 0.0 && continue
        add_to_expression!(rhs, -coefficient, y[j])
    end
    return @constraint(m, theta >= rhs)
end
