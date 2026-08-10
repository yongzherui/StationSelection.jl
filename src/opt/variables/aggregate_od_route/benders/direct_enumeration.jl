"""
    add_direct_enumeration_guide_variables!(m, full_pool, n_scenarios; relax_integrality=false) -> theta_direct

`BendersSolver(direct_enumeration_guide=true)`'s exact-routing-cost guide term
(`direct_enumeration_guide.jl`'s `_add_direct_enumeration_guide!`): one `theta_direct[idx, s]`
per `(enumerated column, scenario)`, `Bin` by default or `[0,1]`-bounded continuous when
`relax_integrality=true`. Distinct shape from `add_benders_lambda_variables!` (which is
unbounded-above continuous, not `[0,1]`-bounded) since this is a route-selection indicator over
the *complete* enumerated universe, not a route-activation count.
"""
function add_direct_enumeration_guide_variables!(
    m::JuMP.Model,
    full_pool::Vector{AggregateODRouteColumn},
    n_scenarios::Int;
    relax_integrality::Bool = false,
)
    return relax_integrality ?
        @variable(m, [1:length(full_pool), 1:n_scenarios], lower_bound = 0.0, upper_bound = 1.0) :
        @variable(m, [1:length(full_pool), 1:n_scenarios], Bin)
end
