"""
Variables for AggregateODRouteProblem.
"""

export add_aggregate_od_route_theta_variables!
export add_benders_cut_placeholder_variables!

"""
    add_benders_cut_placeholder_variables!(m::Model, cut_ids) -> Int

Adds the Benders master's cut-placeholder variable `theta[cut_ids] >= 0.0`
(registered as `m[:theta]`) -- shared across every `AggregateODRouteProblem`
Benders decomposition (`BendersY`/`BendersXY`/`BendersYZ`/`BendersYZH`/
`BranchAndBendersSolver`), which otherwise each declare this identical
variable inline. Distinct from `theta_compat`
([`add_aggregate_od_route_theta_variables!`](@ref)), the compact model's
per-route-column variable -- no key collision.
"""
function add_benders_cut_placeholder_variables!(m::Model, cut_ids)::Int
    before = JuMP.num_variables(m)
    @variable(m, theta[cut_ids] >= 0.0)
    return JuMP.num_variables(m) - before
end

"""
    add_aggregate_od_route_theta_variables!(m, data, mapping; relax_integrality=false, scenarios=1:n_scenarios(data)) -> Int

`scenarios` restricts variable creation to a subset of scenarios -- used by
`AggregateODRouteBendersYXFormulation`'s subproblem, which is solved one scenario at a
time (default keeps every existing caller's behavior unchanged).
"""
function add_aggregate_od_route_theta_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    relax_integrality::Bool=false,
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
)::Int
    before = JuMP.num_variables(m)
    theta = Dict{Tuple{Int, Int}, VariableRef}()
    for column in mapping.columns
        for s in scenarios
            if relax_integrality
                # The upper bound is primal-redundant for this positive-cost set-covering LP:
                # every coverage RHS is at most one, so reducing theta > 1 to one preserves
                # feasibility and cannot increase cost.  Keeping the bound lets its bound dual
                # absorb reduced cost, which makes coverage duals alone invalid for Benders/MW
                # completion.  Use the equivalent standard nonnegative covering relaxation.
                theta[(column.id, s)] = @variable(m, lower_bound = 0.0)
            else
                theta[(column.id, s)] = @variable(m, binary = true)
            end
        end
    end
    m[:theta_compat] = theta
    return JuMP.num_variables(m) - before
end
