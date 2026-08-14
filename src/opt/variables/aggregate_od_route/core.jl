"""
Variables for AggregateODRouteProblem.
"""

export add_aggregate_od_route_theta_variables!

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
