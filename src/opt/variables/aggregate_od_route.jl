"""
Variables for AggregateODRouteModel.
"""

export add_aggregate_od_route_theta_variables!

function add_aggregate_od_route_theta_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    relax_integrality::Bool=false,
)::Int
    before = JuMP.num_variables(m)
    theta = Dict{Tuple{Int, Int}, VariableRef}()
    for column in mapping.columns
        for s in 1:n_scenarios(data)
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
