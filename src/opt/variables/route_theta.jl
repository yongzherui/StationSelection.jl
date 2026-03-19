"""
Route activation variables for TwoStageRouteWithTimeModel.

θ_s[s][r] ∈ {0,1}, stored as `m[:theta_s]` (Vector{Vector{VariableRef}}).
"""

export add_route_theta_variables!

"""
    add_route_theta_variables!(m, data, mapping::TwoStageRouteODMap) -> Int

Add binary route activation variables θ_s[s][r] for all scenarios.

Stored as `m[:theta_s]`, a `Vector{Vector{VariableRef}}` where `m[:theta_s][s][r]`
is the variable for route r of scenario s.

Returns the number of variables added.
"""
function add_route_theta_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::TwoStageRouteODMap
)::Int
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    theta_s = [VariableRef[] for _ in 1:S]
    for s in 1:S
        n_r = length(mapping.routes_s[s])
        for _ in 1:n_r
            push!(theta_s[s], @variable(m, binary = true))
        end
    end
    m[:theta_s] = theta_s

    return JuMP.num_variables(m) - before
end


"""
    add_route_theta_variables!(m, data, mapping::RouteODMap) -> Int

Add binary route activation variables θ_s[s][r] for RouteAlphaCapacityModel /
RouteVehicleCapacityModel.

Stored as `m[:theta_s]`, a `Vector{Vector{VariableRef}}` where `m[:theta_s][s][r]`
is the variable for route r of scenario s.
"""
function add_route_theta_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::RouteODMap
)::Int
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    theta_s = [VariableRef[] for _ in 1:S]
    for s in 1:S
        n_r = length(mapping.routes_s[s])
        for _ in 1:n_r
            push!(theta_s[s], @variable(m, binary = true))
        end
    end
    m[:theta_s] = theta_s

    return JuMP.num_variables(m) - before
end
