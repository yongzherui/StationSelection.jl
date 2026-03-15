"""
Route activation variables for TwoStageRouteModel.

Standard mode:  θ[s, r] ∈ {0,1}, stored as `m[:theta]` (2-D JuMP array).
Temporal mode:  θ_s[s][r] ∈ {0,1}, stored as `m[:theta_s]` (Vector{Vector{VariableRef}}).
"""

export add_route_theta_variables!

"""
    add_route_theta_variables!(m, data, mapping::TwoStageRouteODMap) -> Int

Add binary route activation variables for all scenarios.

**Standard mode** (`mapping.routes_s === nothing`):
  Adds θ[s, r] for s ∈ 1:S, r ∈ 1:n_routes. Stored as `m[:theta]`.

**Temporal mode** (`mapping.routes_s !== nothing`):
  Adds one variable per (scenario, per-scenario route). Stored as `m[:theta_s]`,
  a `Vector{Vector{VariableRef}}` where `m[:theta_s][s][r]` is the variable for
  route r of scenario s.

Returns the number of variables added.
"""
function add_route_theta_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::TwoStageRouteODMap
)::Int
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    if is_temporal_mode(mapping)
        # Per-scenario route pools — variable count differs per scenario
        theta_s = [VariableRef[] for _ in 1:S]
        for s in 1:S
            n_r = length(mapping.routes_s[s])
            for _ in 1:n_r
                push!(theta_s[s], @variable(m, binary = true))
            end
        end
        m[:theta_s] = theta_s
    else
        n_routes = length(mapping.routes)
        @variable(m, theta[1:S, 1:n_routes], Bin)
    end

    return JuMP.num_variables(m) - before
end
