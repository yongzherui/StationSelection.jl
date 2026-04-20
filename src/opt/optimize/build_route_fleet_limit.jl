"""
    _build_fleet_limit_milp(model, data, mapping; optimizer_env) -> BuildResult

Build the RouteFleetLimitModel JuMP problem from a pre-built FleetLimitODMap.
Separated from `build_model` so the fleet search can reuse the same map
(route generation + delay coefficients) across fleet-size iterations.
"""
function _build_fleet_limit_milp(
        model       :: RouteFleetLimitModel,
        data        :: StationSelectionData,
        mapping     :: FleetLimitODMap;
        optimizer_env = nothing
    ) :: BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    inner = mapping.inner
    S = length(data.scenarios)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts   = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts      = Dict{String, Int}()

    n_routes = sum(
        sum(length(v) for v in values(inner.routes_s[s]); init = 0)
        for s in 1:S; init = 0
    )
    total_od_pairs = sum(length(inner.Omega_s[s]) for s in 1:S; init = 0)

    extra_counts["n_routes"]       = n_routes
    extra_counts["total_od_pairs"] = total_od_pairs
    extra_counts["fleet_size"]     = mapping.fleet_size

    m[:vehicle_capacity] = model.vehicle_capacity

    variable_counts["station_selection"]   = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"]          = add_assignment_variables!(m, data, inner)
    variable_counts["alpha_r_jkts"]       = add_alpha_r_jkts_variables!(m, data, inner;
        integer_alpha = false)
    variable_counts["theta_r_ts"]         = add_theta_r_ts_variables!(m, data, inner)
    variable_counts["v_jkts"]             = add_v_jkts_variables!(m, data, mapping)

    set_fleet_limit_objective!(m, data, mapping;
        route_regularization_weight = model.route_regularization_weight,
        delay_weight                = model.delay_weight,
        repositioning_time          = model.repositioning_time,
        unmet_demand_penalty        = model.unmet_demand_penalty)

    constraint_counts["station_limit"] =
        add_station_limit_constraint!(m, data, model.l; equality = true)
    constraint_counts["scenario_activation_limit"] =
        add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] =
        add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] =
        add_assignment_constraints!(m, data, inner)
    constraint_counts["assignment_to_active"] =
        add_assignment_to_active_constraints!(m, data, inner)
    constraint_counts["route_capacity"] =
        add_route_capacity_constraints!(m, data, inner)
    constraint_counts["fleet_limit"] =
        add_fleet_limit_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end


"""
    build_model(model::RouteFleetLimitModel, data; optimizer_env) -> BuildResult

Build the MILP for RouteFleetLimitModel.

Extends RouteVehicleCapacityModel with:
  - Per-passenger delay cost μ · w_delay · d^r_{jk} · α^r_{jkts} in the objective
  - Unmet demand variable v_{jkts} ∈ ℤ₊
  - Route-linking as equality: Σx = v + Σ_r α
  - Fleet-size constraint: Σ_r θ^r_{ts} ≤ F  ∀ t, s
"""
function build_model(
        model::RouteFleetLimitModel,
        data::StationSelectionData;
        optimizer_env = nothing
    )::BuildResult

    mapping = create_map(model, data)
    return _build_fleet_limit_milp(model, data, mapping; optimizer_env=optimizer_env)
end
