# =============================================================================
# RouteAlphaCapacityModel and RouteVehicleCapacityModel
# =============================================================================

"""
    build_model(model::RouteAlphaCapacityModel, data::StationSelectionData;
                optimizer_env=nothing) -> BuildResult

Build the MILP for RouteAlphaCapacityModel (old formulation).

Variables: y (build), z (activate), x (assignment), θ_s (route activation).
The route-covering constraint is time-indexed, calling `compute_alpha_r_jkt`
(placeholder — will error until implemented).
"""
function build_model(
        model::RouteAlphaCapacityModel,
        data::StationSelectionData;
        optimizer_env = nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    mapping = create_map(model, data)

    S = length(data.scenarios)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts   = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts      = Dict{String, Int}()

    # Extra counts
    n_routes = sum(length(mapping.routes_s[s]) for s in 1:S; init = 0)
    total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:S; init = 0)

    extra_counts["n_routes"]       = n_routes
    extra_counts["total_od_pairs"] = total_od_pairs

    # ==========================================================================
    # Variables
    # ==========================================================================

    variable_counts["station_selection"]   = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"]          = add_assignment_variables!(m, data, mapping)
    variable_counts["route_theta"]         = add_route_theta_variables!(m, data, mapping)

    # ==========================================================================
    # Objective
    # ==========================================================================

    set_route_od_objective!(m, data, mapping;
        route_regularization_weight = model.route_regularization_weight,
        route_var_name = :theta_s)

    # ==========================================================================
    # Constraints
    # ==========================================================================

    constraint_counts["station_limit"] =
        add_station_limit_constraint!(m, data, model.l; equality = true)
    constraint_counts["scenario_activation_limit"] =
        add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] =
        add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] =
        add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] =
        add_assignment_to_active_constraints!(m, data, mapping)
    constraint_counts["route_capacity"] =
        add_route_capacity_constraints!(m, data, mapping, model)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end


"""
    build_model(model::RouteVehicleCapacityModel, data::StationSelectionData;
                optimizer_env=nothing) -> BuildResult

Build the MILP for RouteVehicleCapacityModel (new formulation).

Variables: y (build), z (activate), x (assignment),
           d_{jkts} (induced demand, Z+), α^r_{jkts} (route serving, Z+),
           θ^r_{ts} (route deployments, Z+).

Three covering constraints enforce demand definition, demand coverage by routes,
and vehicle capacity per route segment.
"""
function build_model(
        model::RouteVehicleCapacityModel,
        data::StationSelectionData;
        optimizer_env = nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    mapping = create_map(model, data)  # VehicleCapacityODMap

    S = length(data.scenarios)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts   = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts      = Dict{String, Int}()

    # Extra counts
    n_routes = sum(
        sum(length(v) for v in values(mapping.routes_s[s]); init = 0)
        for s in 1:S; init = 0
    )
    total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:S; init = 0)

    extra_counts["n_routes"]       = n_routes
    extra_counts["total_od_pairs"] = total_od_pairs

    # Store vehicle capacity in model dict for constraint function access
    m[:vehicle_capacity] = model.vehicle_capacity

    # ==========================================================================
    # Variables
    # ==========================================================================

    variable_counts["station_selection"]   = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"]          = add_assignment_variables!(m, data, mapping)
    variable_counts["d_jkts"]             = add_d_jkts_variables!(m, data, mapping)
    variable_counts["alpha_r_jkts"]       = add_alpha_r_jkts_variables!(m, data, mapping)
    variable_counts["theta_r_ts"]         = add_theta_r_ts_variables!(m, data, mapping)

    # ==========================================================================
    # Objective
    # ==========================================================================

    set_route_od_objective!(m, data, mapping;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time)

    # ==========================================================================
    # Constraints
    # ==========================================================================

    constraint_counts["station_limit"] =
        add_station_limit_constraint!(m, data, model.l; equality = true)
    constraint_counts["scenario_activation_limit"] =
        add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] =
        add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] =
        add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] =
        add_assignment_to_active_constraints!(m, data, mapping)
    if model.use_lazy_constraints
        constraint_counts["route_capacity"] =
            add_route_capacity_lazy_constraints!(m, data, mapping)
    else
        constraint_counts["route_capacity"] =
            add_route_capacity_constraints!(m, data, mapping)
    end

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
