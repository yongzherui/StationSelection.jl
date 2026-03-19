# =============================================================================
# RouteAlphaCapacityModel and RouteVehicleCapacityModel
# =============================================================================

"""
    build_model(model::Union{RouteAlphaCapacityModel,RouteVehicleCapacityModel},
                data::StationSelectionData; optimizer_env=nothing) -> BuildResult

Build the MILP for RouteAlphaCapacityModel or RouteVehicleCapacityModel.

Both models share the same constraint structure (non-temporal route covering).
The difference in capacity coefficient (α vs C) is encoded in `RouteODMap.routes_by_jks`
at map-construction time.
"""
function build_model(
        model::Union{RouteAlphaCapacityModel, RouteVehicleCapacityModel},
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
        route_regularization_weight = model.route_regularization_weight)

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
        add_route_capacity_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
