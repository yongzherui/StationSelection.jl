# =============================================================================
# TransportationModel (zone-pair anchor transportation flow)
# =============================================================================

function build_model(
        model::TransportationModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Create the transportation map
    mapping = create_map(model, data; optimizer_env=optimizer_env)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    extra_counts["n_clusters"] = mapping.n_clusters
    extra_counts["n_active_anchors"] = length(mapping.active_anchors)

    # Total trips across all anchors/scenarios
    total_trips = 0
    for (g_idx, _) in enumerate(mapping.active_anchors)
        for s in mapping.anchor_scenarios[g_idx]
            total_trips += Int(mapping.M_gs[(g_idx, s)])
        end
    end
    extra_counts["total_trips"] = total_trips

    # ==========================================================================
    # Variables
    # ==========================================================================

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["transportation_assignment"] = add_transportation_assignment_variables!(m, data, mapping)

    # Register :x as alias for x_pick so run_opt can extract solution via m[:x]
    m[:x] = m[:x_pick]

    variable_counts["transportation_aggregation"] = add_transportation_aggregation_variables!(m, data, mapping)
    variable_counts["transportation_flow"] = add_transportation_flow_variables!(m, data, mapping)
    variable_counts["transportation_activation"] = add_transportation_activation_variables!(m, data, mapping)

    # ==========================================================================
    # Objective
    # ==========================================================================

    set_transportation_objective!(
        m,
        data,
        mapping;
        in_vehicle_time_weight=model.in_vehicle_time_weight,
        activation_cost=model.activation_cost
    )

    # ==========================================================================
    # Constraints
    # ==========================================================================

    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)

    constraint_counts["transportation_assignment"] = add_transportation_assignment_constraints!(m, data, mapping)
    constraint_counts["transportation_aggregation"] = add_transportation_aggregation_constraints!(m, data, mapping)
    constraint_counts["transportation_flow_conservation"] = add_transportation_flow_conservation_constraints!(m, data, mapping)
    constraint_counts["transportation_flow_activation"] = add_transportation_flow_activation_constraints!(m, data, mapping)
    constraint_counts["transportation_viability"] = add_transportation_viability_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)

    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
