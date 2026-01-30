function build_model(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        count::Bool=false
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Compute detour combinations first (needed for mapping creation)
    Xi_same_source = find_same_source_detour_combinations(model, data)
    Xi_same_dest = find_same_dest_detour_combinations(model, data)

    # Create the pooling scenario map with Xi for feasible detour computation
    mapping = model.use_walking_distance_limit ?
        create_pooling_scenario_origin_dest_time_map(
            model,
            data;
            Xi_same_source=Xi_same_source,
            Xi_same_dest=Xi_same_dest
        ) :
        create_pooling_scenario_origin_dest_time_map_no_walking_limit(model, data)

    detour_combos = DetourComboData(Xi_same_source, Xi_same_dest)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    if count
        extra_counts["same_source"] = length(Xi_same_source)
        extra_counts["same_dest"] = length(Xi_same_dest)
    end

    # Add variables
    if count
        variable_counts["station_selection"] = add_station_selection_variables!(m, data)
        variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
        variable_counts["flow"] = add_flow_variables!(m, data, mapping)
    else
        add_station_selection_variables!(m, data)
        add_scenario_activation_variables!(m, data)
        add_flow_variables!(m, data, mapping)
    end

    # Use sparse assignment variables when walking limits are enabled
    if has_walking_distance_limit(mapping)
        if count
            variable_counts["assignment"] = add_assignment_variables_with_walking_distance_limit!(m, data, mapping)
        else
            add_assignment_variables_with_walking_distance_limit!(m, data, mapping)
        end
    else
        if count
            variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
        else
            add_assignment_variables!(m, data, mapping)
        end
    end

    if count
        variable_counts["detour"] = add_detour_variables!(m, data, mapping, Xi_same_source, Xi_same_dest)
    else
        add_detour_variables!(m, data, mapping, Xi_same_source, Xi_same_dest)
    end

    # Add constraints
    if count
        constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
        constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
        constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
        constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
        constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(m, data, mapping)
        constraint_counts["assignment_to_flow"] = add_assignment_to_flow_constraints!(m, data, mapping)
        constraint_counts["assignment_to_same_source_detour"] =
            add_assignment_to_same_source_detour_constraints!(m, data, mapping, Xi_same_source)
        constraint_counts["assignment_to_same_dest_detour"] =
            add_assignment_to_same_dest_detour_constraints!(m, data, mapping, Xi_same_dest)
    else
        add_station_limit_constraint!(m, data, model.l; equality=true)
        add_scenario_activation_limit_constraints!(m, data, model.k)
        add_activation_linking_constraints!(m, data)
        add_assignment_constraints!(m, data, mapping)
        add_assignment_to_active_constraints!(m, data, mapping)
        add_assignment_to_flow_constraints!(m, data, mapping)
        add_assignment_to_same_source_detour_constraints!(m, data, mapping, Xi_same_source)
        add_assignment_to_same_dest_detour_constraints!(m, data, mapping, Xi_same_dest)
    end

    # Set objective
    if has_walking_distance_limit(mapping)
        set_two_stage_single_detour_objective!(m, data, mapping, Xi_same_source, Xi_same_dest;
                                                routing_weight=model.routing_weight)
    else
        set_two_stage_single_detour_objective_no_walking_limit!(m, data, mapping, Xi_same_source, Xi_same_dest;
                                                routing_weight=model.routing_weight)
    end

    counts = count ? ModelCounts(variable_counts, constraint_counts, extra_counts) : nothing
    return BuildResult(m, mapping, detour_combos, counts, Dict{String, Any}())
end
