# =============================================================================
# TwoStageSingleDetourNoWalkingLimitModel
# =============================================================================

function build_model(
        model::TwoStageSingleDetourNoWalkingLimitModel,
        data::StationSelectionData,
        optimizer_env
    )::Model
    m, _, _, _ = build_model_with_counts(model, data, optimizer_env)
    return m
end

function build_model_with_counts(
        model::TwoStageSingleDetourNoWalkingLimitModel,
        data::StationSelectionData,
        optimizer_env
    )::Tuple{Model, Dict{String, Int}, Dict{String, Int}, Dict{String, Int}}

    # Create the pooling scenario map (no walking limit)
    mapping = create_pooling_scenario_origin_dest_time_map_no_walking_limit(model, data)

    # Compute detour combinations
    Xi_same_source = find_same_source_detour_combinations(model, data)
    Xi_same_dest = find_same_dest_detour_combinations(model, data)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    detour_combo_counts = Dict{String, Int}()
    detour_combo_counts["same_source"] = length(Xi_same_source)
    detour_combo_counts["same_dest"] = length(Xi_same_dest)

    # Add variables
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["flow"] = add_flow_variables!(m, data, mapping)
    if has_walking_distance_limit(mapping)
        variable_counts["assignment"] = add_assignment_variables_with_walking_distance_limit!(m, data, mapping)
    else
        variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
    end
    variable_counts["detour"] = add_detour_variables!(m, data, mapping, Xi_same_source, Xi_same_dest)

    # Add constraints
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(m, data, mapping)
    constraint_counts["assignment_to_flow"] = add_assignment_to_flow_constraints!(m, data, mapping)
    constraint_counts["assignment_to_same_source_detour"] = add_assignment_to_same_source_detour_constraints!(m, data, mapping, Xi_same_source)
    constraint_counts["assignment_to_same_dest_detour"] = add_assignment_to_same_dest_detour_constraints!(m, data, mapping, Xi_same_dest)

    # Set objective
    set_two_stage_single_detour_objective_no_walking_limit!(m, data, mapping, Xi_same_source, Xi_same_dest;
                                            routing_weight=model.routing_weight)

    return m, variable_counts, constraint_counts, detour_combo_counts
end
