function build_model(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Compute detour combinations first (needed for mapping creation)
    Xi_same_source = find_same_source_detour_combinations(model, data)
    Xi_same_dest = find_same_dest_detour_combinations(model, data)

    # Create the pooling scenario map with Xi for feasible detour computation
    mapping = create_map(
        model,
        data;
        Xi_same_source=Xi_same_source,
        Xi_same_dest=Xi_same_dest
    )

    detour_combos = DetourComboData(Xi_same_source, Xi_same_dest)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    extra_counts["same_source"] = length(Xi_same_source)
    extra_counts["same_dest"] = length(Xi_same_dest)

    # Add variables
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["flow"] = add_flow_variables!(m, data, mapping)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
    variable_counts["detour"] = add_detour_variables!(m, data, mapping, Xi_same_source, Xi_same_dest)

    # Add constraints
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(
        m, data, mapping; tight_constraints=model.tight_constraints
    )
    constraint_counts["assignment_to_flow"] = add_assignment_to_flow_constraints!(m, data, mapping)
    constraint_counts["assignment_to_same_source_detour"] =
        add_assignment_to_same_source_detour_constraints!(
            m, data, mapping, Xi_same_source; tight_constraints=model.tight_constraints
        )
    constraint_counts["assignment_to_same_dest_detour"] =
        add_assignment_to_same_dest_detour_constraints!(
            m, data, mapping, Xi_same_dest; tight_constraints=model.tight_constraints
        )

    # Set objective
    set_two_stage_single_detour_objective!(m, data, mapping, Xi_same_source, Xi_same_dest;
                                            routing_weight=model.routing_weight)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, detour_combos, counts, Dict{String, Any}())
end
