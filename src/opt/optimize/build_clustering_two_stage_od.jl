# =============================================================================
# ClusteringTwoStageODModel
# =============================================================================

function build_model(
        model::ClusteringTwoStageODModel,
        data::StationSelectionData,
        optimizer_env
    )::Model
    m, _, _, _ = build_model_with_counts(model, data, optimizer_env)
    return m
end

function build_model_with_counts(
        model::ClusteringTwoStageODModel,
        data::StationSelectionData,
        optimizer_env
    )::Tuple{Model, Dict{String, Int}, Dict{String, Int}, Dict{String, Int}}

    # Create the clustering scenario OD map
    mapping = create_clustering_scenario_od_map(model, data)

    S = length(data.scenarios)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    # Count total OD pairs across scenarios
    total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:S)
    extra_counts["total_od_pairs"] = total_od_pairs

    # ==========================================================================
    # Variables (reusing shared functions where possible)
    # ==========================================================================

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    if has_walking_distance_limit(mapping)
        variable_counts["assignment"] = add_assignment_variables_with_walking_distance_limit!(m, data, mapping)
    else
        variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
    end

    # ==========================================================================
    # Objective
    # ==========================================================================

    set_clustering_od_objective!(m, data, mapping; routing_weight=model.routing_weight)

    # ==========================================================================
    # Constraints (reusing shared functions where possible)
    # ==========================================================================

    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(m, data, mapping)

    return m, variable_counts, constraint_counts, extra_counts
end
