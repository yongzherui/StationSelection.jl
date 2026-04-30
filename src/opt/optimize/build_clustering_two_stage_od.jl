# =============================================================================
# ClusteringTwoStageODModel
# =============================================================================

function build_model(
        model::ClusteringTwoStageODModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Create the clustering scenario OD map
    mapping = create_map(model, data)

    S = length(data.scenarios)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:S)
    extra_counts["total_od_pairs"] = total_od_pairs

    # ==========================================================================
    # Variables (reusing shared functions where possible)
    # ==========================================================================

    variable_counts["station_selection"]   = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"]          = add_assignment_variables!(m, data, mapping)
    if !isnothing(model.flow_regularization_weight)
        variable_counts["flow_activation"] = add_flow_variables!(m, data, mapping)
    end

    # ==========================================================================
    # Objective
    # ==========================================================================

    if isnothing(model.flow_regularization_weight)
        set_clustering_od_objective!(m, data, mapping;
            in_vehicle_time_weight=model.in_vehicle_time_weight)
    else
        set_clustering_od_flow_regularizer_objective!(m, data, mapping;
            in_vehicle_time_weight=model.in_vehicle_time_weight,
            flow_regularization_weight=Float64(model.flow_regularization_weight))
    end

    # ==========================================================================
    # Constraints
    # ==========================================================================

    constraint_counts["station_limit"]             = add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"]        = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"]                = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"]      = add_assignment_to_active_constraints!(m, data, mapping)
    if !isnothing(model.flow_regularization_weight)
        constraint_counts["flow_activation"] = add_flow_activation_constraints!(m, data, mapping)
    end

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)

    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
