# =============================================================================
# ClusteringTwoStageStationModel
# =============================================================================

function build_model(
        model::ClusteringTwoStageStationModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    mapping = create_map(model, data)
    S = length(data.scenarios)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    total_endpoint_groups = sum(length(mapping.I_s[s]) for s in 1:S)
    extra_counts["total_endpoint_groups"] = total_endpoint_groups

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)

    set_clustering_two_stage_station_objective!(m, data, mapping)

    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
