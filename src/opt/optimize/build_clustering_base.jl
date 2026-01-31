# =============================================================================
# ClusteringBaseModel
# =============================================================================

function build_model(
        model::ClusteringBaseModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Create the clustering base map
    mapping = create_map(model, data)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    total_requests = sum(values(mapping.request_counts))
    extra_counts["total_requests"] = total_requests

    # ==========================================================================
    # Variables
    # ==========================================================================

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)

    # ==========================================================================
    # Objective
    # ==========================================================================

    set_clustering_base_objective!(m, data, mapping)

    # ==========================================================================
    # Constraints
    # ==========================================================================

    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.k; equality=true)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_selected"] = add_assignment_to_selected_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
