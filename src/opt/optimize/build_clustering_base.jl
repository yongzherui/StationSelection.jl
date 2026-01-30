# =============================================================================
# ClusteringBaseModel
# =============================================================================

function build_model(
        model::ClusteringBaseModel,
        data::StationSelectionData,
        optimizer_env
    )::Model
    m, _, _, _ = build_model_with_counts(model, data, optimizer_env)
    return m
end

function build_model_with_counts(
        model::ClusteringBaseModel,
        data::StationSelectionData,
        optimizer_env
    )::Tuple{Model, Dict{String, Int}, Dict{String, Int}, Dict{String, Int}}

    # Create the clustering base map
    mapping = create_clustering_base_map(model, data)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    # Count total requests
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

    return m, variable_counts, constraint_counts, extra_counts
end
