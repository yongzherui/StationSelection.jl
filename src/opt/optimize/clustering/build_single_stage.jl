# =============================================================================
# ClusteringBaseFormulation
# =============================================================================

function build_model(
        problem::StationSelectionProblem,
        formulation::ClusteringBaseFormulation,
        solver::DirectMIPSolver,
    )::BuildResult
    # ---- 1. Parameters ----
    data = problem.data
    mapping = create_map(problem, formulation, data)

    m = Model(() -> Gurobi.Optimizer())

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    total_requests = sum(values(mapping.request_counts))
    extra_counts["total_requests"] = total_requests

    # ---- 2. Variables ----
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)

    # ---- 3. Objective ----
    set_clustering_base_objective!(m, data, mapping)

    # ---- 4. Constraints ----
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, problem.k; equality=true)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_selected"] = add_assignment_to_selected_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
