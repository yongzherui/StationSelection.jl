# =============================================================================
# ClusteringTwoStageFormulation
# =============================================================================

function build_model(
        problem::StationSelectionProblem,
        formulation::ClusteringTwoStageFormulation,
        solver::DirectMIPSolver,
    )::BuildResult
    formulation.l <= problem.k || throw(ArgumentError(
        "l=$(formulation.l) exceeds k=$(problem.k)"
    ))

    # ---- 1. Parameters ----
    data = problem.data
    mapping = create_map(problem, formulation, data)

    m = Model(() -> Gurobi.Optimizer())

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    S = length(data.scenarios)
    total_endpoint_groups = sum(length(mapping.I_s[s]) for s in 1:S)
    extra_counts["total_endpoint_groups"] = total_endpoint_groups

    # ---- 2. Variables ----
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)

    # ---- 3. Objective ----
    set_clustering_two_stage_station_objective!(m, data, mapping)

    # ---- 4. Constraints ----
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, problem.k; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, formulation.l)
    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
