# =============================================================================
# ClusteringTwoStageODFormulation / ClusteringTwoStageODFlowRegularizerFormulation
# =============================================================================

"""
    _build_clustering_two_stage_od_variables!(m, data, mapping, variable_counts, extra_counts)

Variables shared by both `ClusteringTwoStageODFormulation` and
`ClusteringTwoStageODFlowRegularizerFormulation`'s `build_model` bodies below -- station
selection, scenario activation, and OD assignment. Not a dispatch point (both callers
already know their own concrete type); purely to keep the two `build_model` methods from
duplicating this block verbatim, since `flow_activation`/the objective/the rest of the
constraints genuinely differ between them.
"""
function _build_clustering_two_stage_od_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap,
        variable_counts::Dict{String, Int},
        extra_counts::Dict{String, Int}
    )
    S = length(data.scenarios)
    total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:S)
    extra_counts["total_od_pairs"] = total_od_pairs

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
    return nothing
end

"""
    _build_clustering_two_stage_od_constraints!(m, problem, data, mapping, formulation, constraint_counts)

Constraints shared by both `ClusteringTwoStageODFormulation` and
`ClusteringTwoStageODFlowRegularizerFormulation`'s `build_model` bodies. `flow_activation`
(when present) is added by the caller right after this.
"""
function _build_clustering_two_stage_od_constraints!(
        m::Model,
        problem::StationSelectionProblem,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap,
        formulation::AbstractClusteringTwoStageODFormulation,
        constraint_counts::Dict{String, Int}
    )
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, problem.k; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, formulation.l)
    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(m, data, mapping)
    return nothing
end

function build_model(
        problem::StationSelectionProblem,
        formulation::ClusteringTwoStageODFormulation,
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

    # ---- 2. Variables ----
    _build_clustering_two_stage_od_variables!(m, data, mapping, variable_counts, extra_counts)

    # ---- 3. Objective ----
    set_clustering_od_objective!(
        m,
        data,
        mapping;
        in_vehicle_time_weight=formulation.in_vehicle_time_weight
    )

    # ---- 4. Constraints ----
    _build_clustering_two_stage_od_constraints!(m, problem, data, mapping, formulation, constraint_counts)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end

function build_model(
        problem::StationSelectionProblem,
        formulation::ClusteringTwoStageODFlowRegularizerFormulation,
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

    # ---- 2. Variables ----
    _build_clustering_two_stage_od_variables!(m, data, mapping, variable_counts, extra_counts)
    variable_counts["flow_activation"] = add_flow_variables!(m, data, mapping)

    # ---- 3. Objective ----
    set_clustering_od_flow_regularizer_objective!(
        m,
        data,
        mapping;
        in_vehicle_time_weight=formulation.in_vehicle_time_weight,
        flow_regularization_weight=formulation.flow_regularization_weight
    )

    # ---- 4. Constraints ----
    _build_clustering_two_stage_od_constraints!(m, problem, data, mapping, formulation, constraint_counts)
    constraint_counts["flow_activation"] = add_flow_activation_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
