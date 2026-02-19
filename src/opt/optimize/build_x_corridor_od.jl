# =============================================================================
# XCorridorODModel (x-based corridor activation)
# =============================================================================

function build_model(
        model::XCorridorODModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Create the corridor scenario OD map
    mapping = create_map(model, data; optimizer_env=optimizer_env)

    S = length(data.scenarios)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:S)
    extra_counts["total_od_pairs"] = total_od_pairs
    extra_counts["n_clusters"] = mapping.n_clusters
    extra_counts["n_corridors"] = length(mapping.corridor_indices)

    # ==========================================================================
    # Variables (no α — corridor activation is x-based)
    # ==========================================================================

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(
        m, data, mapping; variable_reduction=model.variable_reduction
    )
    variable_counts["corridor"] = add_corridor_variables!(m, data, mapping)

    # ==========================================================================
    # Objective
    # ==========================================================================

    set_corridor_od_objective!(
        m,
        data,
        mapping;
        in_vehicle_time_weight=model.in_vehicle_time_weight,
        corridor_weight=model.corridor_weight,
        variable_reduction=model.variable_reduction
    )

    # ==========================================================================
    # Constraints
    # ==========================================================================

    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)

    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)

    constraint_counts["assignment"] = add_assignment_constraints!(
        m, data, mapping; variable_reduction=model.variable_reduction
    )
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(
        m, data, mapping; variable_reduction=model.variable_reduction, tight_constraints=model.tight_constraints
    )

    # x-based corridor activation: f_{gs} ≥ x_{odjks} for j∈C_a, k∈C_b
    constraint_counts["corridor_x_activation"] = add_corridor_x_activation_constraints!(
        m, data, mapping; variable_reduction=model.variable_reduction
    )

    if model.use_walking_distance_limit && !model.variable_reduction
        constraint_counts["walking_limit_origin_dest"] = add_assignment_walking_limit_constraints!(
            m, data, mapping, model.max_walking_distance
        )
    end

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)

    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
