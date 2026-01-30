# =============================================================================
# ClusteringTwoStageODModel
# =============================================================================

function build_model(
        model::ClusteringTwoStageODModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        count::Bool=false
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Create the clustering scenario OD map
    mapping = create_clustering_scenario_od_map(model, data)

    S = length(data.scenarios)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    if count
        total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:S)
        extra_counts["total_od_pairs"] = total_od_pairs
    end

    # ==========================================================================
    # Variables (reusing shared functions where possible)
    # ==========================================================================

    if count
        variable_counts["station_selection"] = add_station_selection_variables!(m, data)
        variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    else
        add_station_selection_variables!(m, data)
        add_scenario_activation_variables!(m, data)
    end

    if count
        variable_counts["assignment"] = add_assignment_variables!(
            m, data, mapping; variable_reduction=model.variable_reduction
        )
    else
        add_assignment_variables!(
            m, data, mapping; variable_reduction=model.variable_reduction
        )
    end

    # ==========================================================================
    # Objective
    # ==========================================================================

    set_clustering_od_objective!(
        m,
        data,
        mapping;
        routing_weight=model.routing_weight,
        variable_reduction=model.variable_reduction
    )

    # ==========================================================================
    # Constraints (reusing shared functions where possible)
    # ==========================================================================

    if count
        constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
        constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
        constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
        constraint_counts["assignment"] = add_assignment_constraints!(
            m, data, mapping; variable_reduction=model.variable_reduction
        )
        constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(
            m, data, mapping; variable_reduction=model.variable_reduction
        )
    else
        add_station_limit_constraint!(m, data, model.l; equality=true)
        add_scenario_activation_limit_constraints!(m, data, model.k)
        add_activation_linking_constraints!(m, data)
        add_assignment_constraints!(
            m, data, mapping; variable_reduction=model.variable_reduction
        )
        add_assignment_to_active_constraints!(
            m, data, mapping; variable_reduction=model.variable_reduction
        )
    end

    if model.use_walking_distance_limit && !model.variable_reduction
        if count
            constraint_counts["walking_limit_origin_dest"] = add_assignment_walking_limit_constraints!(
                m, data, mapping, model.max_walking_distance
            )
        else
            add_assignment_walking_limit_constraints!(
                m, data, mapping, model.max_walking_distance
            )
        end
    end

    counts = count ? ModelCounts(variable_counts, constraint_counts, extra_counts) : nothing
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
