function _build_compatibility_set_core!(
        model::AnyCompatibilitySetModel,
        data::StationSelectionData,
        optimizer_env;
        relax_integrality::Bool,
        coverage_equality::Bool,
    )::BuildResult
    mapping = create_map(model, data)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    S = n_scenarios(data)
    extra_counts["total_od_pairs"] = sum(length(mapping.Omega_s[s]) for s in 1:S; init=0)
    extra_counts["active_od_pairs"] = sum(length(mapping.active_jk_s[s]) for s in 1:S; init=0)
    extra_counts["compatibility_columns"] = length(mapping.columns)

    m[:compatibility_route_regularization_weight] = model.route_regularization_weight
    m[:compatibility_repositioning_time] = model.repositioning_time
    m[:compatibility_relax_integrality] = relax_integrality
    m[:compatibility_station_budget] = model.l
    m[:compatibility_max_wait_time] = model.max_wait_time
    m[:compatibility_detour_factor] = model.detour_factor
    m[:compatibility_max_stops] = model.max_stops
    m[:compatibility_max_visits_per_node] = model.max_visits_per_node
    m[:compatibility_max_new_columns] = model.max_new_columns
    m[:compatibility_n_candidates] = model.n_candidates
    m[:compatibility_pricing_time_limit_sec] = model.pricing_time_limit_sec
    m[:compatibility_reduced_cost_tol] = model.reduced_cost_tol

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
    variable_counts["od_activation"] = add_od_activation_variables!(
        m,
        data,
        mapping;
        relax_integrality=relax_integrality,
    )
    variable_counts["compatibility_theta"] = add_compatibility_theta_variables!(
        m,
        data,
        mapping;
        relax_integrality=relax_integrality,
    )

    if relax_integrality
        _relax_compatibility_station_and_assignment!(m)
    end

    set_compatibility_set_objective!(
        m,
        data,
        mapping;
        route_regularization_weight=model.route_regularization_weight,
        repositioning_time=model.repositioning_time,
    )

    constraint_counts["station_limit"] =
        add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["assignment"] =
        add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_selected"] =
        add_assignment_to_selected_constraints!(m, data, mapping)
    constraint_counts["assignment_to_od_activation"] =
        add_assignment_to_od_activation_constraints!(m, data, mapping)
    constraint_counts["compatibility_coverage"] =
        add_compatibility_coverage_constraints!(m, data, mapping; equality=coverage_equality)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end

function build_model(
        model::CompatibilitySetModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        relax_integrality::Bool=model.relax_integrality,
    )::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    return _build_compatibility_set_core!(
        model, data, optimizer_env;
        relax_integrality=relax_integrality,
        coverage_equality=false,
    )
end

function build_model(
        model::CompatibilitySetAssignmentModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        relax_integrality::Bool=model.relax_integrality,
    )::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    return _build_compatibility_set_core!(
        model, data, optimizer_env;
        relax_integrality=relax_integrality,
        coverage_equality=true,
    )
end

function _relax_compatibility_station_and_assignment!(m::Model)
    for y_var in m[:y]
        unset_binary(y_var)
        set_lower_bound(y_var, 0.0)
        set_upper_bound(y_var, 1.0)
    end
    for scenario_dict in m[:x]
        for x_vec in values(scenario_dict)
            for x_var in x_vec
                unset_integer(x_var)
            end
        end
    end
    return nothing
end
