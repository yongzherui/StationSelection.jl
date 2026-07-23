function _build_aggregate_od_route_core!(
        model::AnyAggregateODRouteModel,
        data::StationSelectionData,
        optimizer_env;
        relax_integrality::Bool,
    )::BuildResult
    base_model = model isa RouteCoveringProblem ? model.base : model
    mapping = create_map(model, data)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    S = n_scenarios(data)
    extra_counts["total_od_pairs"] = sum(length(mapping.Omega_s[s]) for s in 1:S; init=0)
    extra_counts["active_od_pairs"] = sum(length(mapping.active_jk_s[s]) for s in 1:S; init=0)
    extra_counts["aggregate_od_route_columns"] = length(mapping.columns)

    m[:aggregate_od_route_route_regularization_weight] = base_model.route_regularization_weight
    m[:aggregate_od_route_walk_cost_weight] = base_model.walk_cost_weight
    m[:aggregate_od_route_repositioning_time] = base_model.repositioning_time
    m[:aggregate_od_route_relax_integrality] = relax_integrality
    m[:aggregate_od_route_unmet_demand_penalty] = base_model.unmet_demand_penalty
    m[:aggregate_od_route_station_budget] = base_model.l
    m[:aggregate_od_route_max_wait_time] = base_model.max_wait_time
    m[:aggregate_od_route_detour_factor] = base_model.detour_factor
    m[:aggregate_od_route_max_stops] = base_model.max_stops
    m[:aggregate_od_route_max_visits_per_node] = base_model.max_visits_per_node
    m[:aggregate_od_route_max_new_columns] = base_model.max_new_columns
    m[:aggregate_od_route_n_candidates] = base_model.n_candidates
    m[:aggregate_od_route_pricing_time_limit_sec] = base_model.pricing_time_limit_sec
    m[:aggregate_od_route_reduced_cost_tol] = base_model.reduced_cost_tol

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
    variable_counts["aggregate_od_route_theta"] = add_aggregate_od_route_theta_variables!(
        m,
        data,
        mapping;
        relax_integrality=relax_integrality,
    )

    if relax_integrality
        _relax_aggregate_od_route_station_and_assignment!(m)
    end

    set_aggregate_od_route_objective!(
        m,
        data,
        mapping;
        route_regularization_weight=base_model.route_regularization_weight,
        walk_cost_weight=base_model.walk_cost_weight,
        repositioning_time=base_model.repositioning_time,
    )

    constraint_counts["station_limit"] =
        add_station_limit_constraint!(m, data, base_model.l; equality=true)
    constraint_counts["assignment"] =
        add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_selected"] =
        add_assignment_to_selected_constraints!(m, data, mapping)
    if model isa RouteCoveringProblem
        constraint_counts["fixed_open_stations"] =
            add_fixed_open_station_constraints!(m, data, model)
    elseif base_model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy
        if _is_endpoint_nearest_style(base_model.assignment_policy.feasibility_cut_style)
            # Endpoint styles build independent per-endpoint nearest-open selectors
            # (no ranking over station pairs), so they can support direct walking
            # unlike :pair_chain, which ranks station *pairs* jointly and has no
            # walk-only wiring.
            validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=base_model.allow_walk_only)
            constraint_counts["nearest_open_assignment"] =
                add_nearest_open_endpoint_constraints!(
                    m, data, mapping;
                    allow_walk_only=base_model.allow_walk_only,
                    selector_style=base_model.assignment_policy.feasibility_cut_style,
                )
        else
            assert_no_walk_only_pairs(mapping, "NearestOpenAggregateODAssignmentPolicy(:pair_chain)")
            constraint_counts["nearest_open_assignment"] =
                add_nearest_open_assignment_constraints!(m, data, mapping)
        end
    end
    constraint_counts["aggregate_od_route_coverage"] =
        add_aggregate_od_route_coverage_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end

function build_model(
        model::AggregateODRouteModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        relax_integrality::Bool=model.relax_integrality,
    )::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    return _build_aggregate_od_route_core!(
        model, data, optimizer_env;
        relax_integrality=relax_integrality,
    )
end

function build_model(
        model::RouteCoveringProblem,
        data::StationSelectionData;
        optimizer_env=nothing,
        relax_integrality::Bool=model.relax_integrality,
    )::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    return _build_aggregate_od_route_core!(
        model, data, optimizer_env;
        relax_integrality=relax_integrality,
    )
end

function _relax_aggregate_od_route_station_and_assignment!(m::Model)
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
    if haskey(m, :u) && m[:u] !== nothing
        for scenario_dict in m[:u]
            for u_var in values(scenario_dict)
                unset_integer(u_var)
            end
        end
    end
    return nothing
end
