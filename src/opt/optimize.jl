"""
    run_opt(model, data; optimizer_env=nothing, silent=true)

Construct and solve a station selection optimization model.

# Arguments
- `model::AbstractStationSelectionModel`: The model specification (e.g., TwoStageSingleDetourModel)
- `data::StationSelectionData`: Problem data with stations, requests, and costs

# Keyword Arguments
- `optimizer_env`: Gurobi environment (created if not provided)
- `silent::Bool`: Whether to suppress solver output (default: true)

# Returns
- Tuple of (termination_status, objective_value, solution_values)
"""
function run_opt(
        model::AbstractStationSelectionModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        silent::Bool=true
    )

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Build model
    m = build_model(model, data, optimizer_env)

    if silent
        set_silent(m)
    end

    # Solve the model
    optimize!(m)

    term_status = JuMP.termination_status(m)
    if term_status == MOI.OPTIMAL
        obj = JuMP.objective_value(m)
        x_val = JuMP.value.(m[:x])
        y_val = JuMP.value.(m[:y])
        return term_status, obj, (x_val, y_val)
    end
    return term_status, nothing, nothing
end

function build_model(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData,
        optimizer_env
    )::Model

    # Create the pooling scenario map
    mapping = create_pooling_scenario_origin_dest_time_map(model, data)

    # Compute detour combinations
    Xi_same_source = find_same_source_detour_combinations(model, data)
    Xi_same_dest = find_same_dest_detour_combinations(model, data)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    # Add variables
    add_station_selection_variables!(m, data)
    add_scenario_activation_variables!(m, data)
    add_flow_variables!(m, data, mapping)
    add_assignment_variables!(m, data, mapping)
    add_detour_variables!(m, data, mapping, Xi_same_source, Xi_same_dest)

    # Add constraints
    add_station_limit_constraint!(m, data, model.l; equality=true)
    add_scenario_activation_limit_constraints!(m, data, model.k)
    add_activation_linking_constraints!(m, data)
    add_assignment_constraints!(m, data, mapping)
    add_assignment_to_active_constraints!(m, data, mapping)
    add_assignment_to_flow_constraints!(m, data, mapping)
    add_assignment_to_same_source_detour_constraints!(m, data, mapping, Xi_same_source)
    add_assignment_to_same_dest_detour_constraints!(m, data, mapping, Xi_same_dest)

    # Set objective
    set_two_stage_single_detour_objective!(m, data, mapping, Xi_same_source, Xi_same_dest;
                                            routing_weight=model.routing_weight)

    return m
end

