"""
    run_opt(model, data; optimizer_env=nothing, silent=true, show_counts=false,
            return_model=false, return_counts=false)

Construct and solve a station selection optimization model.

# Arguments
- `model::AbstractStationSelectionModel`: The model specification (e.g., TwoStageSingleDetourModel)
- `data::StationSelectionData`: Problem data with stations, requests, and costs

# Keyword Arguments
- `optimizer_env`: Gurobi environment (created if not provided)
- `silent::Bool`: Whether to suppress solver output (default: true)
- `show_counts::Bool`: Whether to print variable/constraint counts before solving (default: false)
- `return_model::Bool`: Whether to return the JuMP model (default: false)
- `return_counts::Bool`: Whether to return variable/constraint counts (default: false)

# Returns
- Tuple of (termination_status, objective_value, solution_values)
"""
function run_opt(
        model::AbstractStationSelectionModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        silent::Bool=true,
        show_counts::Bool=false,
        return_model::Bool=false,
        return_counts::Bool=false
    )

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Build model (with counts when available)
    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    if hasmethod(build_model_with_counts, Tuple{typeof(model), StationSelectionData, typeof(optimizer_env)})
        m, variable_counts, constraint_counts = build_model_with_counts(model, data, optimizer_env)
    else
        m = build_model(model, data, optimizer_env)
    end

    if show_counts && (!isempty(variable_counts) || !isempty(constraint_counts))
        _print_counts("Variables", variable_counts)
        _print_counts("Constraints", constraint_counts)
    elseif show_counts
        println("Counts unavailable for model type: $(typeof(model))")
    end

    if silent
        set_silent(m)
    end

    # Solve the model
    optimize!(m)

    term_status = JuMP.termination_status(m)
    obj = nothing
    solution = nothing
    if term_status == MOI.OPTIMAL
        obj = JuMP.objective_value(m)
        x_val = JuMP.value.(m[:x])
        y_val = JuMP.value.(m[:y])
        solution = (x_val, y_val)
    end

    if return_model || return_counts
        return term_status, obj, solution, m, variable_counts, constraint_counts
    end
    return term_status, obj, solution
end

function _print_counts(title::String, counts::Dict{String, Int})
    total = sum(values(counts))
    println("$title (total=$total)")
    for key in sort(collect(keys(counts)))
        println("  - $key: $(counts[key])")
    end
end

function build_model(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData,
        optimizer_env
    )::Model
    m, _, _ = build_model_with_counts(model, data, optimizer_env)
    return m
end

function build_model_with_counts(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData,
        optimizer_env
    )::Tuple{Model, Dict{String, Int}, Dict{String, Int}}

    # Create the pooling scenario map
    mapping = create_pooling_scenario_origin_dest_time_map(model, data)

    # Compute detour combinations
    Xi_same_source = find_same_source_detour_combinations(model, data)
    Xi_same_dest = find_same_dest_detour_combinations(model, data)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()

    # Add variables
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["flow"] = add_flow_variables!(m, data, mapping)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
    variable_counts["detour"] = add_detour_variables!(m, data, mapping, Xi_same_source, Xi_same_dest)

    # Add constraints
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, model.l; equality=true)
    constraint_counts["scenario_activation_limit"] = add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(m, data, mapping)
    constraint_counts["assignment_to_flow"] = add_assignment_to_flow_constraints!(m, data, mapping)
    constraint_counts["assignment_to_same_source_detour"] = add_assignment_to_same_source_detour_constraints!(m, data, mapping, Xi_same_source)
    constraint_counts["assignment_to_same_dest_detour"] = add_assignment_to_same_dest_detour_constraints!(m, data, mapping, Xi_same_dest)

    # Set objective
    set_two_stage_single_detour_objective!(m, data, mapping, Xi_same_source, Xi_same_dest;
                                            routing_weight=model.routing_weight)

    return m, variable_counts, constraint_counts
end
