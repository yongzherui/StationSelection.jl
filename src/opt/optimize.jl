"""
    Construct a model based on the StationSelectionData and runs the model
"""
function run_opt(
        model::AbstractStationSelectionModel,
        data::StationSelectionData
        ;
        optimizer_env=nothing,
        silent::Bool=true
    )::OptimizationResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end
    # build model
    m = build_model(model, data)

    if silent
        set_silent(m)
    end

    # solve the model
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

    mapping = create_pooling_scenario_origin_dest_time_map(model, data)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    add_station_selection_variables!(m, data)
    add_scenario_activation_variables!(m, data)

    add_flow_variables!(m, data, mapping)
    add_assignment_variables!(m, data, mapping)

    add_detour_variables!(m, data, mapping)


    # now we add the constraints
end

