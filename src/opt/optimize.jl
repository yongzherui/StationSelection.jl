"""
Generic optimization runner and result extraction for station selection models.

This module provides the main `optimize_model` function that works with any
concrete model type through multiple dispatch on `build_model` and `extract_result`.
"""
module Optimize

using JuMP
using Gurobi
using DataFrames

using ..DataStructs: StationSelectionData, n_scenarios, get_station_id
using ..AbstractModels: AbstractStationSelectionModel, AbstractSingleScenarioModel,
                        AbstractTwoStageModel, AbstractRoutingModel

# Import model types
using ..BaseModelDef: BaseModel
using ..TwoStageLambdaModelDef: TwoStageLambdaModel
using ..TwoStageLModelDef: TwoStageLModel
using ..RoutingTransportModelDef: RoutingTransportModel

# Import variable/constraint/objective modules
using ..Variables
using ..Constraints
using ..Objectives

# Import Result type (we need to reference it from the parent module)
# This will be set up via the main module

export optimize_model, build_model, extract_result

"""
    OptimizationResult

Result struct that describes the output of station selection optimization methods.

# Fields
- `method::String`: Name of the optimization method used
- `status::Bool`: Whether optimization was successful
- `value::Union{Vector{Int}, Nothing}`: Vector of selected station indicators
- `stations::Union{Dict{Int, Bool}, Nothing}`: Dictionary mapping station ID to selection status
- `station_df::DataFrame`: DataFrame containing station information and results
- `model::Union{JuMP.Model, Nothing}`: The JuMP optimization model
- `metadata::Dict{String, Any}`: Additional metadata about the optimization run
"""
struct OptimizationResult
    method::String
    status::Bool
    value::Union{Vector{Int}, Nothing}
    stations::Union{Dict{Int, Bool}, Nothing}
    station_df::DataFrame
    model::Union{JuMP.Model, Nothing}
    metadata::Dict{String, Any}
end

# Convenience constructor without metadata
function OptimizationResult(
    method::String,
    status::Bool,
    value::Union{Vector{Int}, Nothing},
    stations::Union{Dict{Int, Bool}, Nothing},
    station_df::DataFrame,
    model::Union{JuMP.Model, Nothing}
)
    return OptimizationResult(method, status, value, stations, station_df, model, Dict{String, Any}())
end

export OptimizationResult

"""
    optimize_model(model::AbstractStationSelectionModel, data::StationSelectionData;
                   optimizer_env=nothing, silent::Bool=true) -> OptimizationResult

Main optimization entry point. Builds and solves the model, then extracts results.

# Arguments
- `model`: A concrete model configuration (e.g., BaseModel, TwoStageLModel)
- `data`: Problem data encapsulated in StationSelectionData
- `optimizer_env`: Optional Gurobi environment (created if not provided)
- `silent`: Whether to suppress solver output (default: true)

# Returns
- `OptimizationResult` containing solution status, selected stations, and model
"""
function optimize_model(
    model::AbstractStationSelectionModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    silent::Bool=true
)::OptimizationResult
    # Create optimizer environment if not provided
    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Build the JuMP model using dispatch on model type
    m = build_model(model, data, optimizer_env)

    if silent
        set_silent(m)
    end

    # Solve
    JuMP.optimize!(m)

    # Extract and return results
    return extract_result(model, m, data)
end

# ============================================================================
# build_model implementations for each model type
# ============================================================================

"""
    build_model(model::BaseModel, data::StationSelectionData, optimizer_env) -> Model

Build JuMP model for basic k-medoids station selection.
"""
function build_model(model::BaseModel, data::StationSelectionData, optimizer_env)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    # Variables
    add_station_selection_variables!(m, data)
    add_assignment_variables_single_scenario!(m, data)

    # Constraints
    add_single_assignment_constraints!(m, data)
    add_assignment_to_selected_constraints!(m, data)
    add_station_limit_constraint!(m, data, model.k; equality=model.strict_equality)

    # Objective
    walking_cost = create_walking_cost_expression_single!(m, data)
    set_minimize_objective!(m, walking_cost)

    return m
end

"""
    build_model(model::TwoStageLambdaModel, data::StationSelectionData, optimizer_env) -> Model

Build JuMP model for two-stage with λ penalty.
"""
function build_model(model::TwoStageLambdaModel, data::StationSelectionData, optimizer_env)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    # Variables
    add_station_selection_variables!(m, data)
    add_scenario_activation_variables!(m, data)
    add_assignment_variables!(m, data)

    # Constraints
    add_assignment_constraints!(m, data)
    add_assignment_to_active_constraints!(m, data)
    add_activation_linking_constraints!(m, data)
    add_station_limit_constraint!(m, data, model.k; equality=true)

    # Objective: walking cost + activation penalty
    walking_cost = create_walking_cost_expression!(m, data)
    if model.lambda > 0
        activation_penalty = create_activation_penalty_expression!(m, data; lambda=model.lambda)
        set_minimize_objective!(m, walking_cost, activation_penalty)
    else
        set_minimize_objective!(m, walking_cost)
    end

    return m
end

"""
    build_model(model::TwoStageLModel, data::StationSelectionData, optimizer_env) -> Model

Build JuMP model for two-stage with L permanent, k active per scenario.
"""
function build_model(model::TwoStageLModel, data::StationSelectionData, optimizer_env)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    # Variables
    add_station_selection_variables!(m, data)
    add_scenario_activation_variables!(m, data)
    add_assignment_variables!(m, data)

    # Constraints
    add_assignment_constraints!(m, data)
    add_assignment_to_active_constraints!(m, data)
    add_activation_linking_constraints!(m, data)
    add_station_limit_constraint!(m, data, model.l; equality=true)
    add_scenario_activation_limit_constraints!(m, data, model.k)

    # Objective: walking cost only
    walking_cost = create_walking_cost_expression!(m, data)
    set_minimize_objective!(m, walking_cost)

    return m
end

"""
    build_model(model::RoutingTransportModel, data::StationSelectionData, optimizer_env) -> Model

Build JuMP model for routing via transportation problem.
"""
function build_model(model::RoutingTransportModel, data::StationSelectionData, optimizer_env)
    # Validate that routing costs are available
    isnothing(data.routing_costs) && error("RoutingTransportModel requires routing_costs in data")

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    # Variables
    add_station_selection_variables!(m, data)
    add_scenario_activation_variables!(m, data)
    add_pickup_assignment_variables!(m, data)
    add_dropoff_assignment_variables!(m, data)
    add_flow_variables!(m, data)

    # Constraints
    add_pickup_assignment_constraints!(m, data)
    add_dropoff_assignment_constraints!(m, data)
    add_pickup_to_active_constraints!(m, data)
    add_dropoff_to_active_constraints!(m, data)
    add_activation_linking_constraints!(m, data)
    add_station_limit_constraint!(m, data, model.l; equality=true)
    add_scenario_activation_limit_constraints!(m, data, model.k)
    add_flow_supply_constraints!(m, data)
    add_flow_demand_constraints!(m, data)

    # Objective: pickup walking + dropoff walking + routing cost
    pickup_cost = create_pickup_walking_cost_expression!(m, data)
    dropoff_cost = create_dropoff_walking_cost_expression!(m, data)
    routing_cost = create_routing_cost_expression!(m, data; weight=model.lambda)
    set_minimize_objective!(m, pickup_cost, dropoff_cost, routing_cost)

    return m
end

# ============================================================================
# extract_result implementations
# ============================================================================

"""
    extract_result(model::AbstractSingleScenarioModel, m::Model,
                   data::StationSelectionData) -> OptimizationResult

Extract results for single-scenario models.
"""
function extract_result(
    model::AbstractSingleScenarioModel,
    m::Model,
    data::StationSelectionData
)::OptimizationResult
    n = data.n_stations

    if !is_solved_and_feasible(m)
        return OptimizationResult(
            string(typeof(model)),
            false,
            nothing,
            nothing,
            data.stations,
            m
        )
    end

    y_vals = value.(m[:y])

    # Build result DataFrame
    df = DataFrame(
        id=data.stations.id,
        lon=data.stations.lon,
        lat=data.stations.lat,
        selected=y_vals
    )

    # Build station selection dictionary
    stations_dict = Dict(
        get_station_id(data, i) => (y_vals[i] > 0.5)
        for i in 1:n
    )

    return OptimizationResult(
        string(typeof(model)),
        true,
        Int.(y_vals),
        stations_dict,
        df,
        m
    )
end

"""
    extract_result(model::AbstractTwoStageModel, m::Model,
                   data::StationSelectionData) -> OptimizationResult

Extract results for two-stage models (includes scenario activation info).
"""
function extract_result(
    model::AbstractTwoStageModel,
    m::Model,
    data::StationSelectionData
)::OptimizationResult
    n = data.n_stations
    S = n_scenarios(data)

    if !is_solved_and_feasible(m)
        return OptimizationResult(
            string(typeof(model)),
            false,
            nothing,
            nothing,
            data.stations,
            m
        )
    end

    y_vals = value.(m[:y])
    z_vals = value.(m[:z])

    # Build result DataFrame with scenario columns
    df = DataFrame(
        id=data.stations.id,
        lon=data.stations.lon,
        lat=data.stations.lat,
        selected=y_vals
    )

    # Add scenario activation columns
    for s in 1:S
        col_name = Symbol(data.scenarios[s].label)
        df[!, col_name] = z_vals[:, s]
    end

    # Build station selection dictionary
    stations_dict = Dict(
        get_station_id(data, i) => (y_vals[i] > 0.5)
        for i in 1:n
    )

    return OptimizationResult(
        string(typeof(model)),
        true,
        Int.(y_vals),
        stations_dict,
        df,
        m
    )
end

# Use the two-stage extractor for routing models too
function extract_result(
    model::AbstractRoutingModel,
    m::Model,
    data::StationSelectionData
)::OptimizationResult
    # Routing models have the same result structure as two-stage models
    # Just call the two-stage version with explicit typing
    return invoke(extract_result, Tuple{AbstractTwoStageModel, Model, StationSelectionData},
                  model, m, data)
end

end # module
