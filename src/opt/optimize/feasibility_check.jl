"""Placeholder mapping returned when `run_opt` exits early due to a failed feasibility check."""
struct EmptyStationSelectionMap <: AbstractStationSelectionMap end

"""
    check_model_feasibility(model, data) -> Union{Nothing, String}

Pre-solve feasibility check dispatched by model type. Called by `run_opt` before
entering the main solve (or fleet search loop for RouteVehicleCapacityModel).
Returns `nothing` if the instance looks solvable, or a non-nothing String describing
the detected issue for early exit.

Default (non-OD models): no check, always returns `nothing`.
"""
function check_model_feasibility(
        ::AbstractStationSelectionModel,
        ::StationSelectionData
    )
    return nothing
end

"""
    check_model_feasibility(model::AbstractODModel, data) -> Union{Nothing, String}

Runs a `ClusteringTwoStageODModel` solve as a fast feasibility proxy before entering
the main solve. If even this simpler model is infeasible, the more complex model will
be too.

Skipped when the model is already a `ClusteringTwoStageODModel` (would be redundant).
"""
function check_model_feasibility(
        model::AbstractODModel,
        data::StationSelectionData
    )
    model isa ClusteringTwoStageODModel && return nothing

    feasibility_model = ClusteringTwoStageODModel(
        model.k, model.l;
        max_walking_distance   = model.max_walking_distance,
        in_vehicle_time_weight = 1.0
    )
    @info "check_model_feasibility: running ClusteringTwoStageODModel proxy" k=model.k l=model.l
    result = _run_opt_impl(
        feasibility_model, data;
        silent            = true,
        do_optimize       = true,
        warm_start        = false,
        check_feasibility = false,
        mip_gap           = nothing
    )
    if result.termination_status != MOI.OPTIMAL
        return "ClusteringTwoStageODModel feasibility proxy returned $(result.termination_status)"
    end
    return nothing
end

function check_model_feasibility(::NominalTwoStageODModel, ::StationSelectionData)
    return nothing
end

function check_model_feasibility(::RobustTotalDemandCapModel, ::StationSelectionData)
    return nothing
end
