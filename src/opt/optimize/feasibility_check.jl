"""Placeholder mapping returned when `run_opt` exits early due to a failed feasibility check."""
struct EmptyStationSelectionMap <: AbstractStationSelectionMap end

"""
    check_model_feasibility(model, data) -> Union{Nothing, String}

Pre-solve feasibility check dispatched by model type. Called by `run_opt` before
entering the main solve.
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
