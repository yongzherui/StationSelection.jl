using JuMP

const MOI = JuMP.MOI

export ModelCounts
export DetourComboData
export BuildResult
export OptResult

"""
    ModelCounts

Holds variable/constraint counts and extra counters from model building.
"""
struct ModelCounts
    variables::Dict{String, Int}
    constraints::Dict{String, Int}
    extras::Dict{String, Int}
end

"""
    DetourComboData

Detour combination data for single-detour models.
"""
struct DetourComboData
    same_source::Vector{Tuple{Int, Int, Int}}
    same_dest::Vector{Tuple{Int, Int, Int, Int}}
end

"""
    BuildResult

Return type for model construction.
"""
struct BuildResult
    model::JuMP.Model
    mapping::AbstractStationSelectionMap
    detour_combos::Union{DetourComboData, Nothing}
    counts::Union{ModelCounts, Nothing}
    metadata::Dict{String, Any}
end

"""
    OptResult

Return type for optimization runs.
"""
struct OptResult
    termination_status::MOI.TerminationStatusCode
    objective_value::Union{Nothing, Float64}
    solution::Union{Nothing, Tuple}
    runtime_sec::Float64
    model::JuMP.Model
    mapping::AbstractStationSelectionMap
    detour_combos::Union{DetourComboData, Nothing}
    counts::Union{ModelCounts, Nothing}
    warm_start_solution::Union{Nothing, Dict{Symbol, Any}}
    metadata::Dict{String, Any}
end
