export AbstractSolveStrategy
export AbstractIterativeSolveStrategy
export DirectSolveStrategy
export IterativeSolveIterationSummary
export IterativeSolveResult

abstract type AbstractSolveStrategy end
abstract type AbstractIterativeSolveStrategy <: AbstractSolveStrategy end

struct DirectSolveStrategy <: AbstractSolveStrategy end

struct IterativeSolveIterationSummary
    iteration::Int
    objective_value::Float64
    state_size_before::Int
    state_size_after::Int
    added_count::Int
    removed_count::Int
    state_change_ratio::Float64
    objective_improvement::Union{Nothing, Float64}
    objective_delta::Union{Nothing, Float64}
    relative_objective_improvement::Union{Nothing, Float64}
    metadata::Dict{String, Any}
end

struct IterativeSolveResult
    final_result::OptResult
    iterations::Vector{IterativeSolveIterationSummary}
    convergence_reason::String
    final_state::Any
    metadata::Dict{String, Any}
end
