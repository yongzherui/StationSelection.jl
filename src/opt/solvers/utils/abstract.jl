"""
Shared foundation for `src/opt/solvers/`: the `AbstractSolver` root type and the
execution knobs (`SolverOptions`) common to every concrete solver, independent of
algorithm shape (direct solve, column generation, Benders, heuristic, ...).
"""

export AbstractSolver
export SolverOptions

"""
    AbstractSolver

Root type for *how* a built model is solved once `build_model(problem, formulation,
solver)` has produced it -- direct MIP solve, column generation, Benders decomposition,
a heuristic, etc. Concrete subtypes are named `<Algorithm>Solver`, e.g. `DirectMIPSolver`.
Each must implement:

    optimize_model(build_result::BuildResult, solver::AbstractSolver) -> OptResult
"""
abstract type AbstractSolver end

"""
    SolverOptions

Execution knobs shared by every `AbstractSolver`, applied to the underlying JuMP model
before solving. Algorithm-specific knobs (iteration limits, tolerances, ...) live on
the concrete solver struct itself, not here.
"""
struct SolverOptions
    silent::Bool
    mip_gap::Union{Nothing, Float64}
    time_limit_sec::Union{Nothing, Float64}

    function SolverOptions(;
            silent::Bool=true,
            mip_gap::Union{Number, Nothing}=nothing,
            time_limit_sec::Union{Number, Nothing}=nothing,
        )
        isnothing(mip_gap) || mip_gap >= 0 ||
            throw(ArgumentError("mip_gap must be non-negative"))
        isnothing(time_limit_sec) || time_limit_sec > 0 ||
            throw(ArgumentError("time_limit_sec must be positive"))
        new(
            silent,
            isnothing(mip_gap) ? nothing : Float64(mip_gap),
            isnothing(time_limit_sec) ? nothing : Float64(time_limit_sec),
        )
    end
end

function _apply_solver_config!(m::JuMP.Model, config::SolverOptions)
    config.silent && set_silent(m)
    isnothing(config.mip_gap) || set_optimizer_attribute(m, "MIPGap", config.mip_gap)
    isnothing(config.time_limit_sec) || set_time_limit_sec(m, config.time_limit_sec)
    return nothing
end

"""
    optimize_model(build_result::BuildResult, solver::AbstractSolver) -> OptResult

Universal solve entry point dispatched on `solver`. Every concrete `AbstractSolver`
must supply its own method; this fallback only exists to give a clear error for a
solver type that hasn't (yet).
"""
function optimize_model(build_result::BuildResult, solver::AbstractSolver)
    throw(ArgumentError("optimize_model is not implemented for solver type $(typeof(solver))"))
end
