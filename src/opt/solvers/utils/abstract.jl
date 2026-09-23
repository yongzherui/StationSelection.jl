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
    threads::Union{Nothing, Int}
    # Raw solver attributes, applied LAST so they can override anything above. The escape
    # hatch for the knobs that only matter on a specific instance and do not belong in a
    # shared struct -- Gurobi's NoRelHeurTime, MIPFocus, Presolve, Method and friends.
    # Added 2026-09-16 after a ClusteringTwoStageOD model of 46.6M rows / 23.3M columns /
    # 116.6M nonzeros OOM-killed at 64 GB with Gurobi 1500 s into a presolve that had
    # removed zero rows: nothing in the four fields above can express "skip presolve and
    # just find me an incumbent", and that is a solver-effort choice, not a model change.
    attributes::Dict{String, Any}

    function SolverOptions(;
            silent::Bool=true,
            mip_gap::Union{Number, Nothing}=nothing,
            time_limit_sec::Union{Number, Nothing}=nothing,
            threads::Union{Integer, Nothing}=nothing,
            attributes::AbstractDict=Dict{String, Any}(),
        )
        isnothing(mip_gap) || mip_gap >= 0 ||
            throw(ArgumentError("mip_gap must be non-negative"))
        isnothing(time_limit_sec) || time_limit_sec > 0 ||
            throw(ArgumentError("time_limit_sec must be positive"))
        isnothing(threads) || threads > 0 ||
            throw(ArgumentError("threads must be positive"))
        attrs = Dict{String, Any}(String(k) => v for (k, v) in attributes)
        new(
            silent,
            isnothing(mip_gap) ? nothing : Float64(mip_gap),
            isnothing(time_limit_sec) ? nothing : Float64(time_limit_sec),
            isnothing(threads) ? nothing : Int(threads),
            attrs,
        )
    end
end

function _apply_solver_config!(m::JuMP.Model, config::SolverOptions)
    config.silent && set_silent(m)
    isnothing(config.mip_gap) || set_optimizer_attribute(m, "MIPGap", config.mip_gap)
    isnothing(config.time_limit_sec) || set_time_limit_sec(m, config.time_limit_sec)
    isnothing(config.threads) || set_optimizer_attribute(m, "Threads", config.threads)
    # Last, so an explicit attribute wins over the named fields above.
    for (k, v) in config.attributes
        set_optimizer_attribute(m, k, v)
    end
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
