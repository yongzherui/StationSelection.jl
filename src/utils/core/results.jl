using JuMP

const MOI = JuMP.MOI

export ModelCounts
export DetourComboData
export BuildResult
export OptResult
export SolveStatus
export SOLVE_OPTIMAL, SOLVE_FEASIBLE, SOLVE_INFEASIBLE, SOLVE_NOT_SOLVED

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
    SolveStatus

What a solve actually established about the problem `run_opt` was handed. Deliberately
NOT `MOI.TerminationStatusCode`: that enum reports the status of the *model object that
was last optimized*, which for a column-generation run is the master over whatever
restricted column pool the loop happened to stop with. A `CGSolver` run that exhausts its
total budget still leaves that master solving to `MOI.OPTIMAL`, so the raw MOI code reads
`OPTIMAL` for a run that proved nothing about the true optimum -- the failure mode this
enum exists to remove. MOI also has no code for "feasible but not proven optimal"
(`MOI.FEASIBLE_POINT` is a *primal* status, a different enum), so a package-owned type is
required to express it at all.

- `SOLVE_OPTIMAL` -- a certified optimum. For `DirectMIPSolver`, the MIP solved to
  optimality. For `CGSolver`, additionally that pricing *exhausted*: no
  negative-reduced-cost column remains, so the pool is provably complete.
- `SOLVE_FEASIBLE` -- a valid incumbent (a genuine upper bound on a minimization) exists,
  but optimality is NOT proven. This is where a budget-stopped or pricing-inconclusive CG
  run belongs, and where a MIP that hit a time/node limit with an incumbent belongs.
- `SOLVE_INFEASIBLE` -- the problem has no feasible solution: either the solver returned
  an infeasible status, or `check_feasibility`'s necessary-condition gate failed before
  the model was ever solved.
- `SOLVE_NOT_SOLVED` -- no incumbent to report (never optimized, numerical error, or a
  limit hit before any feasible point was found).

The raw MOI code is not discarded -- `_package_result` records it as
`metadata["moi_termination_status"]` so the underlying solver state stays inspectable.

Prints as `OPTIMAL`/`FEASIBLE`/`INFEASIBLE`/`NOT_SOLVED` (without the `SOLVE_` prefix) so
`string(result.termination_status)` stays readable in result CSVs; the Julia-level member
names keep the prefix because `using JuMP` re-exports bare `OPTIMAL`/`INFEASIBLE` from
MOI into scope and the names would otherwise collide.
"""
@enum SolveStatus SOLVE_OPTIMAL SOLVE_FEASIBLE SOLVE_INFEASIBLE SOLVE_NOT_SOLVED

const _SOLVE_STATUS_LABELS = Dict{SolveStatus, String}(
    SOLVE_OPTIMAL => "OPTIMAL",
    SOLVE_FEASIBLE => "FEASIBLE",
    SOLVE_INFEASIBLE => "INFEASIBLE",
    SOLVE_NOT_SOLVED => "NOT_SOLVED",
)

Base.show(io::IO, status::SolveStatus) = print(io, _SOLVE_STATUS_LABELS[status])
Base.string(status::SolveStatus) = _SOLVE_STATUS_LABELS[status]

"""
    OptResult

Return type for optimization runs.

`termination_status` is a `SolveStatus` (above), **not** an `MOI.TerminationStatusCode`:
it describes what the run established about the *problem*, where the MOI code describes
only the last model object that was optimized. The raw MOI code is kept as
`metadata["moi_termination_status"]`.

`duals` is `nothing` for every `AbstractStationSelectionModel` result (the
overwhelming majority) and populated only for `AbstractBendersDualProblem`
results (see `src/opt/abstract.jl`), which have no station/assignment
`solution` to report -- read their solved dual values from here instead.
Defaults to `nothing` so every pre-existing positional `OptResult(...)` call
site (10 args) keeps working unchanged; only dual-problem construction sites
need to pass it explicitly.
"""
struct OptResult
    termination_status::SolveStatus
    objective_value::Union{Nothing, Float64}
    solution::Union{Nothing, Tuple}
    runtime_sec::Float64
    model::JuMP.Model
    mapping::AbstractStationSelectionMap
    detour_combos::Union{DetourComboData, Nothing}
    counts::Union{ModelCounts, Nothing}
    warm_start_solution::Union{Nothing, Dict{Symbol, Any}}
    metadata::Dict{String, Any}
    duals::Union{Nothing, Dict}

    function OptResult(
        termination_status::SolveStatus,
        objective_value::Union{Nothing, Float64},
        solution::Union{Nothing, Tuple},
        runtime_sec::Float64,
        model::JuMP.Model,
        mapping::AbstractStationSelectionMap,
        detour_combos::Union{DetourComboData, Nothing},
        counts::Union{ModelCounts, Nothing},
        warm_start_solution::Union{Nothing, Dict{Symbol, Any}},
        metadata::Dict{String, Any},
        duals::Union{Nothing, Dict}=nothing,
    )
        new(
            termination_status, objective_value, solution, runtime_sec, model,
            mapping, detour_combos, counts, warm_start_solution, metadata, duals,
        )
    end
end
