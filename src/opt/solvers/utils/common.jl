"""
Shared result-packaging logic reused by every concrete `AbstractSolver` in this
directory, so each solver file only has to implement its own outer-loop shape.
"""

"""
    extract_solution(build_result::BuildResult, m::JuMP.Model)

Pull the solved decision-variable values out of `m` in whatever shape the problem
this model was built for wants to report. Problem/formulation-specific (e.g. reading
`m[:y]`/`m[:x]` for a station-selection model); not implemented here. Defaults to
`nothing`, which is a valid `OptResult.solution` value, so solvers work generically
even before a given Problem/Formulation supplies its own method.
"""
function extract_solution(build_result::BuildResult, m::JuMP.Model)
    return nothing
end

"""
    _solve_status(m::JuMP.Model; certified::Bool=true) -> SolveStatus

Translate `m`'s MOI termination/primal status into the package's `SolveStatus`
(`utils/core/results.jl`).

`certified` is the caller's assertion that `MOI.OPTIMAL` on **this model** means optimal
for the **problem** -- true for `DirectMIPSolver`, which optimizes the whole model, but
only conditionally true for `CGSolver`, whose master is optimal over a restricted column
pool that is provably complete only when pricing exhausted. A caller that cannot make
that assertion passes `certified=false` and the best available answer is downgraded from
`SOLVE_OPTIMAL` to `SOLVE_FEASIBLE`; the incumbent is still a valid bound, it is simply
not a proof.

An infeasible status is reported regardless of `certified` -- infeasibility of a
*restricted* master is not infeasibility of the problem, so `CGSolver` must not reach here
with an infeasible restricted master and call the instance infeasible (it breaks the loop
on `master_not_optimal` and re-solves instead; see `cg_solver.jl`).
"""
function _solve_status(m::JuMP.Model; certified::Bool=true)::SolveStatus
    term_status = JuMP.termination_status(m)
    if term_status in (MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED, MOI.LOCALLY_INFEASIBLE)
        return SOLVE_INFEASIBLE
    end
    # `result_count == 0` covers OPTIMIZE_NOT_CALLED and every limit hit before a first
    # incumbent, and is checked first because `primal_status` is only meaningful once a
    # result exists.
    has_incumbent = JuMP.result_count(m) > 0 &&
        JuMP.primal_status(m) in (MOI.FEASIBLE_POINT, MOI.NEARLY_FEASIBLE_POINT)
    has_incumbent || return SOLVE_NOT_SOLVED
    return (term_status == MOI.OPTIMAL && certified) ? SOLVE_OPTIMAL : SOLVE_FEASIBLE
end

"""
    _package_result(build_result, m, runtime_sec; metadata=Dict(), certified=true) -> OptResult

Read `m`'s current status/objective/solution and package them into an `OptResult`
alongside `build_result`'s mapping/counts/detour_combos. Shared by every solver so
termination handling and `OptResult` construction stay identical regardless of which
algorithm produced the final model state.

`certified` is forwarded to `_solve_status` (see there). The objective and solution are
extracted for `SOLVE_FEASIBLE` as well as `SOLVE_OPTIMAL` -- an uncertified incumbent is
still the run's answer, and dropping it would leave a budget-stopped run reporting no
objective at all. The raw MOI code is preserved as `metadata["moi_termination_status"]`
so nothing is lost by narrowing the reported status.
"""
function _package_result(
        build_result::BuildResult,
        m::JuMP.Model,
        runtime_sec::Float64;
        metadata::Dict{String, Any}=Dict{String, Any}(),
        certified::Bool=true,
    )::OptResult
    status = _solve_status(m; certified=certified)
    objective_value = nothing
    solution = nothing
    if status in (SOLVE_OPTIMAL, SOLVE_FEASIBLE)
        objective_value = JuMP.objective_value(m)
        solution = extract_solution(build_result, m)
    end
    metadata["moi_termination_status"] = string(JuMP.termination_status(m))

    return OptResult(
        status,
        objective_value,
        solution,
        runtime_sec,
        m,
        build_result.mapping,
        build_result.detour_combos,
        build_result.counts,
        nothing,
        metadata,
    )
end

"""
    _infeasible_result(build_result, runtime_sec, reason; metadata=Dict()) -> OptResult

A `SOLVE_INFEASIBLE` `OptResult` for a problem proven infeasible **without solving the
built model** -- specifically by `run_opt`'s `check_feasibility` gate
(`optimize/run_opt.jl`), which can refute a necessary condition far more cheaply than the
real solve would. `build_result`'s model is carried through unsolved so callers still get
the mapping/counts they would get from any other result; `reason` is recorded as
`metadata["infeasibility_reason"]`.
"""
function _infeasible_result(
        build_result::BuildResult,
        runtime_sec::Float64,
        reason::AbstractString;
        metadata::Dict{String, Any}=Dict{String, Any}(),
    )::OptResult
    metadata["infeasibility_reason"] = String(reason)

    return OptResult(
        SOLVE_INFEASIBLE,
        nothing,
        nothing,
        runtime_sec,
        build_result.model,
        build_result.mapping,
        build_result.detour_combos,
        build_result.counts,
        nothing,
        metadata,
    )
end
