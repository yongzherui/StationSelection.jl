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
    _package_result(build_result, m, runtime_sec; metadata=Dict()) -> OptResult

Read `m`'s current termination status/objective/solution and package them into an
`OptResult` alongside `build_result`'s mapping/counts/detour_combos. Shared by every
solver so termination handling and `OptResult` construction stay identical regardless
of which algorithm produced the final model state.
"""
function _package_result(
        build_result::BuildResult,
        m::JuMP.Model,
        runtime_sec::Float64;
        metadata::Dict{String, Any}=Dict{String, Any}(),
    )::OptResult
    term_status = JuMP.termination_status(m)
    objective_value = nothing
    solution = nothing
    if term_status == MOI.OPTIMAL
        objective_value = JuMP.objective_value(m)
        solution = extract_solution(build_result, m)
    end

    return OptResult(
        term_status,
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
