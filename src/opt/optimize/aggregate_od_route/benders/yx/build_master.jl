"""
`build_model` for `StationSelectionProblem` paired with `AggregateODRouteBendersYXFormulation`
+ `BendersSolver` -- the Benders master: `y` (station selection) plus one `theta_cut`
cut-placeholder variable per cut group (`formulation.cut_mode`). The route/assignment
universe (`mapping`, exhaustively enumerated `columns`) is built once here, exactly like
`AggregateODRouteBaseFormulation`'s own `build_model`, and stashed on `m` for the Benders
hooks (`hooks.jl`) to reuse when building per-scenario subproblems.
"""

"""
    _aggregate_od_route_benders_yx_cut_groups(cut_mode, S::Int) -> Dict{Int, Vector{Int}}

Maps cut-group id to the scenarios it covers. `SingleCut` -> one group covering every
scenario; `MultiCut(:scenario)` -> one group per scenario. Group ids are always a dense
`1:G` range so `theta_cut` can be a plain `@variable(m, theta_cut[1:G] >= 0)` container.
"""
function _aggregate_od_route_benders_yx_cut_groups(
    cut_mode::AbstractBendersCutMode,
    S::Int,
)::Dict{Int, Vector{Int}}
    cut_mode isa SingleCut && return Dict(1 => collect(1:S))
    return Dict(s => [s] for s in 1:S)
end

function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteBendersYXFormulation,
        solver::BendersSolver,
    )::BuildResult
    data = problem.data
    columns = enumerate_aggregate_od_route_columns(problem, formulation, data)
    mapping = create_aggregate_od_route_map(problem, formulation, data; initial_columns=columns)

    m = Model(() -> Gurobi.Optimizer())

    variable_counts = Dict{String, Int}()
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)

    groups = _aggregate_od_route_benders_yx_cut_groups(formulation.cut_mode, n_scenarios(data))
    G = length(groups)
    @variable(m, theta_cut[1:G] >= 0.0)
    variable_counts["theta_cut"] = G

    constraint_counts = Dict{String, Int}()
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, problem.l; equality = true)

    @objective(m, Min, sum(theta_cut))

    m[:aggregate_od_route_benders_yx_data] = data
    m[:aggregate_od_route_benders_yx_formulation] = formulation
    m[:aggregate_od_route_benders_yx_groups] = groups
    m[:aggregate_od_route_benders_yx_theta] = theta_cut

    extra_counts = Dict{String, Int}(
        "routes_enumerated" => length(columns),
        "cut_groups" => G,
    )
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
