"""
Route-activation variable creation, batch/closed-form style: one `theta[(column_id,s)]`
per `(route column, scenario)`, for a column pool that is already fully known before any
constraint referencing `theta` is built. Lives outside `variables/aggregate_od_route/`
(and is not itself aggregate-OD-route-specific) because the "route that covers a set of
`(j,k)` station pairs, activated per scenario" idea it encodes is reusable by any problem
built from `AggregateODRouteColumn`-shaped route columns -- currently
`AggregateODRouteBaseFormulation`'s `DirectMIPSolver` build
(`optimize/aggregate_od_route/direct/build_base.jl`), and eventually `RouteCoveringProblem`'s
own `DirectMIPSolver` build once that problem type is wired up (`opt/problems/route_covering.jl`,
currently unwired).

Not to be confused with `add_aggregate_od_route_base_column!`
(`constraints/aggregate_od_route/base/route_activation.jl`), the CG-facing *incremental*
adder used when the column pool grows one column at a time after `route_link` already
exists (`CGSolver`'s own build, `optimize/aggregate_od_route/column_generation/build_base.jl`)
-- that one is not a candidate for the same reuse yet, since it also reaches into
formulation-specific `m[...]` state (objective weights, `route_link`'s own index) that a
different problem type would stash under different keys.
"""

export add_route_variables!

"""
    add_route_variables!(m, data, mapping; relax_integrality=false, scenarios=1:n_scenarios(data)) -> Int

One `theta[(column.id, s)]` per `column in mapping.columns` Ã— `s in scenarios` --
`AggregateODRouteColumn` is scenario-agnostic geometry (which `(j,k)` pairs a route
covers), so `theta` is what actually decides whether that route is *activated* in a given
scenario. `scenarios` restricts variable creation to a subset of scenarios -- used by
`AggregateODRouteBendersYXFormulation`'s subproblem, which is solved one scenario at a
time (default keeps every existing caller's behavior unchanged).
"""
function add_route_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    relax_integrality::Bool=false,
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
)::Int
    before = JuMP.num_variables(m)
    theta = Dict{Tuple{Int, Int}, VariableRef}()
    for column in mapping.columns
        for s in scenarios
            if relax_integrality
                # The upper bound is primal-redundant for this positive-cost set-covering LP:
                # every coverage RHS is at most one, so reducing theta > 1 to one preserves
                # feasibility and cannot increase cost.  Keeping the bound lets its bound dual
                # absorb reduced cost, which makes coverage duals alone invalid for Benders/MW
                # completion.  Use the equivalent standard nonnegative covering relaxation.
                theta[(column.id, s)] = @variable(m, lower_bound = 0.0)
            else
                theta[(column.id, s)] = @variable(m, binary = true)
            end
        end
    end
    m[:route_theta] = theta
    return JuMP.num_variables(m) - before
end
