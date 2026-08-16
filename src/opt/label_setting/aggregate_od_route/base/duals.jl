"""
Master-problem-facing dual extraction for `AggregateODRouteBaseFormulation`'s `CGSolver`
master, keyed by scenario -- each scenario's demand groups price against a different
`route_link` dual vector, so pricing needs one `AggregateODRoutePricingDuals` per scenario,
not one shared vector.
"""

export extract_aggregate_od_route_base_duals

"""
    extract_aggregate_od_route_base_duals(m) -> Dict{Int, AggregateODRoutePricingDuals}

Per-scenario per-pair reward `sigma_s[(j,k)] = -sum(dual(route_link[(s,p,j,k)]) for p using (j,k))`.
`route_link[key] = @constraint(m, x[key] <= sum(theta over (j,k)))` normalizes to
`x - sum(theta) <= 0`; for a Min problem's `<=` row, JuMP's dual convention is `<= 0` (the
same convention `extract_joint_routing_assignment_duals`'s `pickup_link`/`dropoff_link`
negation already relies on), so `-dual(...)` should yield the non-negative per-pair "value
of one more unit of route capacity" the label-setting pricer's `AggregateODRoutePricingDuals`
expects. This is a derivation, not something read off existing code -- verified empirically
in `test/opt/test_aggregate_od_route_base_cg.jl`, not just trusted by inspection.
"""
function extract_aggregate_od_route_base_duals(m::JuMP.Model)::Dict{Int, AggregateODRoutePricingDuals}
    data = m[:aggregate_od_route_base_data]
    route_link = m[:aggregate_od_route_base_route_link]
    sigma_by_scenario = Dict{Int, Dict{Tuple{Int, Int}, Float64}}(
        s => Dict{Tuple{Int, Int}, Float64}() for s in 1:n_scenarios(data)
    )
    for ((s, _p, j, k), con) in route_link
        sigma = sigma_by_scenario[s]
        sigma[(j, k)] = get(sigma, (j, k), 0.0) - dual(con)
    end
    return Dict(s => AggregateODRoutePricingDuals(sigma) for (s, sigma) in sigma_by_scenario)
end

"""
`CGSolver` hook real logic (dispatched from
`optimize/aggregate_od_route/column_generation/dispatch.jl`).
"""
_aggregate_od_route_extract_duals(::AggregateODRouteBaseFormulation, build_result, mapping, m::JuMP.Model) =
    extract_aggregate_od_route_base_duals(m)
