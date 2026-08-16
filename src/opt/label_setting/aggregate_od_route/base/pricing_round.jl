"""
`price_columns` `CGSolver` hook for `AggregateODRouteBaseFormulation`. A thin per-scenario
wrapper around the already-existing, already-tested pricer
`aggregate_od_route_pricing_by_label_setting` (`label_setting/aggregate_od_route/search.jl`)
-- no new traversal, no new dominance rules: that function already handles dedup against the
existing pool, reduced-cost/dominance pruning, and graceful time-limit truncation (it never
throws on a budget limit, unlike `enumerate_aggregate_od_route_columns`'s exhaustive DFS).
"""

"""
    _aggregate_od_route_base_price_scenario(m, data, mapping, sigma, s; ...) -> Vector{AggregateODRouteColumn}

Prices one scenario against `AggregateODRoutePricingDuals(sigma)`. `next_column_id=1` is a
throwaway placeholder here: unlike Joint (where the pricer-assigned id *is* the master's real
key), Base's incremental adder (`add_aggregate_od_route_base_column!`) dedups by content
signature and mints its own id on registration, so ids returned by the pricer are never used
as master keys. This also means each scenario's pricing call is independent and could run in
parallel; sequential for now (no `Threads.@threads`, unlike Joint's
`_price_passenger_scenarios`) since nothing has needed the throughput yet.
"""
function _aggregate_od_route_base_price_scenario(
        m::JuMP.Model,
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        sigma::Dict{Tuple{Int, Int}, Float64},
        s::Int;
        max_new_columns::Int,
        n_candidates::Int,
        time_limit::Float64,
        reduced_cost_tol::Float64,
    )::Vector{AggregateODRouteColumn}
    active_pairs = filter(!is_walk_only_pair, mapping.active_jk_s[s])
    isempty(active_pairs) && return AggregateODRouteColumn[]

    max_stops = Int(m[:aggregate_od_route_base_max_stops])
    pricing_data = AggregateODRoutePricingData(
        s,
        m[:aggregate_od_route_base_nodes],
        m[:aggregate_od_route_base_travel_cost],
        active_pairs,
        Float64(m[:aggregate_od_route_base_route_regularization_weight]),
        Float64(m[:aggregate_od_route_base_repositioning_time]),
        Float64(m[:aggregate_od_route_base_max_wait_time]),
        Float64(m[:aggregate_od_route_base_detour_factor]),
        max_stops,
        max_stops != typemax(Int),
    )

    columns_by_id = m[:aggregate_od_route_base_columns_by_id]
    theta = m[:aggregate_od_route_base_theta]
    existing = AggregateODRouteColumn[
        columns_by_id[column_id] for (column_id, sc) in keys(theta) if sc == s
    ]

    columns, _exhausted, _stats = aggregate_od_route_pricing_by_label_setting(
        pricing_data, existing, AggregateODRoutePricingDuals(sigma);
        next_column_id=1, reduced_cost_tol=reduced_cost_tol,
        max_new_columns=max_new_columns, n_candidates=n_candidates, time_limit=time_limit,
    )
    return columns
end

"""
Cross-check that the pricer's reported reduced cost equals the one implied by the master's
own duals, mirroring `_verify_joint_routing_assignment_master_reduced_cost`. A single-term
formula here (unlike Joint's three-constraint-family sum): a Base column only ever touches
`route_link`.
"""
function _verify_aggregate_od_route_base_master_reduced_cost(
        column::AggregateODRouteColumn,
        route_regularization_weight::Float64,
        repositioning_time::Float64,
        sigma::Dict{Tuple{Int, Int}, Float64};
        atol::Float64=1e-5,
    )
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = aggregate_od_route_column_objective_coefficient(
        route_regularization_weight, repositioning_time, column,
    ) - sum(get(sigma, pair, 0.0) for pair in column.od_pairs; init=0.0)
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end

"""
`CGSolver` hook real logic (dispatched from
`optimize/aggregate_od_route/column_generation/dispatch.jl`).
"""
function _aggregate_od_route_price_columns(
        ::AggregateODRouteBaseFormulation,
        build_result::BuildResult,
        mapping::AggregateODRouteMap,
        m::JuMP.Model,
        duals::Dict{Int, AggregateODRoutePricingDuals},
        solver::CGSolver,
    )
    data = m[:aggregate_od_route_base_data]
    route_regularization_weight = Float64(m[:aggregate_od_route_base_route_regularization_weight])
    repositioning_time = Float64(m[:aggregate_od_route_base_repositioning_time])

    all_columns = AggregateODRouteColumn[]
    for s in 1:n_scenarios(data)
        sigma = duals[s].sigma
        columns = _aggregate_od_route_base_price_scenario(
            m, data, mapping, sigma, s;
            max_new_columns=typemax(Int) ÷ 2, n_candidates=typemax(Int) ÷ 2,
            time_limit=30.0, reduced_cost_tol=solver.reduced_cost_tol,
        )
        for column in columns
            ok, pricer_rc, master_rc = _verify_aggregate_od_route_base_master_reduced_cost(
                column, route_regularization_weight, repositioning_time, sigma,
            )
            ok || error(
                "aggregate OD route base pricing reduced cost $(pricer_rc) disagrees with " *
                "the master's dual-implied $(master_rc) for column $(column.od_pairs), " *
                "scenario $(s) -- the pricer and master formulations have drifted apart",
            )
            push!(all_columns, column)
        end
    end
    return all_columns
end
