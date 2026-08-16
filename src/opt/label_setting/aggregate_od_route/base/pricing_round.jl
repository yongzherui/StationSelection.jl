"""
`AggregateODRouteBaseFormulation`'s formulation-level hooks into
`_run_pricing_round` (`round.jl`): one unit per scenario, sequential (no
`_pricing_parallel_units` override -- nothing has needed the throughput yet),
no cross-scenario column budget (no `_pricing_merge_units` override), and a
throwaway starting column id (no `_pricing_next_column_id` override: unlike
Joint, Base's incremental adder `add_aggregate_od_route_base_column!` dedups
by content signature and mints its own id on registration, so ids returned by
the pricer are never used as master keys). Only the two hooks below need an
implementation; the rest fall back to `round.jl`'s defaults.
"""

_pricing_units(::AggregateODRouteBaseFormulation, mapping::AggregateODRouteMap, m::JuMP.Model) =
    1:n_scenarios(m[:aggregate_od_route_base_data])

"""
Build one scenario's pricing context: `active_pairs` (non-walk-only pairs this
scenario actually has), the pricing graph off the model's stashed scalars, and
the existing column pool restricted to this scenario. `nothing` when a
scenario has no active pairs to price at all.
"""
function _pricing_build_unit_context(
    ::AggregateODRouteBaseFormulation, mapping::AggregateODRouteMap, s::Int,
    m::JuMP.Model, duals::Dict{Int, AggregateODRoutePricingDuals},
)
    active_pairs = filter(!is_walk_only_pair, mapping.active_jk_s[s])
    isempty(active_pairs) && return nothing

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

    ctx = AggregateODRouteSearchContext(pricing_data, duals[s])
    return ctx, existing
end
