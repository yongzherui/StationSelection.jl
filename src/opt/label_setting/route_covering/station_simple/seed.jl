"""
Label seeding for the elementary-route (station-simple) pricer: the depth-1
labels `_run_label_setting` (`engine.jl`) seeds its frontier from, via the
`_pricing_initial_labels` hook (wired in `hooks.jl`). See `types.jl` for what
`RouteCoveringStationSimpleLabel` means and `extend.jl` for how a seeded
label grows from here.
"""

function _initial_route_covering_station_simple_labels(
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{RouteCoveringStationSimpleLabel}
    # Only stations that are some pair's origin with positive dual reward are
    # worth opening a pickup clock at from the start; every node still gets a
    # depth-1 label (a route could pass through anywhere), just not all of
    # them start with a live clock.
    positive_origins = Set{Int}(
        pair[1] for pair in pricing_data.active_pairs if get(duals.sigma, pair, 0.0) > 1e-9
    )
    labels = RouteCoveringStationSimpleLabel[]
    for node in pricing_data.nodes
        live = Dict{Int, Float64}()
        node in positive_origins && (live[node] = 0.0)
        push!(labels, RouteCoveringStationSimpleLabel(
            node,
            [node],
            Set{Int}([node]),  # visited starts with just this node
            0.0,
            live,
            Set{Tuple{Int, Int}}(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
        ))
    end
    return labels
end
