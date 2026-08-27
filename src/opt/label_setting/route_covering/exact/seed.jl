"""
Label seeding for the revisit-tolerant route-covering pricer: the depth-1
labels `_run_label_setting` (`engine.jl`) seeds its frontier from, via the
`_pricing_initial_labels` hook (wired in `hooks.jl`). See `types.jl` for what
`RouteCoveringPricingLabel` means and `extend.jl` for how a seeded label
grows from here.
"""

export initial_route_covering_pricing_labels

function initial_route_covering_pricing_labels(
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{RouteCoveringPricingLabel}
    # A route can only ever serve an active pair by visiting one of that
    # pair's two endpoints, so seeding a label at every other node would just
    # waste search on routes that can never certify anything.
    endpoints = Set{Int}()
    for (j, k) in pricing_data.active_pairs
        push!(endpoints, j)
        push!(endpoints, k)
    end

    labels = RouteCoveringPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        # One depth-1 label per relevant node: route so far is just `[node]`,
        # `time = 0` (route clock starts here), this node's own pickup clock
        # starts live at age 0, nothing served yet, and `tau`/`reduced_cost`
        # already carry the fixed `repositioning_time` cost every route pays
        # regardless of length.
        push!(labels, RouteCoveringPricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            Set{Tuple{Int, Int}}(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
            1,
        ))
    end
    return labels
end
