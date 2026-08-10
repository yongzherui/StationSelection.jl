"""
Exact destination-only completion oracle after the pickup cutoff `W`.

Once `label.time >= max_wait_time`, no station visit can open or reset a pickup
clock. With metric travel times and nonnegative travel cost, an optimal suffix
does not revisit a suffix station: a second visit is later (so it cannot certify
more from the same fixed origins), and deleting the intervening cycle is no more
expensive and reaches every later station no later. The residual problem can
therefore be enumerated as an elementary destination path.

This is initially a measurement oracle, not a hot-loop bound. It establishes the
exact post-W completion value used to assess later memoized/time-bucket bounds.
"""

export passenger_free_assignment_post_w_completion

function passenger_free_assignment_post_w_completion(
    label::PassengerFreeAssignmentPricingLabel,
    pricing_data::PassengerFreeAssignmentPricingData;
    time_limit::Float64=30.0,
    max_suffix_stops::Int=length(pricing_data.nodes),
)
    label.time + 1e-9 >= pricing_data.max_wait_time || throw(ArgumentError(
        "post-W completion requires label.time >= max_wait_time",
    ))
    time_limit > 0 || throw(ArgumentError("time_limit must be positive"))
    max_suffix_stops >= 0 || throw(ArgumentError("max_suffix_stops must be nonnegative"))

    best = label
    stack = Tuple{PassengerFreeAssignmentPricingLabel, Set{Int}, Int}[
        (label, Set{Int}([label.current]), 0),
    ]
    states = 0
    exhausted = true
    t_start = time()
    while !isempty(stack)
        if time() - t_start > time_limit
            exhausted = false
            break
        end
        current, suffix_visited, depth = pop!(stack)
        states += 1
        if current.reduced_cost < best.reduced_cost - 1e-9 ||
                (abs(current.reduced_cost - best.reduced_cost) <= 1e-9 && current.tau < best.tau)
            best = current
        end
        depth >= max_suffix_stops && continue
        current.route_length >= pricing_data.max_stops && continue
        next_nodes = _passenger_free_assignment_candidate_next_nodes(current, pricing_data)
        for next_node in next_nodes
            next_node in suffix_visited && continue
            for child in extend_passenger_free_assignment_pricing_label(
                    current, next_node, pricing_data)
                visited = copy(suffix_visited)
                push!(visited, next_node)
                push!(stack, (child, visited, depth + 1))
            end
        end
    end
    return best, exhausted, (
        states=states,
        suffix_stops=length(best.route) - length(label.route),
        improvement=label.reduced_cost - best.reduced_cost,
        wall_seconds=time() - t_start,
    )
end
