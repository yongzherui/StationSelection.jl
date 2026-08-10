"""Passenger-specific decremental state-space relaxation for PFA pricing."""

export passenger_free_assignment_passenger_dssr_bound

function _passenger_dssr_candidates(
    candidates::AbstractVector{PassengerAssignmentCandidate},
    exact_passengers::Set{Int},
)
    transformed = PassengerAssignmentCandidate[]
    real_for_state = Dict{Int, Int}()
    # Negative ids cannot collide with the positive passenger ids used by the
    # master. Exact passengers share their real id; every relaxed assignment
    # receives its own id and can therefore be rewarded independently.
    pseudo = -1
    for candidate in candidates
        state_id = if candidate.passenger in exact_passengers
            candidate.passenger
        else
            id = pseudo
            pseudo -= 1
            id
        end
        push!(transformed, PassengerAssignmentCandidate(
            state_id, candidate.origin, candidate.destination,
            candidate.ride_limit, candidate.reward,
        ))
        real_for_state[state_id] = candidate.passenger
    end
    return transformed, real_for_state
end

"""
    passenger_free_assignment_passenger_dssr_bound(exact_data, candidates; kwargs...)

Run passenger-DSSR to exhaustion or `max_rounds`. Noncritical passengers may be
rewarded once per assignment; passengers duplicated by the relaxed best route
are promoted to exact max-reward state. Returns a valid lower bound only when
the final inner search exhausts. `stats.exact` means the relaxed maximizing
route has no duplicated passenger and its relaxed value equals its exact replay
value; in that case the bound is the exact pricing optimum even if other
passengers have not been promoted.
"""
function passenger_free_assignment_passenger_dssr_bound(
    exact_data::PassengerFreeAssignmentPricingData,
    candidates::AbstractVector{PassengerAssignmentCandidate};
    initial_exact_passengers::AbstractSet{Int}=Set{Int}(),
    max_rounds::Int=20,
    time_limit::Float64=30.0,
    reduced_cost_tol::Float64=1e-6,
)
    max_rounds > 0 || throw(ArgumentError("max_rounds must be positive"))
    time_limit > 0 || throw(ArgumentError("time_limit must be positive"))
    exact_passengers = Set{Int}(initial_exact_passengers)
    bound_trajectory = Float64[]
    promoted_by_round = Vector{Vector{Int}}()
    total_labels = 0
    best_exact_replay_rc = Inf
    final_exact = false
    exhausted = true
    t_start = time()

    for _round in 1:max_rounds
        remaining = time_limit - (time() - t_start)
        if remaining <= 0
            exhausted = false
            break
        end
        relaxed_candidates, real_for_state =
            _passenger_dssr_candidates(candidates, exact_passengers)
        relaxed_data = create_passenger_free_assignment_pricing_data(
            exact_data.scenario, exact_data.nodes, exact_data.travel_cost,
            relaxed_candidates;
            route_regularization_weight=exact_data.route_regularization_weight,
            max_wait_time=exact_data.max_wait_time,
            repositioning_time=exact_data.repositioning_time,
            max_stops=exact_data.max_stops,
            max_visits_per_node=exact_data.max_visits_per_node,
            compensated_dominance=exact_data.compensated_dominance,
        )
        labels, round_exhausted, search_stats =
            _enumerate_passenger_free_assignment_pricing_labels(
                relaxed_data;
                time_limit=remaining,
                reduced_cost_tol=reduced_cost_tol,
                max_visits_per_node=relaxed_data.max_visits_per_node,
                use_reduced_cost_pruning=false,
            )
        total_labels += search_stats.labels_generated
        if !round_exhausted
            exhausted = false
            break
        end
        if isempty(labels)
            bound = exact_data.route_regularization_weight * exact_data.repositioning_time
            push!(bound_trajectory, bound)
            push!(promoted_by_round, Int[])
            final_exact = true
            break
        end

        best_label = argmin(label -> label.reduced_cost, labels)
        bound = best_label.reduced_cost
        push!(bound_trajectory, bound)
        state_assignments = _replay_passenger_free_assignment_route(
            best_label.route, relaxed_data,
        )
        counts = Dict{Int, Int}()
        for state_id in keys(state_assignments)
            real = real_for_state[state_id]
            counts[real] = get(counts, real, 0) + 1
        end
        _assignments, _tau, exact_rc = _passenger_free_assignment_column_from_route(
            best_label.route, exact_data,
        )
        best_exact_replay_rc = min(best_exact_replay_rc, exact_rc)
        offenders = sort!([
            passenger for (passenger, count) in counts
            if count > 1 && passenger ∉ exact_passengers
        ])
        push!(promoted_by_round, offenders)
        if isempty(offenders)
            # With no duplicate real passenger, relaxed and exact reward agree on
            # this route. Since it minimizes a lower-bounding relaxation, equality
            # sandwiches it to the exact optimum.
            abs(bound - exact_rc) <= 1e-6 || error(
                "passenger-DSSR route without duplicates disagrees with exact replay: " *
                "$(bound) vs $(exact_rc)",
            )
            final_exact = true
            break
        end
        union!(exact_passengers, offenders)
    end

    certified = exhausted && !isempty(bound_trajectory)
    lower_bound = certified ? last(bound_trajectory) : -Inf
    return lower_bound, certified, (
        exact=final_exact,
        rounds=length(bound_trajectory),
        bound_trajectory=bound_trajectory,
        promoted_by_round=promoted_by_round,
        n_exact_passengers=length(exact_passengers),
        labels_generated=total_labels,
        best_exact_replay_rc=best_exact_replay_rc,
        proves_no_improving_column=certified && lower_bound >= -reduced_cost_tol,
        wall_seconds=time() - t_start,
    )
end
