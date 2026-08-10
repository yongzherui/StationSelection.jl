"""
Experimental single-route relaxation that dualizes only passenger uniqueness.

Every assignment `(p,j,k)` is represented as a separate pseudo-passenger, so
the existing label search may collect several assignment rewards belonging to
the same real passenger while still following one physical route. Multipliers
penalize every collected alternative and add the Lagrangian constant back to
give a valid lower bound on exact reduced cost.

This is deliberately not wired into column generation. It first needs to show
that repeated-passenger reward is a material relaxation error and that a few
multiplier iterations improve the bound quickly enough to beat exact pricing.
"""

export passenger_free_assignment_lagrangian_bound

function _passenger_lagrangian_candidates(
    candidates::AbstractVector{PassengerAssignmentCandidate},
    lambda::Dict{Int, Float64},
)
    relaxed = PassengerAssignmentCandidate[]
    assignment_for_pseudo = Dict{Int, PassengerAssignmentCandidate}()
    for (pseudo, candidate) in enumerate(candidates)
        adjusted = candidate.reward - get(lambda, candidate.passenger, 0.0)
        push!(relaxed, PassengerAssignmentCandidate(
            pseudo, candidate.origin, candidate.destination,
            candidate.ride_limit, adjusted,
        ))
        assignment_for_pseudo[pseudo] = candidate
    end
    return relaxed, assignment_for_pseudo
end

"""
    passenger_free_assignment_lagrangian_bound(exact_pricing_data, candidates; kwargs...)

Return `(lower_bound, certified, stats)`. `lower_bound` is valid only when
`certified` is true; otherwise it is `-Inf`. `stats.best_exact_replay_rc` is the
best exact reduced cost among the relaxed maximizing routes and can be used to
assess harvesting quality independently of certification.

`step_fraction` scales the projected subgradient update by the largest positive
reward. This is an experimental bound-quality census, not yet a production
multiplier optimizer.
"""
function passenger_free_assignment_lagrangian_bound(
    exact_pricing_data::PassengerFreeAssignmentPricingData,
    candidates::AbstractVector{PassengerAssignmentCandidate};
    max_iterations::Int=5,
    step_fraction::Float64=0.25,
    time_limit::Float64=30.0,
    reduced_cost_tol::Float64=1e-6,
)
    max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
    step_fraction > 0 || throw(ArgumentError("step_fraction must be positive"))
    time_limit > 0 || throw(ArgumentError("time_limit must be positive"))

    passengers = sort!(unique(candidate.passenger for candidate in candidates))
    lambda = Dict(passenger => 0.0 for passenger in passengers)
    reward_scale = maximum((max(0.0, candidate.reward) for candidate in candidates); init=0.0)
    best_bound = -Inf
    best_exact_replay_rc = Inf
    total_labels = 0
    max_multiplicity = 0
    repeated_passengers = 0
    iterations_run = 0
    all_exhausted = true
    t_start = time()

    for iteration in 1:max_iterations
        remaining = time_limit - (time() - t_start)
        if remaining <= 0
            all_exhausted = false
            break
        end
        iterations_run = iteration
        relaxed_candidates, assignment_for_pseudo =
            _passenger_lagrangian_candidates(candidates, lambda)
        relaxed_data = create_passenger_free_assignment_pricing_data(
            exact_pricing_data.scenario,
            exact_pricing_data.nodes,
            exact_pricing_data.travel_cost,
            relaxed_candidates;
            route_regularization_weight=exact_pricing_data.route_regularization_weight,
            max_wait_time=exact_pricing_data.max_wait_time,
            repositioning_time=exact_pricing_data.repositioning_time,
            max_stops=exact_pricing_data.max_stops,
            compensated_dominance=exact_pricing_data.compensated_dominance,
        )
        labels, exhausted, search_stats = _enumerate_passenger_free_assignment_pricing_labels(
            relaxed_data;
            time_limit=remaining,
            reduced_cost_tol=reduced_cost_tol,
            use_reduced_cost_pruning=false,
        )
        total_labels += search_stats.labels_generated
        if !exhausted
            all_exhausted = false
            break
        end

        if isempty(labels)
            route_rc = exact_pricing_data.route_regularization_weight *
                exact_pricing_data.repositioning_time
            counts = Dict{Int, Int}()
        else
            best_label = argmin(label -> label.reduced_cost, labels)
            route_rc = best_label.reduced_cost
            pseudo_assignments = _replay_passenger_free_assignment_route(
                best_label.route, relaxed_data,
            )
            counts = Dict{Int, Int}()
            for pseudo in keys(pseudo_assignments)
                passenger = assignment_for_pseudo[pseudo].passenger
                counts[passenger] = get(counts, passenger, 0) + 1
            end
            _assignments, _tau, exact_rc = _passenger_free_assignment_column_from_route(
                best_label.route, exact_pricing_data,
            )
            best_exact_replay_rc = min(best_exact_replay_rc, exact_rc)
        end

        bound = route_rc - sum(values(lambda); init=0.0)
        best_bound = max(best_bound, bound)
        iteration_max = maximum(values(counts); init=0)
        max_multiplicity = max(max_multiplicity, iteration_max)
        repeated_passengers += count(>(1), values(counts))

        step = reward_scale * step_fraction / sqrt(iteration)
        for passenger in passengers
            multiplicity = get(counts, passenger, 0)
            lambda[passenger] = max(0.0, lambda[passenger] + step * (multiplicity - 1))
        end
    end

    certified = all_exhausted && iterations_run > 0
    return certified ? best_bound : -Inf, certified, (
        iterations=iterations_run,
        labels_generated=total_labels,
        max_passenger_multiplicity=max_multiplicity,
        repeated_passenger_count=repeated_passengers,
        best_exact_replay_rc=best_exact_replay_rc,
        multipliers=lambda,
        proves_no_improving_column=certified && best_bound >= -reduced_cost_tol,
        wall_seconds=time() - t_start,
    )
end
