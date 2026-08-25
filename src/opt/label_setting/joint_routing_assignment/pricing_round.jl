"""
`AggregateODRouteJointRoutingAssignmentFormulation`'s formulation-level hooks
into `_run_pricing_round` (`round.jl`): every scenario in the mapping (no
`_pricing_scenarios` override -- `round.jl`'s default already matches),
threaded (scenarios are independent and pricing touches no Gurobi state, so
concurrent search is safe), a real starting column id (unlike Base, the
pricer-assigned id *is* the master's real key here), and a cross-scenario
merge that sorts and imposes one global `max_new_columns` budget so thread
completion order can't decide which columns enter the RMP.

Lives at the `joint_routing_assignment/` level, not under `exact/` or
`darp/`, because `_pricing_build_scenario_context` below is the one place
that picks *between* those two pricers (`formulation.pricing_mode`, stashed
as `m[:joint_routing_assignment_pricing_mode]`) -- it isn't exact-specific
or darp-specific itself, just the formulation-level wiring both plug into.
Candidate extraction (`joint_routing_assignment_pricing_candidates`) is
mode-agnostic too: both pricers price the exact same `PassengerAssignmentCandidate`s,
differing only in what they do with them once built (running-max reward
layers vs. first-commit credit -- see `exact/types.jl` vs. `darp/types.jl`).
"""

"""
    joint_routing_assignment_pricing_candidates(data, mapping, alpha, gamma_o, gamma_d, walk_cost_weight, detour_factor, scenario)
        -> Vector{PassengerAssignmentCandidate}

Turn the current RMP duals into `PassengerAssignmentCandidate`s for one scenario,
directly off `AggregateODRouteMap` -- no `MasterData`. The search's `p::Int` field is
the demand group's index, the position of `(o,d)` within `mapping.Omega_s[scenario]`
(matching the aggregate model's own convention); it's only ever compared for equality
within one scenario's pricing call, so reusing the same small dense range each scenario
is fine. `WALK_ONLY_PAIR` is excluded: a route can't certify it (see
`add_walk_variables!`'s docstring). Only `rho > 0` survives
(the pricer's reward-layer preprocessing drops the rest anyway).
"""
# ── candidate extraction from RMP duals ─────────────────────────────────────
function joint_routing_assignment_pricing_candidates(
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    alpha::Dict{Tuple{Int, Int}, Float64},
    gamma_o::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
    gamma_d::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
    walk_cost_weight::Float64,
    detour_factor::Float64,
    scenario::Int,
)::Vector{PassengerAssignmentCandidate}
    candidates = PassengerAssignmentCandidate[]
    for (p, (o, d)) in enumerate(mapping.Omega_s[scenario])
        mapping.Q_s[scenario][p] > 0 || continue     # demand group must actually have positive demand
        key2 = (scenario, p)
        a = get(alpha, key2, 0.0)                    # this demand group's coverage dual
        a > 1e-9 || continue                          # zero dual -> no candidate on this group can ever have rho > 0
        for pair in get_valid_jk_pairs(mapping, o, d)
            is_walk_only_pair(pair) && continue
            j, k = pair
            # rho_pjk = alpha_p - gamma^O_pj - gamma^D_pk - walk_cost_weight * walking_cost:
            # coverage dual minus both station-linking duals minus the
            # walking-access cost of this particular (j,k) station pair.
            rho = a - get(gamma_o, (key2, j), 0.0) - get(gamma_d, (key2, k), 0.0) -
                walk_cost_weight * od_pair_walking_cost(data, o, d, pair)
            rho > 1e-9 || continue
            ride_limit = detour_factor * get_routing_cost(data, j, k)
            push!(candidates, PassengerAssignmentCandidate(p, j, k, ride_limit, rho))
        end
    end
    return candidates
end

# ── formulation-level hooks (threading / column ids / scenario context / merge) ──
_pricing_parallel_scenarios(::AggregateODRouteJointRoutingAssignmentFormulation) = true

_pricing_next_column_id(::AggregateODRouteJointRoutingAssignmentFormulation, mapping::AggregateODRouteMap, m::JuMP.Model) =
    maximum(keys(m[:joint_routing_assignment_columns]); init=0) + 1

"""
Build one scenario's pricing context: duals -> candidates -> pricing data ->
search context (`exact/` or `darp/`, per `formulation.pricing_mode`), plus the
existing column pool restricted to this scenario. `nothing` when a scenario
has no positive-reward candidates (and therefore, for `exact/`, no
opportunities) to price at all.
"""
function _pricing_build_scenario_context(
    ::AggregateODRouteJointRoutingAssignmentFormulation, mapping::AggregateODRouteMap, s::Int,
    m::JuMP.Model, duals,
)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    walk_cost_weight = Float64(m[:joint_routing_assignment_walk_cost_weight])
    detour_factor = Float64(m[:joint_routing_assignment_detour_factor])
    candidates = joint_routing_assignment_pricing_candidates(
        data, mapping, alpha, gamma_o, gamma_d, walk_cost_weight, detour_factor, s,
    )
    isempty(candidates) && return nothing

    existing = JointRoutingAssignmentRouteColumn[
        c for c in values(m[:joint_routing_assignment_columns]) if Int(get(c.metadata, "scenario", 0)) == s
    ]

    pricing_mode = m[:joint_routing_assignment_pricing_mode]::Symbol
    if pricing_mode === :darp
        darp_pricing_data = create_joint_routing_assignment_darp_pricing_data(
            s, m[:joint_routing_assignment_nodes], m[:joint_routing_assignment_travel_cost], candidates;
            route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
            max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
            repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
            max_stops=Int(m[:joint_routing_assignment_max_stops]),
            compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
        )
        isempty(darp_pricing_data.candidates) && return nothing
        return JointRoutingAssignmentDarpSearchContext(darp_pricing_data), existing
    end

    pricing_mode === :exact || throw(ArgumentError(
        "unknown joint_routing_assignment pricing_mode $(repr(pricing_mode)) -- expected :exact or :darp",
    ))
    pricing_data = create_joint_routing_assignment_pricing_data(
        s, m[:joint_routing_assignment_nodes], m[:joint_routing_assignment_travel_cost], candidates;
        route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
        max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
        repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
        max_stops=Int(m[:joint_routing_assignment_max_stops]),
        compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
    )
    isempty(pricing_data.opportunities) && return nothing

    return JointRoutingAssignmentSearchContext(pricing_data), existing
end

"""
Sort by `(reduced_cost, tau, scenario, route)` and truncate to one global
`max_new_columns` across every scenario -- the one real behavioral difference
from `AggregateODRouteBaseFormulation`'s `_pricing_merge_scenarios` (`round.jl`'s
default, unbounded identity): scenarios are threaded here, so without a
deterministic global order thread completion order would decide which
columns enter the RMP.
"""
function _pricing_merge_scenarios(
    ::AggregateODRouteJointRoutingAssignmentFormulation, mapping::AggregateODRouteMap,
    candidates::AbstractVector, max_new_columns::Int,
)
    sorted = _sort_pricing_results_by_route(
        candidates, entry -> (entry.reduced_cost, entry.tau, entry.scenario, string(entry.payload.route)),
    )
    return sorted[1:min(length(sorted), max_new_columns)]
end
