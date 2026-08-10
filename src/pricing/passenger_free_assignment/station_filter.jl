function _safe_reduction_ratio(before::Int, after::Int)::Float64
    before > 0 || return 0.0
    return 1.0 - after / before
end

"""
Positive-reward pricing graph size under the given duals.

`n_positive_rho` counts attractive opportunities `(p,j,k)`. `j_positive_rho`
counts endpoint stations incident to at least one attractive opportunity. Pricing
cost usually tracks the first more directly; the second measures whether the
station endpoint set itself is shrinking.
"""
function _positive_rho_stats(
    md::PassengerFreeAssignmentMasterData,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64};
    tol::Float64=1e-9,
)::NamedTuple{(:n_positive_rho, :j_positive_rho), Tuple{Int, Int}}
    n = 0
    stations = Set{Int}()
    for p in md.passengers
        a = get(alpha, p.id, 0.0)
        a > tol || continue
        for (j, k) in md.feasible_assignments[p.id]
            rho = a - get(gamma_o, (p.id, j), 0.0) - get(gamma_d, (p.id, k), 0.0) -
                md.walk_cost_weight * md.assignment_walk_cost[(p.id, j, k)]
            if rho > tol
                n += 1
                push!(stations, j)
                push!(stations, k)
            end
        end
    end
    return (n_positive_rho=n, j_positive_rho=length(stations))
end

_count_positive_rho(md, alpha, gamma_o, gamma_d; tol::Float64=1e-9) =
    _positive_rho_stats(md, alpha, gamma_o, gamma_d; tol=tol).n_positive_rho

function _candidate_positive_rho_stats(
    candidates::AbstractVector{PassengerAssignmentCandidate},
    excluded_stations::Set{Int}=Set{Int}(),
    excluded_opportunities::Set{Tuple{Int, Int, Int}}=Set{Tuple{Int, Int, Int}}(),
)::NamedTuple{(:n_positive_rho, :j_positive_rho), Tuple{Int, Int}}
    n = 0
    stations = Set{Int}()
    for c in candidates
        c.origin in excluded_stations && continue
        c.destination in excluded_stations && continue
        (c.passenger, c.origin, c.destination) in excluded_opportunities && continue
        n += 1
        push!(stations, c.origin)
        push!(stations, c.destination)
    end
    return (n_positive_rho=n, j_positive_rho=length(stations))
end

function _normal_station_reduced_cost_filter_mode(mode)::Symbol
    mode isa Bool && return mode ? :closed_form : :none
    mode isa Symbol && mode in (:none, :closed_form, :joint_lp) && return mode
    throw(ArgumentError(
        "station reduced-cost filter mode must be :none, :closed_form, or :joint_lp",
    ))
end

function _station_reduced_cost_joint_lp_filter(
    all_candidates::AbstractVector{PassengerAssignmentCandidate},
    y_value::AbstractVector{Float64},
    y_lower_rc::AbstractVector{Float64},
    optimizer_env;
    closed_tol::Float64,
    reward_tol::Float64,
    slack_tol::Float64,
)
    closed = Set(j for j in eachindex(y_value)
                 if y_value[j] <= closed_tol && y_lower_rc[j] > slack_tol)
    isempty(closed) && return (
        excluded_opportunities=Set{Tuple{Int, Int, Int}}(),
        excluded_stations=Set{Int}(),
        adjusted_rewards=Dict{Tuple{Int, Int, Int}, Float64}(),
        objective=0.0,
    )

    candidates = [c for c in all_candidates if c.reward > reward_tol &&
        (c.origin in closed || c.destination in closed)]
    isempty(candidates) && return (
        excluded_opportunities=Set{Tuple{Int, Int, Int}}(),
        excluded_stations=Set{Int}(),
        adjusted_rewards=Dict{Tuple{Int, Int, Int}, Float64}(),
        objective=0.0,
    )

    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(m)

    po_keys = sort!(collect(Set((c.passenger, c.origin) for c in candidates if c.origin in closed)))
    pd_keys = sort!(collect(Set((c.passenger, c.destination) for c in candidates if c.destination in closed)))
    po_key_set = Set(po_keys)
    pd_key_set = Set(pd_keys)
    @variable(m, delta_o[po_keys] >= 0.0)
    @variable(m, delta_d[pd_keys] >= 0.0)
    @variable(m, u[1:length(candidates)] >= 0.0)

    for (idx, c) in enumerate(candidates)
        lhs = c.reward
        c.origin in closed && (lhs -= delta_o[(c.passenger, c.origin)])
        c.destination in closed && (lhs -= delta_d[(c.passenger, c.destination)])
        @constraint(m, u[idx] >= lhs)
    end

    for j in closed
        terms = AffExpr()
        for key in po_keys
            key[2] == j && add_to_expression!(terms, delta_o[key])
        end
        for key in pd_keys
            key[2] == j && add_to_expression!(terms, delta_d[key])
        end
        @constraint(m, terms <= y_lower_rc[j])
    end
    @objective(m, Min, sum(u))
    optimize!(m)
    termination_status(m) == MOI.OPTIMAL || error(
        "station reduced-cost joint LP did not solve to optimality: $(termination_status(m))",
    )

    adjusted_rewards = Dict{Tuple{Int, Int, Int}, Float64}()
    excluded_opportunities = Set{Tuple{Int, Int, Int}}()
    for c in all_candidates
        key = (c.passenger, c.origin, c.destination)
        adjusted = c.reward
        if (c.passenger, c.origin) in po_key_set
            adjusted -= value(delta_o[(c.passenger, c.origin)])
        end
        if (c.passenger, c.destination) in pd_key_set
            adjusted -= value(delta_d[(c.passenger, c.destination)])
        end
        adjusted <= c.reward + max(1e-8, reward_tol) || error(
            "station reduced-cost joint LP increased reward for $key: " *
            "$(adjusted) > $(c.reward)",
        )
        adjusted_rewards[key] = adjusted
        adjusted <= reward_tol && push!(excluded_opportunities, key)
    end

    surviving_stats = _candidate_positive_rho_stats(
        all_candidates, Set{Int}(), excluded_opportunities,
    )
    surviving_stations = Set{Int}()
    for c in all_candidates
        (c.passenger, c.origin, c.destination) in excluded_opportunities && continue
        push!(surviving_stations, c.origin)
        push!(surviving_stations, c.destination)
    end
    all_stations = Set{Int}()
    for c in all_candidates
        push!(all_stations, c.origin)
        push!(all_stations, c.destination)
    end
    excluded_stations = setdiff(all_stations, surviving_stations)

    return (
        excluded_opportunities=excluded_opportunities,
        excluded_stations=excluded_stations,
        adjusted_rewards=adjusted_rewards,
        objective=objective_value(m),
        surviving_positive=surviving_stats.n_positive_rho,
    )
end

function _station_reduced_cost_filter_stats(
    master::PassengerFreeAssignmentMaster,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64},
    scenarios::Vector{Int},
    reduced_cost_tol::Float64,
    optimizer_env,
    mode::Symbol=:closed_form,
)
    md = master.master_data
    mode = _normal_station_reduced_cost_filter_mode(mode)
    all_candidates = PassengerAssignmentCandidate[]
    for s in scenarios
        append!(all_candidates,
            passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, s))
    end
    raw_stats = _candidate_positive_rho_stats(all_candidates)
    y_value = Float64[value(var) for var in master.y]
    y_lower_rc = Float64[reduced_cost(var) for var in master.y]
    closed_form_excluded, required = passenger_free_assignment_station_reduced_cost_eliminations(
        all_candidates, y_value, y_lower_rc;
        closed_tol=max(1e-7, reduced_cost_tol),
        slack_tol=max(1e-7, reduced_cost_tol),
    )
    excluded = closed_form_excluded
    excluded_opportunities = Set{Tuple{Int, Int, Int}}()
    joint_objective = missing
    if mode == :joint_lp
        joint = _station_reduced_cost_joint_lp_filter(
            all_candidates, y_value, y_lower_rc, optimizer_env;
            closed_tol=max(1e-7, reduced_cost_tol),
            reward_tol=1e-9,
            slack_tol=max(1e-7, reduced_cost_tol),
        )
        excluded = union(excluded, joint.excluded_stations)
        excluded_opportunities = joint.excluded_opportunities
        joint_objective = joint.objective
    end
    filtered_stats = _candidate_positive_rho_stats(
        all_candidates, excluded, excluded_opportunities,
    )
    filtered_stats.n_positive_rho <= raw_stats.n_positive_rho || error(
        "station reduced-cost filter increased positive opportunities: " *
        "$(filtered_stats.n_positive_rho) > $(raw_stats.n_positive_rho)",
    )
    filtered_stats.j_positive_rho <= raw_stats.j_positive_rho || error(
        "station reduced-cost filter increased positive endpoint stations: " *
        "$(filtered_stats.j_positive_rho) > $(raw_stats.j_positive_rho)",
    )

    closed_with_need = Int[]
    ratios = Float64[]
    for j in eachindex(y_value)
        y_value[j] <= max(1e-7, reduced_cost_tol) || continue
        required[j] > max(1e-7, reduced_cost_tol) || continue
        push!(closed_with_need, j)
        push!(ratios, y_lower_rc[j] / required[j])
    end

    return (
        mode=mode,
        excluded_stations=excluded,
        excluded_opportunities=excluded_opportunities,
        raw_positive_rho=raw_stats.n_positive_rho,
        filtered_positive_rho=filtered_stats.n_positive_rho,
        raw_positive_rho_stations=raw_stats.j_positive_rho,
        filtered_positive_rho_stations=filtered_stats.j_positive_rho,
        positive_rho_reduction=_safe_reduction_ratio(
            raw_stats.n_positive_rho, filtered_stats.n_positive_rho),
        positive_rho_station_reduction=_safe_reduction_ratio(
            raw_stats.j_positive_rho, filtered_stats.j_positive_rho),
        n_closed_stations=count(v -> v <= max(1e-7, reduced_cost_tol), y_value),
        n_closed_stations_with_positive_need=length(closed_with_need),
        n_excluded_stations=length(excluded),
        n_excluded_opportunities=length(excluded_opportunities),
        joint_lp_objective=joint_objective,
        min_slack_need_ratio=isempty(ratios) ? missing : minimum(ratios),
        mean_slack_need_ratio=isempty(ratios) ? missing : sum(ratios) / length(ratios),
        max_slack_need_ratio=isempty(ratios) ? missing : maximum(ratios),
    )
end
