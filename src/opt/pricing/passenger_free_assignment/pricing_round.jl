"""
Price one scenario. Pure with respect to shared state: it only *reads* `master`
and the dual dicts, and allocates everything else itself, which is what makes the
per-scenario calls safe to run concurrently.

Note that pricing touches no Gurobi at all -- it is pure Julia label-setting --
so there is no solver thread-safety question here.
"""
function _price_one_passenger_scenario(
    master::PassengerFreeAssignmentMaster,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64},
    s::Int,
    base_column_id::Int,
    existing::Vector{PassengerFreeAssignmentRouteColumn};
    n_candidates::Int,
    max_new_columns::Int,
    time_limit::Float64,
    reduced_cost_tol::Float64,
    compensated_dominance::Bool=true,
    use_station_simple::Bool=false,
    reward_coarsening_levels::Int=0,
    allowed_stations::Union{Nothing, Set{Int}}=nothing,
)
    md = master.master_data
    candidates = passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, s)
    if !isnothing(allowed_stations)
        filter!(c -> c.origin in allowed_stations && c.destination in allowed_stations, candidates)
    end
    isempty(candidates) && return PassengerFreeAssignmentRouteColumn[], true, 0

    pricing_nodes = isnothing(allowed_stations) ? md.nodes :
        [j for j in md.nodes if j in allowed_stations]

    exact_pricing_data = create_passenger_free_assignment_pricing_data(
        s, pricing_nodes, md.travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time,
        repositioning_time=md.repositioning_time,
        max_stops=md.max_stops,
        compensated_dominance=compensated_dominance,
    )
    isempty(exact_pricing_data.opportunities) && return PassengerFreeAssignmentRouteColumn[], true, 0

    pricing_data = if reward_coarsening_levels > 0
        relaxed_candidates = coarsen_passenger_assignment_rewards(
            candidates, reward_coarsening_levels,
        )
        create_passenger_free_assignment_pricing_data(
            s, pricing_nodes, md.travel_cost, relaxed_candidates;
            route_regularization_weight=md.route_regularization_weight,
            max_wait_time=md.max_wait_time,
            repositioning_time=md.repositioning_time,
            max_stops=md.max_stops,
            compensated_dominance=compensated_dominance,
        )
    else
        exact_pricing_data
    end

    # Under reward coarsening, relaxed assignment signatures can differ from exact
    # replay signatures, so search against an empty pool and re-apply exact novelty.
    search_pool = reward_coarsening_levels > 0 ?
        PassengerFreeAssignmentRouteColumn[] : existing

    columns_s, exhausted_s, stats_s = use_station_simple ?
        passenger_free_assignment_pricing_by_station_simple_label_setting(
            pricing_data, search_pool;
            next_column_id=base_column_id,
            reduced_cost_tol=reduced_cost_tol,
            max_new_columns=max_new_columns,
            n_candidates=n_candidates,
            time_limit=time_limit,
        ) :
        passenger_free_assignment_pricing_by_label_setting(
            pricing_data, search_pool;
            next_column_id=base_column_id,
            reduced_cost_tol=reduced_cost_tol,
            max_new_columns=max_new_columns,
            n_candidates=n_candidates,
            time_limit=time_limit,
        )
    reward_coarsening_levels == 0 &&
        return columns_s, exhausted_s, stats_s.labels_generated

    best_pool_tau = Dict{Any, Float64}()
    for column in existing
        signature = _passenger_free_assignment_column_signature(column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end
    exact_by_signature = Dict{Any, NamedTuple}()
    for column in columns_s
        relaxed_rc = Float64(get(column.metadata, "reduced_cost", Inf))
        assignments, tau, exact_rc = _passenger_free_assignment_column_from_route(
            collect(Int, column.route), exact_pricing_data,
        )
        relaxed_rc <= exact_rc + 1e-6 || error(
            "reward-coarsened pricing violated relaxed_rc <= exact_rc on route " *
            "$(column.route): $(relaxed_rc) > $(exact_rc)",
        )
        isempty(assignments) && continue
        exact_rc < -reduced_cost_tol || continue
        signature = _passenger_free_assignment_column_signature(assignments)
        tau < get(best_pool_tau, signature, Inf) - 1e-9 || continue
        current = get(exact_by_signature, signature, nothing)
        if isnothing(current) || exact_rc < current.reduced_cost - 1e-9 ||
                (abs(exact_rc - current.reduced_cost) <= 1e-9 && tau < current.tau - 1e-9)
            exact_by_signature[signature] = (
                route=collect(Int, column.route), assignments=assignments,
                tau=tau, reduced_cost=exact_rc,
            )
        end
    end
    scored = sort!(collect(values(exact_by_signature));
                   by=entry -> (entry.reduced_cost, entry.tau, string(entry.route)))
    scored = scored[1:min(length(scored), max_new_columns)]
    exact_columns = PassengerFreeAssignmentRouteColumn[]
    for (offset, entry) in enumerate(scored)
        push!(exact_columns, PassengerFreeAssignmentRouteColumn(
            base_column_id + offset - 1, entry.route, entry.assignments, entry.tau;
            metadata=Dict{String, Any}(
                "scenario" => s,
                "route" => Tuple(entry.route),
                "reduced_cost" => entry.reduced_cost,
                "harvester" => "reward_coarsened",
                "reward_coarsening_levels" => reward_coarsening_levels,
            ),
        ))
    end

    certified = exhausted_s && isempty(columns_s)
    return exact_columns, certified, stats_s.labels_generated
end

"""
    _price_passenger_scenarios(...; parallel_scenarios)

The pricing subproblem separates exactly by scenario. Per-scenario searches can
run concurrently because all cross-scenario coupling lives in the master.
"""
function _price_passenger_scenarios(
    master::PassengerFreeAssignmentMaster,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64},
    next_column_id::Int;
    n_candidates::Int,
    max_new_columns::Int,
    time_limit::Float64,
    reduced_cost_tol::Float64,
    verify_reduced_costs::Bool,
    parallel_scenarios::Bool=true,
    compensated_dominance::Bool=true,
    use_station_simple::Bool=false,
    reward_coarsening_levels::Int=0,
    allowed_stations::Union{Nothing, Set{Int}}=nothing,
)
    md = master.master_data
    scenarios = sort!(collect(keys(md.passengers_by_scenario)))
    n_s = length(scenarios)

    empty_pool = PassengerFreeAssignmentRouteColumn[]
    existing_by_scenario = Dict{Int, Vector{PassengerFreeAssignmentRouteColumn}}()
    for c in values(master.columns)
        push!(get!(() -> PassengerFreeAssignmentRouteColumn[],
                   existing_by_scenario, Int(get(c.metadata, "scenario", 0))), c)
    end
    cols_by_s = Vector{Vector{PassengerFreeAssignmentRouteColumn}}(undef, n_s)
    exh_by_s = Vector{Bool}(undef, n_s)
    lab_by_s = Vector{Int}(undef, n_s)

    use_threads = parallel_scenarios && n_s > 1 && Threads.nthreads() > 1
    if use_threads
        Threads.@threads for i in 1:n_s
            cols_by_s[i], exh_by_s[i], lab_by_s[i] = _price_one_passenger_scenario(
                master, alpha, gamma_o, gamma_d, scenarios[i], next_column_id,
                get(existing_by_scenario, scenarios[i], empty_pool);
                n_candidates=n_candidates, max_new_columns=max_new_columns,
                time_limit=time_limit, reduced_cost_tol=reduced_cost_tol,
                compensated_dominance=compensated_dominance,
                use_station_simple=use_station_simple,
                reward_coarsening_levels=reward_coarsening_levels,
                allowed_stations=allowed_stations,
            )
        end
    else
        for i in 1:n_s
            cols_by_s[i], exh_by_s[i], lab_by_s[i] = _price_one_passenger_scenario(
                master, alpha, gamma_o, gamma_d, scenarios[i], next_column_id,
                get(existing_by_scenario, scenarios[i], empty_pool);
                n_candidates=n_candidates, max_new_columns=max_new_columns,
                time_limit=time_limit, reduced_cost_tol=reduced_cost_tol,
                compensated_dominance=compensated_dominance,
                use_station_simple=use_station_simple,
                reward_coarsening_levels=reward_coarsening_levels,
                allowed_stations=allowed_stations,
            )
        end
    end

    # Each scenario over-generates and ranks its own novel routes above.  Merge
    # those pools, verify every reduced cost against the master convention, then
    # impose one global column budget.  This prevents scenario order (and thread
    # completion order) from deciding which columns enter the RMP.
    verified_columns = PassengerFreeAssignmentRouteColumn[]
    exhausted = all(exh_by_s)
    labels_generated = sum(lab_by_s; init=0)

    for i in 1:n_s
        for c in cols_by_s[i]
            if verify_reduced_costs
                ok, pricer_rc, master_rc =
                    _verify_passenger_master_reduced_cost(c, md, alpha, gamma_o, gamma_d)
                ok || error(
                    "passenger pricing reduced cost $(pricer_rc) disagrees with the master's " *
                    "dual-implied $(master_rc) for column route $(c.route) -- the pricer " *
                    "and master formulations have drifted apart",
                )
            end
            push!(verified_columns, c)
        end
    end

    sort!(verified_columns; by=c -> (
        Float64(get(c.metadata, "reduced_cost", Inf)), c.tau,
        Int(get(c.metadata, "scenario", 0)), string(c.route),
    ))
    resize!(verified_columns, min(length(verified_columns), max_new_columns))

    all_columns = PassengerFreeAssignmentRouteColumn[]
    for (offset, c) in enumerate(verified_columns)
        push!(all_columns, PassengerFreeAssignmentRouteColumn(
            next_column_id + offset - 1, c.route, c.assignments, c.tau;
            metadata=c.metadata,
        ))
    end
    return all_columns, exhausted, labels_generated
end
