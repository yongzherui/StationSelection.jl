"""
Fleet-size search solver for RouteVehicleCapacityModel.

Solves RouteFleetLimitModel (delay_weight=0) with increasing fleet size F until
the fleet constraint is strictly non-binding: Σ_r θ^r_{ts} < F for every (t,s).
At that point the fleet bound is not limiting the solution, so the result is
equivalent to the unconstrained RouteVehicleCapacityModel optimum.

Advantages over solving RouteVehicleCapacityModel directly:
  - Bounded θ tightens the LP relaxation, often accelerating MIP solve.
  - Each iteration warm-starts the next via MIP start hints.
  - The route map (expensive DFS + delay coefficients) is built only once.
"""

export run_opt_fleet_search


"""
    run_opt_fleet_search(model, data; ...) -> OptResult

Solve `RouteVehicleCapacityModel` via an increasing-fleet-size search over
`RouteFleetLimitModel` (delay_weight=0) until the fleet constraint becomes
strictly non-binding for all time windows and scenarios.

# Keyword arguments
- `optimizer_env`: shared Gurobi environment (created fresh if not provided)
- `silent::Bool=false`: suppress Gurobi output
- `mip_gap::Union{Float64,Nothing}=nothing`: per-iteration MIP gap tolerance
- `fleet_search_start::Int=1`: initial fleet size F
- `fleet_search_max::Union{Int,Nothing}=20`: hard cap on F
- `fleet_size_increment::Int=1`: how much to increase F per iteration
- `unmet_demand_penalty::Float64=1e9`: objective penalty per unserved passenger-leg;
  must be large enough that the solver prefers serving all demand over saving route cost
- `show_counts::Bool=false`: print variable/constraint counts for the first iteration
"""
function run_opt_fleet_search(
        model               :: RouteVehicleCapacityModel,
        data                :: StationSelectionData;
        optimizer_env                       = nothing,
        silent              :: Bool         = false,
        mip_gap             :: Union{Float64, Nothing} = nothing,
        fleet_search_start  :: Int          = 1,
        fleet_search_max    :: Union{Int, Nothing} = 20,
        fleet_size_increment :: Int         = 1,
        unmet_demand_penalty :: Float64     = 1e9,
        show_counts         :: Bool         = false
    ) :: OptResult

    start_time = now()

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # ── 1. Build FleetLimitODMap once (expensive: DFS route generation) ────────
    @info "[fleet search] building route map (once)"
    map_start = now()

    proxy_fleet_model = RouteFleetLimitModel(
        model.k, model.l;
        fleet_size                  = fleet_search_start,
        route_regularization_weight = model.route_regularization_weight,
        delay_weight                = 0.0,
        repositioning_time          = model.repositioning_time,
        unmet_demand_penalty        = unmet_demand_penalty,
        vehicle_capacity            = model.vehicle_capacity,
        max_route_travel_time       = model.max_route_travel_time,
        max_walking_distance        = model.max_walking_distance,
        max_detour_time             = model.max_detour_time,
        max_detour_ratio            = model.max_detour_ratio,
        time_window_sec             = model.time_window_sec,
        max_stations_visited        = model.max_stations_visited,
        stop_dwell_time             = model.stop_dwell_time,
        routes_file                 = model.routes_file,
    )
    base_mapping = create_fleet_limit_od_map(proxy_fleet_model, data)
    map_time_sec = Dates.value(now() - map_start) / 1000
    @info "[fleet search] route map built" map_time_sec=map_time_sec

    # ── 2. Compute upper bound on F ────────────────────────────────────────────
    # Safe upper bound: total number of routes across all (s, t_id).
    # In practice convergence happens much earlier.
    S = length(data.scenarios)
    total_routes = sum(
        sum(length(v) for v in values(base_mapping.inner.routes_s[s]); init = 0)
        for s in 1:S; init = 0
    )
    F_max = isnothing(fleet_search_max) ? max(total_routes, fleet_search_start) : fleet_search_max

    # ── 3. Precompute per-(s, t_id) theta grouping key set ────────────────────
    # Used at every iteration to check binding; build once from the fixed route set.
    st_pairs = Set{Tuple{Int, Int}}()
    for s in 1:S
        for t_id in keys(base_mapping.inner.routes_s[s])
            push!(st_pairs, (s, t_id))
        end
    end

    # ── 4. Fleet size search loop ──────────────────────────────────────────────
    F = fleet_search_start
    iteration = 0
    last_build_result = nothing
    last_m = nothing
    last_term_status = MOI.OTHER_ERROR

    while F <= F_max
        iteration += 1
        @info "[fleet search] iteration" F=F F_max=F_max iteration=iteration

        # Re-use inner map and delay_coeff; only fleet_size changes (cheap)
        mapping_F = FleetLimitODMap(base_mapping.inner, F, base_mapping.delay_coeff)

        fleet_model_F = RouteFleetLimitModel(
            model.k, model.l;
            fleet_size                  = F,
            route_regularization_weight = model.route_regularization_weight,
            delay_weight                = 0.0,
            repositioning_time          = model.repositioning_time,
            unmet_demand_penalty        = unmet_demand_penalty,
            vehicle_capacity            = model.vehicle_capacity,
            max_route_travel_time       = model.max_route_travel_time,
            max_walking_distance        = model.max_walking_distance,
            max_detour_time             = model.max_detour_time,
            max_detour_ratio            = model.max_detour_ratio,
            time_window_sec             = model.time_window_sec,
            max_stations_visited        = model.max_stations_visited,
            stop_dwell_time             = model.stop_dwell_time,
            routes_file                 = model.routes_file,
        )

        build_result = _build_fleet_limit_milp(fleet_model_F, data, mapping_F;
            optimizer_env=optimizer_env)
        m = build_result.model

        if show_counts && iteration == 1
            _print_counts("Variables",   build_result.counts.variables)
            _print_counts("Constraints", build_result.counts.constraints)
            _print_counts("Extras",      build_result.counts.extras)
        end

        if silent
            set_silent(m)
        end
        if !isnothing(mip_gap)
            set_optimizer_attribute(m, "MIPGap", mip_gap)
        end

        optimize!(m)

        term_status = JuMP.termination_status(m)
        @info "[fleet search] solved" F=F term_status=string(term_status)

        if term_status != MOI.OPTIMAL
            @warn "[fleet search] non-optimal at F=$F: $term_status — increasing F"
            F += fleet_size_increment
            continue
        end

        # ── Check non-binding: max_{t,s} Σ_r θ^r_{ts} < F ────────────────────
        theta_r_ts = m[:theta_r_ts]
        v_jkts     = m[:v_jkts]

        # Sum θ per (s, t_id)
        st_theta = Dict{Tuple{Int, Int}, Float64}()
        for ((s, t_id, _r), theta_var) in theta_r_ts
            st_theta[(s, t_id)] = get(st_theta, (s, t_id), 0.0) + value(theta_var)
        end
        max_theta = isempty(st_theta) ? 0.0 : maximum(values(st_theta))

        # Max unmet demand across all (j,k,t,s)
        max_v = isempty(v_jkts) ? 0.0 : maximum(value(v) for v in values(v_jkts))

        # θ is integer; strictly non-binding means max_theta ≤ F - 1
        fleet_nonbinding = max_theta < F - 0.5
        demand_served    = max_v < 0.5   # integer v, so effectively v=0 everywhere

        @info "[fleet search] binding check" F=F max_theta=round(max_theta;digits=1) max_v=round(max_v;digits=2) fleet_nonbinding=fleet_nonbinding demand_served=demand_served

        last_build_result = build_result
        last_m            = m
        last_term_status  = term_status

        if fleet_nonbinding && demand_served
            @info "[fleet search] converged: fleet non-binding and demand fully served" F=F
            break
        end

        if fleet_nonbinding && !demand_served
            # Fleet is not the bottleneck, but demand is still unmet — the original
            # RouteVehicleCapacityModel would also be infeasible for this instance.
            @warn "[fleet search] fleet non-binding at F=$F but unmet demand persists (max_v=$(round(max_v;digits=2))). Original model likely infeasible."
            break
        end

        F += fleet_size_increment
    end

    # ── 5. Handle non-convergence ──────────────────────────────────────────────
    if isnothing(last_build_result) || last_term_status != MOI.OPTIMAL
        @warn "[fleet search] did not converge within F_max=$F_max"
        runtime_sec = Dates.value(now() - start_time) / 1000
        # Return a dummy OptResult signalling failure; last_m may be nothing
        empty_m = isnothing(last_m) ? Model() : last_m
        return OptResult(
            MOI.OTHER_ERROR, nothing, nothing,
            runtime_sec,
            empty_m,
            base_mapping.inner,
            nothing, nothing, nothing,
            Dict{String, Any}("fleet_search_iterations" => iteration, "fleet_search_F" => F)
        )
    end

    # ── 6. Package result (expose inner VehicleCapacityODMap as mapping) ───────
    runtime_sec = Dates.value(now() - start_time) / 1000

    obj = JuMP.objective_value(last_m)
    x_val = _value_recursive(last_m[:x])
    y_val = _value_recursive(last_m[:y])

    return OptResult(
        last_term_status,
        obj,
        (x_val, y_val),
        runtime_sec,
        last_m,
        base_mapping.inner,   # VehicleCapacityODMap — compatible with downstream analysis
        nothing,
        last_build_result.counts,
        nothing,
        Dict{String, Any}(
            "fleet_search_iterations" => iteration,
            "fleet_search_F"          => F,
            "map_time_sec"            => map_time_sec,
        )
    )
end
