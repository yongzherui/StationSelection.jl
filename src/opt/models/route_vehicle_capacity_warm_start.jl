"""
RouteVehicleCapacityWarmStartModel — warm start model for RouteVehicleCapacityModel.

Uses only length-2 (direct j→k) routes to produce a fast incumbent solution whose
y/z/x/α/θ values can be injected as starting hints into the full model.

With length-2 routes, each (j,k) pair has exactly one route and one segment, so
constraint (iii) simplifies to α ≤ Cap_r·θ and constraint (ii) to Σ_od x ≤ α.
The route pool is O(n²) vs exponential for multi-stop routes, making the MIP fast
to solve while still yielding meaningful station selection and assignment decisions.
"""

export RouteVehicleCapacityWarmStartModel


"""
    RouteVehicleCapacityWarmStartModel <: AbstractODModel

Warm start model for RouteVehicleCapacityModel. Identical formulation but routes are
restricted to direct j→k paths (length 2, no pooling stops). This shrinks the route
pool to O(n²), making the MIP fast to solve while still producing meaningful y/z/x/α/θ
starting values for the full model.

The solution transfers to RouteVehicleCapacityModel by matching warm start length-2
routes to the corresponding length-2 routes in the main model's route pool via station_ids.
"""
struct RouteVehicleCapacityWarmStartModel <: AbstractODModel
    k                           :: Int
    l                           :: Int
    route_regularization_weight :: Float64
    repositioning_time          :: Float64
    vehicle_capacity            :: Int
    max_walking_distance        :: Float64
    max_detour_time             :: Float64
    max_detour_ratio            :: Float64
    time_window_sec             :: Int

    function RouteVehicleCapacityWarmStartModel(
            k::Int,
            l::Int;
            route_regularization_weight :: Number = 1.0,
            repositioning_time          :: Number = 20.0,
            vehicle_capacity            :: Int    = 18,
            max_walking_distance        :: Number = 300,
            max_detour_time             :: Number = 1200,
            max_detour_ratio            :: Number = 2.0,
            time_window_sec             :: Int    = 3600
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        vehicle_capacity > 0 || throw(ArgumentError("vehicle_capacity must be positive"))
        time_window_sec > 0  || throw(ArgumentError("time_window_sec must be positive"))
        new(
            k, l,
            Float64(route_regularization_weight), Float64(repositioning_time),
            vehicle_capacity, Float64(max_walking_distance),
            Float64(max_detour_time), Float64(max_detour_ratio),
            time_window_sec
        )
    end
end


# ── Map creation ────────────────────────────────────────────────────────────────


"""
    create_map(model::RouteVehicleCapacityWarmStartModel, data) -> VehicleCapacityODMap

Build the OD mapping for the warm start model. Identical to the full model's mapping
except routes_s contains only direct length-2 routes (one per valid (j,k) pair per
time bucket), avoiding expensive DFS route generation.
"""
function create_map(
    model :: RouteVehicleCapacityWarmStartModel,
    data  :: StationSelectionData
)::VehicleCapacityODMap

    # ── 1. Index mappings ──────────────────────────────────────────────────────
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    # ── 2. Aggregated OD pairs per scenario ────────────────────────────────────
    S            = n_scenarios(data)
    Omega_s      = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s          = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
    Omega_s_t    = Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}()
    Q_s_t        = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for s in 1:S
        scenario = data.scenarios[s]
        od_count = compute_scenario_od_count(scenario)
        Omega_s[s] = collect(keys(od_count))
        Q_s[s]     = od_count
        union!(all_od_pairs, keys(od_count))

        time_to_od   = compute_time_to_od_count_mapping(scenario, model.time_window_sec)
        Omega_s_t[s] = Dict{Int, Vector{Tuple{Int, Int}}}()
        Q_s_t[s]     = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
        for (t_id, od_cnt) in time_to_od
            Omega_s_t[s][t_id] = collect(keys(od_cnt))
            Q_s_t[s][t_id]     = od_cnt
        end
    end

    # ── 3. Valid (j,k) pairs per OD pair ──────────────────────────────────────
    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs, data,
        station_id_to_array_idx, array_idx_to_station_id,
        model.max_walking_distance
    )

    # ── 4. Length-2 routes: one direct route per (j,k) per (s, t_id) ──────────
    # No DFS — just enumerate jk_set once per time bucket.
    routes_s = Dict{Int, Dict{Int, Vector{RouteData}}}()

    for s in 1:S
        routes_s[s] = Dict{Int, Vector{RouteData}}()

        for (t_id, od_pairs) in Omega_s_t[s]
            jk_set = Set{Tuple{Int, Int}}()
            for (o, d) in od_pairs
                for pair in get(valid_jk_pairs, (o, d), Tuple{Int,Int}[])
                    push!(jk_set, pair)
                end
            end

            if isempty(jk_set)
                routes_s[s][t_id] = RouteData[]
                continue
            end

            routes = RouteData[]
            for (r_idx, (j_idx, k_idx)) in enumerate(sort(collect(jk_set)))
                j_id = array_idx_to_station_id[j_idx]
                k_id = array_idx_to_station_id[k_idx]
                travel_time = get_routing_cost(data, j_id, k_id)
                push!(routes, RouteData(r_idx, [j_id, k_id], travel_time, [(j_id, k_id)]))
            end
            routes_s[s][t_id] = routes
        end
    end

    return VehicleCapacityODMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        Omega_s_t,
        Q_s_t,
        valid_jk_pairs,
        model.max_walking_distance,
        model.time_window_sec,
        routes_s
    )
end


# ── Model build ─────────────────────────────────────────────────────────────────


"""
    build_model(model::RouteVehicleCapacityWarmStartModel, data; optimizer_env=nothing)
                -> BuildResult

Build the warm start MIP. Identical to RouteVehicleCapacityModel but with length-2
routes only and upfront (non-lazy) capacity constraints.
"""
function build_model(
        model         :: RouteVehicleCapacityWarmStartModel,
        data          :: StationSelectionData;
        optimizer_env = nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    mapping = create_map(model, data)

    S = length(data.scenarios)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts   = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts      = Dict{String, Int}()

    extra_counts["n_routes"] = sum(
        sum(length(v) for v in values(mapping.routes_s[s]); init = 0)
        for s in 1:S; init = 0
    )
    extra_counts["total_od_pairs"] = sum(length(mapping.Omega_s[s]) for s in 1:S; init = 0)

    m[:vehicle_capacity] = model.vehicle_capacity

    variable_counts["station_selection"]   = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"]          = add_assignment_variables!(m, data, mapping)
    variable_counts["alpha_r_jkts"]        = add_alpha_r_jkts_variables!(m, data, mapping)
    variable_counts["theta_r_ts"]          = add_theta_r_ts_variables!(m, data, mapping)

    set_route_od_objective!(m, data, mapping;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time)

    constraint_counts["station_limit"] =
        add_station_limit_constraint!(m, data, model.l; equality = true)
    constraint_counts["scenario_activation_limit"] =
        add_scenario_activation_limit_constraints!(m, data, model.k)
    constraint_counts["activation_linking"] =
        add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] =
        add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] =
        add_assignment_to_active_constraints!(m, data, mapping)
    constraint_counts["route_capacity"] =
        add_route_capacity_constraints!(m, data, mapping)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end


# ── Warm start solution extraction ──────────────────────────────────────────────


"""
    get_warm_start_solution(model::RouteVehicleCapacityModel, data, build_result;
                            optimizer_env=nothing, silent=true, kwargs...)
                            -> Union{Dict{Symbol,Any}, Nothing}

Solve the RouteVehicleCapacityWarmStartModel (length-2 routes only) and return
starting-value hints for all variables in the full RouteVehicleCapacityModel.

α/θ hints are transferred by matching warm start length-2 routes to the corresponding
routes in the main model's route pool via station_ids.

Returns `nothing` if the warm start solve produces no feasible solution.
"""
function get_warm_start_solution(
    model        :: RouteVehicleCapacityModel,
    data         :: StationSelectionData,
    build_result;
    optimizer_env :: Union{Gurobi.Env, Nothing} = nothing,
    silent        :: Bool = true,
    kwargs...
)
    ws_model = RouteVehicleCapacityWarmStartModel(
        model.k, model.l;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time,
        vehicle_capacity            = model.vehicle_capacity,
        max_walking_distance        = model.max_walking_distance,
        max_detour_time             = model.max_detour_time,
        max_detour_ratio            = model.max_detour_ratio,
        time_window_sec             = model.time_window_sec
    )

    @info "get_warm_start_solution: building warm start model (length-2 routes only)"
    ws_build = build_model(ws_model, data; optimizer_env=optimizer_env)
    ws_m     = ws_build.model

    silent && set_silent(ws_m)
    optimize!(ws_m)

    ts = JuMP.termination_status(ws_m)
    @info "get_warm_start_solution: warm start solve complete" termination_status=string(ts)

    if !has_values(ws_m)
        @warn "get_warm_start_solution: no feasible solution found" termination_status=string(ts)
        return nothing
    end

    y_vals = JuMP.value.(ws_m[:y])
    z_vals = JuMP.value.(ws_m[:z])
    x_vals = _value_recursive(ws_m[:x])

    alpha_hints, theta_hints = _match_warm_start_routes(
        ws_build.mapping, ws_m, build_result.mapping
    )

    return Dict{Symbol, Any}(
        :y     => y_vals,
        :z     => z_vals,
        :x     => x_vals,
        :alpha => alpha_hints,
        :theta => theta_hints,
    )
end


"""
    _match_warm_start_routes(ws_mapping, ws_m, main_mapping)
                             -> (alpha_hints, theta_hints)

Match warm start α/θ values (indexed by length-2 route indices in ws_mapping) to the
corresponding routes in main_mapping via station_ids lookup.
"""
function _match_warm_start_routes(
    ws_mapping   :: VehicleCapacityODMap,
    ws_m         :: JuMP.Model,
    main_mapping :: VehicleCapacityODMap
)
    # Build lookup: (s, t_id, station_ids) → r_idx in main model
    main_route_lookup = Dict{Tuple{Int, Int, Vector{Int}}, Int}()
    for (s, t_routes) in main_mapping.routes_s
        for (t_id, routes_t) in t_routes
            for (r_idx, route) in enumerate(routes_t)
                main_route_lookup[(s, t_id, route.station_ids)] = r_idx
            end
        end
    end

    ws_alpha = ws_m[:alpha_r_jkts]
    ws_theta = ws_m[:theta_r_ts]

    alpha_hints = Dict{NTuple{5, Int}, Float64}()
    theta_hints = Dict{NTuple{3, Int}, Float64}()

    # Transfer α: warm start route → main model route via station_ids
    for ((s, r_ws, j_idx, k_idx, t_id), avar) in ws_alpha
        ws_route = ws_mapping.routes_s[s][t_id][r_ws]
        r_main   = get(main_route_lookup, (s, t_id, ws_route.station_ids), 0)
        r_main == 0 && continue
        alpha_hints[(s, r_main, j_idx, k_idx, t_id)] = JuMP.value(avar)
    end

    # Transfer θ: same matching
    for ((s, t_id, r_ws), theta_var) in ws_theta
        ws_route = ws_mapping.routes_s[s][t_id][r_ws]
        r_main   = get(main_route_lookup, (s, t_id, ws_route.station_ids), 0)
        r_main == 0 && continue
        theta_hints[(s, t_id, r_main)] = JuMP.value(theta_var)
    end

    return alpha_hints, theta_hints
end
