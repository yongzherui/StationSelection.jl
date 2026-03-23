"""
RouteVehicleCapacityWarmStartModel — warm start model for RouteVehicleCapacityModel.

Solves only for y/z/x (station selection + assignment, walking cost objective) with no
routes, no α, and no θ. The much-smaller MIP gives a fast feasible y/z/x solution.
α/θ hints for the main model are then derived analytically from the x values by
greedily assigning demand to the cheapest route serving each (j,k) leg.
"""

export RouteVehicleCapacityWarmStartModel


"""
    RouteVehicleCapacityWarmStartModel <: AbstractODModel

Warm start model for RouteVehicleCapacityModel. Solves only for y/z/x with a walking
cost objective — no routes, no α, no θ, no route capacity constraints. This is fast
to solve. α/θ hints are derived analytically from the x solution using the main
model's route pool.
"""
struct RouteVehicleCapacityWarmStartModel <: AbstractODModel
    k                    :: Int
    l                    :: Int
    max_walking_distance :: Float64
    time_window_sec      :: Int

    function RouteVehicleCapacityWarmStartModel(
            k::Int,
            l::Int;
            max_walking_distance :: Number = 300,
            time_window_sec      :: Int    = 3600
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        time_window_sec > 0 || throw(ArgumentError("time_window_sec must be positive"))
        new(k, l, Float64(max_walking_distance), time_window_sec)
    end
end


# ── Map creation ────────────────────────────────────────────────────────────────


"""
    create_map(model::RouteVehicleCapacityWarmStartModel, data) -> VehicleCapacityODMap

Build the OD mapping for the warm start model. No routes are generated (routes_s is
empty) since the warm start solves only for y/z/x.
"""
function create_map(
    model :: RouteVehicleCapacityWarmStartModel,
    data  :: StationSelectionData
)::VehicleCapacityODMap

    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

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

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs, data,
        station_id_to_array_idx, array_idx_to_station_id,
        model.max_walking_distance
    )

    # No routes generated — warm start solves y/z/x only
    routes_s = Dict{Int, Dict{Int, Vector{RouteData}}}()

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

Build the warm start MIP: y/z/x variables, walking cost objective, no routes/α/θ.
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
    extra_counts      = Dict{String, Int}(
        "total_od_pairs" => sum(length(mapping.Omega_s[s]) for s in 1:S; init = 0)
    )

    variable_counts["station_selection"]   = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"]          = add_assignment_variables!(m, data, mapping)

    # Walking cost only — no route penalty since there are no θ variables
    x   = m[:x]
    obj = AffExpr(0.0)
    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            for (od_idx, (o, d)) in enumerate(od_pairs)
                x_od = get(get(x[s], t_id, Dict{Int, Vector{VariableRef}}()), od_idx, VariableRef[])
                isempty(x_od) && continue
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (pair_idx, (j, k)) in enumerate(valid_pairs)
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]
                    cost = get_walking_cost(data, o, j_id) + get_walking_cost(data, k_id, d)
                    add_to_expression!(obj, cost, x_od[pair_idx])
                end
            end
        end
    end
    @objective(m, Min, obj)

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

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end


# ── Warm start solution extraction ──────────────────────────────────────────────


"""
    get_warm_start_solution(model::RouteVehicleCapacityModel, data, build_result;
                            optimizer_env=nothing, silent=true, kwargs...)
                            -> Union{Dict{Symbol,Any}, Nothing}

Solve RouteVehicleCapacityWarmStartModel (y/z/x only, no routes) to get a fast
incumbent y/z/x solution, then derive α/θ hints analytically from the x values
using the main model's route pool.

Returns `nothing` if the warm start solve produces no feasible solution.
"""
function get_warm_start_solution(
    model        :: RouteVehicleCapacityModel,
    data         :: StationSelectionData,
    build_result;
    optimizer_env :: Union{Gurobi.Env, Nothing} = nothing,
    silent        :: Bool = false,
    kwargs...
)
    ws_model = RouteVehicleCapacityWarmStartModel(
        model.k, model.l;
        max_walking_distance = model.max_walking_distance,
        time_window_sec      = model.time_window_sec
    )

    println("  [warm start] building model (y/z/x only, no routes)...")
    flush(stdout)
    ws_build = build_model(ws_model, data; optimizer_env=optimizer_env)
    ws_m     = ws_build.model

    silent && set_silent(ws_m)

    println("  [warm start] solving...")
    flush(stdout)
    optimize!(ws_m)

    ts = JuMP.termination_status(ws_m)
    println("  [warm start] solve complete: $(string(ts))")
    flush(stdout)

    if !has_values(ws_m)
        @warn "get_warm_start_solution: no feasible solution found" termination_status=string(ts)
        return nothing
    end

    y_vals = JuMP.value.(ws_m[:y])
    z_vals = JuMP.value.(ws_m[:z])
    x_vals = _value_recursive(ws_m[:x])

    println("  [warm start] deriving α/θ hints from x solution...")
    flush(stdout)
    alpha_hints, theta_hints = _derive_alpha_theta_hints(
        x_vals, ws_build.mapping, build_result.mapping, model.vehicle_capacity
    )

    # Correct x: reassign demand away from (j,k) pairs not covered by any route so that
    # constraint (ii) (Σ x ≤ Σ_r α) is not violated by the warm start hint.
    println("  [warm start] correcting x hints for route coverage...")
    flush(stdout)
    x_vals = _correct_x_for_route_coverage(x_vals, alpha_hints, ws_build.mapping)

    return Dict{Symbol, Any}(
        :y     => y_vals,
        :z     => z_vals,
        :x     => x_vals,
        :alpha => alpha_hints,
        :theta => theta_hints,
    )
end


"""
    _correct_x_for_route_coverage(x_vals, alpha_hints, ws_mapping) -> x_vals

For each OD pair in each (s, t_id), zero out x values assigned to (j,k) legs that have
no alpha coverage (i.e. no route in the main model serves that leg). Reassign the demand
to the first covered (j,k) leg so that constraint (ii) (Σ x ≤ Σ_r α) is satisfied.

Modifies and returns x_vals in place.
"""
function _correct_x_for_route_coverage(
    x_vals      :: Vector,
    alpha_hints :: Dict{NTuple{5,Int}, Float64},
    ws_mapping  :: VehicleCapacityODMap
)
    # Build set of (s, t_id, j_idx, k_idx) legs that have at least one alpha hint
    covered = Set{NTuple{4,Int}}()
    for (s, _r, j_idx, k_idx, t_id) in keys(alpha_hints)
        push!(covered, (s, t_id, j_idx, k_idx))
    end

    S = length(x_vals)
    n_zeroed = 0
    n_reassigned = 0

    for s in 1:S
        for (t_id, od_pairs) in ws_mapping.Omega_s_t[s]
            od_dict = get(x_vals[s], t_id, nothing)
            isnothing(od_dict) && continue
            for (od_idx, (o, d)) in enumerate(od_pairs)
                pair_vals = get(od_dict, od_idx, nothing)
                isnothing(pair_vals) && continue
                isempty(pair_vals) && continue

                valid_pairs = get_valid_jk_pairs(ws_mapping, o, d)
                demand      = sum(pair_vals)
                demand <= 0 && continue

                # Find first covered pair index
                best_idx = 0
                for (pair_idx, (j_idx, k_idx)) in enumerate(valid_pairs)
                    pair_idx > length(pair_vals) && break
                    if (s, t_id, j_idx, k_idx) ∈ covered
                        best_idx = pair_idx
                        break
                    end
                end

                # Count uncovered demand before zeroing
                uncovered = sum(
                    pair_vals[pi]
                    for (pi, (j, k)) in enumerate(valid_pairs)
                    if pi <= length(pair_vals) && (s, t_id, j, k) ∉ covered;
                    init = 0.0
                )
                uncovered > 0 && (n_zeroed += 1)

                # Zero all, reassign demand to first covered pair
                fill!(pair_vals, 0.0)
                if best_idx > 0
                    pair_vals[best_idx] = demand
                    uncovered > 0 && (n_reassigned += 1)
                end
                # If best_idx == 0: no covered pair — all x left at 0 (partial infeasibility)
            end
        end
    end

    if n_zeroed > 0
        println("  [warm start] corrected $(n_zeroed) OD-pair slots with uncovered (j,k) legs; $(n_reassigned) reassigned to a covered pair")
    else
        println("  [warm start] all x assignments already covered by routes")
    end

    return x_vals
end


"""
    _derive_alpha_theta_hints(x_vals, ws_mapping, main_mapping, vehicle_capacity)
                              -> (alpha_hints, theta_hints)

From a warm start x solution, greedily assign demand for each (j,k,t,s) to the
cheapest route in main_mapping that serves (j,k), then derive minimum θ from
segment loads.
"""
function _derive_alpha_theta_hints(
    x_vals          :: Vector,
    ws_mapping      :: VehicleCapacityODMap,
    main_mapping    :: VehicleCapacityODMap,
    vehicle_capacity :: Int
)
    S = length(x_vals)

    # ── Step 1: aggregate x values into demand per (s, t_id, j_idx, k_idx) ────
    demand_jkts = Dict{NTuple{4, Int}, Float64}()

    for s in 1:S
        for (t_id, od_dict) in x_vals[s]
            od_pairs = get(ws_mapping.Omega_s_t[s], t_id, Tuple{Int,Int}[])
            for (od_idx, pair_vals) in od_dict
                od_idx > length(od_pairs) && continue
                (o, d) = od_pairs[od_idx]
                valid_pairs = get_valid_jk_pairs(ws_mapping, o, d)
                for (pair_idx, (j_idx, k_idx)) in enumerate(valid_pairs)
                    pair_idx > length(pair_vals) && break
                    v = pair_vals[pair_idx]
                    v <= 0 && continue
                    key = (s, t_id, j_idx, k_idx)
                    demand_jkts[key] = get(demand_jkts, key, 0.0) + v
                end
            end
        end
    end

    # ── Step 2: for each (j,k,t,s) with demand, assign to cheapest route ──────
    alpha_hints = Dict{NTuple{5, Int}, Float64}()

    for ((s, t_id, j_idx, k_idx), demand) in demand_jkts
        demand <= 0 && continue
        routes_t = get(get(main_mapping.routes_s, s, Dict{Int, Vector{RouteData}}()), t_id, RouteData[])

        best_r    = 0
        best_time = Inf
        for (r_idx, route) in enumerate(routes_t)
            _route_serves_jk(route, j_idx, k_idx, main_mapping.array_idx_to_station_id) || continue
            if route.travel_time < best_time
                best_time = route.travel_time
                best_r    = r_idx
            end
        end
        best_r == 0 && continue

        alpha_hints[(s, best_r, j_idx, k_idx, t_id)] = demand
    end

    # ── Step 3: derive θ from segment loads ────────────────────────────────────
    seg_load    = Dict{NTuple{4, Int}, Float64}()  # (s, t_id, r_idx, l) → load
    theta_hints = Dict{NTuple{3, Int}, Float64}()

    for ((s, r_idx, j_idx, k_idx, t_id), alpha_val) in alpha_hints
        alpha_val <= 0 && continue
        routes_t = get(get(main_mapping.routes_s, s, Dict{Int, Vector{RouteData}}()), t_id, RouteData[])
        r_idx > length(routes_t) && continue
        route  = routes_t[r_idx]
        n_segs = length(route.station_ids) - 1
        for l in 1:n_segs
            compute_beta_r_jkl(route, j_idx, k_idx, l,
                                main_mapping.array_idx_to_station_id) || continue
            seg_key = (s, t_id, r_idx, l)
            seg_load[seg_key] = get(seg_load, seg_key, 0.0) + alpha_val
        end
    end

    for ((s, t_id, r_idx, _l), load) in seg_load
        theta_key = (s, t_id, r_idx)
        min_theta = ceil(load / vehicle_capacity)
        theta_hints[theta_key] = max(get(theta_hints, theta_key, 0.0), min_theta)
    end

    return alpha_hints, theta_hints
end
