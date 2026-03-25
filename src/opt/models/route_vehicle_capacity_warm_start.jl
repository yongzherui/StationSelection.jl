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
        Omega_s[s] = sort!(collect(keys(od_count)))
        Q_s[s]     = od_count
        union!(all_od_pairs, keys(od_count))

        time_to_od   = compute_time_to_od_count_mapping(scenario, model.time_window_sec)
        Omega_s_t[s] = Dict{Int, Vector{Tuple{Int, Int}}}()
        Q_s_t[s]     = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
        for (t_id, od_cnt) in time_to_od
            Omega_s_t[s][t_id] = sort!(collect(keys(od_cnt)))
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
        routes_s,
        nothing   # no alpha profile for the warm start map
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


# ── Fixed-variable feasibility test ─────────────────────────────────────────────


"""
    _check_fixed_start_feasibility(model, data, sol; optimizer_env=nothing)

Rebuild the full RouteVehicleCapacityModel (including lazy constraint callbacks), fix every
variable to its warm start hint value, and solve. Reports feasibility.

If infeasible (warm start violates some constraint including lazy capacity constraints),
computes the IIS and writes it to a .ilp file so the conflicting constraints can be
inspected directly.

This test catches violations that `primal_feasibility_report` misses because lazy
constraints are not part of the upfront model.
"""
function _check_fixed_start_feasibility(
    model         :: RouteVehicleCapacityModel,
    data          :: StationSelectionData,
    sol           :: Dict{Symbol, Any};
    optimizer_env :: Union{Gurobi.Env, Nothing} = nothing
)
    # Build with use_lazy_constraints=false so constraints (ii) and (iii) are added as
    # regular upfront constraints. Lazy callbacks are not triggered on a fully-fixed model,
    # so using the callback version would silently skip the capacity constraints.
    println("  [fixed-start] rebuilding model with explicit (non-lazy) capacity constraints...")
    flush(stdout)
    explicit_model = RouteVehicleCapacityModel(
        model.k, model.l;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time,
        vehicle_capacity            = model.vehicle_capacity,
        max_route_travel_time       = model.max_route_travel_time,
        max_walking_distance        = model.max_walking_distance,
        max_detour_time             = model.max_detour_time,
        max_detour_ratio            = model.max_detour_ratio,
        time_window_sec             = model.time_window_sec,
        use_lazy_constraints        = false,
        max_stations_visited        = model.max_stations_visited,
        routes_file                 = model.routes_file,
        alpha_profile_file          = model.alpha_profile_file
    )
    fixed_build = build_model(explicit_model, data; optimizer_env=optimizer_env)
    fixed_m = fixed_build.model

    # Set start values on fixed_m via the same warm start logic (zeros all variables first,
    # then applies sol values). This populates start_value(v) for every variable.
    _apply_warm_start!(fixed_m, sol)

    # Fix every variable to its start value. Use fix(...; force=true) for all types —
    # we do not need to preserve integrality since there are no lazy callbacks.
    vars          = all_variables(fixed_m)
    n_total       = length(vars)
    n_with_start  = 0
    n_fixed       = 0
    missing_start = String[]

    for v in vars
        sv = start_value(v)
        if sv === nothing
            push!(missing_start, name(v))
            continue
        end
        n_with_start += 1
        fix(v, sv; force = true)
        is_fixed(v) && (n_fixed += 1)
    end

    println("  [fixed-start] variable audit:")
    println("    total variables   : $n_total")
    println("    with start value  : $n_with_start  ($(n_total - n_with_start) missing)")
    println("    fixed (is_fixed)  : $n_fixed")
    if !isempty(missing_start)
        println("    ✗ variables missing a start value:")
        for nm in first(missing_start, 10)
            println("      $nm")
        end
        length(missing_start) > 10 &&
            println("      ... ($(length(missing_start) - 10) more not shown)")
    end
    flush(stdout)

    println("  [fixed-start] solving (all vars fixed, (ii)/(iii) as explicit constraints)...")
    flush(stdout)
    set_silent(fixed_m)
    optimize!(fixed_m)

    ts = termination_status(fixed_m)
    println("  [fixed-start] termination: $(string(ts))")

    if ts == MOI.INFEASIBLE
        println("  [fixed-start] INFEASIBLE — warm start values violate constraints")
        println("  [fixed-start] computing IIS...")
        flush(stdout)
        try
            grb = backend(fixed_m)
            Gurobi.GRBcomputeIIS(grb)
            ilp_path = "warm_start_fixed_$(Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")).ilp"
            Gurobi.GRBwrite(grb, ilp_path)
            println("  [fixed-start] IIS written to: $ilp_path")
            println("  [fixed-start] inspect the .ilp file to identify the conflicting constraints")
        catch e
            @warn "  [fixed-start] IIS computation failed" exception=e
        end
    elseif has_values(fixed_m)
        println("  [fixed-start] FEASIBLE — warm start satisfies all constraints (incl. (ii) and (iii))")
    else
        println("  [fixed-start] status=$(string(ts)) primal=$(string(primal_status(fixed_m)))")
    end
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
    optimizer_env    :: Union{Gurobi.Env, Nothing} = nothing,
    silent           :: Bool = false,
    check_feasibility :: Bool = true,
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

    if isnothing(alpha_hints)
        println("  [warm start] ✗ α/θ derivation failed — some (j,k) demand has no covering route")
        println("  [warm start]   aborting warm start; main model will solve cold")
        flush(stdout)
        return nothing
    end
    println("  [warm start] α/θ hints derived ($(length(alpha_hints)) α entries, $(length(theta_hints)) θ entries)")
    flush(stdout)

    sol = Dict{Symbol, Any}(
        :y     => y_vals,
        :z     => z_vals,
        :x     => x_vals,
        :alpha => alpha_hints,
        :theta => theta_hints,
    )

    println("  [warm start] checking lazy constraint satisfaction on hints...")
    flush(stdout)
    _check_lazy_constraints_on_hints(sol, build_result.mapping, model.vehicle_capacity)

    if check_feasibility
        println("  [warm start] running fixed-variable feasibility test...")
        flush(stdout)
        _check_fixed_start_feasibility(model, data, sol; optimizer_env=optimizer_env)
    end

    return sol
end


"""
    _check_lazy_constraints_on_hints(sol, main_mapping, vehicle_capacity; atol=1e-6)

Manually evaluate the two lazy constraints against the warm start hint values and print
any violations. Called before `optimize!` to diagnose why a warm start may be rejected.

Constraint (ii):  Σ_{od using (j,k)} x[s][t][od][pair]  ≤  Σ_r α^r_{jkts}
Constraint (iii): Σ_{(j,k): β^r_{jkl}=1} α^r_{jkts}    ≤  Cap_r · θ^r_{ts}
"""
function _check_lazy_constraints_on_hints(
    sol              :: Dict{Symbol, Any},
    main_mapping     :: VehicleCapacityODMap,
    vehicle_capacity :: Int;
    atol             :: Float64 = 1e-6
)
    x_vals      = sol[:x]
    alpha_hints = get(sol, :alpha, Dict{NTuple{5,Int}, Float64}())
    theta_hints = get(sol, :theta, Dict{NTuple{3,Int}, Float64}())
    S           = length(x_vals)

    viol_ii_count   = 0;  max_viol_ii   = 0.0
    viol_iii_count  = 0;  max_viol_iii  = 0.0

    # ── Constraint (ii) ───────────────────────────────────────────────────────
    for s in 1:S
        for (t_id, od_pairs) in main_mapping.Omega_s_t[s]
            # Collect every (j,k) leg active in this (s, t_id)
            jk_set = Set{Tuple{Int,Int}}()
            for (o, d) in od_pairs
                for (j, k) in get_valid_jk_pairs(main_mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end

            od_dict = get(x_vals[s], t_id, nothing)

            for (j_idx, k_idx) in jk_set
                # x sum: Σ_{od using (j,k)} x hint value
                x_sum = 0.0
                if !isnothing(od_dict)
                    for (od_idx, (o, d)) in enumerate(od_pairs)
                        valid_pairs = get_valid_jk_pairs(main_mapping, o, d)
                        pair_idx    = findfirst(==((j_idx, k_idx)), valid_pairs)
                        pair_idx === nothing && continue
                        pair_vals = get(od_dict, od_idx, nothing)
                        isnothing(pair_vals) && continue
                        pair_idx > length(pair_vals) && continue
                        x_sum += pair_vals[pair_idx]
                    end
                end

                # alpha sum: Σ_r α^r_{jkts}
                alpha_sum = 0.0
                routes_t  = get(get(main_mapping.routes_s, s, Dict{Int,Vector{RouteData}}()), t_id, RouteData[])
                for r_idx in 1:length(routes_t)
                    alpha_sum += get(alpha_hints, (s, r_idx, j_idx, k_idx, t_id), 0.0)
                end

                viol = x_sum - alpha_sum
                if viol > atol
                    viol_ii_count += 1
                    max_viol_ii    = max(max_viol_ii, viol)
                end
            end
        end
    end

    # ── Constraint (iii) ──────────────────────────────────────────────────────
    for s in 1:S
        for (t_id, routes_t) in get(main_mapping.routes_s, s, Dict{Int,Vector{RouteData}}())
            for (r_idx, route) in enumerate(routes_t)
                n_segs = length(route.station_ids) - 1
                n_segs <= 0 && continue

                theta_val = get(theta_hints, (s, t_id, r_idx), 0.0)
                cap       = vehicle_capacity * theta_val

                # Segment loads from alpha hints for this route
                seg_load = zeros(Float64, n_segs)
                for ((s2, r2, j_idx, k_idx, t2), alpha_val) in alpha_hints
                    (s2 == s && r2 == r_idx && t2 == t_id && alpha_val > 0) || continue
                    for l in 1:n_segs
                        compute_beta_r_jkl(route, j_idx, k_idx, l,
                                           main_mapping.array_idx_to_station_id) || continue
                        seg_load[l] += alpha_val
                    end
                end

                for l in 1:n_segs
                    viol = seg_load[l] - cap
                    if viol > atol
                        viol_iii_count += 1
                        max_viol_iii    = max(max_viol_iii, viol)
                    end
                end
            end
        end
    end

    # ── Report ────────────────────────────────────────────────────────────────
    if viol_ii_count == 0 && viol_iii_count == 0
        println("  [warm start lazy check] all lazy constraints satisfied")
    else
        viol_ii_count  > 0 && println("  [warm start lazy check] (ii)  violations: $(viol_ii_count),  max=$(round(max_viol_ii;  digits=6))")
        viol_iii_count > 0 && println("  [warm start lazy check] (iii) violations: $(viol_iii_count), max=$(round(max_viol_iii; digits=6))")
    end
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
        if best_r == 0
            println("  [warm start] ✗ no covering route for (s=$s, t_id=$t_id, j=$j_idx, k=$k_idx) — seeded routes may be missing for this time bucket")
            flush(stdout)
            return nothing, nothing
        end

        # Use alpha profile hint value if available; otherwise fall back to demand
        alpha_val = demand
        profile = main_mapping.alpha_profile_hint
        if !isnothing(profile) && !isempty(profile)
            route    = routes_t[best_r]
            j_id     = main_mapping.array_idx_to_station_id[j_idx]
            k_id     = main_mapping.array_idx_to_station_id[k_idx]
            hint_val = get(profile, (route.id, j_id, k_id), nothing)
            if !isnothing(hint_val)
                alpha_val = hint_val
            end
        end

        alpha_hints[(s, best_r, j_idx, k_idx, t_id)] = alpha_val
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
