"""
AlphaRouteWarmStartModel — warm start model for AlphaRouteModel.

Solves only for y/z/x (station selection + assignment, walking cost objective) with no
routes, no θ. The smaller MIP gives a fast feasible y/z/x solution. θ hints are then
derived analytically from the x values: for each (j,k,t,s) demand bucket, the
corresponding direct 2-stop route in the main model's route pool is identified and
θ = ceil(demand / α_{r,j,k}) is set to satisfy the covering constraint.
"""

export AlphaRouteWarmStartModel


"""
    AlphaRouteWarmStartModel <: AbstractODModel

Warm start model for AlphaRouteModel. Solves only for y/z/x with a walking cost
objective — no routes, no θ, no covering constraints. θ hints are derived analytically
from the x solution using the main model's direct 2-stop routes and fixed alpha values.
"""
struct AlphaRouteWarmStartModel <: AbstractODModel
    k                    :: Int
    l                    :: Int
    max_walking_distance :: Float64
    time_window_sec      :: Int

    function AlphaRouteWarmStartModel(
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


# ── Map creation ─────────────────────────────────────────────────────────────────


"""
    create_map(model::AlphaRouteWarmStartModel, data) -> AlphaRouteODMap

Build the OD mapping for the warm start model. No routes are generated (routes_s is
empty, alpha_profile is empty) since the warm start solves only for y/z/x.
"""
function create_map(
    model :: AlphaRouteWarmStartModel,
    data  :: StationSelectionData
)::AlphaRouteODMap

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

    # No routes or alpha for the warm start — those come from the main model's mapping
    routes_s      = Dict{Int, Dict{Int, Vector{RouteData}}}()
    alpha_profile = Dict{NTuple{3, Int}, Float64}()

    return AlphaRouteODMap(
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
        alpha_profile
    )
end


# ── Model build ──────────────────────────────────────────────────────────────────


"""
    build_model(model::AlphaRouteWarmStartModel, data; optimizer_env=nothing) -> BuildResult

Build the warm start MIP: y/z/x variables, walking cost objective, no routes/θ/covering
constraints.
"""
function build_model(
        model         :: AlphaRouteWarmStartModel,
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

    # Walking cost only — no route penalty (no θ variables)
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


# ── Warm start solution extraction ───────────────────────────────────────────────


"""
    get_warm_start_solution(model::AlphaRouteModel, data, build_result;
                            optimizer_env=nothing, silent=true, kwargs...)
                            -> Union{Dict{Symbol,Any}, Nothing}

Solve AlphaRouteWarmStartModel (y/z/x only, no routes) to get a fast incumbent
y/z/x solution, then derive θ hints from the x values using the main model's
direct 2-stop routes and fixed alpha values.

For each (j,k,t,s) demand bucket, the direct route [j,k] is found in the main
route pool and θ = ceil(demand / α_{r,j,k}) is set. No alpha hints are needed
since alpha is a fixed parameter in AlphaRouteModel.

Returns `nothing` if the warm start solve produces no feasible solution.
"""
function get_warm_start_solution(
    model        :: AlphaRouteModel,
    data         :: StationSelectionData,
    build_result;
    optimizer_env     :: Union{Gurobi.Env, Nothing} = nothing,
    silent            :: Bool = false,
    check_feasibility :: Bool = true,
    kwargs...
)
    ws_model = AlphaRouteWarmStartModel(
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

    println("  [warm start] deriving θ hints from x solution (direct routes only)...")
    flush(stdout)
    theta_hints = _derive_theta_hints_arm(x_vals, ws_build.mapping, build_result.mapping)
    println("  [warm start] θ hints derived ($(length(theta_hints)) θ entries)")
    flush(stdout)

    sol = Dict{Symbol, Any}(
        :y     => y_vals,
        :z     => z_vals,
        :x     => x_vals,
        :theta => theta_hints,
    )

    if check_feasibility
        println("  [warm start] checking covering constraint satisfaction on hints...")
        flush(stdout)
        _check_covering_constraints_arm(sol, build_result.mapping)
    end

    return sol
end


"""
    _derive_theta_hints_arm(x_vals, ws_mapping, main_mapping) -> Dict{NTuple{3,Int}, Float64}

From a warm start x solution, aggregate demand per (s, t_id, j_idx, k_idx) then assign
each bucket to the corresponding direct 2-stop route in the main route pool.

θ is set to ceil(demand / α_{r,j,k}) where α comes from `main_mapping.alpha_profile`.
For direct routes α = vehicle_capacity, so θ = ceil(demand / C).

Multi-leg routes are left at θ=0; the main solver may activate them if beneficial.
"""
function _derive_theta_hints_arm(
    x_vals       :: Vector,
    ws_mapping   :: AlphaRouteODMap,
    main_mapping :: AlphaRouteODMap
)
    S = length(x_vals)

    # ── Step 1: aggregate x values into demand per (s, t_id, j_idx, k_idx) ─────
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

    # ── Step 2: find direct 2-stop route for each demand bucket and set θ ───────
    theta_hints = Dict{NTuple{3, Int}, Float64}()
    n_missing   = 0

    for ((s, t_id, j_idx, k_idx), demand) in demand_jkts
        demand <= 0 && continue
        j_sid = main_mapping.array_idx_to_station_id[j_idx]
        k_sid = main_mapping.array_idx_to_station_id[k_idx]

        routes_t = get(
            get(main_mapping.routes_s, s, Dict{Int, Vector{RouteData}}()),
            t_id, RouteData[]
        )

        # Find the direct 2-stop route [j_sid, k_sid]
        r_idx = 0
        for (ri, route) in enumerate(routes_t)
            if length(route.station_ids) == 2 &&
               route.station_ids[1] == j_sid &&
               route.station_ids[2] == k_sid
                r_idx = ri
                break
            end
        end

        if r_idx == 0
            n_missing += 1
            continue
        end

        route     = routes_t[r_idx]
        alpha_val = get(main_mapping.alpha_profile, (route.id, j_sid, k_sid), 0.0)
        alpha_val <= 0 && continue

        min_theta = ceil(demand / alpha_val)
        theta_key = (s, t_id, r_idx)
        theta_hints[theta_key] = max(get(theta_hints, theta_key, 0.0), min_theta)
    end

    if n_missing > 0
        println("  [warm start] $(n_missing) (j,k,t,s) buckets had no direct route in pool — θ hints skipped for those")
        flush(stdout)
    end

    return theta_hints
end


"""
    _check_covering_constraints_arm(sol, main_mapping; atol=1e-6)

Evaluate the AlphaRouteModel covering constraint against warm start hint values:
    Σ_{od using (j,k)} x[s][t][od][pair]  ≤  Σ_r α^r_{jk} · θ^r_{ts}   ∀ j,k,t,s

Prints violation count and max violation. Violations are expected when a single bucket's
demand exceeds α (e.g. demand > C for a direct route) — the main solver resolves these
by activating additional routes or increasing θ.
"""
function _check_covering_constraints_arm(
    sol          :: Dict{Symbol, Any},
    main_mapping :: AlphaRouteODMap;
    atol         :: Float64 = 1e-6
)
    x_vals      = sol[:x]
    theta_hints = get(sol, :theta, Dict{NTuple{3,Int}, Float64}())
    S           = length(x_vals)

    viol_count = 0
    max_viol   = 0.0

    for s in 1:S
        for (t_id, od_pairs) in main_mapping.Omega_s_t[s]
            routes_t = get(
                get(main_mapping.routes_s, s, Dict{Int, Vector{RouteData}}()),
                t_id, RouteData[]
            )

            jk_set = Set{Tuple{Int,Int}}()
            for (o, d) in od_pairs
                for (j, k) in get_valid_jk_pairs(main_mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end

            od_dict = get(x_vals[s], t_id, nothing)

            for (j_idx, k_idx) in jk_set
                j_sid = main_mapping.array_idx_to_station_id[j_idx]
                k_sid = main_mapping.array_idx_to_station_id[k_idx]

                # x sum: Σ_{od using (j,k)} x hint
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

                # α·θ sum: Σ_r α^r_{jk} · θ^r_{ts}
                at_sum = 0.0
                for (r_idx, route) in enumerate(routes_t)
                    alpha_val = get(main_mapping.alpha_profile, (route.id, j_sid, k_sid), 0.0)
                    alpha_val > 0 || continue
                    theta_val = get(theta_hints, (s, t_id, r_idx), 0.0)
                    at_sum += alpha_val * theta_val
                end

                viol = x_sum - at_sum
                if viol > atol
                    viol_count += 1
                    max_viol    = max(max_viol, viol)
                end
            end
        end
    end

    if viol_count == 0
        println("  [warm start covering check] all covering constraints satisfied")
    else
        println("  [warm start covering check] $(viol_count) violations, max=$(round(max_viol; digits=6))")
        println("  [warm start covering check]   (expected when demand > α; main solver will resolve)")
    end
end
