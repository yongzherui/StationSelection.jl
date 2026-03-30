"""
    build_model(model::AlphaRouteModel, data::StationSelectionData; optimizer_env=nothing)
                -> BuildResult

Build the MILP for AlphaRouteModel.

Variables: y (build), z (activate), x (assignment), θ^r_{ts} (route deployments, Z+).
Alpha values are fixed parameters from `mapping.alpha_profile` — not decision variables.

Single covering constraint:
    Σ_{od: (j,k) valid} x[s][t][od][pair]  ≤  Σ_r α^r_{jk} · θ^r_{ts}   ∀ j,k,t,s

No constraint (iii).
"""
function build_model(
        model :: AlphaRouteModel,
        data  :: StationSelectionData;
        optimizer_env = nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    mapping = create_map(model, data)   # AlphaRouteODMap

    S = length(data.scenarios)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts   = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts      = Dict{String, Int}()

    n_routes = sum(
        sum(length(v) for v in values(mapping.routes_s[s]); init = 0)
        for s in 1:S; init = 0
    )
    total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:S; init = 0)
    extra_counts["n_routes"]       = n_routes
    extra_counts["total_od_pairs"] = total_od_pairs

    # ==========================================================================
    # Variables
    # ==========================================================================

    variable_counts["station_selection"]   = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"]          = add_assignment_variables!(m, data, mapping)
    variable_counts["theta_r_ts"]         = _arm_add_theta_variables!(m, data, mapping)

    # ==========================================================================
    # Objective
    # ==========================================================================

    set_route_od_objective!(m, data, mapping;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time)

    # ==========================================================================
    # Constraints
    # ==========================================================================

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

    if model.use_lazy_constraints
        constraint_counts["route_capacity"] =
            _arm_add_capacity_lazy_constraints!(m, data, mapping)
    else
        constraint_counts["route_capacity"] =
            _arm_add_capacity_constraints!(m, data, mapping)
    end

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end


# ==============================================================================
# Variable creation
# ==============================================================================

"""
    _arm_add_theta_variables!(m, data, mapping::AlphaRouteODMap) -> Int

Integer route-deployment variables θ^r_{ts} ∈ Z+.

Created for each (s, t_id, r_idx) where the route serves at least one valid (j,k) pair
in the bucket AND has a positive alpha value for that leg in `mapping.alpha_profile`.

Stored as `m[:theta_r_ts]::Dict{NTuple{3,Int}, VariableRef}` keyed `(s, t_id, r_idx)`.
Also stores `m[:arm_alpha_params]::Dict{NTuple{5,Int}, Float64}` keyed
`(s, t_id, r_idx, j_idx, k_idx)` for use in constraint building.
"""
function _arm_add_theta_variables!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: AlphaRouteODMap
)::Int
    before        = JuMP.num_variables(m)
    S             = n_scenarios(data)
    alpha_profile = mapping.alpha_profile

    # Precompute: for each (s, t_id, r_idx, j_idx, k_idx), the fixed alpha param
    # Only entries with alpha > 0 are stored.
    arm_alpha_params = Dict{NTuple{5, Int}, Float64}()
    srt_with_alpha   = Set{NTuple{3, Int}}()

    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            # Collect valid (j_idx, k_idx) pairs for this bucket
            jk_set = Set{Tuple{Int, Int}}()
            for (o, d) in od_pairs
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end
            isempty(jk_set) && continue

            routes_t = get(mapping.routes_s[s], t_id, RouteData[])
            for (r_idx, route) in enumerate(routes_t)
                for (j_idx, k_idx) in jk_set
                    j_id = mapping.array_idx_to_station_id[j_idx]
                    k_id = mapping.array_idx_to_station_id[k_idx]
                    alpha_val = get(alpha_profile, (route.id, j_id, k_id), 0.0)
                    alpha_val > 0 || continue
                    arm_alpha_params[(s, t_id, r_idx, j_idx, k_idx)] = alpha_val
                    push!(srt_with_alpha, (s, t_id, r_idx))
                end
            end
        end
    end

    theta_r_ts = Dict{NTuple{3, Int}, VariableRef}()
    for (s, t_id, r_idx) in srt_with_alpha
        theta_r_ts[(s, t_id, r_idx)] = @variable(m, integer = true, lower_bound = 0)
    end

    m[:theta_r_ts]     = theta_r_ts
    m[:arm_alpha_params] = arm_alpha_params

    n_theta = JuMP.num_variables(m) - before
    println("  AlphaRouteModel: $(length(arm_alpha_params)) alpha param entries, $n_theta theta variables")
    flush(stdout)

    # Warn about (j,k,t,s) pairs with demand but zero total alpha coverage
    _arm_warn_uncovered_jk(data, mapping, arm_alpha_params)

    return n_theta
end


"""
Emit warnings for (j,k,t,s) combinations that have demand but no positive alpha coverage.
These legs will have no capacity constraint and may be freely assigned without route coverage.
"""
function _arm_warn_uncovered_jk(
    data             :: StationSelectionData,
    mapping          :: AlphaRouteODMap,
    arm_alpha_params :: Dict{NTuple{5, Int}, Float64}
)
    S = n_scenarios(data)
    # Build set of (s, t_id, j_idx, k_idx) that have at least one alpha entry
    covered = Set{NTuple{4, Int}}()
    for (s, t_id, r_idx, j_idx, k_idx) in keys(arm_alpha_params)
        push!(covered, (s, t_id, j_idx, k_idx))
    end

    n_uncovered = 0
    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            for (o, d) in od_pairs
                get(mapping.Q_s_t[s][t_id], (o, d), 0) > 0 || continue
                for (j_idx, k_idx) in get_valid_jk_pairs(mapping, o, d)
                    (s, t_id, j_idx, k_idx) ∈ covered && continue
                    if n_uncovered < 5
                        j_id = mapping.array_idx_to_station_id[j_idx]
                        k_id = mapping.array_idx_to_station_id[k_idx]
                        @warn "AlphaRouteModel: no alpha coverage for (s=$s, t_id=$t_id, j=$j_id, k=$k_id) — capacity constraint skipped for this leg"
                    end
                    n_uncovered += 1
                end
            end
        end
    end
    if n_uncovered > 5
        @warn "AlphaRouteModel: $n_uncovered total (j,k,t,s) legs have no alpha coverage (first 5 shown)"
    end
end


# ==============================================================================
# Constraints
# ==============================================================================

"""
    _arm_add_capacity_constraints!(m, data, mapping::AlphaRouteODMap) -> Int

Add the alpha-parameterized capacity covering constraint (upfront):

    Σ_{od: (j,k) valid} x[s][t][od][pair]  ≤  Σ_r α^r_{jk} · θ^r_{ts}   ∀ j,k,t,s

Only added for (j,k,t,s) combinations where at least one route has a positive alpha value.
"""
function _arm_add_capacity_constraints!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: AlphaRouteODMap
)::Int
    before           = _total_num_constraints(m)
    S                = n_scenarios(data)
    x                = m[:x]
    theta_r_ts       = m[:theta_r_ts]
    arm_alpha_params = m[:arm_alpha_params]

    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            # Collect active (j,k) legs in this bucket
            jk_set = Set{Tuple{Int, Int}}()
            for (o, d) in od_pairs
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end

            routes_t = get(mapping.routes_s[s], t_id, RouteData[])

            for (j_idx, k_idx) in jk_set
                # RHS: Σ_r α^r_{jk} · θ^r_{ts}  (only routes with positive alpha)
                rhs = AffExpr(0.0)
                for (r_idx, route) in enumerate(routes_t)
                    alpha_val = get(arm_alpha_params, (s, t_id, r_idx, j_idx, k_idx), 0.0)
                    alpha_val > 0 || continue
                    theta_var = get(theta_r_ts, (s, t_id, r_idx), nothing)
                    theta_var === nothing && continue
                    add_to_expression!(rhs, alpha_val, theta_var)
                end
                isempty(rhs.terms) && continue   # no alpha coverage — skip

                # LHS: Σ_{od: (j,k) valid} x[s][t][od][pair]
                lhs = AffExpr(0.0)
                for (od_idx, (o, d)) in enumerate(od_pairs)
                    valid_pairs = get_valid_jk_pairs(mapping, o, d)
                    pair_idx    = findfirst(==((j_idx, k_idx)), valid_pairs)
                    pair_idx === nothing && continue
                    x_od = get(x[s][t_id], od_idx, VariableRef[])
                    isempty(x_od) && continue
                    add_to_expression!(lhs, 1.0, x_od[pair_idx])
                end

                @constraint(m, lhs <= rhs)
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    _arm_add_capacity_lazy_constraints!(m, data, mapping::AlphaRouteODMap) -> Int

Register a lazy-constraint callback for the alpha-parameterized capacity constraint.
"""
function _arm_add_capacity_lazy_constraints!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: AlphaRouteODMap
)::Int
    before           = _total_num_constraints(m)
    S                = n_scenarios(data)
    x                = m[:x]
    theta_r_ts       = m[:theta_r_ts]
    arm_alpha_params = m[:arm_alpha_params]

    # Precompute active (j,k) set and OD index lookup per (s, t_id)
    jk_by_st     = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()
    od_idx_by_st = Dict{Tuple{Int, Int}, Dict{Tuple{Int, Int}, Int}}()

    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            jk_set    = Set{Tuple{Int, Int}}()
            od_lookup = Dict{Tuple{Int, Int}, Int}()
            for (od_idx, (o, d)) in enumerate(od_pairs)
                od_lookup[(o, d)] = od_idx
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end
            jk_by_st[(s, t_id)]     = collect(jk_set)
            od_idx_by_st[(s, t_id)] = od_lookup
        end
    end

    function lazy_cb(cb_data)
        callback_node_status(cb_data, m) == MOI.CALLBACK_NODE_STATUS_INTEGER || return

        for s in 1:S
            for (t_id, od_pairs) in mapping.Omega_s_t[s]
                od_lookup = get(od_idx_by_st, (s, t_id), Dict{Tuple{Int,Int},Int}())
                routes_t  = get(mapping.routes_s[s], t_id, RouteData[])

                for (j_idx, k_idx) in get(jk_by_st, (s, t_id), Tuple{Int,Int}[])
                    # Evaluate RHS: Σ_r α^r_{jk} · θ^r_{ts}
                    rhs_val    = 0.0
                    has_coverage = false
                    for (r_idx, _route) in enumerate(routes_t)
                        alpha_val = get(arm_alpha_params, (s, t_id, r_idx, j_idx, k_idx), 0.0)
                        alpha_val > 0 || continue
                        theta_var = get(theta_r_ts, (s, t_id, r_idx), nothing)
                        theta_var === nothing && continue
                        has_coverage = true
                        rhs_val += alpha_val * callback_value(cb_data, theta_var)
                    end
                    !has_coverage && continue   # no alpha coverage for this (j,k,t,s)

                    # Evaluate LHS: Σ_{od} x[s][t][od][pair]
                    lhs_val = 0.0
                    for (o, d) in od_pairs
                        valid_pairs = get_valid_jk_pairs(mapping, o, d)
                        pair_idx    = findfirst(==((j_idx, k_idx)), valid_pairs)
                        pair_idx === nothing && continue
                        od_idx = get(od_lookup, (o, d), 0)
                        od_idx == 0 && continue
                        x_od = get(x[s][t_id], od_idx, VariableRef[])
                        isempty(x_od) && continue
                        lhs_val += callback_value(cb_data, x_od[pair_idx])
                    end

                    lhs_val <= rhs_val + 1e-6 && continue

                    # Violated — build and submit constraint
                    rhs_expr = AffExpr(0.0)
                    for (r_idx, _route) in enumerate(routes_t)
                        alpha_val = get(arm_alpha_params, (s, t_id, r_idx, j_idx, k_idx), 0.0)
                        alpha_val > 0 || continue
                        theta_var = get(theta_r_ts, (s, t_id, r_idx), nothing)
                        theta_var === nothing && continue
                        add_to_expression!(rhs_expr, alpha_val, theta_var)
                    end
                    lhs_expr = AffExpr(0.0)
                    for (o, d) in od_pairs
                        valid_pairs = get_valid_jk_pairs(mapping, o, d)
                        pair_idx    = findfirst(==((j_idx, k_idx)), valid_pairs)
                        pair_idx === nothing && continue
                        od_idx = get(od_lookup, (o, d), 0)
                        od_idx == 0 && continue
                        x_od = get(x[s][t_id], od_idx, VariableRef[])
                        isempty(x_od) && continue
                        add_to_expression!(lhs_expr, 1.0, x_od[pair_idx])
                    end
                    MOI.submit(m, MOI.LazyConstraint(cb_data),
                               @build_constraint(lhs_expr <= rhs_expr))
                end
            end
        end
    end

    MOI.set(m, MOI.LazyConstraintCallback(), lazy_cb)
    return _total_num_constraints(m) - before
end
