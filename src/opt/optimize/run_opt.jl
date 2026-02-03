using Dates
using Logging

"""
    run_opt(model, data; optimizer_env=nothing, silent=true, show_counts=false,
            do_optimize=true, warm_start=false)

Construct and solve a station selection optimization model.

# Arguments
- `model::AbstractStationSelectionModel`: The model specification (e.g., TwoStageSingleDetourModel)
- `data::StationSelectionData`: Problem data with stations, requests, and costs

# Keyword Arguments
- `optimizer_env`: Gurobi environment (created if not provided)
- `silent::Bool`: Whether to suppress solver output (default: true)
- `show_counts::Bool`: Whether to print variable/constraint counts before solving (default: false)
- `do_optimize::Bool`: Whether to run `optimize!` (default: true)
- `warm_start::Bool`: Whether to compute a warm-start solution (TwoStageSingleDetourModel only)

# Returns
- `OptResult`
"""
function run_opt(
        model::AbstractStationSelectionModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        silent::Bool=true,
        show_counts::Bool=false,
        do_optimize::Bool=true,
        warm_start::Bool=false
    )

    start_time = now()
    @info "run_opt: start" model_type=string(typeof(model)) do_optimize=do_optimize warm_start=warm_start show_counts=show_counts

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Build model
    build_start = now()
    build_result = build_model(model, data; optimizer_env=optimizer_env)
    m = build_result.model
    build_time_sec = Dates.value(now() - build_start) / 1000
    @info "run_opt: model built" build_time_sec=build_time_sec

    if show_counts
        if !isempty(build_result.counts.variables)
            _print_counts("Variables", build_result.counts.variables)
        end
        if !isempty(build_result.counts.constraints)
            _print_counts("Constraints", build_result.counts.constraints)
        end
        if !isempty(build_result.counts.extras)
            _print_counts("Extras", build_result.counts.extras)
        end
    end

    if silent
        set_silent(m)
    end

    warm_start_solution = nothing
    warm_start_time_sec = nothing
    if warm_start && model isa TwoStageSingleDetourModel
        warm_start_start = now()
        warm_start_solution = StationSelection.warm_start(model, data;
            optimizer_env=optimizer_env,
            silent=silent,
            show_counts=show_counts,
            pooling_map=build_result.mapping,
            detour_combos=build_result.detour_combos
        )
        apply_warm_start!(m, warm_start_solution)
        warm_start_time_sec = Dates.value(now() - warm_start_start) / 1000
        @info "run_opt: warm start applied" warm_start_time_sec=warm_start_time_sec
    end

    # Solve the model
    solve_time_sec = nothing
    if do_optimize
        solve_start = now()
        optimize!(m)
        solve_time_sec = Dates.value(now() - solve_start) / 1000
        @info "run_opt: optimize! finished" solve_time_sec=solve_time_sec
    end

    term_status = do_optimize ? JuMP.termination_status(m) : MOI.OPTIMIZE_NOT_CALLED
    obj = nothing
    solution = nothing

    if term_status == MOI.OPTIMAL
        obj = JuMP.objective_value(m)
        x_val = _value_recursive(m[:x])
        y_val = _value_recursive(m[:y])
        solution = (x_val, y_val)
    end

    runtime_sec = Dates.value(now() - start_time) / 1000
    @info "run_opt: completed" termination_status=string(term_status) runtime_sec=runtime_sec

    return OptResult(
        term_status,
        obj,
        solution,
        runtime_sec,
        m,
        build_result.mapping,
        build_result.detour_combos,
        build_result.counts,
        warm_start_solution,
        Dict{String, Any}(
            "build_time_sec" => build_time_sec,
            "warm_start_time_sec" => warm_start_time_sec,
            "solve_time_sec" => solve_time_sec
        )
    )
end

function warm_start(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        silent::Bool=true,
        show_counts::Bool=false,
        pooling_map::Union{AbstractPoolingMap, Nothing}=nothing,
        detour_combos::Union{DetourComboData, Nothing}=nothing
    )
    return get_warm_start_solution(
        model,
        data;
        optimizer_env=optimizer_env,
        silent=silent,
        show_counts=show_counts,
        pooling_map=pooling_map,
        detour_combos=detour_combos
    )
end

function get_warm_start_solution(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        silent::Bool=true,
        show_counts::Bool=false,
        pooling_map::Union{AbstractPoolingMap, Nothing}=nothing,
        detour_combos::Union{DetourComboData, Nothing}=nothing
    )
    # Build pooling map and detour combos to align warm-start values with the target model structure.
    Xi_same_source = detour_combos === nothing ? find_same_source_detour_combinations(model, data) : detour_combos.same_source
    Xi_same_dest = detour_combos === nothing ? find_same_dest_detour_combinations(model, data) : detour_combos.same_dest
    if pooling_map === nothing
        pooling_map = create_map(
            model,
            data;
            Xi_same_source=Xi_same_source,
            Xi_same_dest=Xi_same_dest
        )
    end

    use_walking_distance_limit = has_walking_distance_limit(pooling_map)
    max_walking_distance = get_max_walking_distance(pooling_map)

    clustering_model = ClusteringTwoStageODModel(
        model.k,
        model.l;
        in_vehicle_time_weight=model.in_vehicle_time_weight,
        use_walking_distance_limit=use_walking_distance_limit,
        max_walking_distance=max_walking_distance,
        tight_constraints=model.tight_constraints
    )

    clustering_result = run_opt(
        clustering_model,
        data;
        optimizer_env=optimizer_env,
        silent=silent,
        show_counts=show_counts,
        do_optimize=true,
        warm_start=false
    )

    # TODO: Map clustering solution variables into TwoStageSingleDetourModel warm-start values.
    return adapt_clustering_solution_to_two_stage_single_detour(
        clustering_result.termination_status,
        clustering_result.objective_value,
        clustering_result.model,
        clustering_result.runtime_sec,
        model,
        data,
        pooling_map,
        Xi_same_source,
        Xi_same_dest,
        clustering_result.mapping,
        clustering_model.variable_reduction
    )
end

function adapt_clustering_solution_to_two_stage_single_detour(
        term_status,
        obj,
        clustering_m::Model,
        runtime_sec,
        model::TwoStageSingleDetourModel,
        data::StationSelectionData,
        pooling_map::AbstractPoolingMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}},
        clustering_map::ClusteringTwoStageODMap,
        clustering_variable_reduction::Bool
    )
    x_cluster = _value_recursive(clustering_m[:x])
    y_cluster = _value_recursive(clustering_m[:y])
    z_cluster = _value_recursive(clustering_m[:z])

    n = data.n_stations
    S = n_scenarios(data)

    # Map clustering OD indices for each scenario.
    od_to_idx = [Dict{Tuple{Int, Int}, Int}() for _ in 1:S]
    for s in 1:S
        for (od_idx, od) in enumerate(clustering_map.Omega_s[s])
            od_to_idx[s][od] = od_idx
        end
    end

    # Build warm-start x aligned with pooling map.
    use_sparse = has_walking_distance_limit(pooling_map)
    use_sparse_cluster = clustering_variable_reduction && has_walking_distance_limit(clustering_map)
    x_ws = [Dict{Int, Dict{Tuple{Int, Int}, Any}}() for _ in 1:S]
    for s in 1:S
        for (time_id, od_vector) in pooling_map.Omega_s_t[s]
            x_ws[s][time_id] = Dict{Tuple{Int, Int}, Any}()
            for od in od_vector
                od_idx = od_to_idx[s][od]

                if use_sparse
                    valid_pairs_pool = get_valid_jk_pairs(pooling_map, od[1], od[2])
                    x_vals = zeros(Float64, length(valid_pairs_pool))

                    if use_sparse_cluster
                        valid_pairs_cluster = get_valid_jk_pairs(clustering_map, od[1], od[2])
                        chosen_idx = findfirst(v -> v > 0.5, x_cluster[s][od_idx])
                        if !isnothing(chosen_idx)
                            (j, k) = valid_pairs_cluster[chosen_idx]
                            for (idx, (jj, kk)) in enumerate(valid_pairs_pool)
                                if jj == j && kk == k
                                    x_vals[idx] = 1.0
                                    break
                                end
                            end
                        end
                    else
                        # Dense clustering x
                        chosen = findfirst(v -> v > 0.5, x_cluster[s][od_idx])
                        if !isnothing(chosen)
                            j, k = Tuple(chosen)
                            for (idx, (jj, kk)) in enumerate(valid_pairs_pool)
                                if jj == j && kk == k
                                    x_vals[idx] = 1.0
                                    break
                                end
                            end
                        end
                    end

                    x_ws[s][time_id][od] = x_vals
                else
                    x_vals = zeros(Float64, n, n)
                    if use_sparse_cluster
                        valid_pairs_cluster = get_valid_jk_pairs(clustering_map, od[1], od[2])
                        chosen_idx = findfirst(v -> v > 0.5, x_cluster[s][od_idx])
                        if !isnothing(chosen_idx)
                            (j, k) = valid_pairs_cluster[chosen_idx]
                            x_vals[j, k] = 1.0
                        end
                    else
                        chosen = findfirst(v -> v > 0.5, x_cluster[s][od_idx])
                        if !isnothing(chosen)
                            j, k = Tuple(chosen)
                            x_vals[j, k] = 1.0
                        end
                    end
                    x_ws[s][time_id][od] = x_vals
                end
            end
        end
    end

    # y, z warm-start values come directly from clustering.
    y_ws = y_cluster
    z_ws = z_cluster

    # f warm-start: set f=1 wherever any x=1 on that edge.
    f_ws = [Dict{Int, Any}() for _ in 1:S]
    for s in 1:S
        for (time_id, od_vector) in pooling_map.Omega_s_t[s]
            if use_sparse
                f_vals = Dict{Tuple{Int, Int}, Float64}()
                for (j, k) in get_valid_f_pairs(pooling_map, s, time_id)
                    f_vals[(j, k)] = 0.0
                end
                for od in od_vector
                    valid_pairs = get_valid_jk_pairs(pooling_map, od[1], od[2])
                    x_vals = x_ws[s][time_id][od]
                    for (idx, (j, k)) in enumerate(valid_pairs)
                        if x_vals[idx] > 0.5
                            f_vals[(j, k)] = 1.0
                        end
                    end
                end
                f_ws[s][time_id] = f_vals
            else
                f_vals = zeros(Float64, n, n)
                for od in od_vector
                    x_vals = x_ws[s][time_id][od]
                    for j in 1:n, k in 1:n
                        if x_vals[j, k] > 0.5
                            f_vals[j, k] = 1.0
                        end
                    end
                end
                f_ws[s][time_id] = f_vals
            end
        end
    end

    # u, v warm-start: greedy feasibility based on x assignments.
    u_ws = [Dict{Int, Vector{Float64}}() for _ in 1:S]
    v_ws = [Dict{Int, Vector{Float64}}() for _ in 1:S]

    for s in 1:S
        for (time_id, od_vector) in pooling_map.Omega_s_t[s]
            # Same-source (u)
            feasible_u_indices = has_walking_distance_limit(pooling_map) ?
                get(pooling_map.feasible_same_source[s], time_id, Int[]) :
                collect(1:length(Xi_same_source))
            if isempty(feasible_u_indices) || length(od_vector) <= 1
                u_ws[s][time_id] = Float64[]
            else
                u_vals = zeros(Float64, length(feasible_u_indices))
                for (local_idx, global_idx) in enumerate(feasible_u_indices)
                    (j_id, k_id, l_id) = Xi_same_source[global_idx]
                    j = pooling_map.station_id_to_array_idx[j_id]
                    k = pooling_map.station_id_to_array_idx[k_id]
                    l = pooling_map.station_id_to_array_idx[l_id]

                    has_jk = false
                    has_jl = false
                    for od in od_vector
                        if use_sparse
                            valid_pairs = get_valid_jk_pairs(pooling_map, od[1], od[2])
                            x_vals = x_ws[s][time_id][od]
                            for (idx, (jj, kk)) in enumerate(valid_pairs)
                                if x_vals[idx] > 0.5
                                    if jj == j && kk == k
                                        has_jk = true
                                    elseif jj == j && kk == l
                                        has_jl = true
                                    end
                                end
                            end
                        else
                            x_vals = x_ws[s][time_id][od]
                            has_jk |= x_vals[j, k] > 0.5
                            has_jl |= x_vals[j, l] > 0.5
                        end
                        if has_jk && has_jl
                            break
                        end
                    end
                    if has_jk && has_jl
                        u_vals[local_idx] = 1.0
                    end
                end
                u_ws[s][time_id] = u_vals
            end

            # Same-dest (v)
            feasible_v_indices = Int[]
            if has_walking_distance_limit(pooling_map)
                feasible_v_indices = get(pooling_map.feasible_same_dest[s], time_id, Int[])
            else
                for (idx, (_, _, _, time_delta)) in enumerate(Xi_same_dest)
                    future_time_id = time_id + time_delta
                    if haskey(pooling_map.Omega_s_t[s], future_time_id)
                        push!(feasible_v_indices, idx)
                    end
                end
            end

            if isempty(feasible_v_indices)
                v_ws[s][time_id] = Float64[]
            else
                v_vals = zeros(Float64, length(feasible_v_indices))
                for (local_idx, global_idx) in enumerate(feasible_v_indices)
                    (j_id, k_id, l_id, time_delta) = Xi_same_dest[global_idx]
                    j = pooling_map.station_id_to_array_idx[j_id]
                    k = pooling_map.station_id_to_array_idx[k_id]
                    l = pooling_map.station_id_to_array_idx[l_id]
                    future_time_id = time_id + time_delta
                    if !haskey(pooling_map.Omega_s_t[s], future_time_id)
                        continue
                    end

                    has_jl = false
                    has_kl = false
                    for od in od_vector
                        if use_sparse
                            valid_pairs = get_valid_jk_pairs(pooling_map, od[1], od[2])
                            x_vals = x_ws[s][time_id][od]
                            for (idx, (jj, kk)) in enumerate(valid_pairs)
                                if x_vals[idx] > 0.5 && jj == j && kk == l
                                    has_jl = true
                                    break
                                end
                            end
                        else
                            x_vals = x_ws[s][time_id][od]
                            has_jl |= x_vals[j, l] > 0.5
                        end
                        if has_jl
                            break
                        end
                    end

                    if has_jl
                        future_od_vector = pooling_map.Omega_s_t[s][future_time_id]
                        for od in future_od_vector
                            if use_sparse
                                valid_pairs = get_valid_jk_pairs(pooling_map, od[1], od[2])
                                x_vals = x_ws[s][future_time_id][od]
                                for (idx, (jj, kk)) in enumerate(valid_pairs)
                                    if x_vals[idx] > 0.5 && jj == k && kk == l
                                        has_kl = true
                                        break
                                    end
                                end
                            else
                                x_vals = x_ws[s][future_time_id][od]
                                has_kl |= x_vals[k, l] > 0.5
                            end
                            if has_kl
                                break
                            end
                        end
                    end

                    if has_jl && has_kl
                        v_vals[local_idx] = 1.0
                    end
                end
                v_ws[s][time_id] = v_vals
            end
        end
    end

    return Dict(
        :term_status => term_status,
        :objective => obj,
        :runtime_sec => runtime_sec,
        :x => x_ws,
        :y => y_ws,
        :z => z_ws,
        :f => f_ws,
        :u => u_ws,
        :v => v_ws,
        :mapping => pooling_map
    )
end

function apply_warm_start!(
        m::Model,
        warm_start_solution::Dict
    )
    x_ws = warm_start_solution[:x]
    y_ws = warm_start_solution[:y]
    z_ws = warm_start_solution[:z]
    f_ws = warm_start_solution[:f]
    u_ws = warm_start_solution[:u]
    v_ws = warm_start_solution[:v]

    # y, z
    for j in eachindex(m[:y])
        JuMP.set_start_value(m[:y][j], y_ws[j])
    end
    for j in 1:size(m[:z], 1), s in 1:size(m[:z], 2)
        JuMP.set_start_value(m[:z][j, s], z_ws[j, s])
    end

    # x (assignment)
    for s in eachindex(m[:x])
        for (time_id, od_dict) in m[:x][s]
            for (od, x_vars) in od_dict
                x_vals = x_ws[s][time_id][od]
                if x_vars isa AbstractArray
                    for idx in eachindex(x_vars)
                        JuMP.set_start_value(x_vars[idx], x_vals[idx])
                    end
                else
                    # Should not happen, but keep safe.
                    JuMP.set_start_value(x_vars, x_vals)
                end
            end
        end
    end

    # f (flow)
    for s in eachindex(m[:f])
        for (time_id, f_vars) in m[:f][s]
            f_vals = f_ws[s][time_id]
            if f_vars isa Dict
                for (jk, f_var) in f_vars
                    JuMP.set_start_value(f_var, get(f_vals, jk, 0.0))
                end
            else
                for j in 1:size(f_vars, 1), k in 1:size(f_vars, 2)
                    JuMP.set_start_value(f_vars[j, k], f_vals[j, k])
                end
            end
        end
    end

    # u, v (detour)
    for s in eachindex(m[:u])
        for (time_id, u_vars) in m[:u][s]
            u_vals = u_ws[s][time_id]
            for idx in eachindex(u_vars)
                JuMP.set_start_value(u_vars[idx], u_vals[idx])
            end
        end
    end
    for s in eachindex(m[:v])
        for (time_id, v_vars) in m[:v][s]
            v_vals = v_ws[s][time_id]
            for idx in eachindex(v_vars)
                JuMP.set_start_value(v_vars[idx], v_vals[idx])
            end
        end
    end

    return nothing
end

function _print_counts(title::String, counts::Dict{String, Int})
    total = sum(values(counts))
    println("$title (total=$total)")
    for key in sort(collect(keys(counts)))
        println("  - $key: $(counts[key])")
    end
end

function _value_recursive(value)
    if value isa JuMP.VariableRef
        return JuMP.value(value)
    elseif value isa AbstractArray
        return map(_value_recursive, value)
    elseif value isa Dict
        return Dict(k => _value_recursive(v) for (k, v) in value)
    end
    return value
end
