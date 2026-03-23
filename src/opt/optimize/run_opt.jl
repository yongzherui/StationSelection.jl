using Dates
using Logging

"""
    run_opt(model, data; optimizer_env=nothing, silent=false, show_counts=false,
            do_optimize=true, warm_start=false, mip_gap=nothing)

Construct and solve a station selection optimization model.

# Arguments
- `model::AbstractStationSelectionModel`: The model specification (e.g., ClusteringTwoStageODModel)
- `data::StationSelectionData`: Problem data with stations, requests, and costs

# Keyword Arguments
- `optimizer_env`: Gurobi environment (created if not provided)
- `silent::Bool`: Whether to suppress solver output (default: false)
- `show_counts::Bool`: Whether to print variable/constraint counts before solving (default: false)
- `do_optimize::Bool`: Whether to run `optimize!` (default: true)
- `warm_start::Bool`: When true, solve a restricted warm start model first and inject its solution as starting hints (supported by RouteVehicleCapacityModel)
- `mip_gap::Union{Float64, Nothing}`: MIP optimality gap tolerance; solver stops when gap ≤ this value (default: nothing = Gurobi default 1e-4)

# Returns
- `OptResult`
"""
function run_opt(
        model::AbstractStationSelectionModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        silent::Bool=false,
        show_counts::Bool=false,
        do_optimize::Bool=true,
        warm_start::Bool=false,
        mip_gap::Union{Float64, Nothing}=nothing
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

    if !isnothing(mip_gap)
        set_optimizer_attribute(m, "MIPGap", mip_gap)
    end

    warm_start_solution = nothing
    warm_start_time_sec = nothing

    if warm_start
        ws_start = now()
        warm_start_solution = get_warm_start_solution(
            model, data, build_result;
            optimizer_env=optimizer_env, silent=true
        )
        warm_start_time_sec = Dates.value(now() - ws_start) / 1000
        @info "run_opt: warm start complete" warm_start_time_sec=warm_start_time_sec found=!isnothing(warm_start_solution)
        if !isnothing(warm_start_solution)
            _apply_warm_start!(m, warm_start_solution)
        end
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

function get_warm_start_solution(
        model::AbstractStationSelectionModel,
        data::StationSelectionData,
        build_result;
        kwargs...
    )
    return nothing
end


"""
    _apply_warm_start!(m, sol)

Apply warm start hint values from `sol` (Dict{Symbol,Any}) to the JuMP model `m`
using `set_start_value`. Keys: :y, :z, :x, :alpha (optional), :theta (optional).
"""
function _apply_warm_start!(m::JuMP.Model, sol::Dict{Symbol, Any})
    # y[j] — JuMP container indexed 1:n
    y_vars = m[:y]
    y_vals = sol[:y]
    for j in eachindex(y_vals)
        set_start_value(y_vars[j], y_vals[j])
    end

    # z[j,s] — JuMP container indexed [1:n, 1:S]
    z_vars = m[:z]
    z_vals = sol[:z]
    n_stat, n_scen = size(z_vals)
    for j in 1:n_stat, s in 1:n_scen
        set_start_value(z_vars[j, s], z_vals[j, s])
    end

    # x[s][t_id][od_idx][pair_idx] — nested Dict/Vector structure
    x_vars = m[:x]
    x_vals = sol[:x]
    for s in eachindex(x_vars)
        for (t_id, od_dict_vars) in x_vars[s]
            od_dict_vals = get(x_vals[s], t_id, nothing)
            isnothing(od_dict_vals) && continue
            for (od_idx, pair_vars) in od_dict_vars
                pair_vals = get(od_dict_vals, od_idx, nothing)
                isnothing(pair_vals) && continue
                for pair_idx in eachindex(pair_vars)
                    pair_idx > length(pair_vals) && break
                    set_start_value(pair_vars[pair_idx], pair_vals[pair_idx])
                end
            end
        end
    end

    # alpha — optional (only present for route-based models)
    if haskey(m, :alpha_r_jkts) && haskey(sol, :alpha)
        alpha_vars  = m[:alpha_r_jkts]
        alpha_hints = sol[:alpha]
        for (key, var) in alpha_vars
            set_start_value(var, get(alpha_hints, key, 0.0))
        end
    end

    # theta — optional
    if haskey(m, :theta_r_ts) && haskey(sol, :theta)
        theta_vars  = m[:theta_r_ts]
        theta_hints = sol[:theta]
        for (key, var) in theta_vars
            set_start_value(var, get(theta_hints, key, 0.0))
        end
    end
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
