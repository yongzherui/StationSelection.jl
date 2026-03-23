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
        check_feasibility::Bool=true,
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
            optimizer_env=optimizer_env, silent=false
        )
        warm_start_time_sec = Dates.value(now() - ws_start) / 1000
        @info "run_opt: warm start complete" warm_start_time_sec=warm_start_time_sec found=!isnothing(warm_start_solution)

        if !isnothing(warm_start_solution)
            _apply_warm_start!(m, warm_start_solution)
            if check_feasibility
                println("  [warm start] verifying start completeness...")
                _verify_start_completeness(m)
                println("  [warm start] checking primal feasibility of hints...")
                _check_warm_start_feasibility(m, warm_start_solution)
            end
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
    # Zero every variable first so Gurobi sees a complete start (no GRB_UNDEFINED).
    for var in all_variables(m)
        set_start_value(var, 0.0)
    end

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
    # All x variables are explicitly set (0.0 for any unmatched) to avoid GRB_UNDEFINED.
    x_vars = m[:x]
    x_vals = sol[:x]
    for s in eachindex(x_vars)
        for (t_id, od_dict_vars) in x_vars[s]
            od_dict_vals = get(x_vals[s], t_id, nothing)
            for (od_idx, pair_vars) in od_dict_vars
                pair_vals = isnothing(od_dict_vals) ? nothing : get(od_dict_vals, od_idx, nothing)
                for pair_idx in eachindex(pair_vars)
                    v = (!isnothing(pair_vals) && pair_idx <= length(pair_vals)) ?
                            pair_vals[pair_idx] : 0.0
                    set_start_value(pair_vars[pair_idx], v)
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

"""
    _verify_start_completeness(m)

Check that every variable in `m` has a start value set (non-`nothing`), that no start
value is negative, and that integer variables have exact integer starts.
"""
function _verify_start_completeness(m::JuMP.Model)
    vars       = all_variables(m)
    n_total    = length(vars)
    n_missing  = count(v -> start_value(v) === nothing, vars)
    n_nonzero  = count(v -> !isnothing(start_value(v)) && start_value(v) != 0.0, vars)
    n_negative = count(v -> !isnothing(start_value(v)) && start_value(v) < -1e-9, vars)
    n_nonint   = count(
        v -> is_integer(v) && !isnothing(start_value(v)) &&
             abs(start_value(v) - round(start_value(v))) > 1e-6,
        vars
    )
    println("  [warm start completeness]")
    println("    total variables : $(n_total)")
    println("    missing starts  : $(n_missing)  ← must be 0 for a complete start")
    println("    non-zero starts : $(n_nonzero)")
    println("    negative starts : $(n_negative)  ← must be 0 (all vars have lb=0)")
    println("    non-integer int : $(n_nonint)   ← must be 0 (integer vars need integer starts)")
end


"""
    _check_warm_start_feasibility(m, sol)

Build a point Dict from `sol` and run `primal_feasibility_report` on `m`.
Prints any violated upfront constraints (lazy constraints are not checked).
"""
function _check_warm_start_feasibility(m::JuMP.Model, sol::Dict{Symbol, Any})
    point = Dict{VariableRef, Float64}()

    # y
    y_vars = m[:y]
    y_vals = sol[:y]
    for j in eachindex(y_vals)
        point[y_vars[j]] = Float64(round(y_vals[j]))
    end

    # z
    z_vars = m[:z]
    z_vals = sol[:z]
    n_stat, n_scen = size(z_vals)
    for j in 1:n_stat, s in 1:n_scen
        point[z_vars[j, s]] = Float64(round(z_vals[j, s]))
    end

    # x — nested Dict structure
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
                    point[pair_vars[pair_idx]] = Float64(round(pair_vals[pair_idx]))
                end
            end
        end
    end

    # alpha (optional)
    if haskey(m, :alpha_r_jkts) && haskey(sol, :alpha)
        alpha_vars  = m[:alpha_r_jkts]
        alpha_hints = sol[:alpha]
        for (key, var) in alpha_vars
            point[var] = Float64(round(get(alpha_hints, key, 0.0)))
        end
    end

    # theta (optional)
    if haskey(m, :theta_r_ts) && haskey(sol, :theta)
        theta_vars  = m[:theta_r_ts]
        theta_hints = sol[:theta]
        for (key, var) in theta_vars
            point[var] = Float64(round(get(theta_hints, key, 0.0)))
        end
    end

    report = primal_feasibility_report(m, point; atol=1e-6)
    if isempty(report)
        println("  [warm start feasibility] all upfront constraints satisfied")
    else
        println("  [warm start feasibility] $(length(report)) violated constraint(s):")
        for (con, viol) in sort(collect(report); by = x -> -x[2])
            println("    violation=$(round(viol; digits=6))  con=$(con)")
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
