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
- `warm_start::Bool`: Reserved (no-op; warm start is not supported by current models)
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
        data::StationSelectionData;
        kwargs...
    )
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
