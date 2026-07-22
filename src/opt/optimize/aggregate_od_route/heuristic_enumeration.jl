"""
Heuristic-enumeration warm start for AggregateODRouteModel.

Given a caller-supplied list of candidate open-station sets (fixed y), for each candidate:
derive the cheapest feasible assignment (nearest-open x), then solve the resulting
fixed-station, fixed-assignment routing sub-problem (RouteCoveringProblem) to proven
optimality via column generation (reusing `_solve_fixed_route_covering_by_cg`). The best
candidate by routing objective is folded into the full AggregateODRouteModel's column pool
and used as a warm start for a direct MIP solve.
"""

function _aggregate_od_route_x_hint(
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    )::Vector{Dict{Int, Vector{Float64}}}
    S = n_scenarios(data)
    x_hint = [Dict{Int, Vector{Float64}}() for _ in 1:S]
    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            vals = zeros(Float64, length(valid_pairs))
            assigned = get(assignments, (s, o, d), nothing)
            if !isnothing(assigned)
                pair_idx = findfirst(==(assigned), valid_pairs)
                isnothing(pair_idx) && throw(ArgumentError(
                    "assigned pair $(assigned) for request $((s, o, d)) is not a valid " *
                    "station pair in the final model's mapping"
                ))
                vals[pair_idx] = 1.0
            end
            x_hint[s][od_idx] = vals
        end
    end
    return x_hint
end

"""
    _apply_aggregate_od_route_warm_start!(m, sol)

Apply warm start hint values from `sol` (keys `:y`, `:x`, `:theta`) to an
`AggregateODRouteModel`/`RouteCoveringProblem` JuMP model. Unlike `_apply_warm_start!` in
run_opt.jl (built for ExactDARPRouteModel's shapes), this model family has no `z` or
`alpha_r_jkts`, a flat `x[s][od_idx][pair_idx]` (no `t_id` level), and `theta_compat`
keyed by `(column_id, s)` rather than `theta_r_ts`.
"""
function _apply_aggregate_od_route_warm_start!(m::JuMP.Model, sol::Dict{Symbol, Any})
    for var in all_variables(m)
        set_start_value(var, 0.0)
    end

    y_vars = m[:y]
    y_vals = sol[:y]
    for j in eachindex(y_vals)
        set_start_value(y_vars[j], y_vals[j])
    end

    x_vars = m[:x]
    x_vals = sol[:x]
    for s in eachindex(x_vars)
        od_dict_vals = x_vals[s]
        for (od_idx, pair_vars) in x_vars[s]
            pair_vals = get(od_dict_vals, od_idx, nothing)
            for pair_idx in eachindex(pair_vars)
                v = (!isnothing(pair_vals) && pair_idx <= length(pair_vals)) ?
                        pair_vals[pair_idx] : 0.0
                set_start_value(pair_vars[pair_idx], v)
            end
        end
    end

    theta_vars = m[:theta_compat]
    theta_vals = sol[:theta]
    for (key, var) in theta_vars
        set_start_value(var, get(theta_vals, key, 0.0))
    end
    return nothing
end

function run_opt(
        instance::StationSelectionData,
        formulation::AggregateODRouteModel,
        solver::HeuristicEnumerationSolver,
    )::OptResult
    start_time = time()
    cfg = solver.config

    mapping = create_map(formulation, instance)
    requests, _demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) &&
        throw(ArgumentError("AggregateODRouteModel heuristic enumeration requires positive demand"))

    n_stations = instance.n_stations
    inner_benders = BendersSolver(config=cfg, inner_solver=solver.cg_solver)

    candidate_rows = NamedTuple[]
    best_idx = nothing
    best_cg_result = nothing
    best_assignments = nothing
    best_objective = Inf

    for (idx, candidate) in enumerate(solver.candidate_open_stations)
        length(candidate) == formulation.l || throw(ArgumentError(
            "candidate_open_stations[$idx] has $(length(candidate)) stations, expected l=$(formulation.l)"
        ))

        y_hat = zeros(Float64, n_stations)
        for j in candidate
            y_hat[j] = 1.0
        end

        assignments, infeasible = _fixed_assignments_from_y(instance, requests, feasible_pairs, y_hat)
        if !isempty(infeasible)
            push!(candidate_rows, (index=idx, feasible=false, objective=nothing, reason="infeasible_assignment"))
            continue
        end

        cg_result = try
            _solve_fixed_route_covering_by_cg(
                instance, formulation, assignments, inner_benders, nothing, sort!(unique(candidate))
            )
        catch err
            if err isa ArgumentError && occursin("pricing exhaustion", err.msg)
                push!(candidate_rows, (index=idx, feasible=false, objective=nothing, reason="cg_not_exhausted"))
                continue
            end
            rethrow()
        end

        obj = cg_result.final_result.objective_value
        push!(candidate_rows, (index=idx, feasible=!isnothing(obj), objective=obj, reason="ok"))
        if !isnothing(obj) && obj < best_objective
            best_objective = obj
            best_idx = idx
            best_cg_result = cg_result
            best_assignments = assignments
        end
    end

    isnothing(best_cg_result) && throw(ArgumentError(
        "HeuristicEnumerationSolver: none of the $(length(solver.candidate_open_stations)) " *
        "candidate station sets produced a feasible, proven-optimal routing solution"
    ))

    build_result = build_model(formulation, instance; optimizer_env=cfg.optimizer_env)
    m = build_result.model
    cfg.silent && set_silent(m)
    isnothing(cfg.mip_gap) || set_optimizer_attribute(m, "MIPGap", cfg.mip_gap)

    winning_columns = [
        c for c in best_cg_result.generated_columns if c.id in best_cg_result.selected_column_ids
    ]

    next_id = isempty(build_result.mapping.column_ids) ? 1 : maximum(build_result.mapping.column_ids) + 1
    resolved_ids = Dict{Int, Int}()
    for column in winning_columns
        candidate_column = AggregateODRouteColumn(next_id, column.od_pairs, column.tau; metadata=column.metadata)
        _theta_var, action = add_or_update_aggregate_od_route_column!(build_result, candidate_column)
        resolved_idx = findfirst(
            c -> _aggregate_od_route_column_signature_for_update(c) ==
                 _aggregate_od_route_column_signature_for_update(column),
            build_result.mapping.columns,
        )
        isnothing(resolved_idx) && throw(ArgumentError(
            "failed to resolve final-model column id for winning route with od_pairs $(column.od_pairs)"
        ))
        resolved_ids[column.id] = build_result.mapping.columns[resolved_idx].id
        action == :added && (next_id += 1)
    end

    S = n_scenarios(instance)
    sub_theta = best_cg_result.final_result.model[:theta_compat]
    theta_hints = Dict{Tuple{Int, Int}, Float64}()
    for column in winning_columns
        resolved_id = resolved_ids[column.id]
        for s in 1:S
            sub_var = get(sub_theta, (column.id, s), nothing)
            active = !isnothing(sub_var) && value(sub_var) > 0.5
            active && (theta_hints[(resolved_id, s)] = 1.0)
        end
    end

    y_hint = zeros(Float64, n_stations)
    for j in solver.candidate_open_stations[best_idx]
        y_hint[j] = 1.0
    end
    x_hint = _aggregate_od_route_x_hint(instance, build_result.mapping, best_assignments)

    sol = Dict{Symbol, Any}(:y => y_hint, :x => x_hint, :theta => theta_hints)
    _apply_aggregate_od_route_warm_start!(m, sol)
    cfg.check_feasibility && _verify_start_completeness(m)

    solve_start = time()
    optimize!(m)
    solve_time_sec = time() - solve_start

    term_status = termination_status(m)
    obj = term_status == MOI.OPTIMAL ? objective_value(m) : nothing
    solution = term_status == MOI.OPTIMAL ? (_value_recursive(m[:x]), _value_recursive(m[:y])) : nothing

    return OptResult(
        term_status,
        obj,
        solution,
        time() - start_time,
        m,
        build_result.mapping,
        build_result.detour_combos,
        build_result.counts,
        sol,
        Dict{String, Any}(
            "solve_method" => "heuristic_enumeration",
            "n_candidates" => length(solver.candidate_open_stations),
            "n_feasible_candidates" => count(r -> r.feasible, candidate_rows),
            "winning_candidate_index" => best_idx,
            "winning_candidate_routing_objective" => best_objective,
            "candidate_rows" => candidate_rows,
            "solve_time_sec" => solve_time_sec,
        ),
    )
end
