using Dates
using Logging

function _compute_current_od_assignment_costs(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap,
        s::Int;
        in_vehicle_time_weight::Float64
    )::Dict{Int, Float64}
    x = m[:x]
    costs = Dict{Int, Float64}()

    for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
        x_od = get(x[s], od_idx, VariableRef[])
        isempty(x_od) && continue
        valid_pairs = get_valid_jk_pairs(mapping, o, d)
        total = 0.0
        for (pair_idx, (j, k)) in enumerate(valid_pairs)
            coeff = _robust_assignment_pair_cost(
                data, o, d, j, k;
                in_vehicle_time_weight=in_vehicle_time_weight
            )
            total += coeff * value(x_od[pair_idx])
        end
        costs[od_idx] = total
    end

    return costs
end


function _separate_budgeted_uncertainty(
        mapping::RobustTotalDemandCapMap,
        s::Int,
        scores::Dict{Int, Float64}
    )::Tuple{Dict{Int, Float64}, Float64}
    q_wc = Dict{Int, Float64}()
    remaining_budget = mapping.B[s]
    od_indices = sort(
        collect(keys(scores));
        by = od_idx -> (scores[od_idx], get(mapping.q_hat[s], mapping.Omega_s[s][od_idx], 0.0)),
        rev = true,
    )

    worst_case_value = 0.0
    for od_idx in od_indices
        remaining_budget <= 0 && break
        od = mapping.Omega_s[s][od_idx]
        q_bar = get(mapping.q_hat[s], od, 0.0)
        q_bar <= 0 && continue
        q_take = min(q_bar, remaining_budget)
        q_take <= 0 && continue
        q_wc[od_idx] = q_take
        worst_case_value += q_take * scores[od_idx]
        remaining_budget -= q_take
    end

    return q_wc, worst_case_value
end


function _initial_cut_seed(
        mapping::RobustTotalDemandCapMap,
        s::Int
    )::Dict{Int, Float64}
    q_wc = Dict{Int, Float64}()
    remaining_budget = mapping.B[s]
    od_indices = sort(
        collect(eachindex(mapping.Omega_s[s]));
        by = od_idx -> get(mapping.q_hat[s], mapping.Omega_s[s][od_idx], 0.0),
        rev = true,
    )

    for od_idx in od_indices
        remaining_budget <= 0 && break
        od = mapping.Omega_s[s][od_idx]
        q_bar = get(mapping.q_hat[s], od, 0.0)
        q_bar <= 0 && continue
        q_take = min(q_bar, remaining_budget)
        q_take <= 0 && continue
        q_wc[od_idx] = q_take
        remaining_budget -= q_take
    end

    return q_wc
end


function _add_initial_robust_cutting_plane_cuts!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap;
        in_vehicle_time_weight::Float64
    )::Int
    S = n_scenarios(data)
    added = 0
    for s in 1:S
        q_wc = _initial_cut_seed(mapping, s)
        isempty(q_wc) && continue
        _add_robust_cutting_plane_cut!(
            m, data, mapping, s, q_wc;
            in_vehicle_time_weight=in_vehicle_time_weight
        )
        added += 1
    end
    return added
end


function _run_robust_total_demand_cap_cutting_plane(
        model::RobustTotalDemandCapModel,
        data::StationSelectionData;
        optimizer_env=nothing,
        silent::Bool=false,
        show_counts::Bool=false,
        do_optimize::Bool=true,
        mip_gap::Union{Float64, Nothing}=nothing,
        cutting_plane_max_iters::Int=100,
        cutting_plane_tol::Float64=1e-6
    )::OptResult

    start_time = now()
    @info "run_opt: start" model_type=string(typeof(model)) solve_mode="cutting_plane" do_optimize=do_optimize

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    build_start = now()
    build_result = build_robust_total_demand_cap_cutting_plane_master(
        model, data; optimizer_env=optimizer_env
    )
    m = build_result.model
    mapping = build_result.mapping
    build_time_sec = Dates.value(now() - build_start) / 1000
    @info "run_opt: cutting-plane master built" build_time_sec=build_time_sec

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

    initial_cuts_added = _add_initial_robust_cutting_plane_cuts!(
        m, data, mapping;
        in_vehicle_time_weight=model.in_vehicle_time_weight
    )

    if !do_optimize
        return OptResult(
            MOI.OPTIMIZE_NOT_CALLED,
            nothing,
            nothing,
            Dates.value(now() - start_time) / 1000,
            m,
            mapping,
            build_result.detour_combos,
            build_result.counts,
            nothing,
            Dict{String, Any}(
                "build_time_sec" => build_time_sec,
                "solve_time_sec" => nothing,
                "cutting_plane_iterations" => 0,
                "initial_cuts_added" => initial_cuts_added,
                "solve_mode" => "cutting_plane",
            ),
        )
    end

    solve_start = now()
    iteration = 0
    converged = false
    final_term_status = MOI.OPTIMIZE_NOT_CALLED
    iteration_log = Vector{Dict{String, Any}}()
    cut_log = Vector{Dict{String, Any}}()
    prev_master_objective = nothing

    for iter in 1:cutting_plane_max_iters
        iteration = iter
        optimize!(m)
        final_term_status = JuMP.termination_status(m)
        @info "run_opt: cutting-plane master solve finished" iteration=iter termination_status=string(final_term_status)

        if final_term_status != MOI.OPTIMAL
            break
        end

        eta = m[:eta]
        pending_cuts = Tuple{Int, Dict{Int, Float64}, Float64, Float64}[]
        worst_case_by_scenario = Dict{Int, Float64}()
        master_objective = JuMP.objective_value(m)
        objective_delta = isnothing(prev_master_objective) ? nothing : master_objective - prev_master_objective

        if isnothing(objective_delta)
            println("  [cutting plane] iter=$(iter) master_obj=$(round(master_objective; digits=6))")
        else
            println("  [cutting plane] iter=$(iter) master_obj=$(round(master_objective; digits=6)) delta=$(round(objective_delta; digits=6))")
        end
        flush(stdout)
        @info "run_opt: cutting-plane objective progress" iteration=iter master_objective=master_objective objective_delta=objective_delta

        for s in 1:n_scenarios(data)
            scores = _compute_current_od_assignment_costs(
                m, data, mapping, s;
                in_vehicle_time_weight=model.in_vehicle_time_weight
            )
            q_wc, worst_case_value = _separate_budgeted_uncertainty(mapping, s, scores)
            worst_case_by_scenario[s] = worst_case_value
            eta_val = value(eta[s])
            if worst_case_value > eta_val + cutting_plane_tol
                push!(pending_cuts, (s, q_wc, worst_case_value, eta_val))
            end
        end

        push!(iteration_log, Dict{String, Any}(
            "iteration" => iter,
            "master_objective" => master_objective,
            "objective_delta" => objective_delta,
            "violated_scenarios" => length(pending_cuts),
            "worst_case_by_scenario" => worst_case_by_scenario,
        ))
        prev_master_objective = master_objective

        if isempty(pending_cuts)
            converged = true
            break
        end

        if iter == cutting_plane_max_iters
            final_term_status = MOI.ITERATION_LIMIT
            break
        end

        for (s, q_wc, worst_case_value, eta_val) in pending_cuts
            violation = worst_case_value - eta_val
            q_support = [
                Dict(
                    "od_pair" => mapping.Omega_s[s][od_idx],
                    "q_value" => q_val,
                )
                for (od_idx, q_val) in sort(collect(q_wc), by=first)
            ]
            cut_event = Dict{String, Any}(
                "iteration" => iter,
                "scenario" => s,
                "worst_case_value" => worst_case_value,
                "eta_value" => eta_val,
                "violation" => violation,
                "support_size" => length(q_wc),
                "q_support" => q_support,
            )
            push!(cut_log, cut_event)
            println("  [cutting plane] add cut iter=$(iter) scenario=$(s) worst_case=$(round(worst_case_value; digits=6)) eta=$(round(eta_val; digits=6)) violation=$(round(violation; digits=6)) support=$(length(q_wc))")
            flush(stdout)
            @info "run_opt: cutting-plane cut added" iteration=iter scenario=s worst_case_value=worst_case_value eta_value=eta_val violation=violation support_size=length(q_wc)
            _add_robust_cutting_plane_cut!(
                m, data, mapping, s, q_wc;
                in_vehicle_time_weight=model.in_vehicle_time_weight
            )
        end
    end

    solve_time_sec = Dates.value(now() - solve_start) / 1000

    obj = nothing
    solution = nothing
    if final_term_status == MOI.OPTIMAL || final_term_status == MOI.ITERATION_LIMIT
        if JuMP.has_values(m)
            obj = JuMP.objective_value(m)
            x_val = _value_recursive(m[:x])
            y_val = _value_recursive(m[:y])
            solution = (x_val, y_val)
        end
    end

    runtime_sec = Dates.value(now() - start_time) / 1000
    return OptResult(
        final_term_status,
        obj,
        solution,
        runtime_sec,
        m,
        mapping,
        build_result.detour_combos,
        build_result.counts,
        nothing,
        Dict{String, Any}(
            "build_time_sec" => build_time_sec,
            "solve_time_sec" => solve_time_sec,
            "cutting_plane_iterations" => iteration,
            "cutting_plane_converged" => converged,
            "initial_cuts_added" => initial_cuts_added,
            "iteration_log" => iteration_log,
            "cut_log" => cut_log,
            "solve_mode" => "cutting_plane",
        ),
    )
end
