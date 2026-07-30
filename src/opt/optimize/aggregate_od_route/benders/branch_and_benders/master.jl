struct _BranchBendersMasterArtifacts
    model::JuMP.Model
    y::Vector{JuMP.VariableRef}
    theta::JuMP.Containers.DenseAxisArray
    chain_cache::Dict
end

"""Gurobi numbers the root MIP node as zero; tolerate its floating-point callback value."""
_is_root_mipnode(node_count::Real) = node_count < 0.5

"""Solve one scenario MCF at fixed fractional `y_hat` and return its dual value-function cut."""
function _solve_projected_mcf_cut(
    data, subproblem_model, requests, feasible_pairs, cut_id, y_hat, optimizer_env;
    tightness_tolerance=1e-5,
)
    auxiliary = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(auxiliary)
    set_optimizer_attribute(auxiliary, "Threads", 1)
    @variable(auxiliary, 0.0 <= y_aux[1:data.n_stations] <= 1.0)
    fixing = @constraint(auxiliary, [j in 1:data.n_stations], y_aux[j] == y_hat[j])
    expr = _build_lifted_routing_lower_bound_exprs!(
        auxiliary, data, subproblem_model, y_aux, [cut_id], requests, feasible_pairs,
    )[cut_id]
    @objective(auxiliary, Min, expr)
    optimize!(auxiliary)
    termination_status(auxiliary) == MOI.OPTIMAL || throw(ArgumentError(
        "projected MCF separator failed for scenario $cut_id at y=$y_hat: " *
        "status=$(termination_status(auxiliary))",
    ))
    dual_status(auxiliary) == MOI.FEASIBLE_POINT || throw(ArgumentError(
        "projected MCF separator has no dual certificate for scenario $cut_id at y=$y_hat",
    ))
    value_at_generation = objective_value(auxiliary)
    beta = Float64[dual(fixing[j]) for j in 1:data.n_stations]
    alpha = value_at_generation - sum(beta[j] * y_hat[j] for j in eachindex(beta))
    rhs = alpha + sum(beta[j] * y_hat[j] for j in eachindex(beta))
    isapprox(rhs, value_at_generation; atol=tightness_tolerance, rtol=tightness_tolerance) ||
        throw(ArgumentError(
            "projected MCF cut is not tight for scenario $cut_id: value=$value_at_generation rhs=$rhs",
        ))
    return (block_id=cut_id, alpha=alpha, beta=beta, value=value_at_generation)
end

function _projected_mcf_cut_constraint(cut, theta, y)
    return @build_constraint(
        theta[cut.block_id] >= cut.alpha +
            sum(cut.beta[j] * y[j] for j in eachindex(y); init=0.0)
    )
end

struct _MCFDualRow
    rhs::Float64
    coefficients::Dict{Int, Float64}
    sense::Symbol
    parameter::Union{Nothing, Tuple{Symbol, Any}}
end

struct _MCFMWCut
    block_id::Int
    alpha::Float64
    beta_y::Vector{Float64}
    beta_z::Dict{Tuple{_AggregateODRouteEndpointChainKey, Int}, Float64}
    value::Float64
    core_value::Float64
end

"""Evaluate an MCF value-function cut at a `(y,z)` master point."""
function _evaluate_mcf_mw_cut(cut::_MCFMWCut, y_values, z_values)
    return cut.alpha +
        sum(cut.beta_y[j] * y_values[j] for j in eachindex(y_values); init=0.0) +
        sum(coef * z_values[key[1]][key[2]] for (key, coef) in cut.beta_z; init=0.0)
end

function _build_fixed_yz_mcf_lp(
    data, subproblem_model, requests, feasible_pairs, cut_id, y_hat, z_hat, optimizer_env,
)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(m)
    set_optimizer_attribute(m, "Threads", 1)
    @variable(m, 0.0 <= y_aux[1:data.n_stations] <= 1.0)
    y_fix = Dict(j => @constraint(m, y_aux[j] == y_hat[j]) for j in 1:data.n_stations)
    z_cache = Dict{_AggregateODRouteEndpointChainKey, Vector{VariableRef}}()
    z_fix = Dict{Tuple{_AggregateODRouteEndpointChainKey, Int}, ConstraintRef}()
    for (key, values) in z_hat
        vars = @variable(m, [1:length(values)], lower_bound=0.0, upper_bound=1.0)
        z_cache[key] = vars
        for i in eachindex(values)
            z_fix[(key, i)] = @constraint(m, vars[i] == values[i])
        end
    end
    m[:nearest_endpoint_chain_cache] = z_cache
    expr = _build_lifted_routing_lower_bound_exprs!(
        m, data, subproblem_model, y_aux, [cut_id], requests, feasible_pairs,
    )[cut_id]
    @objective(m, Min, expr)
    return m, y_fix, z_fix
end

function _mcf_dual_rows(primal, y_fix, z_fix)
    variables = all_variables(primal)
    var_index = Dict(var => i for (i, var) in enumerate(variables))
    parameter_by_constraint = Dict{ConstraintRef, Tuple{Symbol, Any}}()
    for (j, con) in y_fix
        parameter_by_constraint[con] = (:y, j)
    end
    for (key, con) in z_fix
        parameter_by_constraint[con] = (:z, key)
    end
    rows = _MCFDualRow[]
    for (F, S) in list_of_constraint_types(primal)
        F <: JuMP.AbstractJuMPScalar || continue
        F == VariableRef && continue
        for con in all_constraints(primal, F, S)
            object = constraint_object(con)
            func = object.func
            func isa AffExpr || continue
            set = object.set
            sense, set_rhs = if set isa MOI.EqualTo
                (:eq, Float64(set.value))
            elseif set isa MOI.LessThan
                (:le, Float64(set.upper))
            elseif set isa MOI.GreaterThan
                (:ge, Float64(set.lower))
            else
                continue
            end
            coefficients = Dict{Int, Float64}()
            for (var, coef) in func.terms
                coefficients[var_index[var]] = Float64(coef)
            end
            push!(rows, _MCFDualRow(
                set_rhs - Float64(func.constant), coefficients, sense,
                get(parameter_by_constraint, con, nothing),
            ))
        end
    end
    # Variable bounds are dual rows too; omitting them would make the explicit dual incomplete.
    for (idx, var) in enumerate(variables)
        has_lower_bound(var) && push!(rows, _MCFDualRow(
            lower_bound(var), Dict(idx => 1.0), :ge, nothing,
        ))
        has_upper_bound(var) && push!(rows, _MCFDualRow(
            upper_bound(var), Dict(idx => 1.0), :le, nothing,
        ))
    end
    primal_objective = objective_function(primal)
    c = Float64[coefficient(primal_objective, var) for var in variables]
    objective_constant = Float64(primal_objective.constant)
    return rows, c, objective_constant
end

function _solve_mcf_mw_yz_cut(
    data, subproblem_model, requests, feasible_pairs, cut_id, y_hat, z_hat,
    y_core, z_core, optimizer_env; tightness_tolerance=1e-5,
)
    primal, y_fix, z_fix = _build_fixed_yz_mcf_lp(
        data, subproblem_model, requests, feasible_pairs, cut_id, y_hat, z_hat, optimizer_env,
    )
    optimize!(primal)
    termination_status(primal) == MOI.OPTIMAL || throw(ArgumentError(
        "fractional MCF primal failed for scenario $cut_id: $(termination_status(primal))",
    ))
    value_at_generation = objective_value(primal)
    rows, c, objective_constant = _mcf_dual_rows(primal, y_fix, z_fix)
    dual_model = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(dual_model)
    set_optimizer_attribute(dual_model, "Threads", 1)
    dual_vars = VariableRef[]
    for row in rows
        push!(dual_vars, row.sense == :eq ? @variable(dual_model) :
            row.sense == :ge ? @variable(dual_model, lower_bound=0.0) :
            @variable(dual_model, upper_bound=0.0))
    end
    for j in eachindex(c)
        @constraint(dual_model,
            sum(get(rows[r].coefficients, j, 0.0) * dual_vars[r] for r in eachindex(rows)) == c[j]
        )
    end
    generation_expr = objective_constant +
        sum(rows[r].rhs * dual_vars[r] for r in eachindex(rows))
    @constraint(dual_model, generation_expr == value_at_generation)
    core_rhs = Float64[]
    for row in rows
        rhs = row.rhs
        if !isnothing(row.parameter)
            kind, key = row.parameter
            rhs = kind == :y ? y_core[key] : z_core[key[1]][key[2]]
        end
        push!(core_rhs, rhs)
    end
    @objective(dual_model, Max, objective_constant +
        sum(core_rhs[r] * dual_vars[r] for r in eachindex(rows)))
    optimize!(dual_model)
    termination_status(dual_model) == MOI.OPTIMAL || throw(ArgumentError(
        "fractional MCF MW dual failed for scenario $cut_id: $(termination_status(dual_model))",
    ))
    pi = value.(dual_vars)
    beta_y = zeros(Float64, data.n_stations)
    beta_z = Dict{Tuple{_AggregateODRouteEndpointChainKey, Int}, Float64}()
    alpha = objective_constant
    for (r, row) in enumerate(rows)
        if isnothing(row.parameter)
            alpha += row.rhs * pi[r]
        else
            kind, key = row.parameter
            if kind == :y
                beta_y[key] += pi[r]
            else
                beta_z[key] = get(beta_z, key, 0.0) + pi[r]
            end
        end
    end
    cut = _MCFMWCut(
        cut_id, alpha, beta_y, beta_z, value_at_generation, objective_value(dual_model),
    )
    rhs_at_generation = _evaluate_mcf_mw_cut(cut, y_hat, z_hat)
    isapprox(rhs_at_generation, value_at_generation;
             atol=tightness_tolerance, rtol=tightness_tolerance) || throw(ArgumentError(
        "fractional MCF MW cut is not tight for scenario $cut_id: " *
        "value=$value_at_generation rhs=$rhs_at_generation",
    ))
    return cut
end

function _mcf_mw_yz_cut_constraint(cut::_MCFMWCut, theta, y, chain_cache)
    return @build_constraint(
        theta[cut.block_id] >= cut.alpha +
            sum(cut.beta_y[j] * y[j] for j in eachindex(y); init=0.0) +
            sum(coef * chain_cache[key[1]][key[2]] for (key, coef) in cut.beta_z; init=0.0)
    )
end

function _branch_benders_mcf_complexity(
    data, subproblem_model, requests_s,
)
    stations = Set{Int}()
    for (_s, o, d) in requests_s
        union!(stations, _nearest_open_endpoint_candidates(
            data, o, subproblem_model.max_walking_distance, :pickup,
        ))
        union!(stations, _nearest_open_endpoint_candidates(
            data, d, subproblem_model.max_walking_distance, :dropoff,
        ))
    end
    n = length(stations)
    # Aggregate-flow arcs plus one full arc layer per commodity.
    return (length(requests_s) + 1) * n * max(n - 1, 0)
end

function _branch_benders_mcf_scenario_id(
    data, subproblem_model, requests, cut_ids, requested_id,
)
    if !isnothing(requested_id)
        requested_id in cut_ids || throw(ArgumentError(
            "mcf_scenario_id=$requested_id is not one of the recourse blocks $cut_ids",
        ))
        return requested_id
    end
    return argmax(cut_ids) do cut_id
        requests_s = filter(r -> r[1] == cut_id, requests)
        (_branch_benders_mcf_complexity(data, subproblem_model, requests_s), -cut_id)
    end
end

function _add_branch_benders_mcf_lower_bound!(
    master, data, subproblem_model, solver, y, theta, cut_ids, requests, feasible_pairs,
)
    mode = solver.mcf_lower_bound_mode
    if mode == :all_scenarios
        exprs = _build_lifted_routing_lower_bound_exprs!(
            master, data, subproblem_model, y, cut_ids, requests, feasible_pairs,
        )
        for cut_id in cut_ids
            @constraint(master, theta[cut_id] >= exprs[cut_id])
        end
        return (mode=mode, scenario_id=nothing, common_od_count=0)
    elseif mode == :single_scenario
        selected = _branch_benders_mcf_scenario_id(
            data, subproblem_model, requests, cut_ids, solver.mcf_scenario_id,
        )
        expr = _build_lifted_routing_lower_bound_exprs!(
            master, data, subproblem_model, y, [selected], requests, feasible_pairs,
        )[selected]
        @constraint(master, theta[selected] >= expr)
        return (mode=mode, scenario_id=selected, common_od_count=0)
    end

    od_sets = [Set((o, d) for (s, o, d) in requests if s == cut_id) for cut_id in cut_ids]
    common_ods = isempty(od_sets) ? Set{Tuple{Int, Int}}() : intersect(od_sets...)
    representative = first(cut_ids)
    common_requests = [
        request for request in requests
        if request[1] == representative && (request[2], request[3]) in common_ods
    ]
    expr = if isempty(common_requests)
        AffExpr(0.0)
    else
        _build_lifted_routing_lower_bound_exprs!(
            master, data, subproblem_model, y, [representative], common_requests, feasible_pairs,
        )[representative]
    end
    # The common commodity set is a restriction of every scenario MCF. Hence
    # L_common(y) <= Q_s(y) for every s, and |S| L_common(y) <= sum_s Q_s(y).
    @constraint(master, sum(theta[s] for s in cut_ids) >= length(cut_ids) * expr)
    return (
        mode=mode, scenario_id=representative, common_od_count=length(common_ods),
    )
end

function _build_branch_benders_master(
    data, model, solver, subproblem_model, requests, feasible_pairs, cut_ids, master_env,
)
    cfg = solver.config
    master = Model(() -> Gurobi.Optimizer(master_env))
    cfg.silent && set_silent(master)
    set_optimizer_attribute(master, "Threads", 1)
    set_optimizer_attribute(master, "LazyConstraints", 1)
    set_optimizer_attribute(master, "IntFeasTol", solver.integrality_tolerance)
    if !isnothing(solver.log_dir)
        mkpath(solver.log_dir)
        set_optimizer_attribute(
            master, "LogFile",
            joinpath(solver.log_dir, "aggregate_od_route_branch_benders_gurobi.log"),
        )
        set_optimizer_attribute(master, "DisplayInterval", 1)
    end
    master_mip_gap = isnothing(cfg.mip_gap) ? 0.01 : cfg.mip_gap
    set_optimizer_attribute(master, "MIPGap", master_mip_gap)

    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    _add_default_endpoint_coverage_constraints!(master, y, data, model, requests)
    walking_cost_expr, _ = _add_nearest_open_master_walking_cost!(
        master, data, model, y, requests, feasible_pairs,
    )
    chain_cache = master[:nearest_endpoint_chain_cache]
    mcf_info = _add_branch_benders_mcf_lower_bound!(
        master, data, subproblem_model, solver, y, theta, cut_ids, requests, feasible_pairs,
    )
    master[:branch_benders_mcf_info] = mcf_info
    @objective(master, Min,
        walking_cost_expr + model.route_regularization_weight * sum(theta[s] for s in cut_ids)
    )

    for cut in solver.initial_cuts
        _validate_branch_benders_initial_cut!(
            cut, solver, cut_ids, data.n_stations, chain_cache,
        )
        _add_branch_benders_cut!(master, cut, theta, y, chain_cache)
    end
    return _BranchBendersMasterArtifacts(master, y, theta, chain_cache), master_mip_gap
end
