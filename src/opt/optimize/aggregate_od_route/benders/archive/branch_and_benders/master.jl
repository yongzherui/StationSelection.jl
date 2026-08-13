struct _BranchBendersMasterArtifacts
    model::JuMP.Model
    y::Vector{JuMP.VariableRef}
    theta::JuMP.Containers.DenseAxisArray
    chain_cache::Dict
end

function _build_branch_benders_master(
    data, model, solver, requests, feasible_pairs, cut_ids, master_env,
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

    add_station_selection_variables!(master, data)
    y = master[:y]
    add_benders_cut_placeholder_variables!(master, cut_ids)
    theta = master[:theta]
    add_station_limit_constraint!(master, data, model.l)
    _add_default_endpoint_coverage_constraints!(master, y, data, model, requests)
    walking_cost_expr, _ = _add_nearest_open_master_walking_cost!(
        master, data, model, y, requests, feasible_pairs,
    )
    chain_cache = master[:nearest_endpoint_chain_cache]
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
