"""Audit the first n=15 Branch-and-Benders objective mismatch at both competing station sets."""

using CSV, DataFrames, Gurobi, JuMP, StationSelection
const SS = StationSelection

include(joinpath(@__DIR__, "run_method_compare_task.jl"))

const OUTDIR = length(ARGS) >= 1 ? abspath(ARGS[1]) : abspath(joinpath(
    @__DIR__, "..", "results", "branch_benders_objective_audit_n15_p16_s123_q3",
))
const DATA_DIR = length(ARGS) >= 2 ? abspath(ARGS[2]) : abspath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))

function evaluate_station_set(data, model, requests, feasible_pairs, indices, env)
    y_hat = zeros(data.n_stations); y_hat[indices] .= 1.0
    assignments, infeasible = SS._fixed_assignments_from_y(
        data, requests, feasible_pairs, y_hat;
        style=:big_m_nearest, max_walking_distance=model.max_walking_distance,
        allow_walk_only=false, allow_same_station=true,
    )
    isempty(infeasible) || error("station set $indices has infeasible assignments: $infeasible")
    open_stations = SS._open_station_values(y_hat)
    cg_inner = ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=env, silent=true, mip_gap=0.0),
        max_iterations=10_000, max_columns_per_iteration=20, n_candidates=20,
        reduced_cost_tol=1e-7, pricing_time_limit_sec=120.0,
        final_ip_time_limit_sec=600.0,
    )
    cg_solver = BendersSolver(
        config=SolverConfig(optimizer_env=env, silent=true, mip_gap=0.0),
        decomposition=BendersYZ(), cut_derivation=:restricted_mw_fixed_pi,
        inner_solver=cg_inner, lifted_walking_objective=false,
    )
    cg = SS._solve_fixed_route_covering_by_cg(
        data, model, assignments, cg_solver, nothing, open_stations,
    )

    direct_inner = DirectSolver(
        config=SolverConfig(optimizer_env=env, silent=true, mip_gap=0.0),
        max_enumerated_routes=typemax(Int), max_enumeration_time_sec=9_000.0,
    )
    direct_solver = BendersSolver(
        config=SolverConfig(optimizer_env=env, silent=true, mip_gap=0.0),
        decomposition=BendersYZ(), cut_derivation=:standard,
        inner_solver=direct_inner, lifted_walking_objective=false,
    )
    direct = SS._solve_fixed_route_covering_by_cg(
        data, model, assignments, direct_solver, nothing, open_stations,
    )
    return (
        station_indices=join(indices, ";"),
        station_ids=join(sort(data.array_idx_to_station_id[indices]), ";"),
        cg_objective=cg.final_result.objective_value,
        cg_lp_bound=cg.lp_bound,
        cg_pool_size=length(cg.generated_columns),
        cg_iterations=cg.n_cg_iters,
        cg_stop_reason=string(cg.cg_stop_reason),
        direct_objective=direct.final_result.objective_value,
        direct_lp_bound=direct.lp_bound,
        direct_pool_size=length(direct.generated_columns),
        absolute_difference=abs(cg.final_result.objective_value - direct.final_result.objective_value),
    )
end

function main()
    mkpath(OUTDIR)
    cfg = (
        route_reg_weight=10.0, walk_cost_weight=0.1, repositioning_time=20.0,
        max_wait_time=900.0, detour_factor=2.0,
    )
    data, max_walk = build_instance("zhuzhou", 15, 16, 123, DATA_DIR; n_scenarios=3)
    model = build_model(8, 5, max_walk, cfg)
    mapping = SS.create_map(model, data)
    requests, _demand, feasible_pairs = SS._aggregate_od_route_benders_requests(mapping)
    env = Gurobi.Env()
    rows = [
        evaluate_station_set(data, model, requests, feasible_pairs,
                             [2, 3, 4, 9, 10, 12, 13, 14], env),
        evaluate_station_set(data, model, requests, feasible_pairs,
                             [2, 3, 4, 8, 9, 10, 12, 14], env),
    ]
    path = joinpath(OUTDIR, "fixed_station_objective_audit.csv")
    CSV.write(path, DataFrame(rows))
    show(stdout, MIME("text/csv"), DataFrame(rows)); println()
    println("Wrote $path")
end

main()
