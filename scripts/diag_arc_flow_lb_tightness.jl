"""
    scripts/diag_arc_flow_lb_tightness.jl

Diagnostic: how tight is the multicommodity arc-flow lower bound
(_build_lifted_routing_lower_bound_exprs!) relative to the true certified routing subproblem
cost, at a FIXED y -- both at the known optimal y* (from an already-converged run) and at a
couple of alternative y's, on the real zhuzhou n=15/seed=42 instance (ms4).

No optimize!() calls outside small LPs -- safe to run directly per this repo's established
sizing/diagnostic-script exception.

Usage:
    julia --project=. scripts/diag_arc_flow_lb_tightness.jl
"""

using StationSelection, DataFrames, JuMP, Gurobi, Printf
const MOI = JuMP.MOI

include(joinpath(@__DIR__, "zhuzhou_p16_scaling_route100x.jl"))

const N_STATIONS = 15
const SEED = 42
const KNOWN_OPTIMAL_STATION_IDS = [11, 21, 100, 117, 121, 133, 158, 202]
const KNOWN_OPTIMAL_OBJECTIVE = 66851.45479524239

function report_tightness(label::String, y_hat::Vector{Float64}, data, model, mapping, requests, feasible_pairs, cut_ids, optimizer_env)
    open_stations = StationSelection._open_station_values(y_hat)
    assignments, infeasible = StationSelection._fixed_assignments_from_y(
        data, requests, feasible_pairs, y_hat;
        style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
    )
    if !isempty(infeasible)
        println("$label: INFEASIBLE y -- open_stations=$(sort(open_stations)), infeasible requests=$(infeasible)")
        return
    end

    # route_lb(y_hat): fix y, build only the arc-flow exprs, minimize their sum.
    lb_m = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(lb_m)
    @variable(lb_m, 0 <= y[1:N_STATIONS] <= 1)
    for j in 1:N_STATIONS
        fix(y[j], y_hat[j]; force=true)
    end
    route_lb_exprs = StationSelection._build_lifted_routing_lower_bound_exprs!(
        lb_m, data, model, y, cut_ids, requests, feasible_pairs,
    )
    @objective(lb_m, Min, sum(route_lb_exprs[cut_id] for cut_id in cut_ids))
    optimize!(lb_m)
    route_lb = primal_status(lb_m) == MOI.FEASIBLE_POINT ? objective_value(lb_m) : NaN

    # True certified routing cost at y_hat: fix y, derive z via the master's own z-builder, then
    # solve the repriced route-covering LP seeded by a real CG pass -- same pattern as the
    # correctness test.
    zm = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(zm)
    @variable(zm, 0 <= zy[1:N_STATIONS] <= 1)
    for j in 1:N_STATIONS
        fix(zy[j], y_hat[j]; force=true)
    end
    StationSelection._add_nearest_open_master_z!(
        zm, data, zy, requests, feasible_pairs, model.max_walking_distance,
        model.allow_walk_only, model.assignment_policy.feasibility_cut_style,
    )
    optimize!(zm)
    if primal_status(zm) != MOI.FEASIBLE_POINT
        println("$label: z-derivation infeasible")
        return
    end
    z_hat = Dict{StationSelection._AggregateODRouteEndpointChainKey, Vector{Float64}}(
        key => round.(value.(vars)) for (key, vars) in zm[:nearest_endpoint_chain_cache]
    )

    ground_truth_solver = BendersSolver(
        config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        decomposition=BendersY(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
            max_iterations=500, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=60.0,
        ),
    )
    cg_result = StationSelection._solve_fixed_route_covering_by_cg(
        data, model, assignments, ground_truth_solver, nothing, open_stations,
    )

    total_true = 0.0
    for cut_id in cut_ids
        v_hat, _rho, _pool, _n_new, _rounds, exhausted, _delta =
            StationSelection._solve_yz_route_subproblem_lp_with_repricing(
                data, model, mapping, requests, feasible_pairs,
                cg_result.generated_columns, z_hat, optimizer_env, true,
            )
        exhausted || @warn "$label: pricing not exhausted for cut_id=$cut_id"
        total_true += v_hat
    end

    gap_pct = 100 * (total_true - route_lb) / total_true
    @printf("%-30s route_lb=%-14.2f true_routing_cost=%-14.2f gap=%.2f%%  (route_lb/true = %.1f%%)\n",
        label, route_lb, total_true, gap_pct, 100 * route_lb / total_true)
end

function main()
    data, max_walk = build_instance(FAMILY, N_STATIONS, N_PAIRS, SEED, DATA_DIR)
    l = _l_for(N_STATIONS)
    max_stops = resolve_max_stops(:ms4, N_STATIONS)
    model = build_model(l, max_stops, max_walk, CFG)
    mapping = StationSelection.create_map(model, data)
    requests, demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)
    cut_ids = sort!(collect(keys(mapping.Q_s)))
    optimizer_env = Gurobi.Env()

    id_to_idx = Dict(id => idx for (idx, id) in enumerate(mapping.array_idx_to_station_id))

    println("array_idx_to_station_id = ", mapping.array_idx_to_station_id)
    println("id_to_idx = ", id_to_idx)
    y_star = zeros(N_STATIONS)
    for id in KNOWN_OPTIMAL_STATION_IDS
        y_star[id_to_idx[id]] = 1.0
    end
    println("y_star open array indices = ", StationSelection._open_station_values(y_star))
    println("Known-optimal objective (from converged run) = $KNOWN_OPTIMAL_OBJECTIVE")
    report_tightness("y* (known optimal)", y_star, data, model, mapping, requests, feasible_pairs, cut_ids, optimizer_env)

    # Alternative feasible y: swap one open station for a closed one (first closed station in,
    # last open station out), to see tightness away from the optimum too.
    open_idx = findall(==(1.0), y_star)
    closed_idx = findall(==(0.0), y_star)
    y_alt = copy(y_star)
    y_alt[open_idx[end]] = 0.0
    y_alt[closed_idx[1]] = 1.0
    report_tightness("y_alt (one station swapped)", y_alt, data, model, mapping, requests, feasible_pairs, cut_ids, optimizer_env)
end

main()
