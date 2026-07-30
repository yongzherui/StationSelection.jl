"""
    scripts/diag_direct_enumeration_guide_cut_validity.jl

Root-cause diagnostic for the sample09 n=10, max_stops=3, BendersYZ, zero_completion
mismatch (plain=9198.22 correct; guided phase2=11587.82 wrong). Runs phase 1 directly
(capturing its harvested cuts), then checks whether any harvested cut is INVALID: fixes
y to the known true-optimal station set y*=[11,21,22,40,108] (confirmed by DirectSolver
ground truth) in a fresh master with the harvested cuts applied, and compares the
resulting forced lower bound on theta against the true (independently computed)
achievable routing cost at y*. If the forced bound exceeds the true cost, that cut is
mathematically invalid.
"""

using DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

n_stations = 10
l = L_FOR[n_stations]
max_stops = resolve_max_stops(:ms3, n_stations)
data = load_sample09(n_stations)
model = build_model(l, max_stops, MAX_WALK, CFG)

solver = BendersSolver(
    config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
    decomposition=BendersYZ(),
    inner_solver=ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
        max_iterations=CFG.inner_cg_max_iters, max_columns_per_iteration=20, n_candidates=20,
        pricing_time_limit_sec=CFG.inner_pricing_time, final_ip_time_limit_sec=CFG.inner_ip_time_limit,
    ),
    max_iterations=CFG.benders_max_iters,
    reprice_subproblem=true,
    max_reprice_rounds=CFG.max_reprice_rounds,
    cut_derivation=:zero_completion,
    lifted_walking_objective=true,
    direct_enumeration_guide=true,
    direct_enumeration_max_routes=50_000,
    direct_enumeration_time_limit_sec=120.0,
)

println("=== Running phase 1 directly to capture harvested cuts ===")
full_pool = StationSelection.enumerate_aggregate_od_route_columns(
    StationSelection._unit_weighted_routing_model(model), data;
    max_routes=solver.direct_enumeration_max_routes, time_limit_sec=solver.direct_enumeration_time_limit_sec,
)
println("enumerated_routes=", length(full_pool))

harvested = NamedTuple[]
phase1 = StationSelection._run_aggregate_od_route_nearest_open_benders_yz(
    data, model, solver; direct_enumeration_pool=full_pool, harvested_cuts=harvested,
)
println("phase1 objective=", phase1.objective_value, "  n_harvested_cuts=", length(harvested))
for c in harvested
    println("  cut_id=", c.cut_id, "  cut_constant=", c.cut_constant, "  n_coeffs=", length(c.coeffs))
end

println("\n=== Running PLAIN (no direct_enumeration_pool) to also harvest ITS cuts ===")
harvested_plain = NamedTuple[]
plain_result = StationSelection._run_aggregate_od_route_nearest_open_benders_yz(
    data, model, solver; harvested_cuts=harvested_plain,
)
println("plain objective=", plain_result.objective_value, "  n_harvested_cuts=", length(harvested_plain))
for c in harvested_plain
    println("  cut_id=", c.cut_id, "  cut_constant=", c.cut_constant, "  n_coeffs=", length(c.coeffs))
end

# --- Compute the TRUE achievable (unweighted) routing cost at each candidate y*, and
# check whether the harvested cuts force theta above that true value there. ---
mapping = StationSelection.create_map(model, data)
requests, demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)
cut_groups = StationSelection._benders_cut_groups(requests, solver.cut_mode)
cut_ids = sort!(collect(keys(cut_groups)))
unit_model = StationSelection._unit_weighted_routing_model(model)

candidate_y_stars = Dict(
    "ground_truth_DirectSolver" => [11, 21, 22, 40, 108],
    "phase1_own_incumbent" => [11, 22, 40, 106, 108],
)

function check_cuts_at_ystar(label, y_star_ids, cuts_to_check, cuts_label)
    println("\n=== candidate y* = $label : $y_star_ids  (checking $cuts_label cuts) ===")
    y_hat = zeros(Float64, data.n_stations)
    for id in y_star_ids
        y_hat[mapping.station_id_to_array_idx[id]] = 1.0
    end

    assignments, infeasible = StationSelection._fixed_assignments_from_y(
        data, requests, feasible_pairs, y_hat;
        style=model.assignment_policy.feasibility_cut_style,
        max_walking_distance=model.max_walking_distance,
        allow_walk_only=model.allow_walk_only,
        allow_same_station=true,
    )
    println("infeasible at this y*: ", infeasible)
    isempty(infeasible) || return

    cg_result = StationSelection._solve_fixed_route_covering_by_cg(
        data, unit_model, assignments, solver, nothing, StationSelection._open_station_values(y_hat),
    )
    true_Q_ystar = cg_result.final_result.objective_value
    walking_cost_ystar = StationSelection._lifted_walking_cost(data, model, assignments)
    true_combined_ystar = walking_cost_ystar + model.route_regularization_weight * true_Q_ystar
    @printf("true unweighted routing cost Q(y*) = %.6f\n", true_Q_ystar)
    @printf("true combined objective at y* = %.6f\n", true_combined_ystar)

    optimizer_env = Gurobi.Env()
    test_master = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(test_master)
    @variable(test_master, y[1:data.n_stations], Bin)
    @variable(test_master, theta[cut_ids] >= 0.0)
    @constraint(test_master, sum(y) == model.l)
    for j in 1:data.n_stations
        @constraint(test_master, y[j] == y_hat[j])
    end
    walking_cost_expr, x_by_pair_full = StationSelection._add_nearest_open_master_walking_cost!(
        test_master, data, model, y, requests, feasible_pairs,
    )
    chain_cache = test_master[:nearest_endpoint_chain_cache]
    @objective(test_master, Min, walking_cost_expr)
    optimize!(test_master)
    println("chain_cache solve status: ", termination_status(test_master))

    # Evaluate EACH individual harvested cut's own implied theta value at this y* (not just
    # the binding max across all of them), using the now-concrete chain_cache variable values.
    for (idx, c) in enumerate(cuts_to_check)
        implied = c.cut_constant + sum(c.coeffs[k] * value(chain_cache[k[1]][k[2]]) for k in keys(c.coeffs); init=0.0)
        @printf(
            "  cut #%d (cut_id=%d): implied theta value at this y* = %.6f  (true Q(y*) = %.6f)  %s\n",
            idx, c.cut_id, implied, true_Q_ystar, implied > true_Q_ystar + 1e-4 ? "*** INVALID ***" : "ok",
        )
    end
end

for (label, y_star_ids) in candidate_y_stars
    check_cuts_at_ystar(label, y_star_ids, harvested, "phase1 (guided)")
    check_cuts_at_ystar(label, y_star_ids, harvested_plain, "plain (unguided)")
end
