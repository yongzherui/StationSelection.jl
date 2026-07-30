"""
    scripts/diag_cg_vs_direct_pool_completeness.jl

Direct test of whether Benders' correctness claim ("cuts_added_this_iteration==0" proves global
optimality, no incumbent-tracking luck required) actually holds for the two competing n=15 station
sets from the route_regularization_weight_schedule investigation, by checking whether
`_solve_fixed_route_covering_by_cg`'s v_hat for EACH set is truly exact -- i.e. whether its
price-and-branch final IP step (frozen column pool, no re-pricing during branch-and-bound) matches
a pool-independent ground truth from full route enumeration (`DirectSolver`).

Mirrors the exact methodology from the prior, already-documented investigation
(`/home/yongzr/tmp-claude-sample09-smoke/check_fixed_assignment_gap.jl`, see memory
project_bendersy_vs_bendersyz_yz_pool_gap.md) which confirmed this price-and-branch gap in
isolation for a different station set at route_regularization_weight=10 -- this reruns the same
check at the two sets actually in play here, at target beta=1.0:

  A ("terminal", what every 10-stage schedule -- and BendersYZ's own master -- converges its final
     iteration to): {11,21,40,100,106,128,138,196}, reported v_hat=2644.048247607711
  B ("winning", what `direct`'s persistent incumbent tracking reports as the true best):
     {11,21,100,106,128,138,158,196}, reported v_hat=2613.18216821966 (at direct's iteration 111)

If CG's v_hat for A is genuinely exact (matches DirectSolver enumeration), then A really is the
provable global optimum and B's lower reported value must itself be suspect (or vice versa). If
CG's v_hat for either is inflated above the pool-independent DirectSolver ground truth, that
confirms the price-and-branch gap -- not incumbent-bookkeeping luck -- as the actual reason Benders'
"no more cuts needed" termination criterion fails to certify true optimality here.

Usage:
    julia --project=. scripts/diag_cg_vs_direct_pool_completeness.jl <outdir>
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

const SS = StationSelection

const STATION_SETS = [
    ("A_terminal", [11, 21, 40, 100, 106, 128, 138, 196]),
    ("B_winning",  [11, 21, 100, 106, 128, 138, 158, 196]),
]

function _open_indices(mapping, station_ids::Vector{Int})::Vector{Int}
    id_to_idx = Dict(id => idx for (idx, id) in enumerate(mapping.array_idx_to_station_id))
    return sort([id_to_idx[id] for id in station_ids])
end

function main()
    length(ARGS) >= 1 || error("Usage: julia diag_cg_vs_direct_pool_completeness.jl <outdir>")
    outdir = ARGS[1]
    mkpath(outdir)

    n_stations = 15
    l = L_FOR[n_stations]
    max_stops = resolve_max_stops(:ms4, n_stations)
    data = load_sample09(n_stations)
    model = build_model(l, max_stops, MAX_WALK, CFG)
    subproblem_model = SS._unit_weighted_routing_model(model)

    mapping = create_map(model, data)
    requests, demand, feasible_pairs = SS._aggregate_od_route_benders_requests(mapping)

    cg_solver = BendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
        decomposition=BendersYZ(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
            max_iterations=CFG.inner_cg_max_iters, max_columns_per_iteration=20, n_candidates=20,
            pricing_time_limit_sec=CFG.inner_pricing_time, final_ip_time_limit_sec=CFG.inner_ip_time_limit,
        ),
    )
    direct_solver = BendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
        decomposition=BendersYZ(),
        inner_solver=DirectSolver(
            SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap);
            max_enumerated_routes=CFG.direct_max_routes, max_enumeration_time_sec=CFG.direct_time_limit,
        ),
    )

    rows = NamedTuple[]
    for (label, station_ids) in STATION_SETS
        println("=" ^ 70)
        println("Station set $label = $station_ids")
        println("=" ^ 70)

        open_idxs = _open_indices(mapping, station_ids)
        y_hat = zeros(Float64, data.n_stations)
        for idx in open_idxs
            y_hat[idx] = 1.0
        end
        assignments, infeasible = SS._fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=model.assignment_policy.feasibility_cut_style,
            max_walking_distance=model.max_walking_distance,
            allow_walk_only=model.allow_walk_only,
            allow_same_station=true,
        )
        isempty(infeasible) || error("station set $label is infeasible: $(infeasible)")
        walking_cost = SS._lifted_walking_cost(data, model, assignments)

        cg_result = SS._solve_fixed_route_covering_by_cg(data, subproblem_model, assignments, cg_solver, nothing, open_idxs)
        @printf("  CG:     cg_stop_reason=%s  lp_bound=%.6f  final_IP=%.6f\n",
            string(cg_result.cg_stop_reason), cg_result.lp_bound, cg_result.final_result.objective_value)
        flush(stdout)

        direct_result = SS._solve_fixed_route_covering_by_cg(data, subproblem_model, assignments, direct_solver, nothing, open_idxs)
        @printf("  Direct: lp_bound=%.6f  enumerated_IP=%.6f\n",
            direct_result.lp_bound, direct_result.final_result.objective_value)
        flush(stdout)

        cg_combined = walking_cost + 1.0 * cg_result.final_result.objective_value
        direct_combined = walking_cost + 1.0 * direct_result.final_result.objective_value
        gap = cg_result.final_result.objective_value - direct_result.final_result.objective_value
        @printf("  combined objective (CG)     = %.6f\n", cg_combined)
        @printf("  combined objective (Direct) = %.6f\n", direct_combined)
        @printf("  CG - Direct (unweighted routing units, beta=1 here so same scale) = %.6f\n\n", gap)

        push!(rows, (
            station_set=label, station_ids=string(station_ids),
            walking_cost=walking_cost,
            cg_lp_bound=cg_result.lp_bound, cg_final_ip=cg_result.final_result.objective_value,
            direct_lp_bound=direct_result.lp_bound, direct_enumerated_ip=direct_result.final_result.objective_value,
            cg_minus_direct=gap,
            cg_combined_objective=cg_combined, direct_combined_objective=direct_combined,
        ))
    end

    df = DataFrame(rows)
    path = joinpath(outdir, "diag_cg_vs_direct_pool_completeness.csv")
    CSV.write(path, df)
    println("Wrote $path")
    println()
    println(df)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
