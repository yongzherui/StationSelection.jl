"""
    scripts/diag_schedule_same_yhat_gap.jl

Diagnostic for the route_regularization_weight_schedule investigation: `direct`/`jump-2` and every
10-stage schedule variant at n=15 (sample_09 fixture, BendersYZ, cut_derivation=:restricted_mw_fixed_pi,
reprice_subproblem=true, target beta=1.0) all terminate with the IDENTICAL open-station set
{11,21,40,100,106,128,138,196} (confirmed from aggregate_od_route_benders_iterations.csv's
y_hat_signature column), yet report DIFFERENT combined objectives: 2613.18 (direct/jump-2) vs.
2644.05 (every 10-stage schedule tried, regardless of spacing).

Since `_fixed_assignments_from_y` is a pure function of the open-station SET (not of solve
history), the assignments for this y_hat must be identical across every run. The only thing that
legitimately differs between how `direct` and a 10-stage schedule reach this same y_hat is the
`shared_pool` of previously-discovered route columns seeded into `_solve_fixed_route_covering_by_cg`
(schedule runs seed from a MUCH larger, differently-composed historical pool accumulated across many
more stage transitions/iterations). This script isolates that one variable: solve the SAME known
y_hat's fixed-assignment route-covering CG (a) completely fresh (no seed) and (b) reseeded with a
large decoy pool built from several OTHER station sets actually visited during the failing ramp run
(taken from its logged y_hat_signature values), to see whether seeding alone can make the reported
objective regress despite `cg_stop_reason==:optimality_proven`/`termination_status==MOI.OPTIMAL`.

Usage:
    julia --project=. scripts/diag_schedule_same_yhat_gap.jl <outdir>
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

const SS = StationSelection

# The station set every variant (direct, jump-2, every 10-stage schedule) converges its MASTER to
# at n=15 -- confirmed via y_hat_signature in the iteration logs.
const KNOWN_STATION_IDS = [11, 21, 40, 100, 106, 128, 138, 196]

# A handful of OTHER station sets actually visited mid-run by the failing exp10_s001 schedule
# (from its logged y_hat_signature column, iterations 248/250/251/252) -- used to build a decoy
# seed pool of the same rough character (same fixture, same request set) as what a real multi-stage
# schedule accumulates before it ever reaches KNOWN_STATION_IDS.
const DECOY_STATION_ID_SETS = [
    [11, 22, 40, 48, 100, 121, 158, 196],
    [11, 100, 101, 106, 108, 121, 158, 196],
    [21, 48, 100, 106, 108, 121, 129, 196],
    [21, 22, 48, 100, 101, 121, 128, 196],
]

function _open_indices(mapping, station_ids::Vector{Int})::Vector{Int}
    id_to_idx = Dict(id => idx for (idx, id) in enumerate(mapping.array_idx_to_station_id))
    return sort([id_to_idx[id] for id in station_ids])
end

function _y_hat_for(data, open_idxs::Vector{Int})::Vector{Float64}
    y_hat = zeros(Float64, data.n_stations)
    for idx in open_idxs
        y_hat[idx] = 1.0
    end
    return y_hat
end

function main()
    length(ARGS) >= 1 || error("Usage: julia diag_schedule_same_yhat_gap.jl <outdir>")
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

    solver = BendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
        decomposition=BendersYZ(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
            max_iterations=CFG.inner_cg_max_iters, max_columns_per_iteration=20, n_candidates=20,
            pricing_time_limit_sec=CFG.inner_pricing_time, final_ip_time_limit_sec=CFG.inner_ip_time_limit,
        ),
        max_iterations=3000, max_reprice_rounds=CFG.max_reprice_rounds, reprice_subproblem=true,
        cut_derivation=:restricted_mw_fixed_pi, lifted_walking_objective=true,
    )

    known_open = _open_indices(mapping, KNOWN_STATION_IDS)
    known_y_hat = _y_hat_for(data, known_open)
    known_assignments, known_infeasible = SS._fixed_assignments_from_y(
        data, requests, feasible_pairs, known_y_hat;
        style=model.assignment_policy.feasibility_cut_style,
        max_walking_distance=model.max_walking_distance,
        allow_walk_only=model.allow_walk_only,
        allow_same_station=true,
    )
    isempty(known_infeasible) || error("known station set is infeasible: $(known_infeasible)")
    walking_cost = SS._lifted_walking_cost(data, model, known_assignments)
    @printf("known station set = %s\n", string(KNOWN_STATION_IDS))
    @printf("walking_cost_hat (real weight) = %.6f\n\n", walking_cost)

    println("=== [A] fresh CG-priming solve, no seed (run 1) ===")
    cgA1 = SS._solve_fixed_route_covering_by_cg(data, subproblem_model, known_assignments, solver, nothing, known_open)
    objA1 = walking_cost + 1.0 * cgA1.final_result.objective_value
    @printf("route_cost=%.6f  combined_objective=%.6f\n\n", cgA1.final_result.objective_value, objA1)

    println("=== [A'] fresh CG-priming solve, no seed (run 2, determinism check) ===")
    cgA2 = SS._solve_fixed_route_covering_by_cg(data, subproblem_model, known_assignments, solver, nothing, known_open)
    objA2 = walking_cost + 1.0 * cgA2.final_result.objective_value
    @printf("route_cost=%.6f  combined_objective=%.6f\n\n", cgA2.final_result.objective_value, objA2)

    println("=== Building decoy pool from other real (mid-run) station sets ===")
    decoy_pool = SS.AggregateODRouteColumn[]
    for decoy_ids in DECOY_STATION_ID_SETS
        decoy_open = _open_indices(mapping, decoy_ids)
        decoy_y_hat = _y_hat_for(data, decoy_open)
        decoy_assignments, decoy_infeasible = SS._fixed_assignments_from_y(
            data, requests, feasible_pairs, decoy_y_hat;
            style=model.assignment_policy.feasibility_cut_style,
            max_walking_distance=model.max_walking_distance,
            allow_walk_only=model.allow_walk_only,
            allow_same_station=true,
        )
        isempty(decoy_infeasible) || continue
        cg_decoy = SS._solve_fixed_route_covering_by_cg(data, subproblem_model, decoy_assignments, solver, nothing, decoy_open)
        decoy_pool = SS._deduplicate_aggregate_od_route_columns(vcat(decoy_pool, cg_decoy.final_result.mapping.columns))
        @printf("  decoy set %s -> route_cost=%.6f, pool now %d columns\n",
            string(decoy_ids), cg_decoy.final_result.objective_value, length(decoy_pool))
    end
    println()

    println("=== [B] CG-priming solve at KNOWN station set, seeded with the decoy pool ===")
    cgB = SS._solve_fixed_route_covering_by_cg(
        data, subproblem_model, known_assignments, solver, nothing, known_open; seed_columns=decoy_pool,
    )
    objB = walking_cost + 1.0 * cgB.final_result.objective_value
    @printf("route_cost=%.6f  combined_objective=%.6f\n\n", cgB.final_result.objective_value, objB)

    rows = DataFrame([
        (case="A_fresh_run1", route_cost=cgA1.final_result.objective_value, combined_objective=objA1),
        (case="A_fresh_run2", route_cost=cgA2.final_result.objective_value, combined_objective=objA2),
        (case="B_seeded_decoy_pool", route_cost=cgB.final_result.objective_value, combined_objective=objB),
    ])
    path = joinpath(outdir, "diag_same_yhat_gap.csv")
    CSV.write(path, rows)
    println("Wrote $path")

    @printf("\nSUMMARY: direct/jump-2 reported 2613.18216821966 for this exact station set.\n")
    @printf("A (fresh, no seed)          = %.6f  (delta vs direct = %.6f)\n", objA1, objA1 - 2613.18216821966)
    @printf("A' (fresh, repeat)          = %.6f  (delta vs A       = %.6f)\n", objA2, objA2 - objA1)
    @printf("B (seeded w/ decoy pool)    = %.6f  (delta vs A       = %.6f)\n", objB, objB - objA1)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
