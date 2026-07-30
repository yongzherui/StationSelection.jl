"""
    scripts/diag_direct_ly_tiny_fixture.jl

Diagnose the CG-vs-DirectSolver disagreement found by
test/opt/test_aggregate_od_route_direct_ly.jl on the tiny 5-station fixture under
:direct_ly, walk_cost_weight=0.0 (CG: obj=0.7, y={2,3,4,5}; DirectSolver: obj=0.5, y=?).
Prints both solvers' station sets/objectives/selected routes, then re-solves DirectSolver
restricted to CG's own winning y (via RouteCoveringProblem) to separate "CG picked a
genuinely worse y" from "CG's own final IP pool was incomplete for its own y."
"""

using DataFrames, Dates, Gurobi, JuMP, Printf, StationSelection
const MOI = JuMP.MOI

function fixture()
    stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
    requests = DataFrame(
        id=[1, 2],
        start_station_id=[1, 2],
        end_station_id=[5, 4],
        request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
    )
    walking_costs = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:5, j in 1:5
        walking_costs[(i, j)] = 100.0
    end
    walking_costs[(1, 1)] = 0.0
    walking_costs[(1, 2)] = 3.0
    walking_costs[(4, 5)] = 3.0
    walking_costs[(5, 5)] = 0.0
    walking_costs[(2, 2)] = 0.0
    walking_costs[(4, 4)] = 0.0
    routing_costs = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:5, j in 1:5
        routing_costs[(i, j)] = abs(i - j) + 1.0
    end
    return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
end

function model_for(style::Symbol)
    return AggregateODRouteModel(
        4;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
        max_walking_distance=5.0,
        route_regularization_weight=0.1,
        walk_cost_weight=0.0,
        repositioning_time=0.0,
        max_stops=3,
        max_wait_time=1000.0,
        detour_factor=2.0,
    )
end

function open_set(result)
    y = value.(result.model[:y])
    return sort!([j for j in eachindex(y) if y[j] > 0.5])
end

function print_selected_routes(result, label)
    m = result.model
    haskey(m.obj_dict, :theta_compat) || return
    theta = m[:theta_compat]
    mapping = result.mapping
    column_by_id = Dict(c.id => c for c in mapping.columns)
    println("  [$label] selected routes:")
    for ((cid, s), var) in theta
        v = value(var)
        v > 1e-6 || continue
        col = column_by_id[cid]
        @printf("    scenario=%d column=%d tau=%.3f value=%.4f od_pairs=%s\n", s, cid, col.tau, v, col.od_pairs)
    end
end

env = Gurobi.Env()
data = fixture()

println("=== DirectSolver (:direct_ly) ===")
gt = run_opt(data, model_for(:direct_ly), DirectSolver(
    optimizer_env=env, silent=true, mip_gap=0.0, max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
))
@printf("status=%s obj=%.6f y=%s\n", gt.termination_status, gt.objective_value, open_set(gt))
print_selected_routes(gt, "DirectSolver")

println("\n=== ColumnGenerationSolver (:direct_ly) ===")
cg = run_aggregate_od_route_column_generation(
    model_for(:direct_ly), data;
    optimizer_env=env, verbose=false,
    max_cg_iters=200, max_new_columns=20, n_candidates=20,
    ip_time_limit_sec=30.0, mip_gap=0.0, silent=true,
)
@printf(
    "status=%s obj=%.6f y=%s cg_stop_reason=%s n_cg_iters=%d lp_bound=%.6f n_columns=%d\n",
    cg.final_result.termination_status, cg.final_result.objective_value, open_set(cg.final_result),
    cg.cg_stop_reason, cg.n_cg_iters, cg.lp_bound, length(cg.generated_columns),
)
print_selected_routes(cg.final_result, "CG")

cg_y = open_set(cg.final_result)
gt_y = open_set(gt)

if cg_y != gt_y
    println("\n=== Station sets DIFFER -- re-solving DirectSolver restricted to CG's own y=$cg_y ===")
    rcp = RouteCoveringProblem(
        4, cg_y, Dict{NTuple{3,Int}, Tuple{Int,Int}}();
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(:direct_ly),
        max_walking_distance=5.0, route_regularization_weight=0.1, walk_cost_weight=0.0,
        repositioning_time=0.0, max_stops=3, max_wait_time=1000.0, detour_factor=2.0,
    )
    fixed_y_gt = run_opt(data, rcp, DirectSolver(
        optimizer_env=env, silent=true, mip_gap=0.0, max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
    ))
    @printf(
        "DirectSolver at CG's y=%s: status=%s obj=%.6f\n",
        cg_y, fixed_y_gt.termination_status, fixed_y_gt.objective_value,
    )
    print_selected_routes(fixed_y_gt, "DirectAtCGsY")
    if isapprox(fixed_y_gt.objective_value, cg.final_result.objective_value; atol=1e-6)
        println("\n=> CG's own y is genuinely worse than DirectSolver's y (CG picked a worse y, not a pool-completeness gap for its own y).")
    else
        println("\n=> CG's final IP pool was INCOMPLETE for CG's own y (a price-and-branch pool-completeness gap, not a wrong-y bug).")
    end
else
    println("\n=== Station sets AGREE ($cg_y) -- objective gap is pool completeness for the SAME y ===")
end

println("\n=== Manual NP-row RHS check at DirectSolver's optimal y=$gt_y (sanity: no accidental relaxation) ===")
mapping = gt.mapping
y_open = Set(gt_y)
for s in 1:n_scenarios(data)
    for (o, d) in mapping.Omega_s[s]
        pairs = StationSelection.get_valid_jk_pairs(mapping, o, d)
        for pair in pairs
            StationSelection.requires_no_vehicle_route(pair) && continue
            j, k = pair
            open_both = j in y_open && k in y_open
            @printf("  OD=(%d,%d) pair=(%d,%d) open_both=%s\n", o, d, j, k, open_both)
        end
    end
end
