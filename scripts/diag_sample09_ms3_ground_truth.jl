"""
    scripts/diag_sample09_ms3_ground_truth.jl

Ground-truth check for the sample09_direct_enumeration_guide_compare_ms3 n=10 mismatch:
solves the exact same (n_stations=10, l=5, max_stops=3) AggregateODRouteModel via
DirectSolver (exhaustive enumeration + monolithic MIP, no Benders/decomposition at all)
to get an independent, decomposition-free optimum to compare plain (9980.14) and
guided (11870.78, phase1_objective=9198.22) against.
"""

using DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

n_stations = 10
l = L_FOR[n_stations]
max_stops = resolve_max_stops(:ms3, n_stations)
data = load_sample09(n_stations)
model = build_model(l, max_stops, MAX_WALK, CFG)

solver = DirectSolver(
    config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=1e-4),
    max_enumerated_routes=200_000,
    max_enumeration_time_sec=60.0,
)

t0 = time()
result = StationSelection.run_opt(data, model, solver)
elapsed = time() - t0

@printf(
    "DirectSolver ground truth: n_stations=%d l=%d max_stops=%d -> status=%s obj=%s wall=%.1fs\n",
    n_stations, l, max_stops, string(result.termination_status),
    isnothing(result.objective_value) ? "nothing" : string(result.objective_value),
    elapsed,
)

id_map = result.mapping.array_idx_to_station_id
y = result.model[:y]
selected = sort([id_map[i] for i in 1:length(y) if round(value(y[i])) == 1])
println("selected stations: ", selected)
