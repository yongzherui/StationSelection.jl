"""
    scripts/zhuzhou_p16_direct_ly_scaling.jl

Scalability check for NearestOpenAggregateODAssignmentPolicy(:direct_ly) (γ-chain, no x):
wall time, CG iteration count, and certified LP bound across n_stations in
{20,30,40,50,60}, single scenario, seed=123, max_stops=4, walk_cost_weight=0.1 --
identical instance/CFG to scripts/zhuzhou_p16_cg_ms45_singlescenario.jl's own :big_m_nearest
("cg_ms4") sweep, whose wall_time_sec/n_iterations/lp_bound are already on disk at
experiments/zhuzhou_p16_cg_ms45_singlescenario/results/*_s123__cg_ms4.csv -- not re-run here.

Usage:
    julia --project=. scripts/zhuzhou_p16_direct_ly_scaling.jl <n_stations> <outdir>
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = 16
const SEED = 123
const MAX_STOPS = 4
const ROUTE_REG_WEIGHT = 10.0
const WALK_COST_WEIGHT = 0.1
const REPOSITIONING_TIME = 20.0
const DETOUR_FACTOR = 2.0
const MAX_WAIT_TIME = 900.0
const MAX_WALKING_DISTANCE = 600.0

_l_for(n::Int) = ceil(Int, n / 2)

function run_one(n_stations::Int, outdir::String)
    l = _l_for(n_stations)
    inst_name = "zhuzhou_n$(n_stations)_p$(N_PAIRS)_s$(SEED)"
    results_dir = joinpath(outdir, "results")
    mkpath(results_dir)

    data, meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=1, endpoint_overlap=2.0, seed=SEED,
    )
    print_zhuzhou_data_summary(data, meta)

    model = AggregateODRouteModel(
        l;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(:direct_ly),
        max_walking_distance=MAX_WALKING_DISTANCE,
        route_regularization_weight=ROUTE_REG_WEIGHT,
        walk_cost_weight=WALK_COST_WEIGHT,
        repositioning_time=REPOSITIONING_TIME,
        max_stops=MAX_STOPS,
        max_wait_time=MAX_WAIT_TIME,
        detour_factor=DETOUR_FACTOR,
    )

    env = Gurobi.Env()
    t0 = time()
    result = run_aggregate_od_route_column_generation(
        model, data;
        optimizer_env=env, verbose=false,
        max_cg_iters=10000, max_new_columns=20, n_candidates=20,
        pricing_time_limit_sec=120.0, ip_time_limit_sec=300.0, mip_gap=1e-4, silent=true,
    )
    wall = time() - t0

    @printf(
        "[%s / direct_ly] status=%s stop_reason=%s lp_bound=%.4f obj=%.4f n_iters=%d wall=%.1fs\n",
        inst_name, result.final_result.termination_status, result.cg_stop_reason,
        result.lp_bound, result.final_result.objective_value, result.n_cg_iters, wall,
    )
    flush(stdout)

    row = (
        instance=inst_name, n_stations=n_stations, l=l, seed=SEED, method="direct_ly_ms4",
        status=string(result.final_result.termination_status),
        cg_stop_reason=string(result.cg_stop_reason),
        objective_value=result.final_result.objective_value,
        wall_time_sec=wall,
        n_iterations=result.n_cg_iters,
        lp_bound=result.lp_bound,
        n_columns=length(result.generated_columns),
    )
    CSV.write(joinpath(results_dir, "$(inst_name)__direct_ly_ms4.csv"), DataFrame([row]))
    return row
end

function main()
    length(ARGS) >= 2 || error("Usage: zhuzhou_p16_direct_ly_scaling.jl <n_stations> <outdir>")
    n_stations = parse(Int, ARGS[1])
    outdir = ARGS[2]
    run_one(n_stations, outdir)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
