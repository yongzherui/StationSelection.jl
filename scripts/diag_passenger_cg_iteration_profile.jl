# Full column-generation run instrumented per iteration: how does time-per-iteration
# (RMP LP solve + pricing) grow as the column pool accumulates? Unlike the
# station-subset diagnostics (single seeded pricing snapshot), this runs the real CG
# loop and dumps its per-iteration telemetry.
using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection
include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFACG_N_PAIRS", "16"))
const N_SCEN = parse(Int, get(ENV, "PFACG_N_SCENARIOS", "1"))
const RAW_MAX_STOPS = parse(Int, get(ENV, "PFACG_MAX_STOPS", "7"))
const MAX_STOPS = RAW_MAX_STOPS <= 0 ? typemax(Int) : RAW_MAX_STOPS
const MAX_CG_ITERS = parse(Int, get(ENV, "PFACG_MAX_ITERS", "400"))
const TOTAL_TIME = parse(Float64, get(ENV, "PFACG_TOTAL_TIME", "3000"))
const CERT_TIME = parse(Float64, get(ENV, "PFACG_CERT_TIME", "600"))
const PRICING_TIME = parse(Float64, get(ENV, "PFACG_PRICING_TIME", "60"))
# Bound the final integer MIP so it cannot overrun the SLURM wall and cost us the
# summary write. This study cares about LP-bound certification, not the integer solution.
const IP_TIME = parse(Float64, get(ENV, "PFACG_IP_TIME", "600"))
const GRB_ENV = Gurobi.Env()

function main()
    length(ARGS) == 2 || error("usage: ... <n_stations> <output.csv>")
    n = parse(Int, ARGS[1]); output = ARGS[2]
    L = max(2, ceil(Int, n / 2))
    data, _ = generate_zhuzhou_data(DATA_DIR, n, N_PAIRS; n_scenarios=N_SCEN, seed=42)
    model = AggregateODRouteModel(L;
        route_regularization_weight=10.0, walk_cost_weight=0.1,
        repositioning_time=20.0, max_walking_distance=600.0,
        max_wait_time=900.0, detour_factor=2.0,
        max_stops=MAX_STOPS, max_visits_per_node=3)
    @printf("CONFIG n_stations=%d L=%d n_pairs=%d n_scenarios=%d max_stops=%s\n",
        n, L, N_PAIRS, N_SCEN, MAX_STOPS == typemax(Int) ? "inf" : string(MAX_STOPS))
    flush(stdout)

    result = run_passenger_free_assignment_column_generation(model, data;
        optimizer_env=GRB_ENV, max_cg_iters=MAX_CG_ITERS,
        total_time_limit_sec=TOTAL_TIME,
        certification_time_limit_sec=CERT_TIME,
        pricing_time_limit_sec=PRICING_TIME,
        ip_time_limit_sec=IP_TIME, verbose=true)

    rows = NamedTuple[]
    for r in result.iteration_rows
        push!(rows, (; n_stations=n, L=L, n_pairs=N_PAIRS, n_scenarios=N_SCEN,
            max_stops=(MAX_STOPS == typemax(Int) ? -1 : MAX_STOPS),
            round=r.round, iteration=r.iteration, phase=r.phase,
            lp_bound=r.lp_bound, lp_seconds=r.lp_seconds,
            pricing_seconds=r.pricing_seconds, iter_seconds=r.lp_seconds + r.pricing_seconds,
            labels_generated=r.labels_generated, columns_priced=r.columns_priced,
            columns_added=r.columns_added, pool_size=r.pool_size))
    end
    CSV.write(output, DataFrame(rows)); flush(stdout)

    certified = result.cg_stop_reason == :optimality_proven
    summary = (; n_stations=n, L=L, n_pairs=N_PAIRS, n_scenarios=N_SCEN,
        max_stops=(MAX_STOPS == typemax(Int) ? -1 : MAX_STOPS),
        n_passengers=result.n_passengers, status=result.status,
        cg_stop_reason=result.cg_stop_reason, certified=certified,
        lp_bound=result.lp_bound, lp_bound_certified=result.lp_bound_certified,
        mip_objective=(isnothing(result.mip_objective) ? NaN : result.mip_objective),
        certification_exhausted=result.certification_exhausted,
        n_cg_iters=result.n_cg_iters, n_rounds=result.n_rounds, n_columns=result.n_columns,
        n_master_rows=result.n_master_rows,
        total_lp_seconds=result.total_lp_seconds,
        total_pricing_seconds=result.total_pricing_seconds,
        certification_seconds=result.certification_seconds,
        total_labels_generated=result.total_labels_generated,
        total_seconds=result.total_seconds,
        sec_per_iter=(result.n_cg_iters > 0 ?
            (result.total_lp_seconds + result.total_pricing_seconds) / result.n_cg_iters : NaN))
    CSV.write(replace(output, ".csv" => ".summary.csv"), DataFrame([summary])); flush(stdout)
    @printf("DONE n=%d scen=%d certified=%s stop=%s cg_iters=%d columns=%d lp_bound=%.2f mip=%.2f total=%.1fs\n",
        n, N_SCEN, string(certified), string(result.cg_stop_reason), result.n_cg_iters,
        result.n_columns, result.lp_bound,
        isnothing(result.mip_objective) ? NaN : result.mip_objective, result.total_seconds)
end
main()
