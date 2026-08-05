# Matched max-stops-5 free-assignment CG/Direct benchmark task.
using CSV, DataFrames, Gurobi, StationSelection
include(joinpath(@__DIR__, "run_method_compare_task.jl"))

length(ARGS) == 7 || error("usage: <outdir> <data_dir> <n> <p> <seed> <q> <cg|direct>")
const OUTDIR = abspath(ARGS[1])
const DATA_DIR = abspath(ARGS[2])
const N = parse(Int, ARGS[3])
const P = parse(Int, ARGS[4])
const SEED = parse(Int, ARGS[5])
const Q = parse(Int, ARGS[6])
const METHOD = Symbol(ARGS[7])
METHOD in (:cg, :direct) || error("method must be cg or direct")
const L = ceil(Int, N / 2)

function selected_support(result)
    isnothing(result.solution) && return ""
    y = result.solution[2]
    return join(findall(v -> v > 0.5, y), ";")
end

function main()
    mkpath(OUTDIR)
    cfg = (route_reg_weight=10.0, walk_cost_weight=0.1,
        repositioning_time=20.0, max_wait_time=900.0, detour_factor=2.0)
    data, max_walk = build_instance("zhuzhou", N, P, SEED, DATA_DIR; n_scenarios=Q)
    model = AggregateODRouteModel(L;
        assignment_policy=FreeAggregateODAssignmentPolicy(),
        route_regularization_weight=cfg.route_reg_weight,
        walk_cost_weight=cfg.walk_cost_weight,
        repositioning_time=cfg.repositioning_time,
        max_walking_distance=max_walk,
        max_wait_time=cfg.max_wait_time,
        detour_factor=cfg.detour_factor,
        max_stops=5,
        max_visits_per_node=typemax(Int),
        max_new_columns=100,
        n_candidates=100)

    status = "error"; failure = ""; result = nothing; cg = nothing
    t0 = time()
    try
        if METHOD == :cg
            cg = run_passenger_free_assignment_column_generation(model, data;
                optimizer_env=Gurobi.Env(), max_cg_iters=10_000,
                n_candidates=100, max_new_columns=100,
                reduced_cost_tol=1e-7, pricing_time_limit_sec=120.0,
                certification_time_limit_sec=1800.0, ip_time_limit_sec=300.0,
                total_time_limit_sec=5100.0, mip_gap=1e-4,
                station_simple_warm_start=false,
                use_adaptive_cluster_certification=true,
                cluster_initial_num_clusters=max(2, ceil(Int, N / 3)),
                cluster_max_num_clusters=N, cluster_time_limit_sec=900.0,
                verify_reduced_costs=true, verbose=true)
            result = cg.final_result
        else
            solver = DirectSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=false, mip_gap=1e-4),
                max_enumerated_routes=typemax(Int), max_enumeration_time_sec=5100.0)
            result = run_opt(data, model, solver)
        end
        status = "ok"
    catch err
        failure = sprint(showerror, err)
        showerror(stderr, err, catch_backtrace()); println(stderr)
    end
    wall = time() - t0
    row = (
        n_stations=N, l=L, n_pairs=P, seed=SEED, n_scenarios=Q,
        assignment_policy="free", method=String(METHOD), max_stops=5,
        max_visits="uncapped", n_candidates=METHOD == :cg ? 100 : missing,
        max_new_columns=METHOD == :cg ? 100 : missing,
        status=status, error=failure,
        termination_status=isnothing(result) ? "" : string(result.termination_status),
        objective_value=isnothing(result) ? missing : result.objective_value,
        lp_bound=isnothing(cg) ? (isnothing(result) ? missing : get(result.metadata, "lp_relaxation_objective", missing)) : cg.lp_bound,
        lp_certified=isnothing(cg) ? missing : cg.lp_bound_certified,
        cg_iterations=isnothing(cg) ? missing : cg.n_cg_iters,
        cg_rounds=isnothing(cg) ? missing : cg.n_rounds,
        columns=isnothing(cg) ? (isnothing(result) ? missing : get(result.metadata, "enumerated_routes", missing)) : cg.n_columns,
        labels=isnothing(cg) ? missing : cg.total_labels_generated,
        certification_seconds=isnothing(cg) ? missing : cg.certification_seconds,
        solver_seconds=isnothing(cg) ? (isnothing(result) ? missing : result.runtime_sec) : cg.total_seconds,
        wall_time_sec=wall,
        selected_y=isnothing(result) ? "" : selected_support(result),
    )
    path = joinpath(OUTDIR, "free_ms5_n$(N)_p$(P)_s$(SEED)_q$(Q)_$(METHOD).csv")
    CSV.write(path, DataFrame([row]))
    println("SUMMARY ", row)
    status == "ok" || exit(1)
end
main()
