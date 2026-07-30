"""
    scripts/sample09_direct_enumeration_guide_compare.jl

Compares `BendersSolver(direct_enumeration_guide=true)` against a plain
`lifted_walking_objective=true` BendersYZ solve, on the real sample_09 fixture (same
data/loading as scripts/sample09_mw_vs_direct.jl), using `cut_derivation=:restricted_mw_fixed_pi`.

Runs exactly ONE (n_stations, guided) config per invocation, selected via env vars
`DEG_N_STATIONS` (required, one of the keys in `L_FOR`) and `DEG_GUIDED` (required,
"true"/"false") -- this lets each config be submitted as its own separate sbatch job
(see scripts/sbatch_sample09_direct_enumeration_guide_compare.sh), rather than one long
job running every config sequentially: BendersYZ's plain and guided solves have very
different runtimes at n=15, and a single combined job risks the slow configs starving
the fast ones of their share of the wall-clock budget.

Only `BendersYZ` is tested here -- `BendersY`'s much slower convergence on this real
fixture at n=15 (compare scripts/sample09_mw_vs_direct.jl's own findings) makes it a
poor fit for a same-budget plain-vs-guided comparison; BendersYZ is where the
direct-enumeration guide's effect is easiest to see cleanly.

Reuses `load_sample09`/`L_FOR`/`build_model` from sample09_mw_vs_direct.jl (which itself
includes run_method_compare_task.jl as a library) by `include`-ing it -- safe because
that file's own `main()` is guarded by `abspath(PROGRAM_FILE) == @__FILE__`.

Usage:
    DEG_N_STATIONS=15 DEG_GUIDED=true julia --project=. scripts/sample09_direct_enumeration_guide_compare.jl [outdir]

Default output dir: experiments/sample09_direct_enumeration_guide_compare/
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

const GUIDE_MAX_STOPS_MODE = Symbol(get(ENV, "DEG_MAX_STOPS_MODE", "ms3"))
const GUIDE_DIRECT_MAX_ROUTES = parse(Int, get(ENV, "SAMPLE09_GUIDE_MAX_ROUTES", "50000"))
const GUIDE_DIRECT_TIME_LIMIT = parse(Float64, get(ENV, "SAMPLE09_GUIDE_TIME_LIMIT", "120.0"))
const GUIDE_CUT_DERIVATION = Symbol(get(ENV, "DEG_CUT_DERIVATION", "restricted_mw_fixed_pi"))
# Decouples the enumerated phase-1 pool's own max_stops cap from the model's own
# max_stops (which still governs CG pricing/the Benders subproblem in both phases) --
# see BendersSolver's `direct_enumeration_max_stops` docstring. Unset (nothing) means
# enumeration shares the model's own max_stops, today's default behavior.
const GUIDE_ENUM_MAX_STOPS = haskey(ENV, "DEG_ENUM_MAX_STOPS") ? parse(Int, ENV["DEG_ENUM_MAX_STOPS"]) : nothing
# Relaxes phase 1's theta_direct route-selection variables to continuous -- see
# BendersSolver's `direct_enumeration_relax_integrality` docstring.
const GUIDE_ENUM_RELAX = parse(Bool, get(ENV, "DEG_ENUM_RELAX", "false"))

function build_guide_solver(guided::Bool, log_dir::String)
    inner_cg = ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
        max_iterations=CFG.inner_cg_max_iters, max_columns_per_iteration=20, n_candidates=20,
        pricing_time_limit_sec=CFG.inner_pricing_time, final_ip_time_limit_sec=CFG.inner_ip_time_limit,
    )
    return BendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
        decomposition=BendersYZ(),
        inner_solver=inner_cg,
        max_iterations=CFG.benders_max_iters,
        log_dir=log_dir,
        log_subiteration_details=false,
        reprice_subproblem=true,
        max_reprice_rounds=CFG.max_reprice_rounds,
        cut_derivation=GUIDE_CUT_DERIVATION,
        lifted_walking_objective=true,
        direct_enumeration_guide=guided,
        direct_enumeration_max_routes=GUIDE_DIRECT_MAX_ROUTES,
        direct_enumeration_time_limit_sec=GUIDE_DIRECT_TIME_LIMIT,
        direct_enumeration_max_stops=GUIDE_ENUM_MAX_STOPS,
        direct_enumeration_relax_integrality=guided && GUIDE_ENUM_RELAX,
    )
end

function run_one_guide(n_stations::Int, guided::Bool, results_dir::String, iters_dir::String)
    l = L_FOR[n_stations]
    max_stops = resolve_max_stops(GUIDE_MAX_STOPS_MODE, n_stations)
    label = "BendersYZ_$(guided ? "guided" : "plain")"
    inst_name = "sample09_n$(n_stations)"
    summary_path = joinpath(results_dir, "$(inst_name)__$(label).csv")

    @printf("  [%s / %s] l=%d max_stops=%d cut_derivation=%s ... \n", inst_name, label, l, max_stops, GUIDE_CUT_DERIVATION)
    flush(stdout)

    data = load_sample09(n_stations)
    model = build_model(l, max_stops, MAX_WALK, CFG)
    log_dir = joinpath(iters_dir, "$(inst_name)__$(label)")
    mkpath(log_dir)
    solver = build_guide_solver(guided, log_dir)

    t0 = time()
    status = "ok"
    result = nothing
    try
        result = StationSelection.run_opt(data, model, solver)
    catch err
        status = "error: $(sprint(showerror, err))"
    end
    wall_time = time() - t0

    summary = (
        instance             = inst_name,
        n_stations           = n_stations,
        l                    = l,
        n_orders             = nrow(data.scenarios[1].requests),
        decomposition        = "BendersYZ",
        cut_derivation       = string(GUIDE_CUT_DERIVATION),
        guided               = guided,
        enum_max_stops       = isnothing(GUIDE_ENUM_MAX_STOPS) ? "" : string(GUIDE_ENUM_MAX_STOPS),
        enum_relax           = guided && GUIDE_ENUM_RELAX,
        status               = status,
        termination_status   = isnothing(result) ? "" : string(result.termination_status),
        objective_value      = isnothing(result) || isnothing(result.objective_value) ? "" : string(result.objective_value),
        wall_time_sec        = wall_time,
        phase1_objective     = (!isnothing(result) && haskey(result.metadata, "phase1_objective")) ?
            string(result.metadata["phase1_objective"]) : "",
        phase1_iterations    = (!isnothing(result) && haskey(result.metadata, "phase1_iterations")) ?
            string(result.metadata["phase1_iterations"]) : "",
        phase1_cuts_harvested = (!isnothing(result) && haskey(result.metadata, "phase1_cuts_harvested")) ?
            string(result.metadata["phase1_cuts_harvested"]) : "",
        phase2_iterations    = (!isnothing(result) && haskey(result.metadata, "phase2_iterations")) ?
            string(result.metadata["phase2_iterations"]) : "",
        enumerated_routes    = (!isnothing(result) && haskey(result.metadata, "enumerated_routes")) ?
            string(result.metadata["enumerated_routes"]) : "",
        benders_iterations   = (!isnothing(result) && haskey(result.metadata, "benders_iterations")) ?
            string(result.metadata["benders_iterations"]) : "",
    )
    CSV.write(summary_path, DataFrame([summary]))

    @printf("status=%s obj=%s wall=%.1fs\n", status, summary.objective_value, wall_time)
    flush(stdout)
    return summary
end

function main()
    outdir = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "sample09_direct_enumeration_guide_compare")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    haskey(ENV, "DEG_N_STATIONS") || error("DEG_N_STATIONS env var is required (one of $(sort(collect(keys(L_FOR)))))")
    haskey(ENV, "DEG_GUIDED") || error("DEG_GUIDED env var is required (\"true\" or \"false\")")
    n_stations = parse(Int, ENV["DEG_N_STATIONS"])
    guided = parse(Bool, ENV["DEG_GUIDED"])
    haskey(L_FOR, n_stations) || error("DEG_N_STATIONS=$n_stations has no L_FOR entry (known: $(sort(collect(keys(L_FOR)))))")

    println("=== sample_09: BendersYZ $(guided ? "guided" : "plain"), n_stations=$n_stations, cut_derivation=$GUIDE_CUT_DERIVATION ===")
    println()

    run_one_guide(n_stations, guided, results_dir, iters_dir)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
