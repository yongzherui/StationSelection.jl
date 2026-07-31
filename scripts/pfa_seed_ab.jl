"""
    scripts/pfa_seed_ab.jl

A/B for two-stop seeding: same cells, `seed_two_stop_routes` on vs off.

Correctness gate first -- a certified `lp_bound` and the MIP objective must
agree between the two arms, since seeding only pre-populates a pool the pricer
could have generated anyway. Then the actual question: how many iterations and
how much wall time the empty-pool start was spending on coverage.

Usage:
    julia --project=. scripts/pfa_seed_ab.jl <outdir> "<n>:<p>" ...
"""

using CSV, DataFrames, Gurobi, Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_SCENARIOS = 3
const SEED = 42
const CASE_TIME = parse(Float64, get(ENV, "PFA_CASE_TIME", "1800"))

const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

build_model_for(n) = AggregateODRouteModel(
    _l_for(n);
    route_regularization_weight = 10.0,
    walk_cost_weight            = 0.1,
    repositioning_time          = 20.0,
    max_walking_distance        = 600.0,
    max_wait_time               = 900.0,
    detour_factor               = 2.0,
    max_stops                   = typemax(Int),
    max_visits_per_node         = 3,
)

function run_arm(n::Int, p::Int, seed_two_stop::Bool)
    data, _ = generate_zhuzhou_data(DATA_DIR, n, p; n_scenarios=N_SCENARIOS, seed=SEED)
    t0 = time()
    r = run_passenger_free_assignment_column_generation(
        build_model_for(n), data;
        optimizer_env=GRB_ENV,
        max_cg_iters=2000,
        n_candidates=20,
        max_new_columns=20,
        pricing_time_limit_sec=120.0,
        certification_time_limit_sec=1800.0,
        ip_time_limit_sec=900.0,
        total_time_limit_sec=CASE_TIME,
        parallel_scenarios=true,
        station_budget_cap=false,
        compensated_dominance=true,
        seed_two_stop_routes=seed_two_stop,
        verify_reduced_costs=true,
        verbose=true,
    )
    first_lp = isempty(r.iteration_rows) ? missing : r.iteration_rows[1].lp_bound
    return (
        n_stations=n, n_pairs=p, seeded=seed_two_stop,
        cg_stop_reason=String(r.cg_stop_reason),
        lp_bound=r.lp_bound, lp_bound_certified=r.lp_bound_certified,
        mip_objective=isnothing(r.mip_objective) ? missing : r.mip_objective,
        first_iter_lp=first_lp,
        n_seed_columns=get(r.final_result.metadata, "seed_two_stop_columns", missing),
        n_cg_iters=r.n_cg_iters, n_rounds=r.n_rounds, n_columns=r.n_columns,
        n_unserved=length(r.unserved_passengers),
        total_pricing_seconds=r.total_pricing_seconds,
        certification_seconds=r.certification_seconds,
        open_stations=string(r.open_stations),
        wall_sec=time() - t0,
    )
end

function main()
    outdir = abspath(ARGS[1]); mkpath(outdir)
    cells = [(parse(Int, s[1]), parse(Int, s[2])) for s in split.(ARGS[2:end], ':')]
    rows = NamedTuple[]
    for (n, p) in cells, seeded in (false, true)
        @printf("=== n=%d p=%d seeded=%s ===\n", n, p, seeded); flush(stdout)
        try
            push!(rows, run_arm(n, p, seeded))
            CSV.write(joinpath(outdir, "seed_ab.csv"), DataFrame(rows))
        catch err
            showerror(stderr, err, catch_backtrace()); println(stderr)
        end
    end

    df = DataFrame(rows)
    println("\n", "="^100)
    @printf("%-10s %-7s %-20s %14s %14s %8s %9s %10s\n",
            "cell", "seeded", "stop_reason", "lp_bound", "mip_obj", "iters", "columns", "wall_s")
    for r in eachrow(df)
        @printf("n%-3d p%-4d %-7s %-20s %14.4f %14.4f %8d %9d %10.1f\n",
                r.n_stations, r.n_pairs, string(r.seeded), r.cg_stop_reason,
                r.lp_bound, coalesce(r.mip_objective, NaN), r.n_cg_iters, r.n_columns, r.wall_sec)
    end

    # Correctness gate: certified bounds and MIP objectives must match per cell.
    println("\n--- agreement check (only meaningful where BOTH arms certified) ---")
    ok = true
    for g in groupby(df, [:n_stations, :n_pairs])
        nrow(g) == 2 || continue
        a = g[g.seeded .== false, :][1, :]; b = g[g.seeded .== true, :][1, :]
        tag = "n$(a.n_stations)_p$(a.n_pairs)"
        if a.lp_bound_certified && b.lp_bound_certified
            dlp = abs(a.lp_bound - b.lp_bound) / max(1.0, abs(a.lp_bound))
            dmip = abs(coalesce(a.mip_objective, NaN) - coalesce(b.mip_objective, NaN)) /
                max(1.0, abs(coalesce(a.lp_bound, 1.0)))
            good = dlp < 1e-6 && (isnan(dmip) || dmip < 1e-6)
            ok &= good
            @printf("%-10s %s  d_lp=%.3e d_mip=%.3e   iters %d -> %d, wall %.1fs -> %.1fs\n",
                    tag, good ? "MATCH  " : "MISMATCH", dlp, dmip,
                    a.n_cg_iters, b.n_cg_iters, a.wall_sec, b.wall_sec)
        else
            @printf("%-10s skipped (certified: off=%s on=%s)\n",
                    tag, a.lp_bound_certified, b.lp_bound_certified)
        end
    end
    println(ok ? "\nALL CERTIFIED CELLS AGREE" : "\n*** DISAGREEMENT -- seeding changed the optimum ***")
end

main()
