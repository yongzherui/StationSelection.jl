"""
    scripts/diag_direct_ly_lp_bound.jl

One-shot diagnostic: with the γ-chain redesign of NearestOpenAggregateODAssignmentPolicy(:direct_ly)
(real per-request γ variable, x's defining equality folded into the route-coverage row, walking
cost now representable), how does its certified LP relaxation bound on
zhuzhou_n20_p16_s123/max_stops=4 compare to the existing x/z-based :big_m_nearest encoding?

Context: scripts/zhuzhou_p16_cg_ms45_singlescenario.jl already measured, on this exact instance
at walk_cost_weight=0.1 (the same weight used here):
    :big_m_nearest certified LP bound = 7786.880836332251
    DirectSolver true optimum          = 15345.291427345393   (49% LP/IP gap)
The FIRST (per-pair-only, no γ/z at all) attempt at :direct_ly collapsed this LP bound to exactly
0.0 at this same instance -- a provable structural degeneracy, not a search/seeding failure (see
scripts/diag_direct_ly_seeding_check.jl). This script re-solves both :direct_ly (new γ-chain
version) and :big_m_nearest fresh, and compares both against the already-established reference
numbers above rather than re-running DirectSolver (whose full enumeration crashed at this scale
in an earlier run of this diagnostic; not needed since the reference numbers are already trusted
and independently reproduced in scripts/zhuzhou_p16_cg_ms45_singlescenario.jl's own results).

Usage (submit via sbatch, never run interactively on the login node -- see
feedback_run_julia_tests_via_sbatch.md):
    sbatch scripts/sbatch_diag_direct_ly_lp_bound.sh
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_STATIONS = 20
const N_PAIRS = 16
const SEED = 123
const L = 10
const MAX_STOPS = isempty(ARGS) ? 4 : (ARGS[1] == "uncapped" ? typemax(Int) : parse(Int, ARGS[1]))
const ROUTE_REG_WEIGHT = 10.0
const WALK_COST_WEIGHT = 0.1
const REPOSITIONING_TIME = 20.0
const DETOUR_FACTOR = 2.0
const MAX_WAIT_TIME = 900.0
const MAX_WALKING_DISTANCE = 600.0

const REFERENCE_BIG_M_LP_BOUND = 7786.880836332251
const REFERENCE_TRUE_OPTIMUM = 15345.291427345393

function build_data()
    data, meta = generate_zhuzhou_data(
        DATA_DIR, N_STATIONS, N_PAIRS;
        n_scenarios=1, endpoint_overlap=2.0, seed=SEED,
    )
    print_zhuzhou_data_summary(data, meta)
    return data
end

function model_for(style::Symbol)
    return AggregateODRouteModel(
        L;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
        max_walking_distance=MAX_WALKING_DISTANCE,
        route_regularization_weight=ROUTE_REG_WEIGHT,
        walk_cost_weight=WALK_COST_WEIGHT,
        repositioning_time=REPOSITIONING_TIME,
        max_stops=MAX_STOPS,
        max_wait_time=MAX_WAIT_TIME,
        detour_factor=DETOUR_FACTOR,
    )
end

function run_cg(label::String, model, data, env)
    t0 = time()
    result = run_aggregate_od_route_column_generation(
        model, data;
        optimizer_env=env, verbose=false,
        max_cg_iters=10000, max_new_columns=20, n_candidates=20,
        pricing_time_limit_sec=120.0, ip_time_limit_sec=300.0, mip_gap=1e-4, silent=true,
    )
    wall = time() - t0
    @printf(
        "  [%s] status=%s stop_reason=%s lp_bound=%.4f obj=%.4f wall=%.1fs\n",
        label, result.final_result.termination_status, result.cg_stop_reason,
        result.lp_bound, result.final_result.objective_value, wall,
    )
    flush(stdout)
    return result
end

function main()
    println("=== :direct_ly (γ-chain) vs :big_m_nearest LP bound, zhuzhou_n$(N_STATIONS)_p$(N_PAIRS)_s$(SEED), ms$(MAX_STOPS), walk_cost_weight=$(WALK_COST_WEIGHT) ===")
    data = build_data()
    env = Gurobi.Env()
    outdir = joinpath(@__DIR__, "..", "experiments", "diag_direct_ly_lp_bound")
    mkpath(outdir)

    r_direct_ly = run_cg("direct_ly    ", model_for(:direct_ly), data, env)
    r_big_m = run_cg("big_m_nearest", model_for(:big_m_nearest), data, env)

    println("\n=== SUMMARY (vs already-established reference numbers) ===")
    @printf("  reference :big_m_nearest lp_bound = %.4f\n", REFERENCE_BIG_M_LP_BOUND)
    @printf("  reference DirectSolver true optimum = %.4f\n", REFERENCE_TRUE_OPTIMUM)
    rows = [
        (method="direct_ly", lp_bound=r_direct_ly.lp_bound, cg_obj=r_direct_ly.final_result.objective_value,
         cg_stop_reason=string(r_direct_ly.cg_stop_reason)),
        (method="big_m_nearest", lp_bound=r_big_m.lp_bound, cg_obj=r_big_m.final_result.objective_value,
         cg_stop_reason=string(r_big_m.cg_stop_reason)),
    ]
    for r in rows
        gap_str = MAX_STOPS == 4 ?
            @sprintf("%.2f%%", 100 * (REFERENCE_TRUE_OPTIMUM - r.lp_bound) / REFERENCE_TRUE_OPTIMUM) :
            "n/a (ms4 true-opt reference doesn't apply at this max_stops)"
        @printf("  %-16s lp_bound=%-14.4f cg_obj=%-14.4f lp_vs_ms4true_gap=%-10s stop_reason=%s\n",
            r.method, r.lp_bound, r.cg_obj, gap_str, r.cg_stop_reason)
    end
    CSV.write(joinpath(outdir, "summary_gamma_chain.csv"), DataFrame(rows))
    println("\nWrote summary to ", joinpath(outdir, "summary_gamma_chain.csv"))
end

main()
