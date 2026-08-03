"""
    scripts/diag_y_support_churn.jl <n_stations>

Trace how the support of the build vars `y_j` moves across CG iterations. After
each RMP LP solve the CG loop now records (see column_generation.jl
`y_support_rows`): the number of y_j >= 0.5, the number strictly fractional, the
L1 movement of the whole y vector vs the previous iteration, and the churn of the
top-l station set (entries + Jaccard). We print that per-iteration table plus a
summary, and write it to CSV.

The LP bound is very tight here, so `topl_entered`/`topl_jaccard` (churn of the l
stations with the largest y) is the honest "which stations does the LP want"
signal, robust to the occasional fractional split that `support_ge_half` misses.

Env (mirrors passenger_free_assignment_cg_scaling.jl):
    PFA_N_PAIRS (16), PFA_N_SCENARIOS (1), PFA_SEEDS (single, 42),
    PFA_MAX_STOPS (0 => uncapped typemax), PFA_MAX_VISITS (3),
    PFA_CASE_TIME, PFA_CERT_TIME, PFA_PRICING_TIME, PFA_IP_TIME,
    PFA_OUTDIR (optional; write <case>_ysupport.csv there)
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection
include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFA_N_PAIRS", "16"))
const N_SCEN = parse(Int, get(ENV, "PFA_N_SCENARIOS", "1"))
const SEED = parse(Int, split(get(ENV, "PFA_SEEDS", "42"), ',')[1])
const _RMS = parse(Int, get(ENV, "PFA_MAX_STOPS", "0"))
const MAX_STOPS = _RMS <= 0 ? typemax(Int) : _RMS
const _RMV = parse(Int, get(ENV, "PFA_MAX_VISITS", "3"))
const MAX_VISITS = _RMV <= 0 ? typemax(Int) : _RMV
const CASE_TIME = parse(Float64, get(ENV, "PFA_CASE_TIME", "2400"))
const CERT_TIME = parse(Float64, get(ENV, "PFA_CERT_TIME", "1800"))
const PRICING_TIME = parse(Float64, get(ENV, "PFA_PRICING_TIME", "120"))
const IP_TIME = parse(Float64, get(ENV, "PFA_IP_TIME", "300"))
const OUTDIR = get(ENV, "PFA_OUTDIR", "")
const GRB_ENV = Gurobi.Env()
_l_for(n) = max(2, ceil(Int, n / 2))

function main()
    n = parse(Int, ARGS[1])
    data, _ = generate_zhuzhou_data(DATA_DIR, n, N_PAIRS; n_scenarios=N_SCEN, seed=SEED)
    model = AggregateODRouteModel(
        _l_for(n);
        route_regularization_weight=10.0, walk_cost_weight=0.1, repositioning_time=20.0,
        max_walking_distance=600.0, max_wait_time=900.0, detour_factor=2.0,
        max_stops=MAX_STOPS, max_visits_per_node=MAX_VISITS,
    )
    tag = MAX_STOPS == typemax(Int) ? "uncapped" : "ms$(MAX_STOPS)"
    @printf("=== n=%d scen=%d seed=%d l=%d %s ===\n", n, N_SCEN, SEED, _l_for(n), tag)

    r = run_passenger_free_assignment_column_generation(
        model, data; optimizer_env=GRB_ENV,
        max_cg_iters=2000, n_candidates=20, max_new_columns=20,
        pricing_time_limit_sec=PRICING_TIME, certification_time_limit_sec=CERT_TIME,
        ip_time_limit_sec=IP_TIME, total_time_limit_sec=CASE_TIME,
        verbose=false,
    )
    @printf("stop=%s certified=%s mip=%s\n\n", r.cg_stop_reason, r.lp_bound_certified,
        isnothing(r.mip_objective) ? "n/a" : @sprintf("%.2f", r.mip_objective))

    rows = r.final_result.metadata["y_support_rows"]
    if isempty(rows)
        println("no y_support_rows recorded"); return
    end
    df = DataFrame(rows)

    println("iter phase         lp_bound     l  supp>=.5  frac  l1_move  topl_in  topl_jac")
    for row in eachrow(df)
        @printf("%-4d %-13s %10.1f %3d  %6d  %4d  %s  %s  %s\n",
            row.iteration, row.phase, row.lp_bound, row.l,
            row.support_ge_half, row.n_fractional,
            ismissing(row.l1_move) ? "   -   " : @sprintf("%7.3f", row.l1_move),
            ismissing(row.topl_entered) ? " - " : @sprintf("%3d", row.topl_entered),
            ismissing(row.topl_jaccard) ? "  -   " : @sprintf("%6.3f", row.topl_jaccard))
    end

    moves = collect(skipmissing(df.l1_move))
    entered = collect(skipmissing(df.topl_entered))
    # iteration index (1-based over recorded rows) at which the top-l set last changed
    last_change = 0
    for (i, e) in enumerate(skipmissing(df.topl_entered))
        e > 0 && (last_change = i + 1)   # +1: first row has no delta
    end
    n_rec = nrow(df)
    println()
    @printf("recorded iters: %d   total l1 movement: %.2f   mean/iter: %.3f\n",
        n_rec, isempty(moves) ? 0.0 : sum(moves), isempty(moves) ? 0.0 : sum(moves) / length(moves))
    @printf("top-l set: %d iters with any entry; churn STOPS after recorded-iter %d of %d (%.0f%% of the way in)\n",
        count(>(0), entered), last_change, n_rec, 100 * last_change / n_rec)
    @printf("final support>=.5 = %d (budget l=%d), final fractional = %d\n",
        df.support_ge_half[end], df.l[end], df.n_fractional[end])

    if !isempty(OUTDIR)
        mkpath(OUTDIR)
        case = "n$(n)_sc$(N_SCEN)_s$(SEED)_$(tag)"
        # CSV.jl chokes on `missing` mixed columns only if types clash; coerce here
        for col in names(df)
            if any(ismissing, df[!, col])
                df[!, col] = [ismissing(x) ? missing : x for x in df[!, col]]
            end
        end
        CSV.write(joinpath(OUTDIR, "$(case)_ysupport.csv"), df)
        println("\nwrote $(joinpath(OUTDIR, "$(case)_ysupport.csv"))")
    end
end

main()
