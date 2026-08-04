"""
    scripts/diag_y_support_churn.jl <n_stations>

Trace how the support of the build vars `y_j` moves across CG iterations. After
each RMP LP solve the CG loop now records (see column_generation.jl
`y_support_rows`): the number of y_j >= 0.5, the number strictly fractional, the
L1 movement of the whole y vector vs the previous iteration, and the churn of the
top-l station set (entries + Jaccard). It also writes station-level y/reduced-cost
rows, column provenance/route geometry, and a causal join from columns priced at
solve q to support shifts observed at solve q+1.

The LP bound is very tight here, so `topl_entered`/`topl_jaccard` (churn of the l
stations with the largest y) is the honest "which stations does the LP want"
signal, robust to the occasional fractional split that `support_ge_half` misses.

Env (mirrors passenger_free_assignment_cg_scaling.jl):
    PFA_N_PAIRS (16), PFA_N_SCENARIOS (1), PFA_SEEDS (single, 42),
    PFA_MAX_STOPS (0 => uncapped typemax), PFA_MAX_VISITS (3),
    PFA_CASE_TIME, PFA_CERT_TIME, PFA_PRICING_TIME, PFA_IP_TIME,
    PFA_OUTDIR (optional; write <case>_ysupport.csv there)
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, Statistics, StationSelection
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
const N_CANDIDATES = parse(Int, get(ENV, "PFA_N_CANDIDATES", "20"))
const MAX_NEW_COLUMNS = parse(Int, get(ENV, "PFA_MAX_NEW_COLUMNS", string(N_CANDIDATES)))
const EXHAUSTIVE_EACH_ITER = get(ENV, "PFA_EXHAUSTIVE_EACH_ITER", "0") == "1"
const THETA_RHO_CORE_SIZE = parse(Int, get(ENV, "PFA_THETA_RHO_CORE_SIZE", "0"))
const THETA_RHO_OUTSIDERS = parse(Int, get(ENV, "PFA_THETA_RHO_OUTSIDERS", "1"))
const OUTDIR = get(ENV, "PFA_OUTDIR", "")
const GRB_ENV = Gurobi.Env()
_l_for(n) = max(2, ceil(Int, n / 2))
_write_diag_csv(path, table) = CSV.write(
    path, table; transform=(_column, value) -> isnothing(value) ? missing : value,
)

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
        max_cg_iters=2000, n_candidates=N_CANDIDATES,
        max_new_columns=MAX_NEW_COLUMNS,
        exhaustive_pricing_each_iteration=EXHAUSTIVE_EACH_ITER,
        theta_rho_core_size=THETA_RHO_CORE_SIZE,
        theta_rho_n_outsiders=THETA_RHO_OUTSIDERS,
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

    station_df = DataFrame(r.final_result.metadata["y_station_rows"])
    column_df = DataFrame(r.final_result.metadata["column_rows"])
    priced_route_df = DataFrame(r.final_result.metadata["priced_route_rows"])
    route_rho_df = DataFrame(r.final_result.metadata["route_rho_rows"])
    station_rho_df = DataFrame(r.final_result.metadata["station_rho_score_rows"])
    theta_df = DataFrame(r.final_result.metadata["theta_rows"])
    theta_summary_df = DataFrame(r.final_result.metadata["theta_summary_rows"])
    theta_rho_subset_df = DataFrame(r.final_result.metadata["theta_rho_subset_rows"])
    iteration_df = DataFrame(r.iteration_rows)
    station_rho_df[!, :station_id] = data.array_idx_to_station_id[station_rho_df.station_index]
    priced_route_df[!, :route_station_ids] = [join((data.array_idx_to_station_id[j]
        for j in parse.(Int, filter(x -> !isempty(x), split(String(route), '-')))), "-")
        for route in priced_route_df.route]
    route_rho_df[!, :pickup_station_id] = data.array_idx_to_station_id[route_rho_df.pickup_index]
    route_rho_df[!, :dropoff_station_id] = data.array_idx_to_station_id[route_rho_df.dropoff_index]
    support_by_seq = Dict(Int(row.solve_sequence) =>
        Set(parse.(Int, filter(x -> !isempty(x), split(String(row.topl_indices), ';'))))
        for row in eachrow(df))
    station_by_seq_idx = Dict(
        (Int(row.solve_sequence), Int(row.station_index)) => row for row in eachrow(station_df))

    rc_summary_rows = NamedTuple[]
    for support in eachrow(df)
        q = Int(support.solve_sequence)
        at_q = filter(row -> Int(row.solve_sequence) == q, eachrow(station_df))
        closed = [row for row in at_q if row.y_value <= 1e-7]
        rcs = sort!(Float64[row.y_reduced_cost for row in closed])
        next_entered = haskey(support_by_seq, q + 1) ?
            sort!(collect(setdiff(support_by_seq[q + 1], support_by_seq[q]))) : Int[]
        push!(rc_summary_rows, (
            solve_sequence=q, round=support.round, iteration=support.iteration,
            phase=support.phase, n_closed=length(rcs),
            rc_min=isempty(rcs) ? missing : minimum(rcs),
            rc_q25=isempty(rcs) ? missing : quantile(rcs, 0.25),
            rc_median=isempty(rcs) ? missing : median(rcs),
            rc_mean=isempty(rcs) ? missing : mean(rcs),
            rc_q75=isempty(rcs) ? missing : quantile(rcs, 0.75),
            rc_max=isempty(rcs) ? missing : maximum(rcs),
            next_entered_indices=join(next_entered, ";"),
            next_entered_station_ids=join(data.array_idx_to_station_id[next_entered], ";"),
            next_entered_rc=join((station_by_seq_idx[(q, j)].y_reduced_cost
                                  for j in next_entered), ";"),
            next_entered_closed_rc_rank=join((station_by_seq_idx[(q, j)].closed_rc_rank
                                              for j in next_entered), ";"),
        ))
    end
    rc_summary_df = DataFrame(rc_summary_rows)

    safe_cor(x, y) = length(x) < 2 || iszero(std(x)) || iszero(std(y)) ? missing : cor(x, y)
    route_correlation_rows = NamedTuple[]
    subset_guide_rows = NamedTuple[]
    for q in sort!(unique(Int.(priced_route_df.source_solve_sequence)))
        routes = [row for row in eachrow(priced_route_df) if row.source_solve_sequence == q]
        isempty(routes) && continue
        best = routes[argmin([row.reported_reduced_cost for row in routes])]
        rc = Float64[row.reported_reduced_cost for row in routes]
        rho_sum = Float64[row.rho_sum for row in routes]
        time_cost = Float64[row.route_time_cost for row in routes]
        n_assign = Float64[row.n_assignments for row in routes]
        push!(route_correlation_rows, (
            solve_sequence=q, n_routes=length(routes), best_column_id=best.column_id,
            best_route=best.route, best_reduced_cost=best.reported_reduced_cost,
            best_route_time_cost=best.route_time_cost, best_rho_sum=best.rho_sum,
            best_rho_mean=best.rho_mean, best_rho_max=best.rho_max,
            best_n_assignments=best.n_assignments,
            correlation_rc_rho_sum=safe_cor(rc, rho_sum),
            correlation_rc_time_cost=safe_cor(rc, time_cost),
            correlation_rc_n_assignments=safe_cor(rc, n_assign),
        ))

        best_stations = Set(parse.(Int,
            filter(x -> !isempty(x), split(String(best.route), '-'))))
        scores = [row for row in eachrow(station_rho_df) if row.solve_sequence == q]
        lq = Int(df[df.solve_sequence .== q, :l][1])
        for k in unique(sort!([max(2, lq ÷ 2), max(2, lq - 2), lq]))
            for (guide, field) in (("positive_rho_sum", :positive_rho_sum),
                                   ("positive_rho_max", :positive_rho_max),
                                   ("positive_rho_count", :positive_rho_opportunities))
                ranked = sort(scores; by=row -> (-Float64(getproperty(row, field)), row.station_index))
                selected = Set(Int(row.station_index) for row in ranked[1:min(k, length(ranked))])
                retained = length(intersect(best_stations, selected))
                push!(subset_guide_rows, (
                    solve_sequence=q, guide=guide, k=k, l=lq,
                    selected_indices=join(sort!(collect(selected)), ";"),
                    best_route=best.route,
                    best_route_distinct_stations=length(best_stations),
                    best_route_stations_retained=retained,
                    best_route_station_recall=retained / length(best_stations),
                    contains_entire_best_route=issubset(best_stations, selected),
                ))
            end
        end
    end
    route_correlation_df = DataFrame(route_correlation_rows)
    subset_guide_df = DataFrame(subset_guide_rows)

    # A shift observed at solve q was caused by the columns priced after solve q-1.
    # Emit one row per causal column, with the entering stations' previous-solve RCs.
    event_rows = NamedTuple[]
    for shift in eachrow(df)
        q = Int(shift.solve_sequence)
        q > 1 || continue
        entered_count = ismissing(shift.topl_entered) ? 0 : Int(shift.topl_entered)
        entered_count > 0 || continue
        before = get(support_by_seq, q - 1, Set{Int}())
        after = get(support_by_seq, q, Set{Int}())
        entered_idx = sort!(collect(setdiff(after, before)))
        exited_idx = sort!(collect(setdiff(before, after)))
        entering_rc = [station_by_seq_idx[(q - 1, j)].y_reduced_cost for j in entered_idx]
        entering_rank = [station_by_seq_idx[(q - 1, j)].closed_rc_rank for j in entered_idx]
        causal = filter(row -> Int(row.source_solve_sequence) == q - 1,
            eachrow(priced_route_df))
        for col in causal
            route_idx = parse.(Int, filter(x -> !isempty(x), split(String(col.route), '-')))
            outside = sort!(collect(setdiff(Set(route_idx), before)))
            push!(event_rows, (
                shift_solve_sequence=q, shift_round=shift.round,
                shift_iteration=shift.iteration, shift_phase=shift.phase,
                l1_move=shift.l1_move, topl_entered=entered_count,
                entered_indices=join(entered_idx, ";"),
                entered_station_ids=join(data.array_idx_to_station_id[entered_idx], ";"),
                exited_indices=join(exited_idx, ";"),
                exited_station_ids=join(data.array_idx_to_station_id[exited_idx], ";"),
                entered_previous_rc=join(entering_rc, ";"),
                entered_previous_closed_rc_rank=join(entering_rank, ";"),
                column_id=col.column_id, scenario=col.scenario,
                add_action=col.add_action,
                route=col.route, route_station_ids=col.route_station_ids,
                n_stops=col.n_stops, n_distinct_stations=col.n_distinct_stations,
                n_revisits=col.n_revisits, tau=col.tau,
                route_reduced_cost=col.reported_reduced_cost,
                route_time_cost=col.route_time_cost,
                rho_sum=col.rho_sum, rho_min=col.rho_min,
                rho_mean=col.rho_mean, rho_max=col.rho_max,
                rho_top1_share=col.rho_top1_share,
                n_assignments=col.n_assignments,
                outside_previous_support_indices=join(outside, ";"),
                n_distinct_outside_previous_support=length(outside),
                touches_entering_station=!isempty(intersect(Set(route_idx), Set(entered_idx))),
            ))
        end
    end
    event_df = DataFrame(event_rows)

    println()
    println("support-shift route census")
    if isempty(event_rows)
        println("  no support-changing events with causal columns")
    else
        touched = count(identity, event_df.touches_entering_station)
        @printf("  %d causal columns across %d shifts; %d (%.1f%%) touch an entering station\n",
            nrow(event_df), length(unique(event_df.shift_solve_sequence)), touched,
            100 * touched / nrow(event_df))
        @printf("  stops: min=%d median=%.1f mean=%.2f max=%d; tau: min=%.1f median=%.1f mean=%.1f max=%.1f\n",
            minimum(event_df.n_stops), median(event_df.n_stops), mean(event_df.n_stops),
            maximum(event_df.n_stops), minimum(event_df.tau), median(event_df.tau),
            mean(event_df.tau), maximum(event_df.tau))
    end
    println()
    println("theta census")
    @printf("  final: positive=%d fractional=%d >=.5=%d >=.99=%d sum=%.3f max=%.3f\n",
        theta_summary_df.n_positive[end], theta_summary_df.n_fractional[end],
        theta_summary_df.n_ge_half[end], theta_summary_df.n_near_one[end],
        theta_summary_df.theta_sum[end], theta_summary_df.theta_max[end])
    theta_moves = collect(skipmissing(theta_summary_df.theta_l1_move))
    @printf("  L1 movement: total=%.3f mean/solve=%.3f max=%.3f\n",
        sum(theta_moves), mean(theta_moves), maximum(theta_moves))

    if EXHAUSTIVE_EACH_ITER
        early = filter(row -> row.phase == "early_return", r.iteration_rows)
        all(row.pricing_exhausted for row in early) || @warn(
            "at least one requested exhaustive iteration hit a search/time limit",
        )
    end

    if !isempty(OUTDIR)
        mkpath(OUTDIR)
        case = "n$(n)_sc$(N_SCEN)_s$(SEED)_$(tag)"
        # CSV.jl chokes on `missing` mixed columns only if types clash; coerce here
        for col in names(df)
            if any(ismissing, df[!, col])
                df[!, col] = [ismissing(x) ? missing : x for x in df[!, col]]
            end
        end
        _write_diag_csv(joinpath(OUTDIR, "$(case)_ysupport.csv"), df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_iterations.csv"), iteration_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_ystations.csv"), station_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_yrc_summary.csv"), rc_summary_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_columns.csv"), column_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_priced_routes.csv"), priced_route_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_route_rhos.csv"), route_rho_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_station_rho_scores.csv"), station_rho_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_route_rho_correlations.csv"), route_correlation_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_subset_guides.csv"), subset_guide_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_theta.csv"), theta_df)
        _write_diag_csv(joinpath(OUTDIR, "$(case)_theta_summary.csv"), theta_summary_df)
        if ncol(theta_rho_subset_df) > 0
            _write_diag_csv(joinpath(OUTDIR, "$(case)_theta_rho_subsets.csv"), theta_rho_subset_df)
        end
        _write_diag_csv(joinpath(OUTDIR, "$(case)_shift_routes.csv"), event_df)
        early_priced = count(==("early_return"), priced_route_df.source_phase)
        certification_priced = count(==("certification"), priced_route_df.source_phase)
        summary_df = DataFrame([(
            case=case, n_stations=n, n_pairs=N_PAIRS, n_scenarios=N_SCEN,
            seed=SEED, l=_l_for(n), max_stops=MAX_STOPS,
            n_candidates=N_CANDIDATES, max_new_columns=MAX_NEW_COLUMNS,
            exhaustive_each_iteration=EXHAUSTIVE_EACH_ITER,
            theta_rho_core_size=THETA_RHO_CORE_SIZE,
            theta_rho_outsiders=THETA_RHO_OUTSIDERS,
            status=String(r.status), cg_stop_reason=String(r.cg_stop_reason),
            lp_bound=r.lp_bound, lp_bound_certified=r.lp_bound_certified,
            mip_objective=isnothing(r.mip_objective) ? missing : r.mip_objective,
            mip_termination=string(r.mip_termination),
            n_cg_iters=r.n_cg_iters, n_rounds=r.n_rounds,
            n_columns=r.n_columns, n_passengers=r.n_passengers,
            n_master_rows=r.n_master_rows,
            total_seconds=r.total_seconds,
            total_pricing_seconds=r.total_pricing_seconds,
            total_lp_seconds=r.total_lp_seconds,
            certification_seconds=r.certification_seconds,
            certification_exhausted=r.certification_exhausted,
            total_labels_generated=r.total_labels_generated,
            early_routes_priced=early_priced,
            certification_routes_priced=certification_priced,
            final_theta_positive=theta_summary_df.n_positive[end],
            final_theta_near_one=theta_summary_df.n_near_one[end],
            final_theta_fractional=theta_summary_df.n_fractional[end],
            final_theta_sum=theta_summary_df.theta_sum[end],
        )])
        _write_diag_csv(joinpath(OUTDIR, "$(case)_run_summary.csv"), summary_df)
        println("\nwrote enriched y-support/route/rho diagnostics under $OUTDIR for $case")
    end
end

main()
