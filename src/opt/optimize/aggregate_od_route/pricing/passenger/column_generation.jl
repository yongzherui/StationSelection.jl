"""
Column-generation loop for the passenger free-assignment formulation.

# Two-phase pricing: fast early return, then an exhaustive certificate

Each CG iteration prices with a small `n_candidates`, so the label search stops
as soon as it has that many improving columns instead of exploring to
exhaustion. That is a large speed win per iteration but returns *weak* columns
-- measured at up to ~89% short of the true pricing optimum on n=10 zhuzhou
instances (see scripts/diag_passenger_free_assignment_ncandidates_sensitivity.jl),
because acceptance keys on assignment-signature novelty rather than reduced cost.

Weak columns cost iterations, not correctness: any negative-reduced-cost column
is legitimate to add, so the LP bound still converges. What *would* be a
correctness bug is concluding "no improving column exists" from a search that
stopped early. Two properties keep that safe here:

  1. the early stop can only fire *after* at least one column was accepted, so
     "zero columns returned" can never be caused by it -- only by genuine
     exhaustion or by a timeout; and
  2. when the early-return phase stops producing columns, this loop does not
     take that as proof. It runs an explicit **certification pass** with
     `n_candidates` unbounded and a longer time limit, and only reports
     `:optimality_proven` if that pass returns zero columns *and* reports
     `exhausted == true`. A timeout there yields `:no_columns_not_exhausted`,
     and the LP bound is then only a bound on the restricted pool, not on the
     true LP.

The two phases are nested in an **outer round loop**, not run once each. The
early-return phase can stall for a reason other than true convergence -- most
importantly a pricing *timeout* that returns zero columns -- and in that case the
certification pass legitimately finds improving columns. Those get added and the
early-return phase resumes, rather than the run stopping with an uncertified
bound. Rounds continue until certification comes back empty-and-exhausted, or a
cap/budget is hit.

`lp_bound` is therefore trustworthy as an LP relaxation bound **only** when
`cg_stop_reason == :optimality_proven`. This mirrors the aggregate loop's own
distinction in `../column_generation.jl`.
"""

export PassengerFreeAssignmentCGResult
export run_passenger_free_assignment_column_generation

struct PassengerFreeAssignmentCGResult
    final_result::OptResult
    status::Symbol
    cg_stop_reason::Symbol
    lp_bound::Float64
    lp_bound_certified::Bool
    mip_objective::Union{Float64, Nothing}
    mip_termination::Any
    n_cg_iters::Int
    n_rounds::Int
    n_columns::Int
    n_passengers::Int
    n_master_rows::Int
    open_stations::Vector{Int}
    unserved_passengers::Vector{Int}
    certification_seconds::Float64
    certification_exhausted::Bool
    total_pricing_seconds::Float64
    total_lp_seconds::Float64
    total_labels_generated::Int
    iteration_rows::Vector{NamedTuple}
    total_seconds::Float64
    # Mean count of attractive `(p,j,k)` opportunities under the raw RMP duals.
    mean_positive_rho_used::Float64
    mean_raw_positive_rho::Float64
    mean_positive_rho_stations_used::Float64
    mean_raw_positive_rho_stations::Float64
end

"""
    run_passenger_free_assignment_column_generation(model, data; kwargs...)

Full CG scheme: iterate early-return pricing until it stops producing columns,
then run the exhaustive certification pass, then solve the final MIP over the
accumulated pool.

`n_candidates` controls the early-return phase only; the certification pass
always runs unbounded. `certification_time_limit_sec` bounds that pass -- if it
times out, optimality is *not* claimed.
"""
function run_passenger_free_assignment_column_generation(
    model::AggregateODRouteModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    max_cg_iters::Int=1000,
    n_candidates::Int=20,
    max_new_columns::Int=20,
    reduced_cost_tol::Float64=1e-6,
    pricing_time_limit_sec::Float64=60.0,
    certification_time_limit_sec::Float64=600.0,
    ip_time_limit_sec::Float64=600.0,
    mip_gap::Union{Float64, Nothing}=nothing,
    iteration_log_path::Union{Nothing, AbstractString}=nothing,
    column_log_path::Union{Nothing, AbstractString}=nothing,
    # Total budget for the CG phases (excludes the final MIP, which has its own
    # limit). Guards scaling runs against a case that keeps finding columns for
    # thousands of iterations and starves every later case of wall clock.
    total_time_limit_sec::Float64=Inf,
    # Seed the pool with every two-stop route `[j, k]` before the first LP. On by
    # default: it costs one enumerable pass, and starting from an empty pool
    # makes the opening iterations price against `unserved_penalty` duals instead
    # of real service costs. Set false to reproduce the pre-seeding behaviour.
    seed_two_stop_routes::Bool=true,
    # Run the per-scenario label searches concurrently. Exact and deterministic
    # (see `_price_passenger_scenarios`); a no-op with one thread or one scenario.
    parallel_scenarios::Bool=true,
    # Restrict pricing to columns with at most `l` distinct stations -- the most
    # any integer solution can open. Exact for the IP and cannot loosen `lp_bound`,
    # but measured inert on bound quality here and slower to price; see
    # notes/2026-07-30_passenger_pricing_label_search_optimizations.md.
    station_budget_cap::Bool=false,
    # Dominance rule for the label search. `true` is the compensated rule
    # `rc_a + w(A_a \ A_b) <= rc_b`; `false` is the plain `A_a subseteq A_b`.
    # Compensated prices 2.5-3.9x faster but yields ~50% fewer distinct columns
    # per search, so which wins for CG is an end-to-end question.
    compensated_dominance::Bool=true,
    # Price with the elementary (station-simple) label search, which forbids station
    # revisits. Faster (smaller dominance buckets, fewer extensions) but restricts the
    # column universe, so it can weaken `lp_bound` or miss improving columns where the
    # optimum wants a revisiting route -- opt-in and validate against the default
    # pricer. See pricing/passenger/station_simple.jl.
    use_station_simple::Bool=false,
    # Two-phase warm start: price with the fast elementary (station-simple) search
    # until it PROVES no improving elementary column remains (an exhausted, empty
    # certification pass), THEN switch to the exact revisit-tolerant pricer to close
    # the remaining gap and certify. The elementary phase populates the pool cheaply;
    # the exact phase then only has to find the (typically few) revisiting columns the
    # true optimum needs. Overrides `use_station_simple` (which fixes the pricer for
    # the whole run). The final `lp_bound` is still a genuine certificate -- the
    # switch happens precisely because the elementary pool could not be improved, and
    # the exact phase runs to its own exhaustion. See the phase-switch below.
    station_simple_warm_start::Bool=true,
    # Opt-in reward-state relaxation for the early-return harvester. `2` is the
    # measured best quality/speed compromise. Every candidate route is replayed
    # with exact rewards, and exhaustive certification always uses level 0.
    reward_coarsening_levels::Int=0,
    # Iteration-only exact station restriction: at each RMP optimum, closed
    # stations whose lower-bound reduced-cost slack can absorb every positive
    # incident assignment reward are omitted from the current pricing graph.
    use_station_reduced_cost_filter::Bool=false,
    station_reduced_cost_filter_mode::Symbol=:none,
    unserved_penalty::Union{Float64, Nothing}=nothing,
    verify_reduced_costs::Bool=true,
    verbose::Bool=true,
    silent::Bool=!verbose,
)::PassengerFreeAssignmentCGResult
    model.assignment_policy isa FreeAggregateODAssignmentPolicy || throw(ArgumentError(
        "passenger free-assignment column generation requires " *
        "FreeAggregateODAssignmentPolicy, got $(typeof(model.assignment_policy))",
    ))
    max_cg_iters > 0 || throw(ArgumentError("max_cg_iters must be positive"))
    n_candidates > 0 || throw(ArgumentError("n_candidates must be positive"))
    pricing_time_limit_sec > 0 || throw(ArgumentError("pricing_time_limit_sec must be positive"))
    certification_time_limit_sec > 0 ||
        throw(ArgumentError("certification_time_limit_sec must be positive"))
    reward_coarsening_levels >= 0 ||
        throw(ArgumentError("reward_coarsening_levels must be nonnegative"))
    reward_coarsening_levels > 0 && (use_station_simple || station_simple_warm_start) &&
        throw(ArgumentError(
            "reward-coarsened and station-simple harvesting are alternative modes; " *
            "enable only one per run",
        ))
    station_reduced_cost_filter_mode = _normal_station_reduced_cost_filter_mode(
        use_station_reduced_cost_filter && station_reduced_cost_filter_mode == :none ?
            :closed_form : station_reduced_cost_filter_mode,
    )
    station_reduced_cost_filter_active = station_reduced_cost_filter_mode != :none

    t_start = time()
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())

    mapping = create_map(model, data)
    master_data = create_passenger_free_assignment_master_data(
        model, data, mapping; unserved_penalty=unserved_penalty,
    )
    pricing_scenarios = sort!(collect(keys(master_data.passengers_by_scenario)))
    master = build_passenger_free_assignment_master(master_data, optimizer_env; relax_integrality=true)
    m = master.model
    silent && set_silent(m)

    verbose && println(
        "passenger free-assignment CG: $(length(master_data.passengers)) passengers, " *
        "$(length(master.coverage)) coverage rows, $(length(master.pickup_link)) pickup + " *
        "$(length(master.dropoff_link)) dropoff linking rows, l=$(master_data.l)",
    )

    pos_rho_used_sum = 0
    pos_rho_raw_sum = 0
    pos_rho_stations_used_sum = 0
    pos_rho_stations_raw_sum = 0
    pos_rho_samples = 0

    iteration_rows = NamedTuple[]
    lp_bound = NaN
    cg_stop_reason = :max_cg_iters
    n_iters = 0
    n_rounds = 0
    next_column_id = 1
    certification_seconds = 0.0
    cert_exhausted = false
    total_pricing_seconds = 0.0
    total_lp_seconds = 0.0
    total_labels = 0

    # Which pricer the current phase uses. Under `station_simple_warm_start` this
    # starts elementary and flips to revisit-tolerant once the elementary universe is
    # exhausted (see the certification branch); otherwise it is fixed at
    # `use_station_simple` for the whole run. `warm_start_pending` guards the one-time
    # switch.
    pricing_station_simple = station_simple_warm_start ? true : use_station_simple
    warm_start_pending = station_simple_warm_start

    # ── seed: every two-stop route ────────────────────────────────────────────
    # Without this the pool starts empty, so the first several iterations price
    # against big-M duals and are spent finding *any* covering set rather than a
    # cheap one. These columns are the ones `unserved_penalty` was standing in
    # for, they are enumerable, and they make `v[p]` inert unless the instance is
    # genuinely uncoverable. See the seeding function's docstring.
    n_seed_columns = 0
    if seed_two_stop_routes
        seeds = passenger_free_assignment_two_stop_seed_columns(
            master_data; next_column_id=next_column_id,
        )
        for column in seeds
            _theta, action = add_passenger_free_assignment_column!(master, column)
            action == :added && (n_seed_columns += 1)
        end
        next_column_id += length(seeds)
        verbose && println(
            "  seeded $(n_seed_columns) two-stop columns " *
            "(of $(length(seeds)) distinct (scenario, j, k) routes)",
        )
    end

    while true
        n_rounds += 1

        # ── phase 1: early-return pricing until it stops adding columns ────────
        stalled_reason = :max_cg_iters
        while n_iters < max_cg_iters
            if time() - t_start > total_time_limit_sec
                stalled_reason = :total_time_limit
                break
            end
            n_iters += 1
            t_lp = time()
            optimize!(m)
            lp_seconds = time() - t_lp
            total_lp_seconds += lp_seconds

            if primal_status(m) != MOI.FEASIBLE_POINT
                stalled_reason = :no_primal_solution
                break
            end
            lp_bound = objective_value(m)
            alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)

            raw_stats = _positive_rho_stats(master_data, alpha, gamma_o, gamma_d)
            used_stats = raw_stats

            station_filter_stats = station_reduced_cost_filter_active ?
                _station_reduced_cost_filter_stats(
                    master, alpha, gamma_o, gamma_d, pricing_scenarios, reduced_cost_tol,
                    optimizer_env, station_reduced_cost_filter_mode) : nothing
            if !isnothing(station_filter_stats)
                used_stats = (
                    n_positive_rho=station_filter_stats.filtered_positive_rho,
                    j_positive_rho=station_filter_stats.filtered_positive_rho_stations,
                )
            end
            pos_rho_raw_sum += raw_stats.n_positive_rho
            pos_rho_used_sum += used_stats.n_positive_rho
            pos_rho_stations_raw_sum += raw_stats.j_positive_rho
            pos_rho_stations_used_sum += used_stats.j_positive_rho
            pos_rho_samples += 1

            t_price = time()
            new_columns, exhausted, labels = _price_passenger_scenarios(
                master, alpha, gamma_o, gamma_d, next_column_id;
                n_candidates=n_candidates,
                max_new_columns=max_new_columns,
                time_limit=pricing_time_limit_sec,
                reduced_cost_tol=reduced_cost_tol,
                verify_reduced_costs=verify_reduced_costs,
                parallel_scenarios=parallel_scenarios,
                station_budget_cap=station_budget_cap,
                compensated_dominance=compensated_dominance,
                use_station_simple=pricing_station_simple,
                reward_coarsening_levels=reward_coarsening_levels,
                use_station_reduced_cost_filter=station_reduced_cost_filter_active,
                station_reduced_cost_filter_mode=station_reduced_cost_filter_mode,
                station_filter_excluded_stations=isnothing(station_filter_stats) ?
                    nothing : station_filter_stats.excluded_stations,
                station_filter_excluded_opportunities=isnothing(station_filter_stats) ?
                    nothing : station_filter_stats.excluded_opportunities,
                optimizer_env=optimizer_env,
            )
            pricing_seconds = time() - t_price
            total_pricing_seconds += pricing_seconds
            total_labels += labels
            next_column_id += length(new_columns)

            added = 0
            for c in new_columns
                _theta, action = add_passenger_free_assignment_column!(master, c)
                action == :added && (added += 1)
            end

            best_rc = isempty(new_columns) ? nothing :
                minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in new_columns)
            push!(iteration_rows, (
                round=n_rounds, iteration=n_iters, phase="early_return",
                pricer=pricing_station_simple ? "station_simple" :
                    reward_coarsening_levels > 0 ? "reward_coarsened_L$(reward_coarsening_levels)" : "revisit",
                lp_bound=lp_bound, lp_seconds=lp_seconds,
                pricing_seconds=pricing_seconds, labels_generated=labels,
                columns_priced=length(new_columns), columns_added=added,
                best_reduced_cost=best_rc, pricing_exhausted=exhausted,
                raw_positive_rho=raw_stats.n_positive_rho,
                used_positive_rho=used_stats.n_positive_rho,
                raw_positive_rho_stations=raw_stats.j_positive_rho,
                used_positive_rho_stations=used_stats.j_positive_rho,
                station_filter_raw_positive_rho=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.raw_positive_rho,
                station_filter_positive_rho=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.filtered_positive_rho,
                station_filter_positive_rho_reduction=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.positive_rho_reduction,
                station_filter_raw_positive_rho_stations=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.raw_positive_rho_stations,
                station_filter_positive_rho_stations=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.filtered_positive_rho_stations,
                station_filter_positive_rho_station_reduction=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.positive_rho_station_reduction,
                station_filter_closed_stations=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.n_closed_stations,
                station_filter_closed_stations_with_positive_need=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.n_closed_stations_with_positive_need,
                station_filter_excluded_stations=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.n_excluded_stations,
                station_filter_excluded_opportunities=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.n_excluded_opportunities,
                station_filter_joint_lp_objective=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.joint_lp_objective,
                station_filter_min_slack_need_ratio=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.min_slack_need_ratio,
                station_filter_mean_slack_need_ratio=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.mean_slack_need_ratio,
                station_filter_max_slack_need_ratio=isnothing(station_filter_stats) ?
                    missing : station_filter_stats.max_slack_need_ratio,
                pool_size=length(master.theta),
            ))
            verbose && @printf(
                "  [r%d iter %3d] lp=%.4f priced=%d added=%d best_rc=%s labels=%d lp=%.2fs price=%.2fs pool=%d\n",
                n_rounds, n_iters, lp_bound, length(new_columns), added,
                isnothing(best_rc) ? "n/a" : @sprintf("%.4f", best_rc),
                labels, lp_seconds, pricing_seconds, length(master.theta),
            )
            flush(stdout)

            if added == 0
                # Not yet a proof of optimality -- early return and timeouts both
                # land here. The certification pass below is what decides.
                stalled_reason = :early_return_stalled
                break
            end
        end

        if stalled_reason == :no_primal_solution
            cg_stop_reason = :no_primal_solution
            break
        end
        if stalled_reason == :total_time_limit
            # Out of budget mid-generation: skip certification entirely rather than
            # spend the (unbounded-n_candidates) exhaustive pass on a pool we
            # already know is incomplete. Clear any earlier round's exhaustion flag
            # so telemetry cannot read as "certified" when this round never certified.
            cert_exhausted = false
            cg_stop_reason = :total_time_limit
            break
        end

        # ── phase 2: exhaustive certification ─────────────────────────────────
        verbose && println("  [r$(n_rounds)] exhaustive certification pass ...")
        t_cert = time()
        optimize!(m)
        if primal_status(m) != MOI.FEASIBLE_POINT
            certification_seconds += time() - t_cert
            cg_stop_reason = :no_primal_solution
            break
        end
        lp_bound = objective_value(m)
        alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)
        cert_raw_stats = _positive_rho_stats(master_data, alpha, gamma_o, gamma_d)
        cert_station_filter_stats = station_reduced_cost_filter_active ?
            _station_reduced_cost_filter_stats(
                master, alpha, gamma_o, gamma_d, pricing_scenarios, reduced_cost_tol,
                optimizer_env, station_reduced_cost_filter_mode) : nothing
        cert_budget = isfinite(total_time_limit_sec) ?
            min(certification_time_limit_sec, max(1.0, total_time_limit_sec - (time() - t_start))) :
            certification_time_limit_sec
        cert_columns, cert_exhausted, cert_labels = _price_passenger_scenarios(
            master, alpha, gamma_o, gamma_d, next_column_id;
            n_candidates=typemax(Int) ÷ 2,
            max_new_columns=typemax(Int) ÷ 2,
            time_limit=cert_budget,
            reduced_cost_tol=reduced_cost_tol,
            verify_reduced_costs=verify_reduced_costs,
            parallel_scenarios=parallel_scenarios,
            station_budget_cap=station_budget_cap,
            compensated_dominance=compensated_dominance,
            use_station_simple=pricing_station_simple,
            reward_coarsening_levels=0,
            use_station_reduced_cost_filter=station_reduced_cost_filter_active,
            station_reduced_cost_filter_mode=station_reduced_cost_filter_mode,
            station_filter_excluded_stations=isnothing(cert_station_filter_stats) ?
                nothing : cert_station_filter_stats.excluded_stations,
            station_filter_excluded_opportunities=isnothing(cert_station_filter_stats) ?
                nothing : cert_station_filter_stats.excluded_opportunities,
            optimizer_env=optimizer_env,
        )
        round_cert_seconds = time() - t_cert
        certification_seconds += round_cert_seconds
        total_labels += cert_labels
        next_column_id += length(cert_columns)

        cert_added = 0
        for c in cert_columns
            _theta, action = add_passenger_free_assignment_column!(master, c)
            action == :added && (cert_added += 1)
        end
        push!(iteration_rows, (
            round=n_rounds, iteration=n_iters, phase="certification",
            pricer=pricing_station_simple ? "station_simple" : "revisit",
            lp_bound=lp_bound, lp_seconds=0.0,
            pricing_seconds=round_cert_seconds, labels_generated=cert_labels,
            columns_priced=length(cert_columns), columns_added=cert_added,
            best_reduced_cost=isempty(cert_columns) ? nothing :
                minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in cert_columns),
            pricing_exhausted=cert_exhausted,
            raw_positive_rho=cert_raw_stats.n_positive_rho,
            used_positive_rho=isnothing(cert_station_filter_stats) ?
                cert_raw_stats.n_positive_rho : cert_station_filter_stats.filtered_positive_rho,
            raw_positive_rho_stations=cert_raw_stats.j_positive_rho,
            used_positive_rho_stations=isnothing(cert_station_filter_stats) ?
                cert_raw_stats.j_positive_rho : cert_station_filter_stats.filtered_positive_rho_stations,
            station_filter_raw_positive_rho=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.raw_positive_rho,
            station_filter_positive_rho=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.filtered_positive_rho,
            station_filter_positive_rho_reduction=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.positive_rho_reduction,
            station_filter_raw_positive_rho_stations=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.raw_positive_rho_stations,
            station_filter_positive_rho_stations=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.filtered_positive_rho_stations,
            station_filter_positive_rho_station_reduction=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.positive_rho_station_reduction,
            station_filter_closed_stations=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.n_closed_stations,
            station_filter_closed_stations_with_positive_need=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.n_closed_stations_with_positive_need,
            station_filter_excluded_stations=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.n_excluded_stations,
            station_filter_excluded_opportunities=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.n_excluded_opportunities,
            station_filter_joint_lp_objective=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.joint_lp_objective,
            station_filter_min_slack_need_ratio=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.min_slack_need_ratio,
            station_filter_mean_slack_need_ratio=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.mean_slack_need_ratio,
            station_filter_max_slack_need_ratio=isnothing(cert_station_filter_stats) ?
                missing : cert_station_filter_stats.max_slack_need_ratio,
            pool_size=length(master.theta),
        ))
        verbose && println(
            "  [r$(n_rounds)] certification: $(length(cert_columns)) columns " *
            "($(cert_added) new), exhausted=$(cert_exhausted), " *
            "$(round(round_cert_seconds; digits=2))s",
        )
        flush(stdout)

        if warm_start_pending && (
                (isempty(cert_columns) && cert_exhausted) ||
                (!isempty(cert_columns) && cert_added == 0))
            # Elementary pricing can no longer improve the pool -- it either proved the
            # elementary universe exhausted (empty + exhausted) or now only re-finds
            # already-pooled columns. Switch to the exact revisit-tolerant pricer for
            # the rest of the run instead of stopping over the restricted elementary
            # pool. The LP is unchanged (nothing new was added), so the next round
            # re-prices the same duals with the exact pricer, which is what can still
            # find the revisiting columns the true optimum needs.
            warm_start_pending = false
            pricing_station_simple = false
            verbose && println(
                "  [r$(n_rounds)] station-simple warm start exhausted the elementary " *
                "universe; switching to the revisit-tolerant pricer",
            )
            if n_iters >= max_cg_iters
                cg_stop_reason = :max_cg_iters
                break
            end
            if time() - t_start > total_time_limit_sec
                cg_stop_reason = :total_time_limit
                break
            end
            continue
        end

        if isempty(cert_columns)
            # Exhaustive pricing found nothing. Only `exhausted` distinguishes a
            # genuine optimality certificate from a timeout.
            cg_stop_reason = cert_exhausted ? :optimality_proven : :no_columns_not_exhausted
            break
        end
        if cert_added == 0
            # Everything exhaustive pricing found was already pooled -- no progress
            # is possible, so treat it the same as an uncertified stall rather than
            # looping forever.
            cg_stop_reason = :no_progress
            break
        end
        if n_iters >= max_cg_iters
            cg_stop_reason = :max_cg_iters
            break
        end
        if time() - t_start > total_time_limit_sec
            cg_stop_reason = :total_time_limit
            break
        end
        # Certification found genuinely new columns: resume the early-return phase.
    end

    lp_bound_certified = cg_stop_reason == :optimality_proven

    # ── final MIP over the accumulated pool ───────────────────────────────────
    for y_var in master.y
        set_binary(y_var)
    end
    for theta_var in values(master.theta)
        set_binary(theta_var)
    end
    # A fractional no-vehicle-route leg ("half the passenger walks") is not a
    # meaningful solution, so these bind integrally in the final MIP too.
    for x_var in values(master.x_same)
        set_binary(x_var)
    end
    set_optimizer_attribute(m, "TimeLimit", ip_time_limit_sec)
    isnothing(mip_gap) || set_optimizer_attribute(m, "MIPGap", mip_gap)
    optimize!(m)
    mip_term = termination_status(m)
    mip_obj = mip_term == MOI.OPTIMAL ? objective_value(m) : nothing

    open_stations = Int[]
    unserved = Int[]
    if mip_term in (MOI.OPTIMAL, MOI.TIME_LIMIT) && primal_status(m) == MOI.FEASIBLE_POINT
        open_stations = sort([
            data.array_idx_to_station_id[j] for j in eachindex(master.y) if value(master.y[j]) > 0.5
        ])
        unserved = sort([p for (p, var) in master.v if value(var) > 1e-6])
    end

    status = mip_term == MOI.OPTIMAL ?
        (lp_bound_certified ? :optimal : :feasible) :
        mip_term == MOI.TIME_LIMIT ? :timeout :
        mip_term == MOI.INFEASIBLE ? :infeasible : :error

    column_rows = [(
        column_id=id,
        scenario=Int(get(column.metadata, "scenario", 0)),
        tau=column.tau,
        reduced_cost=get(column.metadata, "reduced_cost", nothing),
        route=join(column.route, "-"),
        assignments=join(("$p:$j:$k" for (p, j, k) in column.assignments), ";"),
    ) for (id, column) in sort!(collect(master.columns); by=first)]
    !isnothing(iteration_log_path) && _write_aggregate_od_route_cg_log_csv(
        String(iteration_log_path), iteration_rows,
    )
    !isnothing(column_log_path) && _write_aggregate_od_route_cg_log_csv(
        String(column_log_path), column_rows,
    )

    counts = ModelCounts(
        Dict(
            "y" => length(master.y),
            "v" => length(master.v),
            "x_same" => length(master.x_same),
            "theta" => length(master.theta),
        ),
        Dict(
            "coverage" => length(master.coverage),
            "pickup_link" => length(master.pickup_link),
            "dropoff_link" => length(master.dropoff_link),
            "station_budget" => 1,
        ),
        Dict(
            "passengers" => length(master_data.passengers),
            "columns" => length(master.columns),
            "cg_iterations" => n_iters,
            "cg_rounds" => n_rounds,
        ),
    )

    solution = nothing
    if mip_term == MOI.OPTIMAL
        assignment_values = (
            route_columns=Dict(id => value(var) for (id, var) in master.theta),
            same_station=Dict(key => value(var) for (key, var) in master.x_same),
            unserved=Dict(p => value(var) for (p, var) in master.v),
        )
        solution = (assignment_values, value.(master.y))
    end
    total_seconds = time() - t_start
    final_result = OptResult(
        mip_term,
        mip_obj,
        solution,
        total_seconds,
        m,
        mapping,
        nothing,
        counts,
        nothing,
        Dict{String, Any}(
            "solve_method" => "column_generation",
            "column_generation_formulation" => "passenger_free_assignment",
            "assignment_policy" => "FreeAggregateODAssignmentPolicy",
            "cg_status" => String(status),
            "cg_stop_reason" => String(cg_stop_reason),
            "lp_bound" => lp_bound,
            "lp_bound_certified" => lp_bound_certified,
            "cg_iterations" => n_iters,
            "cg_rounds" => n_rounds,
            "generated_columns" => length(master.theta),
            "seed_two_stop_columns" => n_seed_columns,
            "passengers" => length(master_data.passengers),
            "master_rows" => length(master.coverage) + length(master.pickup_link) + length(master.dropoff_link),
            "open_stations" => open_stations,
            "unserved_passengers" => unserved,
            "certification_seconds" => certification_seconds,
            "certification_exhausted" => cert_exhausted,
            "pricing_seconds" => total_pricing_seconds,
            "lp_seconds" => total_lp_seconds,
            "labels_generated" => total_labels,
            "reward_coarsening_levels" => reward_coarsening_levels,
            "station_simple_warm_start" => station_simple_warm_start,
            "iteration_rows" => copy(iteration_rows),
            "column_rows" => column_rows,
            "mean_positive_rho_used" => (pos_rho_samples == 0 ? 0.0 : pos_rho_used_sum / pos_rho_samples),
            "mean_raw_positive_rho" => (pos_rho_samples == 0 ? 0.0 : pos_rho_raw_sum / pos_rho_samples),
            "mean_positive_rho_stations_used" =>
                (pos_rho_samples == 0 ? 0.0 : pos_rho_stations_used_sum / pos_rho_samples),
            "mean_raw_positive_rho_stations" =>
                (pos_rho_samples == 0 ? 0.0 : pos_rho_stations_raw_sum / pos_rho_samples),
            "iteration_log_path" => isnothing(iteration_log_path) ? nothing : String(iteration_log_path),
            "column_log_path" => isnothing(column_log_path) ? nothing : String(column_log_path),
        ),
    )

    return PassengerFreeAssignmentCGResult(
        final_result, status, cg_stop_reason, lp_bound, lp_bound_certified,
        mip_obj, mip_term, n_iters, n_rounds, length(master.theta),
        length(master_data.passengers),
        length(master.coverage) + length(master.pickup_link) + length(master.dropoff_link),
        open_stations, unserved,
        certification_seconds, cert_exhausted,
        total_pricing_seconds, total_lp_seconds, total_labels,
        iteration_rows, total_seconds,
        pos_rho_samples == 0 ? 0.0 : pos_rho_used_sum / pos_rho_samples,
        pos_rho_samples == 0 ? 0.0 : pos_rho_raw_sum / pos_rho_samples,
        pos_rho_samples == 0 ? 0.0 : pos_rho_stations_used_sum / pos_rho_samples,
        pos_rho_samples == 0 ? 0.0 : pos_rho_stations_raw_sum / pos_rho_samples,
    )
end

"""
Route free-assignment aggregate-OD models through the passenger-level master and
label-setting pricer. Other assignment policies continue to use the aggregate
station-pair column-generation implementation.
"""
function run_opt(
    instance::StationSelectionData,
    formulation::AggregateODRouteModel,
    solver::ColumnGenerationSolver,
)
    if !(formulation.assignment_policy isa FreeAggregateODAssignmentPolicy)
        return _run_aggregate_od_route_column_generation_opt(instance, formulation, solver)
    end

    cfg = solver.config
    result = run_passenger_free_assignment_column_generation(
        formulation,
        instance;
        optimizer_env=cfg.optimizer_env,
        max_cg_iters=solver.max_iterations,
        n_candidates=solver.n_candidates,
        max_new_columns=solver.max_columns_per_iteration,
        reduced_cost_tol=solver.reduced_cost_tol,
        pricing_time_limit_sec=solver.pricing_time_limit_sec,
        ip_time_limit_sec=solver.final_ip_time_limit_sec,
        mip_gap=cfg.mip_gap,
        iteration_log_path=_aggregate_od_route_cg_log_path(
            solver, "passenger_free_assignment_cg_iterations.csv",
        ),
        column_log_path=_aggregate_od_route_cg_log_path(
            solver, "passenger_free_assignment_cg_columns.csv",
        ),
        verbose=!cfg.silent,
        silent=cfg.silent,
    )
    return result.final_result
end
