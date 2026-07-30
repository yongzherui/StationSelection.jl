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
    # Empty unless pricing-aware dual selection was enabled.
    selector_logs::Vector{PassengerDualSelectionRoundLog}
    # Total time in the selector LPs, and how many iterations actually used its
    # duals (vs falling back to the RMP's own). The certificate is always
    # `cg_stop_reason == :optimality_proven`; the selector never certifies.
    total_selector_seconds::Float64
    selector_iterations_used::Int
    # Mean count of attractive (p,j,k) per iteration under the duals ACTUALLY used
    # for pricing, and under the raw RMP duals. Divergence between them is the
    # direct measure of whether dual selection helps or hurts the label search.
    mean_positive_rho_used::Float64
    mean_raw_positive_rho::Float64
end


"""
Number of `(p,j,k)` with `rho > tol` under the given duals -- i.e. how many
opportunities the pricer will have to carry. Pricing cost scales with this, so it
is logged for BOTH the raw RMP duals and (when enabled) the selected duals, to
test whether dual selection is making pricing easier or harder.
"""
function _count_positive_rho(
    md::PassengerFreeAssignmentMasterData,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64};
    tol::Float64=1e-9,
)::Int
    n = 0
    for p in md.passengers
        a = get(alpha, p.id, 0.0)
        a > tol || continue
        for (j, k) in md.feasible_assignments[p.id]
            rho = a - get(gamma_o, (p.id, j), 0.0) - get(gamma_d, (p.id, k), 0.0) -
                md.walk_cost_weight * md.assignment_walk_cost[(p.id, j, k)]
            rho > tol && (n += 1)
        end
    end
    return n
end

"""
Price one scenario. Pure with respect to shared state: it only *reads* `master`
and the dual dicts, and allocates everything else itself, which is what makes the
per-scenario calls safe to run concurrently.

Note that pricing touches no Gurobi at all -- it is pure Julia label-setting --
so there is no solver thread-safety question here.
"""
function _price_one_passenger_scenario(
    master::PassengerFreeAssignmentMaster,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64},
    s::Int,
    base_column_id::Int,
    existing::Vector{PassengerFreeAssignmentRouteColumn};
    n_candidates::Int,
    max_new_columns::Int,
    time_limit::Float64,
    reduced_cost_tol::Float64,
    station_budget_cap::Bool=false,
)
    md = master.master_data
    candidates = passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, s)
    isempty(candidates) && return PassengerFreeAssignmentRouteColumn[], true, 0

    pricing_data = create_passenger_free_assignment_pricing_data(
        s, md.nodes, md.travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time,
        repositioning_time=md.repositioning_time,
        max_stops=md.max_stops,
        max_visits_per_node=md.max_visits_per_node,
        # Restrict pricing to columns an integer solution could actually use: with
        # `theta_r >= 1` forcing `y_j = 1` at every assignment-carrying station,
        # `|A_r| <= sum(y) = l`. This leaves the IP optimum untouched while making
        # `lp_bound` tighter -- it directly excludes the broad multi-station "hub"
        # columns that earn dual credit in the LP but are unusable in the IP.
        # Costs pricing time (see the note in
        # notes/2026-07-30_passenger_pricing_label_search_optimizations.md), so it
        # is opt-in and judged on bound quality, not pricing speed.
        max_distinct_stations=station_budget_cap ? md.l : typemax(Int),
    )
    isempty(pricing_data.opportunities) && return PassengerFreeAssignmentRouteColumn[], true, 0

    columns_s, exhausted_s, stats_s = passenger_free_assignment_pricing_by_label_setting(
        pricing_data, existing;
        next_column_id=base_column_id,
        reduced_cost_tol=reduced_cost_tol,
        max_new_columns=max_new_columns,
        n_candidates=n_candidates,
        time_limit=time_limit,
    )
    return columns_s, exhausted_s, stats_s.labels_generated
end

"""
    _price_passenger_scenarios(...; parallel_scenarios)

The pricing subproblem separates exactly by scenario: a column belongs to one
scenario, and its reduced cost
`Phi_r = sum(alpha_p - u_pj - v_pk - W) - beta*c_r` touches only that scenario's
duals (the `y`-side duals `eta`/`s_j` live in the `y` column's dual row, not
`theta`'s). All cross-scenario coupling -- `theta <= y_j`, `sum y = L` -- is in the
master. So the per-scenario searches can run concurrently with no interaction.

Determinism is preserved regardless of thread count: results are written into
preallocated per-scenario slots, concatenated in sorted scenario order, and only
THEN assigned sequential column ids. Ids therefore do not depend on completion
order, which they would if each thread drew from a shared counter.
"""
function _price_passenger_scenarios(
    master::PassengerFreeAssignmentMaster,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64},
    next_column_id::Int;
    n_candidates::Int,
    max_new_columns::Int,
    time_limit::Float64,
    reduced_cost_tol::Float64,
    verify_reduced_costs::Bool,
    parallel_scenarios::Bool=true,
    station_budget_cap::Bool=false,
)
    md = master.master_data
    scenarios = sort!(collect(keys(md.passengers_by_scenario)))
    n_s = length(scenarios)

    # Group the pool by scenario ONCE. Each scenario previously rebuilt its own
    # `existing` list by scanning every column, i.e. O(S * pool) per iteration; at
    # 15 scenarios with a 10.7k pool that is ~160k filter operations per iteration
    # and it grows with the pool. Grouping first makes it O(pool).
    #
    # Behaviour is deliberately identical, including the `"scenario"` default of 0:
    # a column lacking that metadata key lands in bucket 0 and matches no real
    # scenario, exactly as the old filter did. (That is a latent inefficiency --
    # such a column is invisible to the pool-novelty check and may be regenerated
    # -- but changing it here would confound the performance measurement.)
    #
    # Built before the parallel region and only read inside it, so it is safe to
    # share across threads.
    empty_pool = PassengerFreeAssignmentRouteColumn[]
    existing_by_scenario = Dict{Int, Vector{PassengerFreeAssignmentRouteColumn}}()
    for c in values(master.columns)
        push!(get!(() -> PassengerFreeAssignmentRouteColumn[],
                   existing_by_scenario, Int(get(c.metadata, "scenario", 0))), c)
    end
    cols_by_s = Vector{Vector{PassengerFreeAssignmentRouteColumn}}(undef, n_s)
    exh_by_s = Vector{Bool}(undef, n_s)
    lab_by_s = Vector{Int}(undef, n_s)

    use_threads = parallel_scenarios && n_s > 1 && Threads.nthreads() > 1
    if use_threads
        Threads.@threads for i in 1:n_s
            cols_by_s[i], exh_by_s[i], lab_by_s[i] = _price_one_passenger_scenario(
                master, alpha, gamma_o, gamma_d, scenarios[i], next_column_id,
                get(existing_by_scenario, scenarios[i], empty_pool);
                n_candidates=n_candidates, max_new_columns=max_new_columns,
                time_limit=time_limit, reduced_cost_tol=reduced_cost_tol,
                station_budget_cap=station_budget_cap,
            )
        end
    else
        for i in 1:n_s
            cols_by_s[i], exh_by_s[i], lab_by_s[i] = _price_one_passenger_scenario(
                master, alpha, gamma_o, gamma_d, scenarios[i], next_column_id,
                get(existing_by_scenario, scenarios[i], empty_pool);
                n_candidates=n_candidates, max_new_columns=max_new_columns,
                time_limit=time_limit, reduced_cost_tol=reduced_cost_tol,
                station_budget_cap=station_budget_cap,
            )
        end
    end

    all_columns = PassengerFreeAssignmentRouteColumn[]
    exhausted = all(exh_by_s)
    labels_generated = sum(lab_by_s; init=0)
    column_id = next_column_id

    for i in 1:n_s
        # Every scenario priced from the SAME `next_column_id`, so ids collide until
        # renumbered here in sorted-scenario order.
        for c in cols_by_s[i]
            renumbered = PassengerFreeAssignmentRouteColumn(
                column_id, c.route, c.assignments, c.tau; metadata=c.metadata,
            )
            column_id += 1
            # Verification runs OUTSIDE the parallel region so a mismatch raises a
            # clean, deterministic error rather than a TaskFailedException whose
            # reported column depends on which thread happened to fail first.
            if verify_reduced_costs
                ok, pricer_rc, master_rc =
                    _verify_passenger_master_reduced_cost(renumbered, md, alpha, gamma_o, gamma_d)
                ok || error(
                    "passenger pricing reduced cost $(pricer_rc) disagrees with the master's " *
                    "dual-implied $(master_rc) for column route $(renumbered.route) -- the pricer " *
                    "and master formulations have drifted apart",
                )
            end
            push!(all_columns, renumbered)
        end
    end
    return all_columns, exhausted, labels_generated
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
    model::AnyAggregateODRouteModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    max_cg_iters::Int=1000,
    n_candidates::Int=20,
    max_new_columns::Int=20,
    reduced_cost_tol::Float64=1e-6,
    pricing_time_limit_sec::Float64=60.0,
    certification_time_limit_sec::Float64=600.0,
    ip_time_limit_sec::Float64=600.0,
    # Total budget for the CG phases (excludes the final MIP, which has its own
    # limit). Guards scaling runs against a case that keeps finding columns for
    # thousands of iterations and starves every later case of wall clock.
    total_time_limit_sec::Float64=Inf,
    # Pricing-aware dual selection (dual_selection.jl). Disabled by default; when
    # disabled, not one line of the selector runs and this loop behaves exactly as
    # it did before that feature existed.
    dual_selector::PassengerDualSelectorConfig=PassengerDualSelectorConfig(),
    # Run the per-scenario label searches concurrently. Exact and deterministic
    # (see `_price_passenger_scenarios`); a no-op with one thread or one scenario.
    parallel_scenarios::Bool=true,
    # Restrict pricing to columns with at most `l` distinct stations -- the most
    # any integer solution can open. Exact for the IP and tightens `lp_bound`, but
    # slows pricing (the soundness companion `U_a subseteq U_b` weakens dominance);
    # see notes/2026-07-30_passenger_pricing_label_search_optimizations.md.
    station_budget_cap::Bool=false,
    unserved_penalty::Union{Float64, Nothing}=nothing,
    verify_reduced_costs::Bool=true,
    verbose::Bool=true,
    silent::Bool=!verbose,
)::PassengerFreeAssignmentCGResult
    max_cg_iters > 0 || throw(ArgumentError("max_cg_iters must be positive"))
    n_candidates > 0 || throw(ArgumentError("n_candidates must be positive"))
    pricing_time_limit_sec > 0 || throw(ArgumentError("pricing_time_limit_sec must be positive"))
    certification_time_limit_sec > 0 ||
        throw(ArgumentError("certification_time_limit_sec must be positive"))

    t_start = time()
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())

    mapping = create_map(model, data)
    master_data = create_passenger_free_assignment_master_data(
        model, data, mapping; unserved_penalty=unserved_penalty,
    )
    master = build_passenger_free_assignment_master(master_data, optimizer_env; relax_integrality=true)
    m = master.model
    set_silent(m)

    verbose && println(
        "passenger free-assignment CG: $(length(master_data.passengers)) passengers, " *
        "$(length(master.coverage)) coverage rows, $(length(master.pickup_link)) pickup + " *
        "$(length(master.dropoff_link)) dropoff linking rows, l=$(master_data.l)",
    )

    selector = dual_selector.use_pricing_aware_dual_selection ?
        build_dual_selector(master_data, dual_selector, optimizer_env) : nothing
    selector_reference_rewards = Dict{Tuple{Int, Int, Int}, Float64}()
    selector_logs = PassengerDualSelectionRoundLog[]
    total_selector_seconds = 0.0
    selector_used_count = 0
    pos_rho_used_sum = 0
    pos_rho_raw_sum = 0
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

            raw_pos_rho = _count_positive_rho(master_data, alpha, gamma_o, gamma_d)

            # Single-pass dual selection: swap in a pricing-friendlier point on the
            # SAME optimal dual face, then price exactly as ordinary CG would. The
            # selector performs no pricing and certifies nothing; substituting an
            # RMP-optimal dual cannot invalidate the certificate.
            if !isnothing(selector)
                original_D = sum(values(alpha); init=0.0)
                sel_alpha, sel_u, sel_v, sel_rewards, sel_ok, sel_log =
                    select_pricing_duals!(
                        selector, master, lp_bound, original_D, selector_reference_rewards,
                    )
                push!(selector_logs, sel_log)
                total_selector_seconds += sel_log.selector_seconds
                if sel_ok
                    alpha, gamma_o, gamma_d = sel_alpha, sel_u, sel_v
                    selector_reference_rewards = sel_rewards
                    selector_used_count += 1
                end
            end

            pos_rho_raw_sum += raw_pos_rho
            pos_rho_used_sum += _count_positive_rho(master_data, alpha, gamma_o, gamma_d)
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
                lp_bound=lp_bound, lp_seconds=lp_seconds,
                pricing_seconds=pricing_seconds, labels_generated=labels,
                columns_priced=length(new_columns), columns_added=added,
                best_reduced_cost=best_rc, pricing_exhausted=exhausted,
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
            lp_bound=lp_bound, lp_seconds=0.0,
            pricing_seconds=round_cert_seconds, labels_generated=cert_labels,
            columns_priced=length(cert_columns), columns_added=cert_added,
            best_reduced_cost=isempty(cert_columns) ? nothing :
                minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in cert_columns),
            pricing_exhausted=cert_exhausted,
            pool_size=length(master.theta),
        ))
        verbose && println(
            "  [r$(n_rounds)] certification: $(length(cert_columns)) columns " *
            "($(cert_added) new), exhausted=$(cert_exhausted), " *
            "$(round(round_cert_seconds; digits=2))s",
        )
        flush(stdout)

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

    return PassengerFreeAssignmentCGResult(
        status, cg_stop_reason, lp_bound, lp_bound_certified,
        mip_obj, mip_term, n_iters, n_rounds, length(master.theta),
        length(master_data.passengers),
        length(master.coverage) + length(master.pickup_link) + length(master.dropoff_link),
        open_stations, unserved,
        certification_seconds, cert_exhausted,
        total_pricing_seconds, total_lp_seconds, total_labels,
        iteration_rows, time() - t_start,
        selector_logs, total_selector_seconds, selector_used_count,
        pos_rho_samples == 0 ? 0.0 : pos_rho_used_sum / pos_rho_samples,
        pos_rho_samples == 0 ? 0.0 : pos_rho_raw_sum / pos_rho_samples,
    )
end
