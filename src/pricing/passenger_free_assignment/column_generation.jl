"""
Column-generation loop for the passenger free-assignment formulation.

# Two-phase pricing: fast early return, then an exhaustive certificate

Each CG iteration prices with a small `n_candidates`, so the label search stops
as soon as it has that many improving columns instead of exploring to
exhaustion. That is a large speed win per iteration but returns *weak* columns
-- measured at up to ~89% short of the true pricing optimum on n=10 zhuzhou
instances (see `julia scripts/diagnose.jl ncandidates_sensitivity`),
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

# Structure

`run_passenger_free_assignment_column_generation` is a thin wrapper around the generic
`_run_column_generation` outer loop (`../generic_runner.jl`), exactly like
`run_aggregate_od_route_column_generation`. The original single-function loop's early-return-phase
solve/price/add cycle and its exhaustive-certification-phase counterpart are now
`PassengerFreeAssignmentCG` methods of the shared hooks (`_cg_build_master`/`_cg_solve_master!`/
`_cg_extract_duals`/`_cg_price_and_add_columns!`/`_cg_finalize_result`, defined below), dispatched
on `state.phase` (`:early_return` or `:certification`) -- see `_cg_price_and_add_columns!`'s own
docstring for exactly how the original's nested round/phase loop structure collapses onto the
shared flat outer loop. All of the state these hooks close over -- accumulated diagnostic rows,
running totals, which pricing phase is active -- lives in `PassengerFreeAssignmentCGState`,
threaded through explicitly rather than captured as closure variables.
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

# Per-iteration y-support churn diagnostics. `yv` is the current RMP LP solution of
# the build vars (`value.(master.y)`, length n); `prev` is the previous iteration's
# (or `nothing` at the first). `l` is the build budget (`sum_j y[j] = l`). Returns a
# NamedTuple describing the support and how much it moved since `prev`. The "top-l"
# set is the l stations with the largest y value -- since the LP is tight this is
# almost always the {y_j ~ 1} set, but it is robust to fractional splits.
function _y_support_metrics(yv::Vector{Float64}, prev::Union{Vector{Float64}, Nothing}, l::Int)
    n = length(yv)
    n_ge_half = count(>(0.5), yv)
    n_fractional = count(v -> 1e-6 < v < 1 - 1e-6, yv)
    topl = Set(partialsortperm(yv, 1:min(l, n); rev=true))
    if prev === nothing
        return (support_ge_half=n_ge_half, n_fractional=n_fractional,
            l1_move=missing, topl_entered=missing, topl_jaccard=missing,
            topl=topl)
    end
    l1_move = sum(abs.(yv .- prev))
    prev_topl = Set(partialsortperm(prev, 1:min(l, n); rev=true))
    entered = length(setdiff(topl, prev_topl))
    uni = length(union(topl, prev_topl))
    jac = uni == 0 ? 1.0 : length(intersect(topl, prev_topl)) / uni
    return (support_ge_half=n_ge_half, n_fractional=n_fractional,
        l1_move=l1_move, topl_entered=entered, topl_jaccard=jac, topl=topl)
end

function _record_priced_route!(
    route_rows::Vector{NamedTuple}, rho_rows::Vector{NamedTuple},
    column::PassengerFreeAssignmentRouteColumn,
    md::PassengerFreeAssignmentMasterData,
    alpha::Dict{Int, Float64}, gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64}, source_topl::Set{Int};
    solve_sequence::Int, round::Int, iteration::Int, phase::String, action::Symbol,
)
    rewards = Float64[]
    for (p, j, k) in column.assignments
        walk_term = md.walk_cost_weight * md.assignment_walk_cost[(p, j, k)]
        rho = get(alpha, p, 0.0) - get(gamma_o, (p, j), 0.0) -
            get(gamma_d, (p, k), 0.0) - walk_term
        push!(rewards, rho)
        push!(rho_rows, (
            source_solve_sequence=solve_sequence, source_round=round,
            source_iteration=iteration, source_phase=phase,
            column_id=column.id, scenario=Int(get(column.metadata, "scenario", 0)),
            passenger=p, pickup_index=j, dropoff_index=k,
            alpha=get(alpha, p, 0.0), gamma_origin=get(gamma_o, (p, j), 0.0),
            gamma_destination=get(gamma_d, (p, k), 0.0),
            walking_dual_term=walk_term, rho=rho,
            pickup_in_source_topl=j in source_topl,
            dropoff_in_source_topl=k in source_topl,
        ))
    end
    reward_sum = sum(rewards)
    route_time_cost = md.route_regularization_weight *
        (column.tau + md.repositioning_time)
    distinct_route = Set(column.route)
    outside = setdiff(distinct_route, source_topl)
    push!(route_rows, (
        source_solve_sequence=solve_sequence, source_round=round,
        source_iteration=iteration, source_phase=phase,
        column_id=column.id, scenario=Int(get(column.metadata, "scenario", 0)),
        add_action=String(action), route=join(column.route, "-"),
        n_stops=length(column.route), n_distinct_stations=length(distinct_route),
        n_revisits=length(column.route) - length(distinct_route), tau=column.tau,
        route_time_cost=route_time_cost, n_assignments=length(rewards),
        rho_sum=reward_sum, rho_min=minimum(rewards), rho_mean=mean(rewards),
        rho_max=maximum(rewards), rho_top1_share=reward_sum <= 1e-12 ? 0.0 : maximum(rewards) / reward_sum,
        recomputed_reduced_cost=route_time_cost - reward_sum,
        reported_reduced_cost=Float64(get(column.metadata, "reduced_cost", NaN)),
        source_topl=join(sort!(collect(source_topl)), ";"),
        outside_source_topl=join(sort!(collect(outside)), ";"),
        n_distinct_outside_source_topl=length(outside),
    ))
    return nothing
end

function _record_station_rho_scores!(
    rows::Vector{NamedTuple}, opportunity_rows::Vector{NamedTuple},
    md::PassengerFreeAssignmentMasterData,
    alpha::Dict{Int, Float64}, gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64}, scenarios::Vector{Int};
    solve_sequence::Int, round::Int, iteration::Int, phase::String,
)
    count_incident = zeros(Int, length(md.nodes))
    sum_incident = zeros(Float64, length(md.nodes))
    max_incident = zeros(Float64, length(md.nodes))
    sum_pickup = zeros(Float64, length(md.nodes))
    sum_dropoff = zeros(Float64, length(md.nodes))
    for scenario in scenarios
        for c in passenger_free_assignment_pricing_candidates(
            md, alpha, gamma_o, gamma_d, scenario,
        )
            push!(opportunity_rows, (
                solve_sequence=solve_sequence, round=round, iteration=iteration,
                phase=phase, scenario=scenario, passenger=c.passenger,
                pickup_index=c.origin, dropoff_index=c.destination,
                rho=c.reward, direct_travel_time=md.travel_cost[(c.origin, c.destination)],
            ))
            for j in unique((c.origin, c.destination))
                count_incident[j] += 1
                sum_incident[j] += c.reward
                max_incident[j] = max(max_incident[j], c.reward)
            end
            sum_pickup[c.origin] += c.reward
            sum_dropoff[c.destination] += c.reward
        end
    end
    for j in eachindex(md.nodes)
        push!(rows, (
            solve_sequence=solve_sequence, round=round, iteration=iteration, phase=phase,
            station_index=j, positive_rho_opportunities=count_incident[j],
            positive_rho_sum=sum_incident[j], positive_rho_max=max_incident[j],
            positive_rho_pickup_sum=sum_pickup[j],
            positive_rho_dropoff_sum=sum_dropoff[j],
        ))
    end
    return nothing
end

function _record_theta_snapshot!(
    rows::Vector{NamedTuple}, summary_rows::Vector{NamedTuple},
    master::PassengerFreeAssignmentMaster, previous::Dict{Int, Float64};
    solve_sequence::Int, round::Int, iteration::Int, phase::String,
)
    current = Dict{Int, Float64}()
    for (id, var) in master.theta
        theta = value(var)
        current[id] = theta
        column = master.columns[id]
        push!(rows, (
            solve_sequence=solve_sequence, round=round, iteration=iteration, phase=phase,
            column_id=id, scenario=Int(get(column.metadata, "scenario", 0)),
            theta_value=theta, theta_previous=get(previous, id, 0.0),
            theta_change=theta - get(previous, id, 0.0),
            source_solve_sequence=get(column.metadata, "cg_source_solve_sequence", missing),
            n_stops=length(column.route), n_distinct_stations=length(unique(column.route)),
        ))
    end
    ids = union(keys(previous), keys(current))
    l1_move = isempty(previous) ? missing :
        sum(abs(get(current, id, 0.0) - get(previous, id, 0.0)) for id in ids)
    vals = collect(values(current))
    push!(summary_rows, (
        solve_sequence=solve_sequence, round=round, iteration=iteration, phase=phase,
        n_columns=length(vals), n_positive=count(>(1e-7), vals),
        n_fractional=count(v -> 1e-7 < v < 1 - 1e-7, vals),
        n_ge_half=count(>=(0.5), vals), n_near_one=count(>=(0.99), vals),
        theta_sum=sum(vals), theta_max=isempty(vals) ? 0.0 : maximum(vals),
        theta_l1_move=l1_move,
    ))
    empty!(previous)
    merge!(previous, current)
    return nothing
end

function _theta_rho_pricing_subset(
    master::PassengerFreeAssignmentMaster,
    alpha::Dict{Int, Float64}, gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64}, scenarios::Vector{Int},
    core_size::Int, n_outsiders::Int,
)
    md = master.master_data
    load = zeros(Float64, length(md.nodes))
    near_one = Set{Int}()
    for (id, var) in master.theta
        theta = value(var)
        theta > 1e-7 || continue
        endpoints = Set{Int}()
        for (_, j, k) in master.columns[id].assignments
            push!(endpoints, j)
            push!(endpoints, k)
        end
        for j in endpoints
            load[j] += theta
            theta >= 0.99 && push!(near_one, j)
        end
    end
    ranked_load = sort!(collect(eachindex(load)); by=j -> (-load[j], j))
    ranked_near = sort!(collect(near_one); by=j -> (-load[j], j))
    core = Set(ranked_near[1:min(core_size, length(ranked_near))])
    for j in ranked_load
        length(core) >= core_size && break
        push!(core, j)
    end

    rho_score = zeros(Float64, length(md.nodes))
    for scenario in scenarios
        for c in passenger_free_assignment_pricing_candidates(
            md, alpha, gamma_o, gamma_d, scenario,
        )
            for j in unique((c.origin, c.destination))
                rho_score[j] += c.reward
            end
        end
    end
    ranked_outside = sort!([j for j in eachindex(rho_score) if !(j in core)];
        by=j -> (-rho_score[j], j))
    outsiders = ranked_outside[1:min(n_outsiders, length(ranked_outside))]
    return union(core, outsiders), core, outsiders, load, rho_score
end

"""
    PassengerFreeAssignmentCGState <: AbstractCGState

Mutable state for one `run_passenger_free_assignment_column_generation` solve -- the local
variables the original single-function loop closed over, now threaded explicitly through the
`PassengerFreeAssignmentCG` methods of the generic column-generation hooks
(`pricing/generic_runner.jl`'s `_cg_build_master`/`_cg_solve_master!`/`_cg_extract_duals`/
`_cg_price_and_add_columns!`/`_cg_finalize_result`). Built once (via the keyword constructor
below) from the entry function's own kwargs, then mutated in place for the life of the solve.

Defined here rather than in `generic_runner.jl` (unlike `AggregateODRouteCGState`): it types
against `PassengerFreeAssignmentMaster`/`StationClusterHierarchy`/etc., all defined under
`pricing/passenger/`, included after `generic_runner.jl` -- the same reason
`variables/constraints/objectives/aggregate_od_route/column_generation/master.jl` leave
`master_data` untyped. The `PassengerFreeAssignmentCG` hook methods live here for the same
reason, alongside this state type and the helpers they call.

`phase` (`:early_return` or `:certification`) is what the original's nested round/phase loop
structure collapses onto in the flat generic outer loop -- see `_cg_price_and_add_columns!`'s
docstring for exactly how. `cg_iterations` exists only to satisfy `AbstractCGState`'s contract
(the shared loop writes the *outer-loop* call count there); it is not what this algorithm reports
as its own iteration count -- that is `n_iters` (early-return-phase solves only), matching the
original's `n_iters`/`n_cg_iters` semantics exactly.
"""
mutable struct PassengerFreeAssignmentCGState <: AbstractCGState
    # ── config, fixed for the life of the solve ──────────────────────────────
    max_cg_iters::Int
    n_candidates::Int
    max_new_columns::Int
    exhaustive_pricing_each_iteration::Bool
    theta_rho_core_size::Int
    theta_rho_n_outsiders::Int
    theta_rho_subset_active::Bool
    reduced_cost_tol::Float64
    pricing_time_limit_sec::Float64
    certification_time_limit_sec::Float64
    total_time_limit_sec::Float64
    parallel_scenarios::Bool
    station_budget_cap::Bool
    compensated_dominance::Bool
    station_simple_warm_start::Bool
    reward_coarsening_levels::Int
    station_reduced_cost_filter_active::Bool
    station_reduced_cost_filter_mode::Symbol
    use_adaptive_cluster_certification::Bool
    cluster_initial_num_clusters::Union{Int, Nothing}
    cluster_max_num_clusters::Union{Int, Nothing}
    cluster_max_size::Union{Int, Nothing}
    cluster_time_limit_sec::Float64
    verify_reduced_costs::Bool
    verbose::Bool
    optimizer_env::Any
    t_start::Float64
    pricing_scenarios::Vector{Int}

    # ── mutable across the whole solve ───────────────────────────────────────
    # `_cg_build_master`'s hook contract only returns `(master, state)`, so anything else a later
    # hook needs that `_cg_build_master` alone computes or receives -- the full station/OD
    # `mapping`, `data` itself (`_cg_solve_master!`'s hook signature carries no `data` parameter,
    # unlike `_cg_price_and_add_columns!`'s, but its diagnostic rows need
    # `data.array_idx_to_station_id`), the two-stop seed count -- is stashed here rather than
    # recomputed or threaded through extra hook parameters.
    data::StationSelectionData
    mapping::AggregateODRouteMap
    n_seed_columns::Int
    phase::Symbol
    # Wall-clock start of the current certification pass, set by `_cg_solve_master!` and read
    # back by `_cg_price_and_add_columns!` to compute one certification-pass duration spanning
    # both hook calls -- matching the original's single `round_cert_seconds = time() - t_cert`
    # measured across its own solve+duals+price+add block, not split into two additions.
    phase_start_time::Float64
    cg_iterations::Int
    pricing_station_simple::Bool
    warm_start_pending::Bool
    n_iters::Int
    n_rounds::Int
    next_column_id::Int
    lp_bound::Float64
    solve_sequence::Int
    current_topl::Set{Int}
    prev_yv::Union{Nothing, Vector{Float64}}
    previous_theta::Dict{Int, Float64}
    certification_seconds::Float64
    cert_exhausted::Bool
    total_pricing_seconds::Float64
    total_lp_seconds::Float64
    total_labels::Int
    pos_rho_used_sum::Int
    pos_rho_raw_sum::Int
    pos_rho_stations_used_sum::Int
    pos_rho_stations_raw_sum::Int
    pos_rho_samples::Int

    # ── diagnostic row accumulators ──────────────────────────────────────────
    iteration_rows::Vector{NamedTuple}
    y_support_rows::Vector{NamedTuple}
    y_station_rows::Vector{NamedTuple}
    priced_route_rows::Vector{NamedTuple}
    route_rho_rows::Vector{NamedTuple}
    station_rho_score_rows::Vector{NamedTuple}
    opportunity_rho_rows::Vector{NamedTuple}
    theta_rows::Vector{NamedTuple}
    theta_summary_rows::Vector{NamedTuple}
    theta_rho_subset_rows::Vector{NamedTuple}
    cluster_certificate_rows::Vector{NamedTuple}
    cluster_hierarchies::Dict{Int, StationClusterHierarchy}
    cluster_caches::Dict{Int, ClusterPricingCache}
end

function PassengerFreeAssignmentCGState(;
    max_cg_iters::Int, n_candidates::Int, max_new_columns::Int,
    exhaustive_pricing_each_iteration::Bool, theta_rho_core_size::Int,
    theta_rho_n_outsiders::Int, theta_rho_subset_active::Bool,
    reduced_cost_tol::Float64, pricing_time_limit_sec::Float64,
    certification_time_limit_sec::Float64, total_time_limit_sec::Float64,
    parallel_scenarios::Bool, station_budget_cap::Bool, compensated_dominance::Bool,
    station_simple_warm_start::Bool, reward_coarsening_levels::Int,
    station_reduced_cost_filter_active::Bool, station_reduced_cost_filter_mode::Symbol,
    use_adaptive_cluster_certification::Bool,
    cluster_initial_num_clusters::Union{Int, Nothing},
    cluster_max_num_clusters::Union{Int, Nothing}, cluster_max_size::Union{Int, Nothing},
    cluster_time_limit_sec::Float64, verify_reduced_costs::Bool, verbose::Bool,
    optimizer_env, t_start::Float64, pricing_scenarios::Vector{Int},
    use_station_simple::Bool, data::StationSelectionData, mapping::AggregateODRouteMap,
)
    return PassengerFreeAssignmentCGState(
        max_cg_iters, n_candidates, max_new_columns, exhaustive_pricing_each_iteration,
        theta_rho_core_size, theta_rho_n_outsiders, theta_rho_subset_active,
        reduced_cost_tol, pricing_time_limit_sec, certification_time_limit_sec,
        total_time_limit_sec, parallel_scenarios, station_budget_cap, compensated_dominance,
        station_simple_warm_start, reward_coarsening_levels, station_reduced_cost_filter_active,
        station_reduced_cost_filter_mode, use_adaptive_cluster_certification,
        cluster_initial_num_clusters, cluster_max_num_clusters, cluster_max_size,
        cluster_time_limit_sec, verify_reduced_costs, verbose, optimizer_env, t_start,
        pricing_scenarios,
        data, mapping, 0,
        # Starts in the early-return phase, round 1 (mirrors the original's `n_rounds=0`
        # immediately incremented to 1 at the top of its first `while true` pass, before any
        # work happens -- starting at 1 directly avoids an awkward "first round is special" case).
        :early_return, 0.0, 0,
        # Which pricer the current phase uses. Under `station_simple_warm_start` this
        # starts elementary and flips to revisit-tolerant once the elementary universe is
        # exhausted; otherwise it is fixed at `use_station_simple` for the whole run.
        station_simple_warm_start ? true : use_station_simple,
        station_simple_warm_start,
        0, 1, 1, NaN, 0, Set{Int}(), nothing, Dict{Int, Float64}(),
        0.0, false, 0.0, 0.0, 0,
        0, 0, 0, 0, 0,
        NamedTuple[], NamedTuple[], NamedTuple[], NamedTuple[], NamedTuple[],
        NamedTuple[], NamedTuple[], NamedTuple[], NamedTuple[], NamedTuple[],
        NamedTuple[], Dict{Int, StationClusterHierarchy}(), Dict{Int, ClusterPricingCache}(),
    )
end

# ---------------------------------------------------------------------------
# PassengerFreeAssignmentCG hooks. `rmp` is the `PassengerFreeAssignmentMaster`
# returned by `build_passenger_free_assignment_master` -- the same object the
# original called `master`; every computation below is that function's
# original loop body, split at hook boundaries, not rewritten.
# ---------------------------------------------------------------------------

"""
    _cg_build_master(data, formulation::AggregateODRouteModel, algorithm::PassengerFreeAssignmentCG, solver, optimizer_env)

Builds the passenger free-assignment RMP (no route columns yet), seeds it with every two-stop
route when `algorithm.seed_two_stop_routes`, and builds the `PassengerFreeAssignmentCGState`,
resolving `solver`'s shared knobs (`n_candidates`, `reduced_cost_tol`, `pricing_time_limit_sec`,
`max_columns_per_iteration`) and `algorithm`'s own fields into it.
"""
function _cg_build_master(
    data::StationSelectionData,
    formulation::AggregateODRouteModel,
    algorithm::PassengerFreeAssignmentCG,
    solver::ColumnGenerationSolver,
    optimizer_env,
)
    t_start = time()
    mapping = create_map(formulation, data)
    master_data = create_passenger_free_assignment_master_data(
        formulation, data, mapping; unserved_penalty=algorithm.unserved_penalty,
    )
    master = build_passenger_free_assignment_master(master_data, optimizer_env; relax_integrality=true)
    solver.config.silent && set_silent(master.model)

    algorithm.verbose && println(
        "passenger free-assignment CG: $(length(master_data.passengers)) passengers, " *
        "$(length(master.coverage)) coverage rows, $(length(master.pickup_link)) pickup + " *
        "$(length(master.dropoff_link)) dropoff linking rows, l=$(master_data.l)",
    )

    state = PassengerFreeAssignmentCGState(;
        max_cg_iters=solver.max_iterations, n_candidates=solver.n_candidates,
        max_new_columns=solver.max_columns_per_iteration,
        exhaustive_pricing_each_iteration=algorithm.exhaustive_pricing_each_iteration,
        theta_rho_core_size=algorithm.theta_rho_core_size,
        theta_rho_n_outsiders=algorithm.theta_rho_n_outsiders,
        theta_rho_subset_active=algorithm.theta_rho_core_size > 0,
        reduced_cost_tol=solver.reduced_cost_tol, pricing_time_limit_sec=solver.pricing_time_limit_sec,
        certification_time_limit_sec=algorithm.certification_time_limit_sec,
        total_time_limit_sec=algorithm.total_time_limit_sec,
        parallel_scenarios=algorithm.parallel_scenarios,
        station_budget_cap=algorithm.station_budget_cap,
        compensated_dominance=algorithm.compensated_dominance,
        station_simple_warm_start=algorithm.station_simple_warm_start,
        reward_coarsening_levels=algorithm.reward_coarsening_levels,
        station_reduced_cost_filter_active=algorithm.use_station_reduced_cost_filter,
        station_reduced_cost_filter_mode=algorithm.station_reduced_cost_filter_mode,
        use_adaptive_cluster_certification=algorithm.use_adaptive_cluster_certification,
        cluster_initial_num_clusters=algorithm.cluster_initial_num_clusters,
        cluster_max_num_clusters=algorithm.cluster_max_num_clusters,
        cluster_max_size=algorithm.cluster_max_size,
        cluster_time_limit_sec=algorithm.cluster_time_limit_sec,
        verify_reduced_costs=algorithm.verify_reduced_costs,
        verbose=algorithm.verbose, optimizer_env=optimizer_env, t_start=t_start,
        pricing_scenarios=sort!(collect(keys(master_data.passengers_by_scenario))),
        use_station_simple=algorithm.use_station_simple, data=data, mapping=mapping,
    )

    # ── seed: every two-stop route ────────────────────────────────────────────
    # Without this the pool starts empty, so the first several iterations price
    # against big-M duals and are spent finding *any* covering set rather than a
    # cheap one. These columns are the ones `unserved_penalty` was standing in
    # for, they are enumerable, and they make `v[p]` inert unless the instance is
    # genuinely uncoverable. See the seeding function's docstring.
    if algorithm.seed_two_stop_routes
        seeds = passenger_free_assignment_two_stop_seed_columns(
            master_data; next_column_id=state.next_column_id,
        )
        n_seed_columns = 0
        for column in seeds
            _theta, action = add_passenger_free_assignment_column!(master, column)
            action == :added && (n_seed_columns += 1)
        end
        state.next_column_id += length(seeds)
        state.n_seed_columns = n_seed_columns
        algorithm.verbose && println(
            "  seeded $(n_seed_columns) two-stop columns " *
            "(of $(length(seeds)) distinct (scenario, j, k) routes)",
        )
    end

    return master, state
end

"""
    _cg_max_outer_iterations(algorithm::PassengerFreeAssignmentCG, solver) -> Int

A large safety constant, not `solver.max_iterations` -- this algorithm's real iteration budget
(`state.max_cg_iters`, counting early-return-phase solves only) is enforced inside
`_cg_solve_master!`'s own phase-transition check, since the shared outer loop's own per-call
counter also advances once per *certification*-phase call, which the original never counted
against `max_cg_iters`.
"""
_cg_max_outer_iterations(::PassengerFreeAssignmentCG, ::ColumnGenerationSolver) = typemax(Int) ÷ 2

"""
    _cg_time_budget_exceeded(algorithm::PassengerFreeAssignmentCG, solver, state, t_start) -> Bool

Only applies during the early-return phase -- the original checks this at the top of its
early-return `while` loop only; once early-return stalls, certification always runs at least once
regardless of the clock (its own `cert_budget` bounds the pricing call instead). Also clears
`state.cert_exhausted` when it fires, matching the original's "skip certification entirely, don't
let this round read as certified" comment at the equivalent site.
"""
function _cg_time_budget_exceeded(
    ::PassengerFreeAssignmentCG,
    ::ColumnGenerationSolver,
    state::PassengerFreeAssignmentCGState,
    ::Float64,
)
    exceeded = state.phase == :early_return && (time() - state.t_start > state.total_time_limit_sec)
    exceeded && (state.cert_exhausted = false)
    return exceeded
end

# Shared by both phases (only the `phase` tag differs) -- the diagnostic block the original
# repeats verbatim right after each of its two `optimize!(m)` calls.
function _pfa_record_lp_snapshot!(
    master::PassengerFreeAssignmentMaster,
    state::PassengerFreeAssignmentCGState,
    phase::String,
)
    data = state.data
    let yv = value.(master.y), l = round(Int, sum(value.(master.y)))
        state.solve_sequence += 1
        ym = _y_support_metrics(yv, state.prev_yv, l)
        state.current_topl = ym.topl
        push!(state.y_support_rows, (
            solve_sequence=state.solve_sequence, round=state.n_rounds,
            iteration=state.n_iters, phase=phase,
            lp_bound=state.lp_bound, l=l,
            support_ge_half=ym.support_ge_half, n_fractional=ym.n_fractional,
            l1_move=ym.l1_move, topl_entered=ym.topl_entered,
            topl_jaccard=ym.topl_jaccard,
            topl_indices=join(sort!(collect(ym.topl)), ";"),
            topl_station_ids=join(sort!(data.array_idx_to_station_id[collect(ym.topl)]), ";"),
        ))
        yrc = Float64[reduced_cost(var) for var in master.y]
        closed = sort!([j for j in eachindex(yv) if yv[j] <= 1e-7]; by=j -> (yrc[j], j))
        closed_rank = Dict(j => rank for (rank, j) in enumerate(closed))
        for j in eachindex(yv)
            push!(state.y_station_rows, (
                solve_sequence=state.solve_sequence, round=state.n_rounds,
                iteration=state.n_iters, phase=phase,
                station_index=j, station_id=data.array_idx_to_station_id[j],
                y_value=yv[j], y_reduced_cost=yrc[j],
                in_topl=j in ym.topl,
                closed_rc_rank=get(closed_rank, j, missing),
            ))
        end
        state.prev_yv = yv
    end
    _record_theta_snapshot!(
        state.theta_rows, state.theta_summary_rows, master, state.previous_theta;
        solve_sequence=state.solve_sequence, round=state.n_rounds,
        iteration=state.n_iters, phase=phase,
    )
    return nothing
end

"""
    _cg_solve_master!(rmp::PassengerFreeAssignmentMaster, algorithm::PassengerFreeAssignmentCG, state, iteration)

Re-solves the current RMP LP for whichever phase `state.phase` is in, plus that phase's diagnostic
snapshot -- both phases do the exact same diagnostic recording (`_pfa_record_lp_snapshot!`),
differing only in the `phase` tag and (for early-return) the `n_iters`/`total_lp_seconds`
bookkeeping the original's inner `while` loop did before this solve.

If `state.phase == :early_return` but `state.n_iters` has already reached `state.max_cg_iters`,
flips to `:certification` first and solves that phase's LP instead -- this is what the original's
inner `while n_iters < max_cg_iters` loop condition becoming false (without ever `break`ing, i.e.
the last early-return iteration added columns right up to the budget) falls through to, within the
same `while true` pass.
"""
function _cg_solve_master!(
    rmp::PassengerFreeAssignmentMaster,
    ::PassengerFreeAssignmentCG,
    state::PassengerFreeAssignmentCGState,
    iteration::Int,
)
    if state.phase == :early_return && state.n_iters >= state.max_cg_iters
        state.phase = :certification
    end

    m = rmp.model
    if state.phase == :early_return
        state.n_iters += 1
        t_lp = time()
        optimize!(m)
        lp_seconds = time() - t_lp
        state.total_lp_seconds += lp_seconds
        term_status = termination_status(m)
        if primal_status(m) != MOI.FEASIBLE_POINT
            return term_status, nothing, false, lp_seconds
        end
        state.lp_bound = objective_value(m)
        _pfa_record_lp_snapshot!(rmp, state, "early_return")
        return term_status, state.lp_bound, true, lp_seconds
    else
        state.verbose && println("  [r$(state.n_rounds)] exhaustive certification pass ...")
        t_cert = time()
        optimize!(m)
        secs = time() - t_cert
        term_status = termination_status(m)
        if primal_status(m) != MOI.FEASIBLE_POINT
            state.certification_seconds += secs
            return term_status, nothing, false, secs
        end
        state.lp_bound = objective_value(m)
        _pfa_record_lp_snapshot!(rmp, state, "certification")
        state.phase_start_time = t_cert
        return term_status, state.lp_bound, true, secs
    end
end

"""
    _cg_extract_duals(rmp::PassengerFreeAssignmentMaster, algorithm::PassengerFreeAssignmentCG, state)

Pure dual extraction plus the rho-score diagnostic recording the original does immediately after
-- identical for both phases (only the `phase` string tag differs).
"""
function _cg_extract_duals(
    rmp::PassengerFreeAssignmentMaster,
    ::PassengerFreeAssignmentCG,
    state::PassengerFreeAssignmentCGState,
)
    alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(rmp)
    _record_station_rho_scores!(
        state.station_rho_score_rows, state.opportunity_rho_rows,
        rmp.master_data, alpha, gamma_o, gamma_d,
        state.pricing_scenarios; solve_sequence=state.solve_sequence, round=state.n_rounds,
        iteration=state.n_iters, phase=string(state.phase),
    )
    return alpha, gamma_o, gamma_d
end

# The early-return-phase half of `_cg_price_and_add_columns!` -- prices, dedupes/adds, and logs
# one early-return iteration, identical in substance to the original inner `while` loop's body
# (minus the solve+duals pieces, now in `_cg_solve_master!`/`_cg_extract_duals`).
function _pfa_price_early_return!(
    master::PassengerFreeAssignmentMaster,
    master_data::PassengerFreeAssignmentMasterData,
    state::PassengerFreeAssignmentCGState,
    alpha::Dict{Int, Float64}, gamma_o::Dict{Tuple{Int, Int}, Float64}, gamma_d::Dict{Tuple{Int, Int}, Float64},
    lp_solve_seconds::Float64,
)
    allowed_stations = nothing
    if state.theta_rho_subset_active
        allowed, core, outsiders, theta_load, rho_score = _theta_rho_pricing_subset(
            master, alpha, gamma_o, gamma_d, state.pricing_scenarios,
            state.theta_rho_core_size, state.theta_rho_n_outsiders,
        )
        allowed_stations = allowed
        push!(state.theta_rho_subset_rows, (
            solve_sequence=state.solve_sequence, round=state.n_rounds, iteration=state.n_iters,
            phase="early_return", core_size=length(core),
            n_outsiders=length(outsiders), allowed_size=length(allowed),
            core_indices=join(sort!(collect(core)), ";"),
            outsider_indices=join(outsiders, ";"),
            allowed_indices=join(sort!(collect(allowed)), ";"),
            core_theta_load=sum(theta_load[j] for j in core),
            outsider_rho_score=sum(rho_score[j] for j in outsiders),
        ))
    end

    raw_stats = _positive_rho_stats(master_data, alpha, gamma_o, gamma_d)
    used_stats = raw_stats

    station_filter_stats = state.station_reduced_cost_filter_active ?
        _station_reduced_cost_filter_stats(
            master, alpha, gamma_o, gamma_d, state.pricing_scenarios, state.reduced_cost_tol,
            state.optimizer_env, state.station_reduced_cost_filter_mode) : nothing
    if !isnothing(station_filter_stats)
        used_stats = (
            n_positive_rho=station_filter_stats.filtered_positive_rho,
            j_positive_rho=station_filter_stats.filtered_positive_rho_stations,
        )
    end
    state.pos_rho_raw_sum += raw_stats.n_positive_rho
    state.pos_rho_used_sum += used_stats.n_positive_rho
    state.pos_rho_stations_raw_sum += raw_stats.j_positive_rho
    state.pos_rho_stations_used_sum += used_stats.j_positive_rho
    state.pos_rho_samples += 1

    t_price = time()
    new_columns, exhausted, labels = _price_passenger_scenarios(
        master, alpha, gamma_o, gamma_d, state.next_column_id;
        n_candidates=state.exhaustive_pricing_each_iteration ? typemax(Int) ÷ 2 : state.n_candidates,
        max_new_columns=state.exhaustive_pricing_each_iteration ? typemax(Int) ÷ 2 : state.max_new_columns,
        time_limit=state.pricing_time_limit_sec,
        reduced_cost_tol=state.reduced_cost_tol,
        verify_reduced_costs=state.verify_reduced_costs,
        parallel_scenarios=state.parallel_scenarios,
        station_budget_cap=state.station_budget_cap,
        compensated_dominance=state.compensated_dominance,
        use_station_simple=state.pricing_station_simple,
        reward_coarsening_levels=state.reward_coarsening_levels,
        use_station_reduced_cost_filter=state.station_reduced_cost_filter_active,
        station_reduced_cost_filter_mode=state.station_reduced_cost_filter_mode,
        station_filter_excluded_stations=isnothing(station_filter_stats) ?
            nothing : station_filter_stats.excluded_stations,
        station_filter_excluded_opportunities=isnothing(station_filter_stats) ?
            nothing : station_filter_stats.excluded_opportunities,
        allowed_stations=allowed_stations,
        optimizer_env=state.optimizer_env,
    )
    pricing_seconds = time() - t_price
    state.total_pricing_seconds += pricing_seconds
    state.total_labels += labels
    state.next_column_id += length(new_columns)

    added = 0
    for c in new_columns
        c.metadata["cg_source_solve_sequence"] = state.solve_sequence
        c.metadata["cg_source_round"] = state.n_rounds
        c.metadata["cg_source_iteration"] = state.n_iters
        c.metadata["cg_source_phase"] = "early_return"
        c.metadata["cg_source_topl"] = sort!(collect(state.current_topl))
        _theta, action = add_passenger_free_assignment_column!(master, c)
        c.metadata["cg_add_action"] = String(action)
        _record_priced_route!(
            state.priced_route_rows, state.route_rho_rows, c, master_data,
            alpha, gamma_o, gamma_d, state.current_topl;
            solve_sequence=state.solve_sequence, round=state.n_rounds,
            iteration=state.n_iters, phase="early_return", action=action,
        )
        action == :added && (added += 1)
    end

    best_rc = isempty(new_columns) ? nothing :
        minimum(Float64(get(c.metadata, "reduced_cost", Inf)) for c in new_columns)
    push!(state.iteration_rows, (
        round=state.n_rounds, iteration=state.n_iters, phase="early_return",
        pricer=state.pricing_station_simple ? "station_simple" :
            state.reward_coarsening_levels > 0 ? "reward_coarsened_L$(state.reward_coarsening_levels)" : "revisit",
        lp_bound=state.lp_bound, lp_seconds=lp_solve_seconds,
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
    state.verbose && @printf(
        "  [r%d iter %3d] lp=%.4f priced=%d added=%d best_rc=%s labels=%d lp=%.2fs price=%.2fs pool=%d\n",
        state.n_rounds, state.n_iters, state.lp_bound, length(new_columns), added,
        isnothing(best_rc) ? "n/a" : @sprintf("%.4f", best_rc),
        labels, lp_solve_seconds, pricing_seconds, length(master.theta),
    )
    flush(stdout)

    # Not yet a proof of optimality -- early return and timeouts both land here. The
    # certification pass is what decides; flipping the phase here is what the original's
    # `stalled_reason = :early_return_stalled; break` falls through to.
    added == 0 && (state.phase = :certification)
    return added, exhausted, nothing
end

# The certification-phase half of `_cg_price_and_add_columns!` -- one round's exhaustive
# certification pass, identical in substance to the original "phase 2" block (minus the
# solve+duals pieces, now in `_cg_solve_master!`/`_cg_extract_duals`).
function _pfa_price_certification!(
    master::PassengerFreeAssignmentMaster,
    master_data::PassengerFreeAssignmentMasterData,
    state::PassengerFreeAssignmentCGState,
    alpha::Dict{Int, Float64}, gamma_o::Dict{Tuple{Int, Int}, Float64}, gamma_d::Dict{Tuple{Int, Int}, Float64},
)
    t_cert = state.phase_start_time
    cert_raw_stats = _positive_rho_stats(master_data, alpha, gamma_o, gamma_d)
    cert_station_filter_stats = state.station_reduced_cost_filter_active ?
        _station_reduced_cost_filter_stats(
            master, alpha, gamma_o, gamma_d, state.pricing_scenarios, state.reduced_cost_tol,
            state.optimizer_env, state.station_reduced_cost_filter_mode) : nothing
    cert_budget = isfinite(state.total_time_limit_sec) ?
        min(state.certification_time_limit_sec, max(1.0, state.total_time_limit_sec - (time() - state.t_start))) :
        state.certification_time_limit_sec
    cluster_certified = false
    if state.use_adaptive_cluster_certification
        all_cluster_certified = true
        for scenario in state.pricing_scenarios
            candidates = passenger_free_assignment_pricing_candidates(
                master_data, alpha, gamma_o, gamma_d, scenario)
            physical = create_passenger_free_assignment_pricing_data(
                scenario, master_data.nodes, master_data.travel_cost, candidates;
                route_regularization_weight=master_data.route_regularization_weight,
                max_wait_time=master_data.max_wait_time,
                repositioning_time=master_data.repositioning_time,
                max_stops=master_data.max_stops,
                max_visits_per_node=master_data.max_visits_per_node,
                compensated_dominance=state.compensated_dominance)
            if !haskey(state.cluster_hierarchies, scenario)
                nstations = length(master_data.nodes)
                k0 = isnothing(state.cluster_initial_num_clusters) ? max(2, ceil(Int, nstations / 3)) : state.cluster_initial_num_clusters
                kmax = isnothing(state.cluster_max_num_clusters) ? nstations : state.cluster_max_num_clusters
                cfg = StationClusteringConfig(initial_num_clusters=k0,
                    max_num_clusters=kmax, max_cluster_size=state.cluster_max_size,
                    pricing_tolerance=state.reduced_cost_tol, time_limit=state.cluster_time_limit_sec)
                hierarchy = initial_station_clustering(master_data.travel_cost, nstations, cfg)
                state.cluster_hierarchies[scenario] = hierarchy
                state.cluster_caches[scenario] = build_cluster_pricing_cache(
                    hierarchy, physical, candidates)
            else
                refresh_cluster_rewards!(state.cluster_hierarchies[scenario],
                    state.cluster_caches[scenario], candidates)
            end
            hierarchy = state.cluster_hierarchies[scenario]
            cache = state.cluster_caches[scenario]
            k_before = length(hierarchy.clusters)
            ct0 = time()
            cluster_result, cluster_stop = solve_adaptive_cluster_lower_bound(hierarchy, cache)
            cluster_seconds = time() - ct0
            push!(state.cluster_certificate_rows, (
                round=state.n_rounds, iteration=state.n_iters, scenario=scenario,
                num_clusters_before=k_before, num_clusters_after=length(hierarchy.clusters),
                lower_bound_reduced_cost=cluster_result.lower_bound_reduced_cost,
                certified=cluster_result.certified_no_negative_column,
                stop_reason=string(cluster_stop),
                labels_generated=cluster_result.labels_generated,
                seconds=cluster_seconds, cluster_route=join(cluster_result.cluster_route, "-")))
            all_cluster_certified &= cluster_result.certified_no_negative_column
        end
        cluster_certified = all_cluster_certified
    end
    cert_columns, cert_exhausted, cert_labels = cluster_certified ?
        (PassengerFreeAssignmentRouteColumn[], true, 0) : _price_passenger_scenarios(
        master, alpha, gamma_o, gamma_d, state.next_column_id;
        n_candidates=typemax(Int) ÷ 2,
        max_new_columns=typemax(Int) ÷ 2,
        time_limit=cert_budget,
        reduced_cost_tol=state.reduced_cost_tol,
        verify_reduced_costs=state.verify_reduced_costs,
        parallel_scenarios=state.parallel_scenarios,
        station_budget_cap=state.station_budget_cap,
        compensated_dominance=state.compensated_dominance,
        use_station_simple=state.pricing_station_simple,
        reward_coarsening_levels=0,
        use_station_reduced_cost_filter=state.station_reduced_cost_filter_active,
        station_reduced_cost_filter_mode=state.station_reduced_cost_filter_mode,
        station_filter_excluded_stations=isnothing(cert_station_filter_stats) ?
            nothing : cert_station_filter_stats.excluded_stations,
        station_filter_excluded_opportunities=isnothing(cert_station_filter_stats) ?
            nothing : cert_station_filter_stats.excluded_opportunities,
        optimizer_env=state.optimizer_env,
    )
    round_cert_seconds = time() - t_cert
    state.certification_seconds += round_cert_seconds
    state.cert_exhausted = cert_exhausted
    state.total_labels += cert_labels
    state.next_column_id += length(cert_columns)

    cert_added = 0
    for c in cert_columns
        c.metadata["cg_source_solve_sequence"] = state.solve_sequence
        c.metadata["cg_source_round"] = state.n_rounds
        c.metadata["cg_source_iteration"] = state.n_iters
        c.metadata["cg_source_phase"] = "certification"
        c.metadata["cg_source_topl"] = sort!(collect(state.current_topl))
        _theta, action = add_passenger_free_assignment_column!(master, c)
        c.metadata["cg_add_action"] = String(action)
        _record_priced_route!(
            state.priced_route_rows, state.route_rho_rows, c, master_data,
            alpha, gamma_o, gamma_d, state.current_topl;
            solve_sequence=state.solve_sequence, round=state.n_rounds,
            iteration=state.n_iters, phase="certification", action=action,
        )
        action == :added && (cert_added += 1)
    end
    push!(state.iteration_rows, (
        round=state.n_rounds, iteration=state.n_iters, phase="certification",
        pricer=state.pricing_station_simple ? "station_simple" : "revisit",
        lp_bound=state.lp_bound, lp_seconds=0.0,
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
    state.verbose && println(
        "  [r$(state.n_rounds)] certification: $(length(cert_columns)) columns " *
        "($(cert_added) new), exhausted=$(cert_exhausted), " *
        "$(round(round_cert_seconds; digits=2))s",
    )
    flush(stdout)

    if state.warm_start_pending && (
            (isempty(cert_columns) && cert_exhausted) ||
            (!isempty(cert_columns) && cert_added == 0))
        # Elementary pricing can no longer improve the pool -- it either proved the
        # elementary universe exhausted (empty + exhausted) or now only re-finds
        # already-pooled columns. Switch to the exact revisit-tolerant pricer for
        # the rest of the run instead of stopping over the restricted elementary
        # pool. The LP is unchanged (nothing new was added), so the next round
        # re-prices the same duals with the exact pricer, which is what can still
        # find the revisiting columns the true optimum needs.
        state.warm_start_pending = false
        state.pricing_station_simple = false
        state.verbose && println(
            "  [r$(state.n_rounds)] station-simple warm start exhausted the elementary " *
            "universe; switching to the revisit-tolerant pricer",
        )
        state.n_iters >= state.max_cg_iters && return 0, cert_exhausted, :max_cg_iters
        time() - state.t_start > state.total_time_limit_sec && return 0, cert_exhausted, :total_time_limit
        state.phase = :early_return
        state.n_rounds += 1
        return 0, cert_exhausted, nothing
    end

    if isempty(cert_columns)
        # Exhaustive pricing found nothing. Only `exhausted` distinguishes a
        # genuine optimality certificate from a timeout.
        return 0, cert_exhausted, _cg_default_stop_reason(true, cert_exhausted)
    end
    if cert_added == 0
        # Everything exhaustive pricing found was already pooled -- no progress
        # is possible, so treat it the same as an uncertified stall rather than
        # looping forever.
        return cert_added, cert_exhausted, :no_progress
    end
    cert_added > 0 && state.n_iters >= state.max_cg_iters && return cert_added, cert_exhausted, :max_cg_iters
    time() - state.t_start > state.total_time_limit_sec &&
        return cert_added, cert_exhausted, :total_time_limit
    # Certification found genuinely new columns: resume the early-return phase.
    state.phase = :early_return
    state.n_rounds += 1
    return cert_added, cert_exhausted, nothing
end

"""
    _cg_price_and_add_columns!(data, rmp::PassengerFreeAssignmentMaster, algorithm::PassengerFreeAssignmentCG, solver, state, duals, iteration, lp_bound, lp_solve_seconds)

Dispatches on `state.phase` to `_pfa_price_early_return!`/`_pfa_price_certification!` -- the
central hook, mirroring the original's own early-return-vs-certification branch structure, now
expressed as this state machine instead of two nested `while`/`if` blocks. Every branch in the two
helper functions maps onto an existing `break`/`continue`/`cg_stop_reason=...` site in the
original function.
"""
function _cg_price_and_add_columns!(
    data::StationSelectionData,
    rmp::PassengerFreeAssignmentMaster,
    ::PassengerFreeAssignmentCG,
    solver::ColumnGenerationSolver,
    state::PassengerFreeAssignmentCGState,
    duals,
    iteration::Int,
    lp_bound::Float64,
    lp_solve_seconds::Float64,
)
    alpha, gamma_o, gamma_d = duals
    master_data = rmp.master_data
    if state.phase == :early_return
        return _pfa_price_early_return!(rmp, master_data, state, alpha, gamma_o, gamma_d, lp_solve_seconds)
    else
        return _pfa_price_certification!(rmp, master_data, state, alpha, gamma_o, gamma_d)
    end
end

"""
    _cg_finalize_result(data, rmp::PassengerFreeAssignmentMaster, algorithm::PassengerFreeAssignmentCG, formulation, solver, state, stop_reason, lp_bound) -> OptResult

The original function's final-MIP-and-metadata tail: binarize `y`/`theta`/`x_same`, solve the MIP
over the accumulated pool, write the CSV logs, then build the `OptResult` (with the same rich
metadata dict the original attached, including every diagnostic row vector) -- everything except
reshaping into `PassengerFreeAssignmentCGResult` itself, which the wrapper does (mirroring
`AggregateODRouteCG`'s own `_cg_finalize_result`/wrapper split).
"""
function _cg_finalize_result(
    data::StationSelectionData,
    rmp::PassengerFreeAssignmentMaster,
    algorithm::PassengerFreeAssignmentCG,
    formulation::AggregateODRouteModel,
    solver::ColumnGenerationSolver,
    state::PassengerFreeAssignmentCGState,
    stop_reason::Symbol,
    lp_bound::Float64,
)::OptResult
    master = rmp
    master_data = master.master_data
    m = master.model
    lp_bound_certified = stop_reason == :optimality_proven

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
    set_optimizer_attribute(m, "TimeLimit", solver.final_ip_time_limit_sec)
    isnothing(solver.config.mip_gap) || set_optimizer_attribute(m, "MIPGap", solver.config.mip_gap)
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
        source_solve_sequence=get(column.metadata, "cg_source_solve_sequence", missing),
        source_round=get(column.metadata, "cg_source_round", missing),
        source_iteration=get(column.metadata, "cg_source_iteration", missing),
        source_phase=get(column.metadata, "cg_source_phase", "seed"),
        add_action=get(column.metadata, "cg_add_action", "seed"),
        tau=column.tau,
        reduced_cost=get(column.metadata, "reduced_cost", missing),
        route=join(column.route, "-"),
        route_station_ids=join((data.array_idx_to_station_id[j] for j in column.route), "-"),
        n_stops=length(column.route),
        n_distinct_stations=length(unique(column.route)),
        n_revisits=length(column.route) - length(unique(column.route)),
        source_topl=join(get(column.metadata, "cg_source_topl", Int[]), ";"),
        n_stops_outside_source_topl=count(
            j -> !(j in get(column.metadata, "cg_source_topl", Int[])), unique(column.route)),
        assignments=join(("$p:$j:$k" for (p, j, k) in column.assignments), ";"),
        n_assignments=length(column.assignments),
    ) for (id, column) in sort!(collect(master.columns); by=first)]
    isnothing(algorithm.iteration_log_path) || _write_aggregate_od_route_cg_log_csv(
        algorithm.iteration_log_path, state.iteration_rows,
    )
    isnothing(algorithm.column_log_path) || _write_aggregate_od_route_cg_log_csv(
        algorithm.column_log_path, column_rows,
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
            "cg_iterations" => state.n_iters,
            "cg_rounds" => state.n_rounds,
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
    total_seconds = time() - state.t_start
    return OptResult(
        mip_term,
        mip_obj,
        solution,
        total_seconds,
        m,
        state.mapping,
        nothing,
        counts,
        nothing,
        Dict{String, Any}(
            "solve_method" => "column_generation",
            "column_generation_formulation" => "passenger_free_assignment",
            "assignment_policy" => "FreeAggregateODAssignmentPolicy",
            "cg_status" => String(status),
            "cg_stop_reason" => String(stop_reason),
            "lp_bound" => state.lp_bound,
            "lp_bound_certified" => lp_bound_certified,
            "cg_iterations" => state.n_iters,
            "cg_rounds" => state.n_rounds,
            "generated_columns" => length(master.theta),
            "seed_two_stop_columns" => state.n_seed_columns,
            "passengers" => length(master_data.passengers),
            "master_rows" => length(master.coverage) + length(master.pickup_link) + length(master.dropoff_link),
            "open_stations" => open_stations,
            "unserved_passengers" => unserved,
            "certification_seconds" => state.certification_seconds,
            "certification_exhausted" => state.cert_exhausted,
            "pricing_seconds" => state.total_pricing_seconds,
            "lp_seconds" => state.total_lp_seconds,
            "labels_generated" => state.total_labels,
            "reward_coarsening_levels" => state.reward_coarsening_levels,
            "station_simple_warm_start" => state.station_simple_warm_start,
            "iteration_rows" => copy(state.iteration_rows),
            "y_support_rows" => copy(state.y_support_rows),
            "y_station_rows" => copy(state.y_station_rows),
            "priced_route_rows" => copy(state.priced_route_rows),
            "route_rho_rows" => copy(state.route_rho_rows),
            "station_rho_score_rows" => copy(state.station_rho_score_rows),
            "opportunity_rho_rows" => copy(state.opportunity_rho_rows),
            "theta_rows" => copy(state.theta_rows),
            "theta_summary_rows" => copy(state.theta_summary_rows),
            "adaptive_cluster_certification" => state.use_adaptive_cluster_certification,
            "cluster_certificate_rows" => copy(state.cluster_certificate_rows),
            "exhaustive_pricing_each_iteration" => state.exhaustive_pricing_each_iteration,
            "theta_rho_subset_rows" => copy(state.theta_rho_subset_rows),
            "theta_rho_core_size" => state.theta_rho_core_size,
            "theta_rho_n_outsiders" => state.theta_rho_n_outsiders,
            "column_rows" => column_rows,
            "mean_positive_rho_used" => (state.pos_rho_samples == 0 ? 0.0 : state.pos_rho_used_sum / state.pos_rho_samples),
            "mean_raw_positive_rho" => (state.pos_rho_samples == 0 ? 0.0 : state.pos_rho_raw_sum / state.pos_rho_samples),
            "mean_positive_rho_stations_used" =>
                (state.pos_rho_samples == 0 ? 0.0 : state.pos_rho_stations_used_sum / state.pos_rho_samples),
            "mean_raw_positive_rho_stations" =>
                (state.pos_rho_samples == 0 ? 0.0 : state.pos_rho_stations_raw_sum / state.pos_rho_samples),
            "iteration_log_path" => algorithm.iteration_log_path,
            "column_log_path" => algorithm.column_log_path,
        ),
    )
end

function run_passenger_free_assignment_column_generation(
    model::AggregateODRouteModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    max_cg_iters::Int=1000,
    n_candidates::Int=100,
    max_new_columns::Int=20,
    exhaustive_pricing_each_iteration::Bool=false,
    theta_rho_core_size::Int=0,
    theta_rho_n_outsiders::Int=1,
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
    # Run the adaptive station-cluster lower bound before each exhaustive exact
    # certification pass.  A nonnegative bound for every scenario replaces that
    # exact pass; a negative bound falls back to the unchanged exact pricer.
    use_adaptive_cluster_certification::Bool=false,
    cluster_initial_num_clusters::Union{Int,Nothing}=nothing,
    cluster_max_num_clusters::Union{Int,Nothing}=nothing,
    cluster_max_size::Union{Int,Nothing}=nothing,
    cluster_time_limit_sec::Float64=60.0,
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
    n_candidates >= max_new_columns ||
        throw(ArgumentError("n_candidates must be >= max_new_columns"))
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
    theta_rho_core_size >= 0 || throw(ArgumentError("theta_rho_core_size must be nonnegative"))
    theta_rho_n_outsiders >= 0 || throw(ArgumentError("theta_rho_n_outsiders must be nonnegative"))

    resolved_env = isnothing(optimizer_env) ? Gurobi.Env() : optimizer_env
    solver = ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=resolved_env, silent=silent, mip_gap=mip_gap),
        max_iterations=max_cg_iters,
        max_columns_per_iteration=max_new_columns,
        n_candidates=n_candidates,
        reduced_cost_tol=reduced_cost_tol,
        pricing_time_limit_sec=pricing_time_limit_sec,
        final_ip_time_limit_sec=ip_time_limit_sec,
        algorithm=PassengerFreeAssignmentCG(
            exhaustive_pricing_each_iteration=exhaustive_pricing_each_iteration,
            theta_rho_core_size=theta_rho_core_size,
            theta_rho_n_outsiders=theta_rho_n_outsiders,
            certification_time_limit_sec=certification_time_limit_sec,
            total_time_limit_sec=total_time_limit_sec,
            seed_two_stop_routes=seed_two_stop_routes,
            parallel_scenarios=parallel_scenarios,
            station_budget_cap=station_budget_cap,
            compensated_dominance=compensated_dominance,
            use_station_simple=use_station_simple,
            station_simple_warm_start=station_simple_warm_start,
            reward_coarsening_levels=reward_coarsening_levels,
            use_station_reduced_cost_filter=use_station_reduced_cost_filter,
            station_reduced_cost_filter_mode=station_reduced_cost_filter_mode,
            use_adaptive_cluster_certification=use_adaptive_cluster_certification,
            cluster_initial_num_clusters=cluster_initial_num_clusters,
            cluster_max_num_clusters=cluster_max_num_clusters,
            cluster_max_size=cluster_max_size,
            cluster_time_limit_sec=cluster_time_limit_sec,
            unserved_penalty=unserved_penalty,
            verify_reduced_costs=verify_reduced_costs,
            verbose=verbose,
            iteration_log_path=iteration_log_path,
            column_log_path=column_log_path,
        ),
    )

    final_result, metadata, state = _run_column_generation(data, model, solver, solver.algorithm)
    cg_stop_reason = metadata["cg_stop_reason"]::Symbol
    lp_bound_certified = cg_stop_reason == :optimality_proven

    status = final_result.termination_status == MOI.OPTIMAL ?
        (lp_bound_certified ? :optimal : :feasible) :
        final_result.termination_status == MOI.TIME_LIMIT ? :timeout :
        final_result.termination_status == MOI.INFEASIBLE ? :infeasible : :error

    fm = final_result.metadata
    return PassengerFreeAssignmentCGResult(
        final_result, status, cg_stop_reason, state.lp_bound, lp_bound_certified,
        final_result.objective_value, final_result.termination_status,
        state.n_iters, state.n_rounds,
        fm["generated_columns"]::Int,
        fm["passengers"]::Int,
        fm["master_rows"]::Int,
        fm["open_stations"]::Vector{Int},
        fm["unserved_passengers"]::Vector{Int},
        state.certification_seconds, state.cert_exhausted,
        state.total_pricing_seconds, state.total_lp_seconds, state.total_labels,
        state.iteration_rows, final_result.runtime_sec,
        state.pos_rho_samples == 0 ? 0.0 : state.pos_rho_used_sum / state.pos_rho_samples,
        state.pos_rho_samples == 0 ? 0.0 : state.pos_rho_raw_sum / state.pos_rho_samples,
        state.pos_rho_samples == 0 ? 0.0 : state.pos_rho_stations_used_sum / state.pos_rho_samples,
        state.pos_rho_samples == 0 ? 0.0 : state.pos_rho_stations_raw_sum / state.pos_rho_samples,
    )
end
