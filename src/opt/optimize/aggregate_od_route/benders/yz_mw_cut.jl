"""
Cut-derivation companion for `BendersSolver{BendersYZ}` on `AggregateODRouteModel` with
`NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`: `:zero_completion` and
`:restricted_mw_fixed_pi` cut modes, mirroring `y_mw_cut.jl`'s design for `BendersY` but over a
simpler primal.

`BendersYZ`'s subproblem (`_build_yz_route_subproblem_lp`) fixes `z` directly via an equality
constraint, with **no other structural row on `z`** inside the subproblem (no `sum(z)==1`, no
Big-M row -- those live only in the master). So, unlike `BendersY`'s completion LP (which needs a
`lambda`/`mu`/`nu` block to relate `z` back to `y` through the Big-M chain), this completion LP
only needs the `alpha`/`rhoO`/`rhoD`/`sigma` block from the `x`-linking rows
(`_add_endpoint_x_linking!`, confirmed style-agnostic -- no `selector_style` parameter) plus the
route-covering duals fixed at `pi_full` (Sections C/D, reused unchanged from `y_mw_cut.jl`).

The core point for the Magnanti-Wong variant is *not* the independent per-chain simplex center a
first cut of this file used -- `z` is a real master variable here (unlike `BendersY`, where only
`y` is a master variable and `z` lives inside the subproblem), linked to `y` in the actual
`BendersYZ` master (`_add_nearest_open_master_z!`) via the exact same `sum(z)==1`/`z<=y`/Big-M-
ordering/nearest-open-lower-bound row family `_endpoint_big_m_variable!` builds, and every chain
competes with every other chain (and with the endpoint-coverage rows) for the same `sum(y)==l`
station budget. A point that is relative-interior for one chain's simplex in isolation need not be
anywhere near relative-interior for the *projection onto `z`* of the true joint `(y,z)` master
polytope -- so `_yz_joint_core_point` below builds that joint polytope directly (reproducing
`_endpoint_big_m_variable!`'s row family rather than calling it, since that function has
cache/model side effects not appropriate for a standalone structural-region LP) and runs the same
two-stage affine-hull-then-normalized-max-min-slack procedure `_y_master_core_point` runs for
`BendersY`'s `y`, jointly over every row (`y` bounds, endpoint rows, and every chain's `z` bounds,
`z<=y`, Big-M ordering, and nearest-open lower-bound rows) under one shared `delta`.

See notes/2026-07-17_restricted_mw_cut_benders_y.md for the BendersY derivation this mirrors, and
the `BendersYZ` docstring in `iterative_strategy_types.jl` for why repricing was previously the only
route to a provably-optimal result.
"""

# ---------------------------------------------------------------------------
# Core point: joint (y,z) structural region, exactly mirroring the real master's row family
# ---------------------------------------------------------------------------

struct AggregateODRouteYZCorePoint
    y::Vector{Float64}
    z::Dict{Any, Vector{Float64}}
    delta::Float64
    fixed_zero::Vector{Int}
    fixed_one::Vector{Int}
    n_endpoint_rows::Int
    n_always_tight_endpoint_rows::Int
    n_z_rows::Int
    n_always_tight_z_rows::Int
end

"""
    _yz_joint_core_point(data, model, requests, optimizer_env, silent; kwargs...) -> AggregateODRouteYZCorePoint

Builds `{sum(y)==l, 0<=y<=1, sum_{j in row} y_j>=1 for every permanent endpoint row} ×
{per chain: sum(z)==1, 0<=z<=1, z[idx]<=y[station], selected_cost<=cost[idx]+M[idx]*(1-y[station]),
z[idx]>=y[station]-sum(y[cheaper stations in chain])}` -- the exact `:big_m_nearest` row family
`_endpoint_big_m_variable!` builds in the real master, reproduced directly here (tie-break-adjusted
costs included, using the identical formula) rather than via that function, plus the endpoint rows
`_add_default_endpoint_coverage_constraints!` adds. Then, jointly over every row from both `y` and
every chain's `z`: (B1) an affine-hull analysis (max slack per row; rows whose max slack is within
`affine_hull_tol` of zero are always-tight/structurally-fixed) followed by (B2) one normalized
max-min-slack LP (`maximize delta s.t. s_i(y,z) >= delta*s_i^max` for every row with positive max
slack, shared `delta` across `y` and `z` rows alike). Computed once per outer `BendersYZ` solve
(does not depend on `y_hat`/`z_hat`).
"""
function _yz_joint_core_point(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    optimizer_env,
    silent::Bool;
    affine_hull_tol::Float64=1e-7,
    core_point_tol::Float64=1e-7,
)::AggregateODRouteYZCorePoint
    base = _base_aggregate_od_route_model(model)
    n = data.n_stations
    endpoint_rows = _restricted_mw_endpoint_rows(data, model, requests)
    chains = _restricted_mw_chains(data, model, requests)

    lp = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(lp)
    @variable(lp, 0 <= y[1:n] <= 1)
    @constraint(lp, sum(y) == base.l)
    for row in endpoint_rows
        @constraint(lp, sum(y[j] for j in row) >= 1.0)
    end

    z = Dict{Any, Vector{VariableRef}}()
    # Every non-equality row touching z, as `expr(y,z) >= 0`, so B1/B2 below can treat them
    # uniformly with y's own bound/endpoint rows.
    z_slack_exprs = AffExpr[]
    for (key, chain) in chains
        n_chain = length(chain.stations)
        zvar = @variable(lp, [1:n_chain], lower_bound = 0.0, upper_bound = 1.0)
        z[key] = zvar
        @constraint(lp, sum(zvar) == 1.0)
        # Matches `_endpoint_big_m_variable!` (aggregate_od_route.jl) exactly, including its
        # tie-break perturbation -- this must be the identical row family the real master builds,
        # not merely an equivalent-looking one, for the resulting core point to mean anything.
        tie_break_scale = max(1e-4, maximum(abs, chain.costs; init=0.0) * 1e-6)
        tb_costs = [chain.costs[idx] + tie_break_scale * (idx - 1) for idx in 1:n_chain]
        max_cost = maximum(tb_costs)
        selected_cost = sum(tb_costs[idx] * zvar[idx] for idx in 1:n_chain)
        for (idx, station) in enumerate(chain.stations)
            big_m = max_cost - tb_costs[idx]
            @constraint(lp, zvar[idx] <= y[station])
            @constraint(lp, selected_cost <= tb_costs[idx] + big_m * (1.0 - y[station]))
            cheaper_sum = sum(y[chain.stations[p]] for p in 1:(idx - 1); init=0.0)
            @constraint(lp, zvar[idx] >= y[station] - cheaper_sum)

            push!(z_slack_exprs, 1.0 * zvar[idx])                                      # zvar >= 0
            push!(z_slack_exprs, 1.0 - zvar[idx])                                      # zvar <= 1
            push!(z_slack_exprs, y[station] - zvar[idx])                               # zvar <= y[station]
            push!(z_slack_exprs, tb_costs[idx] + big_m * (1.0 - y[station]) - selected_cost)  # Big-M ordering
            push!(z_slack_exprs, zvar[idx] - y[station] + cheaper_sum)                 # nearest-open lower bound
        end
    end

    function _max_slack(expr)
        @objective(lp, Max, expr)
        optimize!(lp)
        primal_status(lp) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersYZ joint core-point affine-hull LP failed with status $(termination_status(lp))"))
        return objective_value(lp)
    end

    lb_slack_max = [_max_slack(1.0 * y[j]) for j in 1:n]
    ub_slack_max = [_max_slack(1.0 - y[j]) for j in 1:n]
    endpoint_slack_max = [_max_slack(sum(y[j] for j in row) - 1.0) for row in endpoint_rows]
    z_slack_max = [_max_slack(expr) for expr in z_slack_exprs]

    fixed_zero = [j for j in 1:n if lb_slack_max[j] <= affine_hull_tol]
    fixed_one = [j for j in 1:n if ub_slack_max[j] <= affine_hull_tol]
    n_always_tight_endpoint = count(<=(affine_hull_tol), endpoint_slack_max)
    n_always_tight_z = count(<=(affine_hull_tol), z_slack_max)

    @variable(lp, 0 <= delta <= 1)
    @objective(lp, Max, delta)
    for j in 1:n
        lb_slack_max[j] > affine_hull_tol && @constraint(lp, y[j] >= delta * lb_slack_max[j])
        ub_slack_max[j] > affine_hull_tol && @constraint(lp, 1.0 - y[j] >= delta * ub_slack_max[j])
    end
    for (i, row) in enumerate(endpoint_rows)
        endpoint_slack_max[i] > affine_hull_tol || continue
        @constraint(lp, sum(y[j] for j in row) - 1.0 >= delta * endpoint_slack_max[i])
    end
    for (i, expr) in enumerate(z_slack_exprs)
        z_slack_max[i] > affine_hull_tol || continue
        @constraint(lp, expr >= delta * z_slack_max[i])
    end
    optimize!(lp)
    primal_status(lp) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("BendersYZ joint core-point normalized max-min-slack LP failed with status $(termination_status(lp))"))
    delta_val = value(delta)
    y_core = [value(y[j]) for j in 1:n]
    z_core = Dict{Any, Vector{Float64}}(key => [value(zv) for zv in vars] for (key, vars) in z)

    delta_val > core_point_tol || @warn "BendersYZ joint core point: delta is at/near zero -- " *
        "no strictly relative-interior point could be certified; using this boundary point" delta = delta_val fixed_zero fixed_one

    return AggregateODRouteYZCorePoint(
        y_core, z_core, delta_val, fixed_zero, fixed_one,
        length(endpoint_rows), n_always_tight_endpoint, length(z_slack_exprs), n_always_tight_z,
    )
end

# ---------------------------------------------------------------------------
# Completion LP: alpha/rhoO/rhoD/sigma only (no lambda/mu/nu -- z has no chain structure here)
# ---------------------------------------------------------------------------

function _yz_phi_expr(
    z_point::Dict{Any, Vector{Float64}},
    alpha::Dict{NTuple{3, Int}, VariableRef},
    rhoO::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
    rhoD::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
    sigma::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    pk_key_of::Dict{NTuple{3, Int}, Any},
    dp_key_of::Dict{NTuple{3, Int}, Any},
    pk_rank_of::Dict{NTuple{3, Int}, Dict{Int, Int}},
    dp_rank_of::Dict{NTuple{3, Int}, Dict{Int, Int}},
)::AffExpr
    expr = AffExpr(0.0)
    for p in requests
        add_to_expression!(expr, 1.0, alpha[p])
        for pair in feasible_pairs[p]
            is_walk_only_pair(pair) && continue
            j, k = pair
            zpk = z_point[pk_key_of[p]][pk_rank_of[p][j]]
            zdp = z_point[dp_key_of[p]][dp_rank_of[p][k]]
            add_to_expression!(expr, zpk + zdp - 1.0, sigma[(p, pair)])
            add_to_expression!(expr, -zpk, rhoO[(p, pair)])
            add_to_expression!(expr, -zdp, rhoD[(p, pair)])
        end
    end
    return expr
end

"""
    _yz_completion_lp(data, model, requests, feasible_pairs, z_hat, z_core, pi_full, Q_bar,
                       objective_mode, optimizer_env, silent)

Builds the dual-feasibility LP over `alpha` (free, row 1), `rhoO`/`rhoD`/`sigma` (>=0, rows 2-4 of
`_add_endpoint_x_linking!`), with `pi` fixed at `pi_full` (row 8), subject to `Phi(z_hat;d)==Q_bar`.
`objective_mode=:zero` gives the `:zero_completion` baseline (any dual-feasible completion tight at
`z_hat`); `:maximize_core` maximizes `Phi(z_core;d)`, the Magnanti-Wong-style refinement.
"""
function _yz_completion_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    z_core::Dict{Any, Vector{Float64}},
    pi_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
    Q_bar::Float64,
    objective_mode::Symbol,
    optimizer_env,
    silent::Bool,
)
    objective_mode in (:maximize_core, :zero) ||
        throw(ArgumentError("unsupported objective_mode $(objective_mode)"))
    base = _base_aggregate_od_route_model(model)

    pk_key_of = Dict{NTuple{3, Int}, Any}()
    dp_key_of = Dict{NTuple{3, Int}, Any}()
    pk_rank_of = Dict{NTuple{3, Int}, Dict{Int, Int}}()
    dp_rank_of = Dict{NTuple{3, Int}, Dict{Int, Int}}()
    for p in requests
        _s, o, d = p
        pk_key, pk_stations, _pk_costs = _sorted_endpoint_chain(data, o, base.max_walking_distance, :pickup)
        dp_key, dp_stations, _dp_costs = _sorted_endpoint_chain(data, d, base.max_walking_distance, :dropoff)
        pk_key_of[p] = pk_key
        dp_key_of[p] = dp_key
        pk_rank_of[p] = Dict(station => idx for (idx, station) in enumerate(pk_stations))
        dp_rank_of[p] = Dict(station => idx for (idx, station) in enumerate(dp_stations))
    end

    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)

    alpha = Dict{NTuple{3, Int}, VariableRef}(p => @variable(m) for p in requests)
    rhoO = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    rhoD = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    sigma = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        rhoO[(p, pair)] = @variable(m, lower_bound = 0.0)
        rhoD[(p, pair)] = @variable(m, lower_bound = 0.0)
        sigma[(p, pair)] = @variable(m, lower_bound = 0.0)
    end

    # x-dual constraint: identical algebra to `_restricted_mw_completion_lp`'s (rows 1/4/8 are
    # unaffected by whether z or y is the decomposition's fixed master variable).
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        c_walk = _assignment_pair_cost(data, p, pair; weight=base.walk_cost_weight)
        pi_val = pi_full[(p, pair)]
        @constraint(m, alpha[p] - rhoO[(p, pair)] - rhoD[(p, pair)] + sigma[(p, pair)] - pi_val <= c_walk)
    end

    z_hat_generic = Dict{Any, Vector{Float64}}(key => vals for (key, vals) in z_hat)
    phi_core_expr = _yz_phi_expr(
        z_core, alpha, rhoO, rhoD, sigma, requests, feasible_pairs, pk_key_of, dp_key_of, pk_rank_of, dp_rank_of,
    )
    phi_zhat_expr = _yz_phi_expr(
        z_hat_generic, alpha, rhoO, rhoD, sigma, requests, feasible_pairs, pk_key_of, dp_key_of, pk_rank_of, dp_rank_of,
    )
    @constraint(m, phi_zhat_expr == Q_bar)

    if objective_mode == :maximize_core
        @objective(m, Max, phi_core_expr)
    else
        @objective(m, Max, 0.0)
    end

    return (
        model=m, alpha=alpha, rhoO=rhoO, rhoD=rhoD, sigma=sigma,
        pk_key_of=pk_key_of, dp_key_of=dp_key_of, pk_rank_of=pk_rank_of, dp_rank_of=dp_rank_of,
        phi_core_expr=phi_core_expr, phi_zhat_expr=phi_zhat_expr,
    )
end

struct AggregateODRouteYZCompletion
    status::Symbol
    cut_constant::Float64
    beta::Dict{Tuple{Any, Int}, Float64}
    phi_core::Float64
    phi_zhat::Float64
    runtime_sec::Float64
end

function _solve_yz_completion(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    z_core::Dict{Any, Vector{Float64}},
    pi_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
    Q_bar::Float64,
    objective_mode::Symbol,
    optimizer_env,
    silent::Bool;
    tightness_tol::Float64=1e-5,
)::AggregateODRouteYZCompletion
    start = time()
    built = _yz_completion_lp(
        data, model, requests, feasible_pairs, z_hat, z_core, pi_full, Q_bar, objective_mode, optimizer_env, silent,
    )
    optimize!(built.model)
    runtime = time() - start
    if primal_status(built.model) != MOI.FEASIBLE_POINT
        return AggregateODRouteYZCompletion(:infeasible, NaN, Dict{Tuple{Any, Int}, Float64}(), NaN, NaN, runtime)
    end

    alpha_val = Dict(k => value(v) for (k, v) in built.alpha)
    rhoO_val = Dict(k => value(v) for (k, v) in built.rhoO)
    rhoD_val = Dict(k => value(v) for (k, v) in built.rhoD)
    sigma_val = Dict(k => value(v) for (k, v) in built.sigma)

    cut_constant = sum(values(alpha_val); init=0.0) - sum(values(sigma_val); init=0.0)
    beta = Dict{Tuple{Any, Int}, Float64}()
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        j, k = pair
        pk_key = built.pk_key_of[p]
        pk_idx = built.pk_rank_of[p][j]
        dp_key = built.dp_key_of[p]
        dp_idx = built.dp_rank_of[p][k]
        s_val = sigma_val[(p, pair)]
        beta[(pk_key, pk_idx)] = get(beta, (pk_key, pk_idx), 0.0) + s_val - rhoO_val[(p, pair)]
        beta[(dp_key, dp_idx)] = get(beta, (dp_key, dp_idx), 0.0) + s_val - rhoD_val[(p, pair)]
    end

    phi_core = cut_constant + sum((beta[key] * z_core[key[1]][key[2]] for key in keys(beta)); init=0.0)
    phi_zhat = cut_constant + sum((beta[key] * z_hat[key[1]][key[2]] for key in keys(beta)); init=0.0)
    isapprox(phi_zhat, Q_bar; atol=tightness_tol * max(1.0, abs(Q_bar))) || throw(ArgumentError(
        "restricted YZ completion: cut is not tight at z_hat -- cut_constant + beta'z_hat = " *
        "$(phi_zhat), Q_bar = $(Q_bar)"
    ))

    return AggregateODRouteYZCompletion(:optimal, cut_constant, beta, phi_core, phi_zhat, runtime)
end

# ---------------------------------------------------------------------------
# Assemble the cut
# ---------------------------------------------------------------------------

struct AggregateODRouteYZCutResult
    status::Symbol   # :ok or :completion_infeasible
    Q_bar::Float64
    cut_constant::Float64
    beta::Dict{Tuple{Any, Int}, Float64}
    n_routes::Int
    n_cg_iterations::Int
    completion_runtime_sec::Float64
    phi_core::Float64
    phi_core_baseline::Union{Nothing, Float64}
end

"""
    _restricted_yz_optimality_cut(data, model, solver, requests, feasible_pairs, z_hat,
                                   assignments, open_stations, z_core, optimizer_env,
                                   objective_mode; certified=nothing, Q_bar=nothing)

`BendersYZ` analogue of `_restricted_mw_optimality_cut`: certifies `pi_full`/`Q_bar` (reusing the
caller's if already computed), solves the requested completion (`:maximize_core` or `:zero`), and
(only for `:maximize_core`) also solves the `:zero` baseline to sanity-check
`Phi(z_core;d_star) >= Phi(z_core;d_baseline)`. Never mutates `master`.
"""
function _restricted_yz_optimality_cut(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    z_core::Dict{Any, Vector{Float64}},
    optimizer_env,
    objective_mode::Symbol;
    certified::Union{Nothing, AggregateODRouteCertifiedRouteCoveringDuals}=nothing,
    Q_bar::Union{Nothing, Float64}=nothing,
)::AggregateODRouteYZCutResult
    model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy &&
        model.assignment_policy.feasibility_cut_style == :big_m_nearest ||
        throw(ArgumentError("restricted YZ cut derivation only supports NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)"))
    model.allow_walk_only &&
        throw(ArgumentError("restricted YZ cut derivation does not support allow_walk_only=true"))

    if isnothing(certified) || isnothing(Q_bar)
        certified, Q_bar = _certified_qbar(data, model, solver, requests, assignments, open_stations)
    end
    pi_full = _zero_extended_pi(requests, feasible_pairs, assignments, certified.pi_by_request)

    silent = solver.config.silent
    completion = _solve_yz_completion(
        data, model, requests, feasible_pairs, z_hat, z_core, pi_full, Q_bar, objective_mode, optimizer_env, silent,
    )
    completion.status == :optimal || return AggregateODRouteYZCutResult(
        :completion_infeasible, Q_bar, NaN, Dict{Tuple{Any, Int}, Float64}(), length(certified.pool),
        certified.n_cg_iterations, completion.runtime_sec, NaN, nothing,
    )

    phi_core_baseline = nothing
    if objective_mode == :maximize_core
        baseline = _solve_yz_completion(
            data, model, requests, feasible_pairs, z_hat, z_core, pi_full, Q_bar, :zero, optimizer_env, silent,
        )
        if baseline.status == :optimal
            phi_core_baseline = baseline.phi_core
            completion.phi_core >= baseline.phi_core - 1e-4 * max(1.0, abs(baseline.phi_core)) ||
                @warn "restricted YZ cut: maximize-core completion's Phi(z_core) is worse than the " *
                    "zero-completion baseline's -- should not happen for a correctly-solved maximization" phi_core =
                    completion.phi_core baseline = baseline.phi_core
        end
    end

    return AggregateODRouteYZCutResult(
        :ok, Q_bar, completion.cut_constant, completion.beta, length(certified.pool),
        certified.n_cg_iterations, completion.runtime_sec, completion.phi_core, phi_core_baseline,
    )
end

"""
    _add_aggregate_od_route_benders_yz_optimality_cut!(master, theta, cut_id, data, model, solver,
        group_requests, feasible_pairs, z_hat, assignments, open_stations, z_core, optimizer_env,
        v_hat, rho; certified=nothing, Q_bar=nothing, certification_already_failed=false)

`BendersYZ` analogue of `_add_aggregate_od_route_benders_y_optimality_cut!`: dispatches on
`solver.cut_derivation`. `:standard` reproduces the pre-existing subgradient cut exactly. The two
restricted-completion modes attempt `_restricted_yz_optimality_cut` and fall back to the standard
cut (with a `@warn`) if the completion LP comes back infeasible or the derivation throws.
"""
function _add_aggregate_od_route_benders_yz_optimality_cut!(
    master::Model,
    theta,
    cut_id::Int,
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    z_core::Union{Nothing, Dict{Any, Vector{Float64}}},
    optimizer_env,
    v_hat::Float64,
    rho::AbstractDict;
    certified::Union{Nothing, AggregateODRouteCertifiedRouteCoveringDuals}=nothing,
    Q_bar::Union{Nothing, Float64}=nothing,
    certification_already_failed::Bool=false,
)
    chain_cache = master[:nearest_endpoint_chain_cache]
    if solver.cut_derivation == :standard
        @constraint(master, theta[cut_id] >= v_hat + sum(
            rho[(key, i)] * (chain_cache[key][i] - z_hat[key][i]) for (key, i) in keys(rho)
        ))
        return (
            mode=:standard, mw_status=:not_attempted, Q_bar=v_hat, phi_core=NaN,
            phi_core_baseline=NaN, completion_runtime_sec=0.0, n_routes=0,
            n_cg_iterations=0, fallback=false,
        )
    end

    objective_mode = solver.cut_derivation == :restricted_mw_fixed_pi ? :maximize_core : :zero
    yz_result = certification_already_failed ? nothing : try
        _restricted_yz_optimality_cut(
            data, model, solver, group_requests, feasible_pairs, z_hat, assignments,
            open_stations, z_core, optimizer_env, objective_mode;
            certified=certified, Q_bar=Q_bar,
        )
    catch err
        @warn "BendersYZ restricted cut derivation failed; falling back to the standard cut " *
            "for this (iteration, cut_id)" cut_id error = err
        nothing
    end

    if isnothing(yz_result) || yz_result.status != :ok
        @constraint(master, theta[cut_id] >= v_hat + sum(
            rho[(key, i)] * (chain_cache[key][i] - z_hat[key][i]) for (key, i) in keys(rho)
        ))
        return (
            mode=solver.cut_derivation, mw_status=isnothing(yz_result) ? :error : yz_result.status,
            Q_bar=v_hat, phi_core=NaN, phi_core_baseline=NaN, completion_runtime_sec=0.0,
            n_routes=0, n_cg_iterations=0, fallback=true,
        )
    end

    cut_constant = yz_result.cut_constant
    beta = yz_result.beta
    @constraint(master, theta[cut_id] >= cut_constant + sum(
        beta[key] * chain_cache[key[1]][key[2]] for key in keys(beta)
    ))
    return (
        mode=solver.cut_derivation, mw_status=:ok, Q_bar=yz_result.Q_bar, phi_core=yz_result.phi_core,
        phi_core_baseline=isnothing(yz_result.phi_core_baseline) ? NaN : yz_result.phi_core_baseline,
        completion_runtime_sec=yz_result.completion_runtime_sec, n_routes=yz_result.n_routes,
        n_cg_iterations=yz_result.n_cg_iterations, fallback=false,
    )
end
