"""
Restricted, fixed-pricing-dual Magnanti-Wong-style ("restricted dual-completion cut")
optimality-cut derivation for `BendersSolver{BendersY}` on `AggregateODRouteModel` with
`NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`.

Not a full Magnanti-Wong procedure: the route-covering dual block (`pi`) is fixed at the
vector certified by exact column-generation pricing on the *restricted*, fixed-assignment
route-covering problem; only the remaining (nearest-open selector / assignment-linking) dual
blocks are optimized, against a relative-interior core point of the y-master's permanent
structural region. Not claimed to be globally Pareto-optimal.

See notes/2026-07-17_restricted_mw_cut_benders_y.md for the full derivation and audit this
file implements.
"""

# ---------------------------------------------------------------------------
# Section B: relative-interior core point of the y-master's structural region
# ---------------------------------------------------------------------------

struct AggregateODRouteYCorePoint
    y::Vector{Float64}
    delta::Float64
    fixed_zero::Vector{Int}
    fixed_one::Vector{Int}
    n_endpoint_rows::Int
    n_always_tight_endpoint_rows::Int
end

"""
    _restricted_mw_endpoint_rows(data, model, requests) -> Vector{Vector{Int}}

Every distinct `sum_{j in candidates(endpoint,side)} y_j >= 1` row implied by `requests`
(deduplicated by candidate set). Derivable purely from precomputed walking-cost data, so this
is a *permanent* structural restriction, unlike the lazily-discovered feasibility cuts the
running BendersY loop adds only after hitting infeasibility for a given `y_hat`.
"""
function _restricted_mw_endpoint_rows(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
)::Vector{Vector{Int}}
    base = _base_aggregate_od_route_model(model)
    rows = Set{Vector{Int}}()
    for (_s, o, d) in requests
        push!(rows, sort(_nearest_open_endpoint_candidates(data, o, base.max_walking_distance, :pickup)))
        push!(rows, sort(_nearest_open_endpoint_candidates(data, d, base.max_walking_distance, :dropoff)))
    end
    return collect(rows)
end

"""
    _y_master_core_point(data, model, requests, optimizer_env, silent; kwargs...) -> AggregateODRouteYCorePoint

Section B: builds `Y_LP = {sum y = l, 0<=y<=1, sum_{j in row} y_j >= 1 for every permanent
endpoint row}` and solves (B1) an affine-hull analysis (max slack per inequality; rows whose
max slack is within `affine_hull_tol` of zero are always-tight / structurally-fixed variables)
followed by (B2) a single normalized max-min-slack LP (`maximize delta s.t. s_i(y) >= delta *
s_i^max` for every row with positive max slack). Computed once per outer BendersY solve (does
not depend on `y_hat`); B3's dynamic core-point blending is not implemented (spec allows
starting with the static point only).
"""
function _y_master_core_point(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    optimizer_env,
    silent::Bool;
    affine_hull_tol::Float64=1e-7,
    core_point_tol::Float64=1e-7,
)::AggregateODRouteYCorePoint
    base = _base_aggregate_od_route_model(model)
    n = data.n_stations
    endpoint_rows = _restricted_mw_endpoint_rows(data, model, requests)

    lp = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(lp)
    @variable(lp, 0 <= y[1:n] <= 1)
    @constraint(lp, sum(y) == base.l)
    for row in endpoint_rows
        @constraint(lp, sum(y[j] for j in row) >= 1.0)
    end

    function _max_slack(expr)
        @objective(lp, Max, expr)
        optimize!(lp)
        primal_status(lp) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersY core-point affine-hull LP failed with status $(termination_status(lp))"))
        return objective_value(lp)
    end

    lb_slack_max = [_max_slack(1.0 * y[j]) for j in 1:n]
    ub_slack_max = [_max_slack(1.0 - y[j]) for j in 1:n]
    endpoint_slack_max = [_max_slack(sum(y[j] for j in row) - 1.0) for row in endpoint_rows]

    fixed_zero = [j for j in 1:n if lb_slack_max[j] <= affine_hull_tol]
    fixed_one = [j for j in 1:n if ub_slack_max[j] <= affine_hull_tol]
    n_always_tight = count(<=(affine_hull_tol), endpoint_slack_max)

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
    optimize!(lp)
    primal_status(lp) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("BendersY core-point normalized max-min-slack LP failed with status $(termination_status(lp))"))
    delta_val = value(delta)
    y_core = [value(y[j]) for j in 1:n]

    delta_val > core_point_tol || @warn "BendersY restricted-MW core point: delta is at/near zero -- " *
        "no strictly relative-interior point could be certified; using this boundary point" delta = delta_val fixed_zero fixed_one

    return AggregateODRouteYCorePoint(y_core, delta_val, fixed_zero, fixed_one, length(endpoint_rows), n_always_tight)
end

# ---------------------------------------------------------------------------
# Sections C/D: certified route-covering duals and zero-extension
# ---------------------------------------------------------------------------

struct AggregateODRouteCertifiedRouteCoveringDuals
    pi_by_request::Dict{NTuple{3, Int}, Float64}
    r_value::Float64
    pool::Vector{AggregateODRouteColumn}
    n_cg_iterations::Int
    exact::Bool
end

"""
    _certified_route_covering_pi(data, model, assignments, open_stations, requests, solver, iteration)

Section C/D. Solves `R(x_bar)`, the fixed-assignment route-covering problem, by exact column
generation to `cg_stop_reason == :optimality_proven`, re-solves the LP relaxation on the
converged pool to extract per-request (not `(j,k,s)`-aggregated) coverage-row duals, then runs
one further exact label-setting pricing pass against those duals over every scenario touched
by `requests` to certify `min_r rc(r) >= -reduced_cost_tol` before accepting them (mirrors
`_solve_nearest_open_y_subproblem_lp_with_repricing`'s certification pattern). Throws rather
than returning an uncertified dual vector.
"""
function _certified_route_covering_pi(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    requests::Vector{NTuple{3, Int}},
    solver::BendersSolver,
)::AggregateODRouteCertifiedRouteCoveringDuals
    inner = solver.inner_solver
    inner isa ColumnGenerationSolver ||
        throw(ArgumentError("restricted MW cut derivation requires solver.inner_solver isa ColumnGenerationSolver"))
    cfg = inner.config
    optimizer_env = isnothing(cfg.optimizer_env) ? solver.config.optimizer_env : cfg.optimizer_env
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    silent = cfg.silent || solver.config.silent
    mip_gap = isnothing(cfg.mip_gap) ? solver.config.mip_gap : cfg.mip_gap

    route_problem = _route_covering_problem_from_assignments(model, assignments, open_stations)
    cg_result = run_aggregate_od_route_column_generation(
        route_problem, data;
        optimizer_env=optimizer_env, verbose=!silent,
        max_cg_iters=inner.max_iterations, max_new_columns=inner.max_columns_per_iteration,
        n_candidates=inner.n_candidates, reduced_cost_tol=inner.reduced_cost_tol,
        pricing_time_limit_sec=inner.pricing_time_limit_sec, ip_time_limit_sec=inner.final_ip_time_limit_sec,
        mip_gap=mip_gap, silent=silent,
    )
    cg_result.cg_stop_reason == :optimality_proven ||
        throw(ArgumentError(
            "restricted MW cut: fixed route-covering CG did not prove pricing exhaustion; " *
            "stop_reason=$(cg_result.cg_stop_reason)"
        ))

    pool = cg_result.generated_columns
    lp_problem = _copy_with_initial_columns(route_problem, pool; relax_integrality=true)
    build = build_model(lp_problem, data; optimizer_env=optimizer_env, relax_integrality=true)
    m = build.model
    silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("restricted MW cut: fixed route-covering LP re-solve failed with status $(termination_status(m))"))
    r_value = objective_value(m)
    isapprox(r_value, cg_result.lp_bound; atol=1e-6 * max(1.0, abs(cg_result.lp_bound))) ||
        throw(ArgumentError(
            "restricted MW cut: re-solved route-covering LP value $(r_value) does not match " *
            "CG's certified lp_bound $(cg_result.lp_bound)"
        ))

    mapping = build.mapping
    coverage = m[:aggregate_od_route_coverage_constraints]
    pi_by_request = Dict{NTuple{3, Int}, Float64}()
    for (key, con) in coverage
        _j, _k, s, od_idx, _pair_idx = key
        o, d = mapping.Omega_s[s][od_idx]
        request = (s, o, d)
        request in requests || continue
        pi_by_request[request] = dual(con)
    end
    # A request whose assigned pair `requires_no_vehicle_route` (same-station or walk-only) gets
    # no coverage row anywhere in the codebase (`add_aggregate_od_route_coverage_constraints!`,
    # `_singleton_aggregate_od_route_columns`, and this LP's own build all skip such pairs) --
    # there is genuinely no `pi` dual to certify for it, not a certification failure. Missing here
    # is therefore expected, not an error; `_zero_extended_pi` below zero-extends it, which is
    # correct since such a pair contributes no route/covering cost to `Q_bar` in the first place.
    for request in requests
        requires_no_vehicle_route(assignments[request]) && continue
        haskey(pi_by_request, request) ||
            throw(ArgumentError("restricted MW cut: fixed route-covering LP has no coverage row for request $(request)"))
    end

    duals = extract_aggregate_od_route_coverage_duals(m)
    scenarios_touched = sort!(unique!([s for (s, _o, _d) in requests]))
    for s in scenarios_touched
        pricing_duals = _scenario_pricing_duals(duals, s)
        pricing_data = create_aggregate_od_route_pricing_data(lp_problem, data, mapping, s, pricing_duals)
        new_columns, exhausted, _stats = aggregate_od_route_pricing_by_label_setting(
            pricing_data, pool, pricing_duals;
            next_column_id=(isempty(pool) ? 1 : maximum(c.id for c in pool) + 1),
            reduced_cost_tol=lp_problem.reduced_cost_tol, max_new_columns=lp_problem.max_new_columns,
            n_candidates=lp_problem.n_candidates, time_limit=lp_problem.pricing_time_limit_sec,
            max_visits_per_node=lp_problem.max_visits_per_node,
        )
        exhausted ||
            throw(ArgumentError("restricted MW cut: certification pricing pass hit its time limit for scenario $(s)"))
        isempty(new_columns) ||
            throw(ArgumentError(
                "restricted MW cut: certification pricing pass found $(length(new_columns)) improving " *
                "column(s) beyond the CG-converged pool for scenario $(s) -- pool was not actually complete"
            ))
    end

    return AggregateODRouteCertifiedRouteCoveringDuals(pi_by_request, r_value, pool, cg_result.n_cg_iters, true)
end

"""
    _zero_extended_pi(requests, feasible_pairs, assignments, pi_by_request) -> Dict

Section D: zero-extends the certified, retained-row-only route-covering duals over every
`(request, pair)` in `feasible_pairs`, not just the one retained (assigned) pair per request.
Missing components are exactly the pairs `R(x_bar)` never built a coverage row for (`x_bar=0`
there, or `x_bar=1` but `requires_no_vehicle_route` -- same-station/walk-only assignments get no
coverage row even when assigned, see `_certified_route_covering_pi`); zero credit there cannot
change any route's reduced cost, since reduced cost only sums dual credit over the pairs a route
actually serves, and a `requires_no_vehicle_route` pair is never served by any route regardless.
"""
function _zero_extended_pi(
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    pi_by_request::Dict{NTuple{3, Int}, Float64},
)::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64}
    pi_full = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64}()
    for p in requests
        assigned = assignments[p]
        assigned_has_coverage_row = !requires_no_vehicle_route(assigned)
        for pair in feasible_pairs[p]
            is_walk_only_pair(pair) && continue
            pi_full[(p, pair)] = (pair == assigned && assigned_has_coverage_row) ? pi_by_request[p] : 0.0
        end
    end
    return pi_full
end

# ---------------------------------------------------------------------------
# Sections E-G: restricted completion LP
# ---------------------------------------------------------------------------

struct _RestrictedMWChain
    side::Symbol
    stations::Vector{Int}
    costs::Vector{Float64}
end

function _sorted_endpoint_chain(
    data::StationSelectionData,
    endpoint::Int,
    max_walking_distance::Float64,
    side::Symbol,
)
    candidates = _nearest_open_endpoint_candidates(data, endpoint, max_walking_distance, side)
    costs = [
        side == :pickup ? get_walking_cost(data, endpoint, j) : get_walking_cost(data, j, endpoint)
        for j in candidates
    ]
    order = sortperm(collect(eachindex(candidates)); by=i -> (costs[i], candidates[i]))
    sorted_stations = candidates[order]
    sorted_costs = costs[order]
    key = _endpoint_chain_key(side, sorted_stations, sorted_costs)
    return key, sorted_stations, sorted_costs
end

"""
    _restricted_mw_chains(data, model, requests) -> Dict{Any, _RestrictedMWChain}

Builds the exact same `(side, sorted-candidates, sorted-costs)`-keyed chain grouping the real
primal build (`_endpoint_big_m_variable!`) uses -- necessary for correctness, not just
convenience: a completion LP that gave two physical endpoints with an identical candidate/cost
profile *separate* dual variables would be solving the dual of a different (unshared) primal,
and its completed dual would not actually be feasible for the real, shared-variable subproblem.
"""
function _restricted_mw_chains(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
)::Dict{Any, _RestrictedMWChain}
    base = _base_aggregate_od_route_model(model)
    mwd = base.max_walking_distance
    chains = Dict{Any, _RestrictedMWChain}()
    for (_s, o, d) in requests
        for (side, endpoint) in ((:pickup, o), (:dropoff, d))
            key, stations, costs = _sorted_endpoint_chain(data, endpoint, mwd, side)
            haskey(chains, key) || (chains[key] = _RestrictedMWChain(side, stations, costs))
        end
    end
    return chains
end

function _restricted_mw_phi_expr(
    y_point::Vector{Float64},
    chains::Dict{Any, _RestrictedMWChain},
    lambda::Dict{Any, VariableRef},
    mu::Dict{Tuple{Any, Int}, VariableRef},
    nu::Dict{Tuple{Any, Int}, VariableRef},
    alpha::Dict{NTuple{3, Int}, VariableRef},
    sigma::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
)::AffExpr
    expr = AffExpr(0.0)
    for p in requests
        add_to_expression!(expr, 1.0, alpha[p])
        for pair in feasible_pairs[p]
            is_walk_only_pair(pair) && continue
            add_to_expression!(expr, -1.0, sigma[(p, pair)])
        end
    end
    for (key, chain) in chains
        add_to_expression!(expr, 1.0, lambda[key])
        max_cost = maximum(chain.costs)
        for (idx, station) in enumerate(chain.stations)
            cost = chain.costs[idx]
            m_big = max_cost - cost
            add_to_expression!(expr, -y_point[station], mu[(key, idx)])
            add_to_expression!(expr, -(cost + m_big), nu[(key, idx)])
            add_to_expression!(expr, m_big * y_point[station], nu[(key, idx)])
        end
    end
    return expr
end

"""
    _restricted_mw_completion_lp(...)

Sections E-G. Builds the dual-feasibility LP of `_build_nearest_open_y_subproblem_lp`'s primal
(with `y` symbolic, not fixed -- see the note for the row-by-row sign derivation), with `pi`
fixed at `pi_full`, and either maximizes `Phi(y_core; d)` (`objective_mode=:maximize_core`) or
uses a zero objective (`objective_mode=:zero`, the `:zero_completion` baseline), subject to
`Phi(y_hat; d) == Q_bar`.
"""
function _restricted_mw_completion_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    y_hat::Vector{Float64},
    y_core::Vector{Float64},
    pi_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
    Q_bar::Float64,
    objective_mode::Symbol,
    optimizer_env,
    silent::Bool,
)
    objective_mode in (:maximize_core, :zero) ||
        throw(ArgumentError("unsupported objective_mode $(objective_mode)"))
    base = _base_aggregate_od_route_model(model)
    chains = _restricted_mw_chains(data, model, requests)

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
        pk_rank_of[p] = Dict(s => i for (i, s) in enumerate(pk_stations))
        dp_rank_of[p] = Dict(s => i for (i, s) in enumerate(dp_stations))
    end

    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)

    lambda = Dict{Any, VariableRef}(key => @variable(m) for key in keys(chains))
    mu = Dict{Tuple{Any, Int}, VariableRef}()
    nu = Dict{Tuple{Any, Int}, VariableRef}()
    for (key, chain) in chains, idx in eachindex(chain.stations)
        mu[(key, idx)] = @variable(m, lower_bound = 0.0)
        nu[(key, idx)] = @variable(m, lower_bound = 0.0)
    end
    alpha = Dict(p => @variable(m) for p in requests)
    rhoO = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    rhoD = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    sigma = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        rhoO[(p, pair)] = @variable(m, lower_bound = 0.0)
        rhoD[(p, pair)] = @variable(m, lower_bound = 0.0)
        sigma[(p, pair)] = @variable(m, lower_bound = 0.0)
    end

    # x-dual constraints: alpha[p] - rhoO - rhoD + sigma - pi_full <= c_walk
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        c_walk = _assignment_pair_cost(data, p, pair; weight=base.walk_cost_weight)
        pi_val = pi_full[(p, pair)]
        @constraint(m, alpha[p] - rhoO[(p, pair)] - rhoD[(p, pair)] + sigma[(p, pair)] - pi_val <= c_walk)
    end

    # Precompute, per (chain key, rank), the (request, pair) terms that reference that z index --
    # avoids an O(chains * candidates * requests * pairs) nested scan.
    pickup_terms = Dict{Tuple{Any, Int}, Vector{Tuple{NTuple{3, Int}, Tuple{Int, Int}}}}()
    dropoff_terms = Dict{Tuple{Any, Int}, Vector{Tuple{NTuple{3, Int}, Tuple{Int, Int}}}}()
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        j, k = pair
        push!(get!(pickup_terms, (pk_key_of[p], pk_rank_of[p][j]), Tuple{NTuple{3, Int}, Tuple{Int, Int}}[]), (p, pair))
        push!(get!(dropoff_terms, (dp_key_of[p], dp_rank_of[p][k]), Tuple{NTuple{3, Int}, Tuple{Int, Int}}[]), (p, pair))
    end

    # z-dual constraints: lambda - mu - cost*sum(nu over chain) + role-specific (rhoO/rhoD - sigma) <= 0
    for (key, chain) in chains
        max_cost = maximum(chain.costs)
        nu_sum = sum(nu[(key, idx2)] for idx2 in eachindex(chain.stations))
        for (idx, _station) in enumerate(chain.stations)
            cost = chain.costs[idx]
            expr = AffExpr(0.0)
            add_to_expression!(expr, 1.0, lambda[key])
            add_to_expression!(expr, -1.0, mu[(key, idx)])
            add_to_expression!(expr, -cost, nu_sum)
            terms = chain.side == :pickup ? get(pickup_terms, (key, idx), Tuple{NTuple{3, Int}, Tuple{Int, Int}}[]) :
                    get(dropoff_terms, (key, idx), Tuple{NTuple{3, Int}, Tuple{Int, Int}}[])
            for (p, pair) in terms
                if chain.side == :pickup
                    add_to_expression!(expr, 1.0, rhoO[(p, pair)])
                else
                    add_to_expression!(expr, 1.0, rhoD[(p, pair)])
                end
                add_to_expression!(expr, -1.0, sigma[(p, pair)])
            end
            @constraint(m, expr <= 0.0)
        end
    end

    phi_core_expr = _restricted_mw_phi_expr(y_core, chains, lambda, mu, nu, alpha, sigma, requests, feasible_pairs)
    phi_ybar_expr = _restricted_mw_phi_expr(y_hat, chains, lambda, mu, nu, alpha, sigma, requests, feasible_pairs)
    @constraint(m, phi_ybar_expr == Q_bar)

    if objective_mode == :maximize_core
        @objective(m, Max, phi_core_expr)
    else
        @objective(m, Max, 0.0)
    end

    return (
        model=m, lambda=lambda, mu=mu, nu=nu, alpha=alpha, rhoO=rhoO, rhoD=rhoD, sigma=sigma,
        chains=chains, phi_core_expr=phi_core_expr, phi_ybar_expr=phi_ybar_expr,
    )
end

struct AggregateODRouteRestrictedMWCompletion
    status::Symbol
    cut_constant::Float64
    beta::Dict{Int, Float64}
    phi_core::Float64
    phi_ybar::Float64
    runtime_sec::Float64
end

function _solve_restricted_mw_completion(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    y_hat::Vector{Float64},
    y_core::Vector{Float64},
    pi_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
    Q_bar::Float64,
    objective_mode::Symbol,
    optimizer_env,
    silent::Bool;
    tightness_tol::Float64=1e-5,
)::AggregateODRouteRestrictedMWCompletion
    start = time()
    built = _restricted_mw_completion_lp(
        data, model, requests, feasible_pairs, y_hat, y_core, pi_full, Q_bar, objective_mode, optimizer_env, silent,
    )
    optimize!(built.model)
    runtime = time() - start
    if primal_status(built.model) != MOI.FEASIBLE_POINT
        return AggregateODRouteRestrictedMWCompletion(:infeasible, NaN, Dict{Int, Float64}(), NaN, NaN, runtime)
    end

    lambda_val = Dict(k => value(v) for (k, v) in built.lambda)
    mu_val = Dict(k => value(v) for (k, v) in built.mu)
    nu_val = Dict(k => value(v) for (k, v) in built.nu)
    alpha_val = Dict(k => value(v) for (k, v) in built.alpha)
    sigma_val = Dict(k => value(v) for (k, v) in built.sigma)

    n = data.n_stations
    cut_constant = 0.0
    beta = Dict{Int, Float64}(j => 0.0 for j in 1:n)
    for p in requests
        cut_constant += alpha_val[p]
        for pair in feasible_pairs[p]
            is_walk_only_pair(pair) && continue
            cut_constant -= sigma_val[(p, pair)]
        end
    end
    for (key, chain) in built.chains
        cut_constant += lambda_val[key]
        max_cost = maximum(chain.costs)
        for (idx, station) in enumerate(chain.stations)
            cost = chain.costs[idx]
            m_big = max_cost - cost
            cut_constant -= (cost + m_big) * nu_val[(key, idx)]
            beta[station] -= mu_val[(key, idx)]
            beta[station] += m_big * nu_val[(key, idx)]
        end
    end

    phi_core = cut_constant + sum(beta[j] * y_core[j] for j in 1:n)
    phi_ybar = cut_constant + sum(beta[j] * y_hat[j] for j in 1:n)
    isapprox(phi_ybar, Q_bar; atol=tightness_tol * max(1.0, abs(Q_bar))) || throw(ArgumentError(
        "restricted MW completion: cut is not tight at y_hat -- cut_constant + beta'y_hat = " *
        "$(phi_ybar), Q_bar = $(Q_bar)"
    ))

    return AggregateODRouteRestrictedMWCompletion(:optimal, cut_constant, beta, phi_core, phi_ybar, runtime)
end

# ---------------------------------------------------------------------------
# Section H: assemble and (by the caller) add the cut
# ---------------------------------------------------------------------------

struct AggregateODRouteRestrictedMWCutResult
    status::Symbol   # :ok or :completion_infeasible
    Q_bar::Float64
    cut_constant::Float64
    beta::Dict{Int, Float64}
    n_routes::Int
    n_cg_iterations::Int
    completion_runtime_sec::Float64
    phi_core::Float64
    phi_core_baseline::Union{Nothing, Float64}
end

"""
    _certified_qbar(data, model, solver, requests, assignments, open_stations)

Section C's certified `R(x_bar)` solve plus `Q_bar = sum(c_walk*x_bar) + R(x_bar)`, split out
from `_restricted_mw_optimality_cut` so the caller can compute the *true*, pool-complete value
of a fixed `y_hat` **before** deciding whether a cut is even needed. This matters: the
pre-existing `theta_hat < v_hat - tol` gate uses `v_hat` from
`_solve_nearest_open_y_subproblem_lp` (or its repriced variant), which is only as good as
`shared_pool`'s completeness *for that specific `y_hat`* when `reprice_subproblem=false` --
an incomplete pool can only ever *inflate* `v_hat` (fewer columns can't reduce covering cost),
never deflate it, so an inflated `v_hat` can make the master think it has already converged
(`theta_hat >= v_hat - tol`) when it has not, and the cut-derivation code below this point never
even runs. `Q_bar` here is independent of `shared_pool`/`reprice_subproblem` -- it is *always*
backed by a from-scratch, exactly-certified CG solve on the narrower, fixed-assignment
`R(x_bar)` -- so replacing `v_hat` with `min(v_hat, Q_bar)` for the gating decision closes that
gap for the `:zero_completion`/`:restricted_mw_fixed_pi` modes without requiring
`reprice_subproblem=true`. See notes/2026-07-17_restricted_mw_cut_benders_y.md.
"""
function _certified_qbar(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
    requests::Vector{NTuple{3, Int}},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
)::Tuple{AggregateODRouteCertifiedRouteCoveringDuals, Float64}
    certified = _certified_route_covering_pi(data, model, assignments, open_stations, requests, solver)
    # `certified.r_value` is `RouteCoveringProblem`'s own LP objective value, which -- via the
    # *same* `set_aggregate_od_route_objective!` every AggregateODRouteModel build uses -- already
    # sums BOTH the (forced-constant, since x is fixed to `assignments`) walking-cost terms AND
    # the route-covering terms. It is already `Q_bar = sum(c_walk*x_bar) + R(x_bar)`, not `R(x_bar)`
    # alone; adding the walking-cost sum again here would double-count it.
    Q_bar = certified.r_value
    return certified, Q_bar
end

"""
    _restricted_mw_optimality_cut(data, model, solver, requests, feasible_pairs, y_hat,
                                   assignments, open_stations, y_core, optimizer_env,
                                   objective_mode; certified=nothing, Q_bar=nothing)

Sections C-H end to end for one cut group: certifies `pi_full`/`Q_bar` (reusing the
already-computed `certified`/`Q_bar` if the caller passed them, e.g. from an earlier
`_certified_qbar` call used for the gating decision, rather than re-running CG), solves the
requested completion (`objective_mode ∈ (:maximize_core, :zero)`), and (only for
`:maximize_core`) also solves the `:zero` baseline purely to verify
`Phi(y_core;d_star) >= Phi(y_core;d_baseline)` before returning -- both completions reuse the
same fixed `pi_full`/`Q_bar`, so this is one extra small LP solve, not another pricing pass
(`n_pricing_calls_during_completion` is always zero: Sections E-G never call the pricing
oracle). Never mutates `master`; the caller adds the JuMP constraint and handles the
`:completion_infeasible` fallback to the standard cut.
"""
function _restricted_mw_optimality_cut(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    y_hat::Vector{Float64},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    y_core::Vector{Float64},
    optimizer_env,
    objective_mode::Symbol;
    certified::Union{Nothing, AggregateODRouteCertifiedRouteCoveringDuals}=nothing,
    Q_bar::Union{Nothing, Float64}=nothing,
)::AggregateODRouteRestrictedMWCutResult
    model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy &&
        model.assignment_policy.feasibility_cut_style == :big_m_nearest ||
        throw(ArgumentError("restricted MW cut derivation only supports NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)"))
    model.allow_walk_only &&
        throw(ArgumentError("restricted MW cut derivation does not support allow_walk_only=true"))

    if isnothing(certified) || isnothing(Q_bar)
        certified, Q_bar = _certified_qbar(data, model, solver, requests, assignments, open_stations)
    end
    pi_full = _zero_extended_pi(requests, feasible_pairs, assignments, certified.pi_by_request)

    silent = solver.config.silent
    completion = _solve_restricted_mw_completion(
        data, model, requests, feasible_pairs, y_hat, y_core, pi_full, Q_bar, objective_mode, optimizer_env, silent,
    )
    completion.status == :optimal || return AggregateODRouteRestrictedMWCutResult(
        :completion_infeasible, Q_bar, NaN, Dict{Int, Float64}(), length(certified.pool),
        certified.n_cg_iterations, completion.runtime_sec, NaN, nothing,
    )

    phi_core_baseline = nothing
    if objective_mode == :maximize_core
        baseline = _solve_restricted_mw_completion(
            data, model, requests, feasible_pairs, y_hat, y_core, pi_full, Q_bar, :zero, optimizer_env, silent,
        )
        if baseline.status == :optimal
            phi_core_baseline = baseline.phi_core
            completion.phi_core >= baseline.phi_core - 1e-4 * max(1.0, abs(baseline.phi_core)) ||
                @warn "restricted MW cut: maximize-core completion's Phi(y_core) is worse than the " *
                    "zero-completion baseline's -- should not happen for a correctly-solved maximization" phi_core =
                    completion.phi_core baseline = baseline.phi_core
        end
    end

    return AggregateODRouteRestrictedMWCutResult(
        :ok, Q_bar, completion.cut_constant, completion.beta, length(certified.pool),
        certified.n_cg_iterations, completion.runtime_sec, completion.phi_core, phi_core_baseline,
    )
end

"""
    _add_aggregate_od_route_benders_y_optimality_cut!(master, y, theta, cut_id, data, model,
        solver, group_requests, feasible_pairs, y_hat, assignments, open_stations, y_core,
        optimizer_env, v_hat, rho)

Adds one optimality cut for `cut_id` to `master`, dispatching on `solver.cut_derivation`.
`:standard` reproduces the pre-existing subgradient cut exactly (byte-identical). The two
restricted-completion modes attempt `_restricted_mw_optimality_cut` and fall back to the
standard cut (with a `@warn`) if the completion LP comes back infeasible or the derivation
throws -- this keeps the outer BendersY loop making progress even on iterations where the
restricted completion isn't feasible, per the spec's explicit anticipation of that case.
Returns a diagnostics `NamedTuple` for the caller's iteration log.
"""
function _add_aggregate_od_route_benders_y_optimality_cut!(
    master::Model,
    y,
    theta,
    cut_id::Int,
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    y_hat::Vector{Float64},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    y_core::Union{Nothing, AggregateODRouteYCorePoint},
    optimizer_env,
    v_hat::Float64,
    rho::Dict{Int, Float64};
    certified::Union{Nothing, AggregateODRouteCertifiedRouteCoveringDuals}=nothing,
    Q_bar::Union{Nothing, Float64}=nothing,
    certification_already_failed::Bool=false,
)
    n = data.n_stations
    if solver.cut_derivation == :standard
        alpha = v_hat - sum(rho[j] * y_hat[j] for j in 1:n)
        @constraint(master, theta[cut_id] >= alpha + sum(rho[j] * y[j] for j in 1:n))
        return (
            mode=:standard, mw_status=:not_attempted, Q_bar=v_hat, phi_core=NaN,
            phi_core_baseline=NaN, completion_runtime_sec=0.0, n_routes=0,
            n_cg_iterations=0, fallback=false,
        )
    end

    # If the caller's own certified-Q_bar attempt (used for the gating decision) already failed
    # this (iteration, cut_id), retrying here would just repeat the identical, deterministic CG
    # certification failure -- skip straight to the standard-cut fallback instead of paying for
    # a second, doomed CG solve.
    objective_mode = solver.cut_derivation == :restricted_mw_fixed_pi ? :maximize_core : :zero
    mw_result = certification_already_failed ? nothing : try
        _restricted_mw_optimality_cut(
            data, model, solver, group_requests, feasible_pairs, y_hat, assignments,
            open_stations, y_core.y, optimizer_env, objective_mode;
            certified=certified, Q_bar=Q_bar,
        )
    catch err
        @warn "BendersY restricted-MW cut derivation failed; falling back to the standard cut " *
            "for this (iteration, cut_id)" cut_id error = err
        nothing
    end

    if isnothing(mw_result) || mw_result.status != :ok
        alpha = v_hat - sum(rho[j] * y_hat[j] for j in 1:n)
        @constraint(master, theta[cut_id] >= alpha + sum(rho[j] * y[j] for j in 1:n))
        return (
            mode=solver.cut_derivation, mw_status=isnothing(mw_result) ? :error : mw_result.status,
            Q_bar=v_hat, phi_core=NaN, phi_core_baseline=NaN, completion_runtime_sec=0.0,
            n_routes=0, n_cg_iterations=0, fallback=true,
        )
    end

    cut_constant = mw_result.cut_constant
    beta = mw_result.beta
    @constraint(master, theta[cut_id] >= cut_constant + sum(get(beta, j, 0.0) * y[j] for j in 1:n))
    return (
        mode=solver.cut_derivation, mw_status=:ok, Q_bar=mw_result.Q_bar, phi_core=mw_result.phi_core,
        phi_core_baseline=isnothing(mw_result.phi_core_baseline) ? NaN : mw_result.phi_core_baseline,
        completion_runtime_sec=mw_result.completion_runtime_sec, n_routes=mw_result.n_routes,
        n_cg_iterations=mw_result.n_cg_iterations, fallback=false,
    )
end
