"""
Benders-YZH decomposition for AggregateODRouteProblem (NearestOpen policy only): master =
y,z,h (h scenario-compressed per physical OD pair); subproblem = theta only (see
`iterative_strategy_types.jl`'s `BendersYZH` docstring).

**Repricing is not actually a structural non-issue here** (an earlier version of this docstring
claimed it was, based on an incomplete argument -- see the `BendersYZH` docstring's 2026-07-21
correction). `h` being fixed fully removes the master's own assignment-selection degeneracy, but
`_build_yzh_route_subproblem_lp`'s route-covering LP (fixed `h`, free continuous `lambda`) is
still a set-cover-style LP with a potentially degenerate dual-optimal face -- CG-priming's own
exhaustiveness proof only certifies the *one* dual vertex CG happened to land on, and this
subproblem LP is a separately-solved formulation with no guarantee of landing on that same vertex.
`solver.reprice_subproblem` is fully wired here (not a no-op) via
`_solve_yzh_route_subproblem_lp_with_repricing`, mirroring BendersYZ's repricing loop exactly, and
is one way to close that gap empirically. A cheaper, provably-valid alternative is also
implemented: `BendersSolver(cut_derivation=:zero_completion)` reuses CG's own already-certified
dual directly (zero-extended over `(request, pair)` rows not present in CG's restricted model,
`_zero_completion_yzh_rho` below) instead of re-solving this subproblem LP at all -- any
dual-feasible point tight at `h_hat` gives a valid cut by LP duality, regardless of which point on
a degenerate optimal face is chosen, so CG's own certified dual suffices without exposure to this
LP's own re-solve degeneracy. This is the same idea as `BendersY`'s `cut_derivation=:zero_completion`
(notes/2026-07-17_restricted_mw_cut_benders_y.md), simplified further for `BendersYZH` since there
is no remaining free y/z/x block left to complete once `h` is fixed -- the certified dual *is*
the whole thing needed for the cut, computed by a plain sum rather than a completion LP.
`reprice_subproblem=false` is sound under `cut_derivation=:zero_completion`.
"""

"""
    _selected_assignments_from_h(physical_pairs, occurrences, feasible_pairs_by_p, h_hat)

Expands a rounded, scenario-compressed `h_hat` back into the flat
`Dict{(s,o,d), (j,k)}` shape `_solve_fixed_route_covering_by_cg` expects --
mirrors `_selected_assignments_from_x`, but each physical pair's selected
station pair is replicated across every scenario in which it occurs.
"""
function _selected_assignments_from_h(
    physical_pairs::Vector{Tuple{Int, Int}},
    occurrences::Dict{Tuple{Int, Int}, Vector{Int}},
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    h_hat::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64},
)
    assignments = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
    for p in physical_pairs
        o, d = p
        pairs = feasible_pairs_by_p[p]
        selected_pair = pairs[argmax([get(h_hat, (p, pair), 0.0) for pair in pairs])]
        get(h_hat, (p, selected_pair), 0.0) < 0.5 &&
            throw(ArgumentError("BendersYZH master produced no selected assignment for physical pair $(p)"))
        for s in occurrences[p]
            assignments[(s, o, d)] = selected_pair
        end
    end
    return assignments
end

"""
    _build_yzh_route_subproblem_lp(data, model, group_requests, feasible_pairs_by_p, columns, h_hat, optimizer_env, silent)

BendersYZH's per-cut-group subproblem LP: `h` is fixed *fully* (one
`fix_cons` per `(p, pair)`, mirroring `_build_xy_route_subproblem_lp`'s `x`
fixing exactly) rather than merely linked through `zp`/`zd` the way
BendersYZ's `x` is -- so, like `BendersXY`, this subproblem has no free
assignment variable at all, only `lambda` (route selection). This makes it
structurally immune to the stale-cut gap `BendersYZ`/`BendersY` have (see
`_solve_yz_route_subproblem_lp_with_repricing`'s docstring): the CG-priming
pool is exhaustive for exactly the one fixed assignment this LP also uses, so
no repricing companion is needed here. Objective is route cost only (no `h`
term -- that cost is already fully paid in the master via
`occurrence_count`, exactly as `BendersXY`'s subproblem carries no `x` cost).

`group_requests` is the flat `(s,o,d)` list for one cut group (from
`_benders_cut_groups`, unchanged); this function derives its own
per-group physical-pair/occurrence grouping from it, so a single `h`
variable can feed multiple scenarios' coverage rows within the same group
(the compression point of this whole decomposition) without ever being
duplicated.

Also returns `cover_cons`, keyed `(request, pair) => ConstraintRef` with `request=(s,o,d)`
reconstructed from each physical pair's `group_occurrences` -- deliberately the identical
shape `_build_nearest_open_y_subproblem_lp`/`_build_yz_route_subproblem_lp` use, so
`_extract_nearest_open_y_subproblem_coverage_duals` can be reused unmodified by
`_solve_yzh_route_subproblem_lp_with_repricing`.
"""
function _build_yzh_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteProblem,
    group_requests,
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    h_hat::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64},
    optimizer_env,
    silent::Bool,
)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    group_physical_pairs, group_occurrences = _yzh_group_physical_pairs(group_requests)
    h, fix_cons = add_fixed_physical_pair_variables!(m, group_physical_pairs, feasible_pairs_by_p, h_hat)

    lambda = add_benders_lambda_variables!(m, columns, n_scenarios(data))
    # Walk-only and same-station assignments use no vehicle route, so no route column can (or
    # needs to) cover them -- already handled inside the shared function.
    cover_cons = add_benders_route_coverage_constraints!(
        m, lambda, group_physical_pairs, group_occurrences, feasible_pairs_by_p, columns, h,
    )

    route_expr = benders_route_regularization_cost_expr(model, columns, lambda, n_scenarios(data))
    set_benders_subproblem_objective!(m, AffExpr(0.0), route_expr)
    return m, fix_cons, cover_cons
end

"""
    _solve_yzh_route_subproblem_lp_with_repricing(data, model, mapping, group_requests, feasible_pairs_by_p, columns, h_hat, optimizer_env, silent; max_reprice_rounds)

Diagnostic/soundness-check companion to `_build_yzh_route_subproblem_lp`, mirroring
`_solve_yz_route_subproblem_lp_with_repricing` exactly: after each LP solve, extracts the
covering-constraint duals (via `_extract_nearest_open_y_subproblem_coverage_duals`, reused
unmodified since `cover_cons` has the identical `(request, pair) => ConstraintRef` shape) and
runs genuine label-setting pricing against them. If pricing finds a column with negative
reduced cost beyond the seeded pool, the pool folds it in and the LP is re-solved, repeating
until an exhaustive pricing pass finds no negative columns; hitting `max_reprice_rounds` first
is an error. This is a certification check for dual-basis degeneracy. The priming solve already
established the activated assignment's objective, so alternate columns may change the dual basis
but must not improve that objective. An improvement indicates a pricing or formulation-alignment
defect, and this routine throws.
"""
function _solve_yzh_route_subproblem_lp_with_repricing(
    data::StationSelectionData,
    model::AnyAggregateODRouteProblem,
    mapping::AggregateODRouteMap,
    group_requests,
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    h_hat::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64},
    optimizer_env,
    silent::Bool;
    max_reprice_rounds::Int=10_000,
)
    return _solve_benders_subproblem_lp_with_repricing(
        data, model, mapping, columns, optimizer_env, silent, "BendersYZH";
        build_lp=pool -> begin
            m, fix_cons, cover_cons = _build_yzh_route_subproblem_lp(
                data, model, group_requests, feasible_pairs_by_p, pool, h_hat, optimizer_env, silent,
            )
            (m, fix_cons, cover_cons, nothing)
        end,
        max_reprice_rounds=max_reprice_rounds,
    )
end

"""
    _yzh_group_physical_pairs(group_requests) -> (group_physical_pairs, group_occurrences)

Factors out the `(o,d) -> [scenarios]` grouping `_build_yzh_route_subproblem_lp` already does
inline, so the outer loop's zero-completion cut derivation can reuse it without touching the
subproblem builder.
"""
function _yzh_group_physical_pairs(
    group_requests,
)::Tuple{Vector{Tuple{Int, Int}}, Dict{Tuple{Int, Int}, Vector{Int}}}
    group_occurrences = Dict{Tuple{Int, Int}, Vector{Int}}()
    for (s, o, d) in group_requests
        push!(get!(group_occurrences, (o, d), Int[]), s)
    end
    return collect(keys(group_occurrences)), group_occurrences
end

"""
    _zero_completion_yzh_rho(data, model, cg_result, group_requests, feasible_pairs,
                              assignments) -> (Q_bar, rho, certified)

`BendersYZH`'s zero-completion: since `h` is fixed with no other structural row in the subproblem
(same shape as `BendersY`'s `y` and `BendersYZ`'s `z`), and `h`'s only other appearance is in the
coverage rows (`sum(lambda) >= h[(p,pair)]`, one row per scenario occurrence), the fix-constraint's
dual is *exactly* the sum of the certified, zero-extended coverage-row duals across those
occurrences -- no completion LP needed at all, unlike `BendersY`/`BendersYZ` (see the module
docstring above and `notes/2026-07-17_restricted_mw_cut_benders_y.md`). `cg_result` is this
iteration's own priming CG solve (`_solve_fixed_route_covering_by_cg`, already run once before any
cut group is processed) -- its own pricing pass already proved `cg_stop_reason ==
:optimality_proven`, so `_certified_qbar` reads its per-request duals directly with no re-solve.
"""
function _zero_completion_yzh_rho(
    data::StationSelectionData,
    model::AggregateODRouteProblem,
    cg_result::AggregateODRouteColumnGenerationResult,
    group_requests,
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
)
    group_requests_vec = collect(group_requests)
    assignments_for_group = Dict(request => assignments[request] for request in group_requests_vec)
    certified, full_Q_bar = _certified_qbar(data, model, cg_result, group_requests_vec, assignments_for_group)
    # `_certified_qbar` already returns this group's own share of RouteCoveringProblem's LP
    # objective, which contains both the fixed walking term and route cost. BendersYZH's master
    # already prices walking through `h`, while its theta-only subproblem represents route
    # recourse only. Remove that fixed walking constant here or the zero-completion cut applies
    # walk_cost_weight twice.
    fixed_walking_cost = sum(
        _assignment_pair_cost(data, request, assignments_for_group[request]; weight=model.walk_cost_weight)
        for request in group_requests_vec;
        init=0.0,
    )
    Q_bar = full_Q_bar - fixed_walking_cost
    feasible_pairs_flat = Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}}(
        request => feasible_pairs_by_p[(request[2], request[3])] for request in group_requests_vec
    )
    pi_full = _zero_extended_pi(group_requests_vec, feasible_pairs_flat, assignments_for_group, certified.pi_by_request)

    _group_physical_pairs, group_occurrences = _yzh_group_physical_pairs(group_requests_vec)
    rho = Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64}()
    for p in _group_physical_pairs, pair in feasible_pairs_by_p[p]
        requires_no_vehicle_route(pair) && continue
        rho[(p, pair)] = sum(
            get(pi_full, ((s, p[1], p[2]), pair), 0.0) for s in group_occurrences[p]; init=0.0,
        )
    end
    return Q_bar, rho, certified
end

"""
    _add_aggregate_od_route_benders_yzh_optimality_cut!(master, h, theta, cut_id, data, model,
        solver, group_requests, feasible_pairs_by_p, h_hat, assignments, cg_result, v_hat,
        rho; certified=nothing, Q_bar=nothing, certification_already_failed=false)

`BendersYZH` analogue of `_add_aggregate_od_route_benders_y_optimality_cut!`/
`_add_aggregate_od_route_benders_yz_optimality_cut!`: `:standard` reproduces the pre-existing
subgradient cut exactly. `:zero_completion` uses `_zero_completion_yzh_rho`'s certified sum
directly (no completion LP, no fallback needed beyond the certification itself failing).
`:restricted_mw_fixed_pi` is rejected earlier (`BendersSolver` construction) since there is no free
dual block left to optimize a core-point objective over once `h` is fixed fully.
"""
function _add_aggregate_od_route_benders_yzh_optimality_cut!(
    master::Model,
    h,
    theta,
    cut_id::Int,
    data::StationSelectionData,
    model::AggregateODRouteProblem,
    solver::BendersSolver,
    group_requests,
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    h_hat::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    cg_result::AggregateODRouteColumnGenerationResult,
    v_hat::Float64,
    rho::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64};
    zc_result=nothing,
    certification_already_failed::Bool=false,
)
    if solver.cut_derivation == :standard
        add_benders_optimality_cut!(master, theta, cut_id, v_hat + sum(
            rho[key] * (h[key] - get(h_hat, key, 0.0)) for key in keys(rho)
        ))
        return (mode=:standard, fallback=false)
    end

    certification_already_failed && throw(ErrorException(
        "BendersYZH zero-completion cut cannot be constructed after certification failed"
    ))
    if isnothing(zc_result)
        zc_result = try
            _zero_completion_yzh_rho(
                data, model, cg_result, group_requests, feasible_pairs_by_p, assignments,
            )
        catch err
            throw(ErrorException(
                "BendersYZH zero-completion cut derivation failed for cut_id=$(cut_id); refusing " *
                "to fall back to an uncertified standard cut: " * sprint(showerror, err)
            ))
        end
    end

    Q_bar, zero_rho, _certified = zc_result
    add_benders_optimality_cut!(master, theta, cut_id, Q_bar + sum(
        get(zero_rho, key, 0.0) * (h[key] - get(h_hat, key, 0.0)) for key in keys(zero_rho)
    ))
    return (mode=solver.cut_derivation, fallback=false)
end
"""
    _run_aggregate_od_route_nearest_open_benders_yzh(data, model, solver) -> OptResult

`BendersYZH`'s outer-loop entry point: a thin wrapper around the generic
`_run_benders_decomposition` (`benders/generic_runner.jl`). Unlike `BendersY`/`BendersXY`/
`BendersYZ`, this decomposition never accepted `direct_enumeration_pool`/`seed_cuts`/
`harvested_cuts` kwargs (`_run_direct_enumeration_guided_benders` only ever dispatches to
`BendersY`/`BendersYZ`), so this wrapper doesn't accept them either -- matches the original
signature exactly.
"""
function _run_aggregate_od_route_nearest_open_benders_yzh(
    data::StationSelectionData,
    model::AggregateODRouteProblem,
    solver::BendersSolver,
)
    return _run_benders_decomposition(data, model, solver)
end
