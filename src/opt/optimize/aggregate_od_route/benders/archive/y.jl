"""
Benders-Y decomposition for AggregateODRouteProblem (NearestOpen policy): master = y
only; subproblem = z,x,theta together (see `iterative_strategy_types.jl`'s
`BendersY` docstring). Companion cut-derivation logic (`:standard`/`:zero_completion`/
`:restricted_mw_fixed_pi`) lives in `aggregate_od_route_benders_y_mw_cut.jl`.
"""

function _build_nearest_open_y_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteProblem,
    mapping::AggregateODRouteMap,
    requests,
    demand,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    y_hat::Vector{Float64},
    optimizer_env,
    silent::Bool;
    lambda_binary::Bool=false,
)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    if !lambda_binary
        set_optimizer_attribute(m, "Method", 1)
        set_optimizer_attribute(m, "Presolve", 0)
    end
    y, fix_cons = add_fixed_station_selection_variables!(m, data, y_hat)

    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    m[:debug_sum_x_cons] = Dict{NTuple{3, Int}, ConstraintRef}()  # DEBUG (temporary instrumentation)
    if _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        for request in requests
            pairs = feasible_pairs[request]
            x_by_pair, sum_con = _add_nearest_open_pair_assignment!(
                m, data, y, request, pairs, model.max_walking_distance;
                allow_walk_only=model.allow_walk_only,
                selector_style=model.assignment_policy.feasibility_cut_style,
                debug_key_prefix=request,
            )
            m[:debug_sum_x_cons][request] = sum_con
            for (pair, var) in x_by_pair
                x[(request, pair)] = var
            end
        end
    else
        for request in requests
            x_by_pair, _sum_con = add_ranked_pair_assignment_constraints!(
                m, data, y, request, feasible_pairs[request],
            )
            for (pair, var) in x_by_pair
                x[(request, pair)] = var
            end
        end
    end

    lambda = add_benders_lambda_variables!(m, columns, n_scenarios(data); binary=lambda_binary)
    # Walk-only and same-station assignments use no vehicle route, so no route column can (or
    # needs to) cover them -- a coverage row here would wrongly force x[(request, pair)] to 0
    # even when the endpoint-collision constraint (_add_nearest_open_endpoint_linked_x!) forces
    # it to 1, making the LP infeasible. add_benders_route_coverage_constraints! already skips
    # these (requires_no_vehicle_route).
    cover_cons = add_benders_route_coverage_constraints!(m, lambda, requests, feasible_pairs, columns, x)

    walking_expr = assignment_walking_cost_expr(data, requests, feasible_pairs, x; weight=model.walk_cost_weight)
    route_expr = benders_route_regularization_cost_expr(model, columns, lambda, n_scenarios(data))
    set_benders_subproblem_objective!(m, walking_expr, route_expr)
    return m, fix_cons, x, cover_cons
end

"""
    _assert_x_matches_nearest_open(x, data, requests, feasible_pairs, y_hat; atol=1e-6)

Runtime check (not just a constraint-design argument) that a solved
`_build_nearest_open_y_subproblem_lp` LP's `x` values, for `y` fixed to
`y_hat`, actually reproduce nearest-open assignment: exactly one `x[request,
pair]` at (near-)1 per request, and that pair must equal the pair
independently computed by `_fixed_assignments_from_y` (the same routine
`_run_aggregate_od_route_nearest_open_benders_y` uses to fix assignments for
priming CG). Throws `ArgumentError` naming the first mismatch found, rather
than silently trusting the chain-constraint encoding.
"""
function _assert_x_matches_nearest_open(
    x::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
    data::StationSelectionData,
    requests,
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    y_hat::Vector{Float64},
    model::AnyAggregateODRouteProblem;
    atol::Float64=1e-6,
)::Nothing
    expected, infeasible = _fixed_assignments_from_y(
        data, collect(requests), feasible_pairs, y_hat;
        style=model.assignment_policy.feasibility_cut_style,
        max_walking_distance=model.max_walking_distance,
        allow_walk_only=model.allow_walk_only,
        allow_same_station=true,
    )
    isempty(infeasible) || throw(ArgumentError(
        "nearest-open subproblem LP check: y_hat=$(y_hat) leaves requests infeasible: $(infeasible)"
    ))
    for request in requests
        ranked = _ranked_request_pairs(data, request, feasible_pairs[request])
        positive = [(pair, value(x[(request, pair)])) for pair in ranked if value(x[(request, pair)]) > atol]
        length(positive) == 1 || throw(ArgumentError(
            "nearest-open subproblem LP check failed for request $(request): expected exactly one " *
            "positive x at y_hat=$(y_hat), got $(positive)"
        ))
        selected_pair, val = positive[1]
        isapprox(val, 1.0; atol=atol) || throw(ArgumentError(
            "nearest-open subproblem LP check failed for request $(request): x[$(selected_pair)]=$(val) " *
            "is not binary (not within atol=$(atol) of 1.0) at y_hat=$(y_hat)"
        ))
        selected_pair == expected[request] || throw(ArgumentError(
            "nearest-open subproblem LP check failed for request $(request): LP selected pair " *
            "$(selected_pair) but independently-computed nearest-open assignment is $(expected[request]) " *
            "at y_hat=$(y_hat)"
        ))
    end
    return nothing
end

"""
    _solve_nearest_open_y_subproblem_lp_with_repricing(...)

Guarantees `v_hat`/`rho` are valid against the *full* route universe, not just whatever
`columns` (the shared pool) happens to contain. Trusting `columns` outright (a single LP
solve on `_build_nearest_open_y_subproblem_lp`, no repricing) is sound only if the pool is
already complete for *this* subproblem's own dual structure, which is a different, more
general LP (free `x` over every globally feasible pair, all `data.n_stations` as potential
route nodes) than the restricted, fixed-assignment problem
`_solve_fixed_route_covering_by_cg`'s priming CG actually proved complete for. This
function closes that gap directly: after each LP solve, it extracts the covering-constraint
duals (see `_extract_nearest_open_y_subproblem_coverage_duals`) and runs genuine
label-setting pricing against them, over every scenario, exactly mirroring
`generate_aggregate_od_route_columns`'s own pricing round. If pricing finds any column with
negative reduced cost, that pool is *not* actually complete for this subproblem -- a
real completeness gap regardless of cause, though dual degeneracy (an alternate optimal
dual vertex under which a column looks non-improving) is one plausible source, since the
duals used are whichever vertex of the LP's optimal face the solver happened to return.
Either way the newly found columns are folded in and the LP is re-solved, repeating until
pricing finds nothing more (mirroring standard CG's own convergence, `cg_stop_reason ==
:optimality_proven`). Hitting `max_reprice_rounds` before that pass is an error. This is a
certification check, not a corrective CG loop for an underpriced LP: the fixed-assignment priming
solve has already established the objective that the activated-assignment subproblem must attain.
Alternate columns exposed by a different dual basis must preserve that objective. An improvement
therefore indicates invalid pricing output or a mismatch between the priming and activated-
assignment formulations, and the routine throws rather than hiding that upstream defect.
Returns `(v_hat, rho, pool, n_new_columns_total, n_rounds, fully_exhausted,
max_objective_delta)`; `n_new_columns_total > 0` is itself the signal worth
surfacing -- see notes/2026-07-15_bendersy_stale_cut_soundness.md.
"""
function _solve_nearest_open_y_subproblem_lp_with_repricing(
    data::StationSelectionData,
    model::AnyAggregateODRouteProblem,
    mapping::AggregateODRouteMap,
    requests,
    demand,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    y_hat::Vector{Float64},
    optimizer_env,
    silent::Bool;
    max_reprice_rounds::Int=10_000,
)
    return _solve_benders_subproblem_lp_with_repricing(
        data, model, mapping, columns, optimizer_env, silent, "BendersY";
        build_lp=pool -> begin
            m, fix_cons, x, cover_cons = _build_nearest_open_y_subproblem_lp(
                data, model, mapping, requests, demand, feasible_pairs, pool, y_hat, optimizer_env, silent,
            )
            (m, fix_cons, cover_cons, x)
        end,
        extra_checks! = (m, x) -> begin
            _assert_x_matches_nearest_open(x, data, requests, feasible_pairs, y_hat, model)
            assert_endpoint_chain_near_binary(m)
        end,
        max_reprice_rounds=max_reprice_rounds,
    )
end

"""
    _run_aggregate_od_route_nearest_open_benders_y(data, model, solver; kwargs...) -> OptResult

`BendersY`'s outer-loop entry point: a thin wrapper around the generic
`_run_benders_decomposition` (`benders/generic_runner.jl`), which implements the
build-master / solve-master / per-cut-group subproblem+cut / convergence-check loop shape
shared by every decomposition. Kept as its own named function (rather than inlined at the
`BendersSolver` dispatch site) purely so `dispatch.jl` and
`direct_enumeration_guide.jl`'s `_run_direct_enumeration_guided_benders` (which calls this
function twice, once per direct-enumeration phase) don't need to know the generic runner
exists.
"""
function _run_aggregate_od_route_nearest_open_benders_y(
    data::StationSelectionData,
    model::AggregateODRouteProblem,
    solver::BendersSolver;
    direct_enumeration_pool::Union{Nothing, Vector{AggregateODRouteColumn}}=nothing,
    seed_cuts::Vector{<:NamedTuple}=NamedTuple[],
    harvested_cuts::Union{Nothing, Vector{<:NamedTuple}}=nothing,
)
    return _run_benders_decomposition(
        data, model, solver;
        direct_enumeration_pool=direct_enumeration_pool, seed_cuts=seed_cuts, harvested_cuts=harvested_cuts,
    )
end
