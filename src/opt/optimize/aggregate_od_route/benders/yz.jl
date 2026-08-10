"""
Benders-YZ decomposition for AggregateODRouteModel (NearestOpen policy only): master =
y,z; subproblem = x,theta (see `iterative_strategy_types.jl`'s `BendersYZ` docstring).
Requires `reprice_subproblem=true` for a provably optimal result -- see
`_solve_yz_route_subproblem_lp_with_repricing`'s docstring.
"""

"""
    _build_yz_route_subproblem_lp(data, model, requests, feasible_pairs, columns, z_hat, optimizer_env, silent)

BendersYZ's per-cut-group subproblem LP: unlike `_build_xy_route_subproblem_lp`
(which fixes `x` and leaves the assignment cost entirely to the master), this
fixes `z` -- `x` and `θ` (via `lambda`) are both free here, and the walking
cost lives in this LP's objective since the master carries none (mirrors
`_build_nearest_open_y_subproblem_lp`'s objective shape, not
`_build_xy_route_subproblem_lp`'s). There is no `y` in this LP at all: `z` is
built bare and fixed directly to `z_hat`, using `_sorted_endpoint_chain`
(`aggregate_od_route_benders_y_mw_cut.jl`) to get the same
`(key, sorted_stations)` a request's physical endpoints resolve to in the
master's `nearest_endpoint_chain_cache` -- guaranteed to line up positionally
since both use the identical `sortperm`+`_endpoint_chain_key` logic. A local
(not model-attached) cache dedupes `z` construction within this one LP build
when a physical endpoint recurs across the group's requests.
"""
function _build_yz_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    optimizer_env,
    silent::Bool,
)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    # One pre-pass to resolve each request's pickup/dropoff chain key (and sorted-candidate
    # list, reused below to avoid recomputing it) so add_fixed_endpoint_chain_variables! can
    # eagerly build z for exactly the keys this group's requests need -- not every key in z_hat,
    # which is the master's full nearest_endpoint_chain_cache (see that function's docstring).
    chain_info = Dict{NTuple{3, Int}, Any}()
    needed_keys = Set{keytype(z_hat)}()
    for request in requests
        _s, o, d = request
        pickup_key, sorted_pickups, _pickup_costs = _sorted_endpoint_chain(data, o, model.max_walking_distance, :pickup)
        dropoff_key, sorted_dropoffs, _dropoff_costs = _sorted_endpoint_chain(data, d, model.max_walking_distance, :dropoff)
        chain_info[request] = (pickup_key, sorted_pickups, dropoff_key, sorted_dropoffs)
        push!(needed_keys, pickup_key)
        push!(needed_keys, dropoff_key)
    end
    z_by_key, fix_cons = add_fixed_endpoint_chain_variables!(m, z_hat, needed_keys)

    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for request in requests
        pairs = feasible_pairs[request]
        x_by_pair, _sum_con = add_unlinked_pair_assignment_constraints!(m, request, pairs)
        for (pair, var) in x_by_pair
            x[(request, pair)] = var
        end

        pickup_key, sorted_pickups, dropoff_key, sorted_dropoffs = chain_info[request]
        zp = z_by_key[pickup_key]
        zd = z_by_key[dropoff_key]
        pickup_rank = Dict(station => idx for (idx, station) in enumerate(sorted_pickups))
        dropoff_rank = Dict(station => idx for (idx, station) in enumerate(sorted_dropoffs))
        real_pairs = filter(!is_walk_only_pair, pairs)
        _add_endpoint_x_linking!(
            m, real_pairs, pairs, x_by_pair, zp, zd, pickup_rank, dropoff_rank, sorted_pickups, sorted_dropoffs,
        )
    end

    lambda = add_benders_lambda_variables!(m, columns, n_scenarios(data))
    # See _build_nearest_open_y_subproblem_lp: walk-only and same-station assignments use no
    # vehicle route, so no coverage row for them -- already handled inside the shared function.
    cover_cons = add_benders_route_coverage_constraints!(m, lambda, requests, feasible_pairs, columns, x)

    walking_expr = assignment_walking_cost_expr(data, requests, feasible_pairs, x; weight=model.walk_cost_weight)
    route_expr = benders_route_regularization_cost_expr(model, columns, lambda, n_scenarios(data))
    set_benders_subproblem_objective!(m, walking_expr, route_expr)
    m[:x] = x
    return m, fix_cons, cover_cons
end

"""
    _solve_yz_route_subproblem_lp_with_repricing(...)

BendersYZ analogue of [`_solve_nearest_open_y_subproblem_lp_with_repricing`](@ref):
`_build_yz_route_subproblem_lp` also lets `x` vary freely (only `z` is fixed),
so a column pool proven exhaustive by `_solve_fixed_route_covering_by_cg` for
just the one nearest-open assignment at `y_hat` is not necessarily complete
for *this* LP's own, more general dual structure -- the same completeness gap
`_solve_nearest_open_y_subproblem_lp_with_repricing`'s docstring describes for
BendersY (confirmed empirically: a single plain, non-repricing LP solve on
`_build_yz_route_subproblem_lp` converges BendersYZ to a genuinely
suboptimal-but-correctly-costed `y` on the real-data alignment fixture).
Reuses `_extract_nearest_open_y_subproblem_coverage_duals` unchanged since
`cover_cons` has the identical `(request, pair) => ConstraintRef` shape. Like
`_solve_nearest_open_y_subproblem_lp_with_repricing`, this is a certification
check for dual-basis degeneracy. The priming solve already established the activated assignment's
objective, so newly priced columns may change the dual basis but must not improve that objective.
An improvement indicates a pricing or formulation-alignment defect, and this routine throws.
Successful return also requires an exhaustive pricing pass with no negative columns.
"""
function _solve_yz_route_subproblem_lp_with_repricing(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    mapping::AggregateODRouteMap,
    requests,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    optimizer_env,
    silent::Bool;
    max_reprice_rounds::Int=10_000,
)
    return _solve_benders_subproblem_lp_with_repricing(
        data, model, mapping, columns, optimizer_env, silent, "BendersYZ";
        build_lp=pool -> begin
            m, fix_cons, cover_cons = _build_yz_route_subproblem_lp(
                data, model, requests, feasible_pairs, pool, z_hat, optimizer_env, silent,
            )
            (m, fix_cons, cover_cons, nothing)
        end,
        extra_checks! = (m, _extra) -> assert_endpoint_chain_near_binary(m),
        max_reprice_rounds=max_reprice_rounds,
    )
end

"""
    _run_aggregate_od_route_nearest_open_benders_yz(data, model, solver; kwargs...) -> OptResult

`BendersYZ`'s outer-loop entry point: a thin wrapper around the generic
`_run_benders_decomposition` (`benders/generic_runner.jl`). Its signature is unchanged, so
`dispatch.jl` needed no changes.
"""
function _run_aggregate_od_route_nearest_open_benders_yz(
    data::StationSelectionData,
    model::AggregateODRouteModel,
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
