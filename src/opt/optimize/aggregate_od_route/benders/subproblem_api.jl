"""
Public API for testing AggregateODRouteModel's Benders decomposition
master/subproblem builders directly, without going through the full outer
`run_opt(..., ::BendersSolver)` loop (no cut accumulation, no master
iteration). Exists specifically so "always feasible" mode
(`AggregateODRouteModel(unmet_demand_penalty=...)`) -- and in particular
whether the service indicator `u` always resolves to exactly 0/1 -- can be
exercised directly against a caller-supplied `y_hat`, including adversarial
inputs a real master would never propose (e.g. `y_hat` opening nothing at
all). The internal `_build_*`/`_solve_*` functions these wrap are untouched;
this is purely an additive public layer.
"""

export BendersSubproblemResult
export solve_benders_y_subproblem
export solve_benders_yz_subproblem
export solve_benders_yzh_master

"""
    BendersSubproblemResult

Flat result type for the public Benders subproblem/master API (mirrors
`OptResult`'s style, `src/utils/core/results.jl`).

- `decomposition`: `:y`, `:yz`, or `:yzh_master`.
- `objective_value`: includes the `unmet_demand_penalty` term when active.
- `assignment`: `x`/`h` values, keyed like the corresponding internal dict
  (`(request,pair)` for `:y`/`:yz`, `(physical_pair,pair)` for `:yzh_master`).
- `service`: `u` values, same keys as `assignment` collapsed to the request/
  physical-pair level; `nothing` when `unmet_demand_penalty` is off.
- `duals`: the `rho` values used for Benders cuts (fix_cons duals); empty for
  `:yzh_master` (no cut is derived here, this solves the whole master).
"""
struct BendersSubproblemResult
    decomposition::Symbol
    objective_value::Float64
    assignment::Dict{Any, Float64}
    service::Union{Nothing, Dict{Any, Float64}}
    duals::Dict{Any, Float64}
    termination_status::MOI.TerminationStatusCode
    metadata::Dict{String, Any}
end

function _benders_subproblem_api_setup(data::StationSelectionData, model::AggregateODRouteModel)
    _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style) ||
        throw(ArgumentError(
            "solve_benders_*_subproblem/master require NearestOpenAggregateODAssignmentPolicy" *
            "(:big_m_nearest) or (:endpoint_chain); got :$(model.assignment_policy.feasibility_cut_style)"
        ))
    mapping = create_map(model, data)
    validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))
    return mapping, requests, demand, feasible_pairs
end

"""
    solve_benders_y_subproblem(data, model, y_hat; columns=AggregateODRouteColumn[], reprice=false, optimizer_env=nothing, silent=true) -> BendersSubproblemResult

Solves `BendersY`'s fixed-`y` subproblem LP (`z`,`x`,`θ` jointly, `y` fixed to
`y_hat`) directly. `reprice=true` uses `_solve_nearest_open_y_subproblem_lp_with_repricing`
instead of the plain (potentially column-pool-incomplete) LP -- see that
function's docstring for why repricing matters for provable optimality, not
just for this API.
"""
function solve_benders_y_subproblem(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    y_hat::AbstractVector{<:Real};
    columns::Vector{AggregateODRouteColumn}=AggregateODRouteColumn[],
    reprice::Bool=false,
    optimizer_env=nothing,
    silent::Bool=true,
    max_reprice_rounds::Int=20,
)::BendersSubproblemResult
    mapping, requests, demand, feasible_pairs = _benders_subproblem_api_setup(data, model)
    env = isnothing(optimizer_env) ? Gurobi.Env() : optimizer_env
    y_hat_f = Float64.(y_hat)
    metadata = Dict{String, Any}()
    if reprice
        v_hat, rho, pool, n_new, rounds, exhausted, delta = _solve_nearest_open_y_subproblem_lp_with_repricing(
            data, model, mapping, requests, demand, feasible_pairs, columns, y_hat_f, env, silent;
            max_reprice_rounds=max_reprice_rounds,
        )
        metadata["reprice_columns_found"] = n_new
        metadata["reprice_rounds"] = rounds
        metadata["reprice_exhausted"] = exhausted
        metadata["reprice_objective_delta"] = delta
        objective_value = v_hat
        rho_out = Dict{Any, Float64}(j => v for (j, v) in rho)
    else
        objective_value, rho = _solve_nearest_open_y_subproblem_lp(
            data, model, mapping, requests, demand, feasible_pairs, columns, y_hat_f, env, silent,
        )
        rho_out = Dict{Any, Float64}(j => v for (j, v) in rho)
    end
    # Re-solve once more to read x/u off a live model (the functions above only
    # return the objective/duals) -- cheap relative to the LP solve itself, and
    # keeps this API's result self-contained rather than requiring a second call.
    m, _fix_cons, x, _cover_cons = _build_nearest_open_y_subproblem_lp(
        data, model, mapping, requests, demand, feasible_pairs, columns, y_hat_f, env, silent,
    )
    optimize!(m)
    assignment = Dict{Any, Float64}(key => value(var) for (key, var) in x)
    service = haskey(m, :u) && !isnothing(m[:u]) ?
        Dict{Any, Float64}(request => value(var) for (request, var) in m[:u]) :
        nothing
    return BendersSubproblemResult(
        :y, objective_value, assignment, service, rho_out, termination_status(m), metadata,
    )
end

"""
    solve_benders_yz_subproblem(data, model, y_hat; columns=AggregateODRouteColumn[], reprice=false, optimizer_env=nothing, silent=true) -> BendersSubproblemResult

Solves `BendersYZ`'s fixed-`z` subproblem LP (`x`,`θ` free, `z` fixed) given
`y_hat`. `z_hat` is derived from `y_hat` first (a throwaway LP: `y` fixed to
`y_hat`, `_add_nearest_open_master_z!` builds/solves the same chain
machinery the real master uses) since `z_hat`'s own key type
(`_AggregateODRouteEndpointChainKey`) is internal -- callers only ever need
to supply `y_hat`, matching `solve_benders_y_subproblem`'s signature.
`reprice=true` uses `_solve_yz_route_subproblem_lp_with_repricing` -- see
`BendersYZ`'s docstring (`iterative_strategy_types.jl`) for why this is
required for a provably optimal result, not just for this API.
"""
function solve_benders_yz_subproblem(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    y_hat::AbstractVector{<:Real};
    columns::Vector{AggregateODRouteColumn}=AggregateODRouteColumn[],
    reprice::Bool=false,
    optimizer_env=nothing,
    silent::Bool=true,
    max_reprice_rounds::Int=20,
)::BendersSubproblemResult
    mapping, requests, demand, feasible_pairs = _benders_subproblem_api_setup(data, model)
    env = isnothing(optimizer_env) ? Gurobi.Env() : optimizer_env
    y_hat_f = Float64.(y_hat)

    zm = Model(() -> Gurobi.Optimizer(env))
    silent && set_silent(zm)
    zm[:aggregate_od_route_unmet_demand_penalty] = model.unmet_demand_penalty
    @variable(zm, 0 <= y[1:data.n_stations] <= 1)
    for j in 1:data.n_stations
        fix(y[j], y_hat_f[j]; force=true)
    end
    _add_nearest_open_master_z!(
        zm, data, y, requests, feasible_pairs, model.max_walking_distance, model.allow_walk_only,
        model.assignment_policy.feasibility_cut_style,
    )
    optimize!(zm)
    primal_status(zm) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("solve_benders_yz_subproblem: z derivation from y_hat failed with status $(termination_status(zm))"))
    z_hat = Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}}(
        key => round.(value.(vars)) for (key, vars) in zm[:nearest_endpoint_chain_cache]
    )

    metadata = Dict{String, Any}()
    if reprice
        v_hat, rho, pool, n_new, rounds, exhausted, delta = _solve_yz_route_subproblem_lp_with_repricing(
            data, model, mapping, requests, feasible_pairs, columns, z_hat, env, silent;
            max_reprice_rounds=max_reprice_rounds,
        )
        metadata["reprice_columns_found"] = n_new
        metadata["reprice_rounds"] = rounds
        metadata["reprice_exhausted"] = exhausted
        metadata["reprice_objective_delta"] = delta
        objective_value = v_hat
        rho_out = Dict{Any, Float64}(key => v for (key, v) in rho)
    else
        objective_value, rho = _solve_yz_route_subproblem_lp(
            data, model, requests, feasible_pairs, columns, z_hat, env, silent,
        )
        rho_out = Dict{Any, Float64}(key => v for (key, v) in rho)
    end
    m, _fix_cons, _cover_cons = _build_yz_route_subproblem_lp(
        data, model, requests, feasible_pairs, columns, z_hat, env, silent,
    )
    optimize!(m)
    assignment = Dict{Any, Float64}(key => value(var) for (key, var) in m[:x])
    service = haskey(m, :u) && !isnothing(m[:u]) ?
        Dict{Any, Float64}(request => value(var) for (request, var) in m[:u]) :
        nothing
    return BendersSubproblemResult(
        :yz, objective_value, assignment, service, rho_out, termination_status(m), metadata,
    )
end

"""
    solve_benders_yzh_master(data, model; optimizer_env=nothing, silent=true) -> BendersSubproblemResult

Builds and solves `BendersYZH`'s full master (`y,z,h,u`) directly, with no
`θ`/cut machinery at all (there is nothing to cut yet -- this solves the
master's own assignment-cost objective, `Σ occurrence_count[p]·cost·h[p] +
Σ occurrence_count[p]·penalty·(1-u[p])`, standalone). Unlike
`solve_benders_y_subproblem`/`solve_benders_yz_subproblem`, there is no
`y_hat` to fix: `y` is chosen by the solve itself (`sum(y)==model.l`), since
`h`/`u` live in the master alongside `y` for this decomposition -- see
`_add_nearest_open_master_h!`'s docstring for why (`h` is fixed fully in the
subproblem, unlike `BendersYZ`'s `x`).
"""
function solve_benders_yzh_master(
    data::StationSelectionData,
    model::AggregateODRouteModel;
    optimizer_env=nothing,
    silent::Bool=true,
)::BendersSubproblemResult
    mapping, requests, demand, feasible_pairs = _benders_subproblem_api_setup(data, model)
    physical_pairs, occurrences, feasible_pairs_by_p = _aggregate_od_route_benders_physical_pairs(mapping)
    occurrence_count = Dict(p => length(occurrences[p]) for p in physical_pairs)
    env = isnothing(optimizer_env) ? Gurobi.Env() : optimizer_env

    m = Model(() -> Gurobi.Optimizer(env))
    silent && set_silent(m)
    m[:aggregate_od_route_unmet_demand_penalty] = model.unmet_demand_penalty
    @variable(m, y[1:data.n_stations], Bin)
    @constraint(m, sum(y) == model.l)
    h = _add_nearest_open_master_h!(
        m, data, y, physical_pairs, feasible_pairs_by_p, model.max_walking_distance, model.allow_walk_only,
        model.assignment_policy.feasibility_cut_style,
    )
    unmet_demand_active = !isnothing(model.unmet_demand_penalty)
    u = unmet_demand_active ? m[:u] : nothing

    obj = AffExpr(0.0)
    for p in physical_pairs, pair in feasible_pairs_by_p[p]
        o, d = p
        add_to_expression!(obj, occurrence_count[p] * od_pair_walking_cost(data, o, d, pair), h[(p, pair)])
    end
    if unmet_demand_active
        for p in physical_pairs
            add_to_expression!(obj, occurrence_count[p] * model.unmet_demand_penalty)
            add_to_expression!(obj, -occurrence_count[p] * model.unmet_demand_penalty, u[p])
        end
    end
    @objective(m, Min, obj)
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("solve_benders_yzh_master failed with status $(termination_status(m))"))
    assert_endpoint_chain_near_binary(m)
    assert_service_near_binary(m)

    assignment = Dict{Any, Float64}(key => value(var) for (key, var) in h)
    service = unmet_demand_active ? Dict{Any, Float64}(p => value(var) for (p, var) in u) : nothing
    return BendersSubproblemResult(
        :yzh_master, objective_value(m), assignment, service, Dict{Any, Float64}(), termination_status(m),
        Dict{String, Any}("open_stations" => _open_station_values([round(value(y[j])) for j in 1:data.n_stations])),
    )
end
