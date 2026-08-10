"""
Shared machinery for `BendersSolver(lifted_walking_objective=true)`
(`AggregateODRouteModel` + `NearestOpenAggregateODAssignmentPolicy`, `BendersY`/`BendersYZ`
only). See `solver_types.jl`'s `BendersSolver` docstring for the option's contract.

Two pieces:

- `_unit_weighted_routing_model`: a shadow copy of the real model with
  `walk_cost_weight=0.0, route_regularization_weight=1.0`. Every subproblem/pricing/
  cut-derivation entry point in `y.jl`/`yz.jl`/`y_mw_cut.jl`/`yz_mw_cut.jl` already threads
  both fields through everywhere cost enters (that's `walk_cost_weight`'s documented purpose
  today), so passing this shadow model in place of the real one needs zero changes to any of
  that code: it already computes `0 * walking + 1 * routing` correctly by construction.
  Candidate ordering/tie-breaking in the chain builders is driven by raw walking distance, never
  by `walk_cost_weight`, so this is exact -- it changes only which objective coefficient is
  charged, never which station is nearest or how ties resolve. And because
  `route_regularization_weight` multiplies every route-column/pricing cost by the same constant,
  computing that sum at weight 1 and multiplying the *total* by the real weight afterward
  (done by the master and by the caller's incumbent bookkeeping) is algebraically identical to
  computing it at the real weight throughout.
- `_add_nearest_open_master_walking_cost!`: builds the exact walking-cost objective term in the
  master as a function of `y`, by reusing the *same* nearest-open chain + AND-linking machinery
  the subproblem already builds fresh every iteration (`_add_nearest_open_endpoint_linked_x!`),
  just built once in the master instead. For `BendersYZ` this is a superset of (and replaces)
  `_add_nearest_open_master_z!` -- it populates the identical `nearest_endpoint_chain_cache`, so
  the existing `z_hat` extraction is unaffected.
"""

function _unit_weighted_routing_model(
    model::AggregateODRouteModel; max_stops::Union{Int, Nothing}=model.max_stops,
)::AggregateODRouteModel
    return AggregateODRouteModel(
        model.l;
        route_regularization_weight=1.0,
        walk_cost_weight=0.0,
        repositioning_time=model.repositioning_time,
        max_walking_distance=model.max_walking_distance,
        max_wait_time=model.max_wait_time,
        detour_factor=model.detour_factor,
        max_stops=max_stops,
        max_new_columns=model.max_new_columns,
        n_candidates=model.n_candidates,
        pricing_time_limit_sec=model.pricing_time_limit_sec,
        reduced_cost_tol=model.reduced_cost_tol,
        initial_columns=model.initial_columns,
        relax_integrality=model.relax_integrality,
        assignment_policy=model.assignment_policy,
        allow_walk_only=model.allow_walk_only,
    )
end

"""
    _add_nearest_open_master_walking_cost!(master, data, model, y, requests, feasible_pairs) -> (AffExpr, Dict)

Builds, once in the master, the same per-request `x`/chain-selector structure
`_build_nearest_open_y_subproblem_lp`/`_build_yz_route_subproblem_lp` build fresh every
iteration (`_add_nearest_open_endpoint_linked_x!`), and returns a tuple
`(walking_cost, x_by_pair_full)`: `walking_cost` is
`sum(_assignment_pair_cost(data, request, pair; weight=model.walk_cost_weight) * x[request, pair])`
as a `JuMP.AffExpr` for the caller to fold into the master's objective; `x_by_pair_full` is
every request's `x` variable, keyed `(request, pair)` (the same shape used by
`_build_nearest_open_y_subproblem_lp`'s own `x` dict), for callers that need to reference the
master's exact assignment structure directly (e.g. `direct_enumeration_guide.jl`'s route-covering
linking constraints). Requires `feasibility_cut_style in (:big_m_nearest, :endpoint_chain)` --
`:pair_chain` ranks station pairs jointly with no addressable per-side chain to reuse (mirrors the
existing restriction that non-`:standard` `cut_derivation` already requires `:big_m_nearest`).
"""
function _add_nearest_open_master_walking_cost!(
    master::Model,
    data::StationSelectionData,
    model::AggregateODRouteModel,
    y,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
)::Tuple{AffExpr, Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}}
    model.assignment_policy.feasibility_cut_style == :pair_chain && throw(ArgumentError(
        "lifted_walking_objective has no addressable per-side chain to reuse under " *
        ":pair_chain -- use :big_m_nearest or :endpoint_chain"
    ))
    walking_cost = AffExpr(0.0)
    x_by_pair_full = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for request in requests
        pairs = feasible_pairs[request]
        x_by_pair, _sum_con = _add_nearest_open_pair_assignment!(
            master, data, y, request, pairs, model.max_walking_distance;
            allow_walk_only=model.allow_walk_only,
            selector_style=model.assignment_policy.feasibility_cut_style,
        )
        for pair in pairs
            add_to_expression!(
                walking_cost,
                _assignment_pair_cost(data, request, pair; weight=model.walk_cost_weight),
                x_by_pair[pair],
            )
            x_by_pair_full[(request, pair)] = x_by_pair[pair]
        end
    end
    return walking_cost, x_by_pair_full
end

"""
    _lifted_walking_cost(data, model, assignments) -> Float64

Closed-form walking cost of a nearest-open assignment `assignments` (as returned by
`_fixed_assignments_from_y`) -- no LP involved. Used for per-`y_hat` incumbent bookkeeping
(the master's own `walking_cost` `AffExpr` already gives the optimization-time value; this is
the cheap per-iteration readout used to reconstruct the true combined objective from a subproblem
solved against `_unit_weighted_routing_model`).
"""
function _lifted_walking_cost(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
)::Float64
    return sum(
        _assignment_pair_cost(data, request, pair; weight=model.walk_cost_weight)
        for (request, pair) in assignments;
        init=0.0,
    )
end

"""
    _with_objective_value(result::OptResult, objective_value::Float64) -> OptResult

Copy of `result` with `objective_value` overridden. Used under `lifted_walking_objective` to
report the true combined (walking + `route_regularization_weight`-weighted routing) objective as
`OptResult.objective_value`, since the underlying `result.model` was solved against
`_unit_weighted_routing_model` (unweighted routing, zero walking) and its own `objective_value(m)`
reflects only that unweighted routing value.
"""
function _with_objective_value(result::OptResult, objective_value::Float64)::OptResult
    return OptResult(
        result.termination_status,
        objective_value,
        result.solution,
        result.runtime_sec,
        result.model,
        result.mapping,
        result.detour_combos,
        result.counts,
        result.warm_start_solution,
        result.metadata,
        result.duals,
    )
end
