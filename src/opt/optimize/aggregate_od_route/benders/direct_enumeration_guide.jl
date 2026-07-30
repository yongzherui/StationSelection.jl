"""
Direct-enumeration-guided Benders for `AggregateODRouteModel` (`BendersY`/`BendersYZ`,
`lifted_walking_objective=true` only). See `BendersSolver`'s `direct_enumeration_guide`
docstring (`solver_types.jl`) for the two-phase contract this module implements.

Phase 1 augments the usual lifted master with the complete enumerated route universe as
an additional, exact routing-cost term (`theta_direct`), guiding the master's own `y_hat`
choices and harvesting the cuts the outer loop derives along the way. Phase 2 rebuilds
the plain lifted master (no enumerated pool) seeded with those harvested cuts and re-runs
the outer loop to a certified result.
"""

"""
    _add_direct_enumeration_guide!(master, data, model, requests, feasible_pairs, x_by_pair_full, full_pool) -> AffExpr

Adds one binary `theta_direct[idx, s]` per (enumerated column, scenario) to `master`, plus
route-covering constraints linking them to the master's own per-request assignment
`x_by_pair_full` (built by `_add_nearest_open_master_walking_cost!`), mirroring the
identical coverage-row shape already used in `_build_nearest_open_y_subproblem_lp`
(`y.jl`) / `_build_yz_route_subproblem_lp` (`yz.jl`):

    sum(theta_direct[idx, s] for idx covering (request's pair)) >= x_by_pair_full[(request, pair)]

Returns the exact (unit-route-regularization-weight) routing-cost `AffExpr` for the
caller to fold into the master's objective, scaled by the caller's own `current_beta`
(matching how `theta`'s coefficient is applied today). A `pair -> column indices` reverse
index is built once here rather than scanning `full_pool` per `(request, pair)`, since the
pool can be large.
"""
function _add_direct_enumeration_guide!(
    master::Model,
    data::StationSelectionData,
    model::AggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    x_by_pair_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
    full_pool::Vector{AggregateODRouteColumn};
    relax_integrality::Bool=false,
)::AffExpr
    S = n_scenarios(data)
    pair_to_columns = Dict{Tuple{Int, Int}, Vector{Int}}()
    for (idx, column) in enumerate(full_pool), pair in column.od_pairs
        push!(get!(pair_to_columns, pair, Int[]), idx)
    end

    theta_direct = relax_integrality ?
        @variable(master, [1:length(full_pool), 1:S], lower_bound = 0.0, upper_bound = 1.0) :
        @variable(master, [1:length(full_pool), 1:S], Bin)
    master[:theta_direct] = theta_direct
    for request in requests
        s, _o, _d = request
        for pair in feasible_pairs[request]
            requires_no_vehicle_route(pair) && continue
            covering = get(pair_to_columns, pair, Int[])
            @constraint(
                master,
                sum(theta_direct[idx, s] for idx in covering; init=0.0) >= x_by_pair_full[(request, pair)]
            )
        end
    end

    direct_cost = AffExpr(0.0)
    for (idx, column) in enumerate(full_pool), s in 1:S
        add_to_expression!(
            direct_cost,
            aggregate_od_route_column_objective_coefficient(1.0, model.repositioning_time, column),
            theta_direct[idx, s],
        )
    end
    return direct_cost
end

"""
    _seed_y_cuts!(master, y, theta, seed_cuts)

Replays previously-harvested BendersY cuts (each `(cut_id, cut_constant, coeffs)`, with
`coeffs::Dict{Int,Float64}` over station indices) into a fresh master, before its own
outer loop begins. Every BendersY cut, regardless of `cut_derivation` mode, reduces to
exactly this affine shape (`y_mw_cut.jl`), so no re-derivation is needed to replay one.
"""
function _seed_y_cuts!(master::Model, y, theta, seed_cuts::Vector{<:NamedTuple})::Nothing
    for c in seed_cuts
        @constraint(master, theta[c.cut_id] >= c.cut_constant + sum(c.coeffs[j] * y[j] for j in keys(c.coeffs); init=0.0))
    end
    return nothing
end

"""
    _seed_yz_cuts!(master, theta, seed_cuts)

BendersYZ analogue of [`_seed_y_cuts!`](@ref): `coeffs` keys are `(chain_key, i)` pairs
referencing `master[:nearest_endpoint_chain_cache]`, matching the shape every BendersYZ
cut (`yz_mw_cut.jl`) already reduces to. Requires the walking-cost/chain-cache machinery
(`_add_nearest_open_master_walking_cost!`) to already have run on `master`.
"""
function _seed_yz_cuts!(master::Model, theta, seed_cuts::Vector{<:NamedTuple})::Nothing
    chain_cache = master[:nearest_endpoint_chain_cache]
    for c in seed_cuts
        @constraint(
            master,
            theta[c.cut_id] >= c.cut_constant + sum(c.coeffs[k] * chain_cache[k[1]][k[2]] for k in keys(c.coeffs); init=0.0)
        )
    end
    return nothing
end

"""
    _run_direct_enumeration_guided_benders(data, model, solver) -> OptResult

Orchestrates `BendersSolver(direct_enumeration_guide=true)`'s two phases (see this
module's docstring and `BendersSolver`'s own docstring). Enumerates the full route
universe once via `enumerate_aggregate_od_route_columns` (unit-weighted, matching the
lifted-objective convention already used for subproblem/pricing costs), runs phase 1
(direct-enumeration-augmented master) to harvest cuts, then phase 2 (plain master seeded
with those cuts) to obtain the certified result. Returns phase 2's `OptResult`, with
phase-1 diagnostics recorded under `phase1_*`/`enumerated_routes` metadata keys.
"""
function _run_direct_enumeration_guided_benders(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
)::OptResult
    enumeration_max_stops = isnothing(solver.direct_enumeration_max_stops) ?
        model.max_stops : solver.direct_enumeration_max_stops
    full_pool = enumerate_aggregate_od_route_columns(
        _unit_weighted_routing_model(model; max_stops=enumeration_max_stops), data;
        max_routes=solver.direct_enumeration_max_routes,
        time_limit_sec=solver.direct_enumeration_time_limit_sec,
    )
    run_phase = solver.decomposition isa BendersY ?
        _run_aggregate_od_route_nearest_open_benders_y : _run_aggregate_od_route_nearest_open_benders_yz

    harvested_cuts = NamedTuple[]
    phase1 = run_phase(data, model, solver; direct_enumeration_pool=full_pool, harvested_cuts=harvested_cuts)
    phase2 = run_phase(data, model, solver; seed_cuts=harvested_cuts)

    if !isnothing(phase1.objective_value) && !isnothing(phase2.objective_value)
        isapprox(
            phase1.objective_value, phase2.objective_value;
            atol=1e-6 * max(1.0, abs(phase2.objective_value)),
        ) || @warn(
            "direct_enumeration_guide: phase 1 (direct-enumeration-guided) and phase 2 " *
            "(cut-seeded pure Benders) objectives disagree -- phase 2's result is still " *
            "returned as the certified answer, but this mismatch is worth investigating",
            phase1_objective=phase1.objective_value, phase2_objective=phase2.objective_value,
        )
    end
    phase2.metadata["phase1_objective"] = phase1.objective_value
    phase2.metadata["phase1_iterations"] = get(phase1.metadata, "benders_iterations", nothing)
    phase2.metadata["phase1_cuts_harvested"] = length(harvested_cuts)
    phase2.metadata["phase2_iterations"] = get(phase2.metadata, "benders_iterations", nothing)
    phase2.metadata["enumerated_routes"] = length(full_pool)
    return phase2
end
