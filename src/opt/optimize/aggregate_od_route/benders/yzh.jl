"""
Benders-YZH decomposition for AggregateODRouteModel (NearestOpen policy only): master =
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
is the currently-implemented way to close that gap empirically. A cheaper, provably-valid
alternative exists but is not implemented: reuse CG's own already-certified dual directly
(zero-extended over `(request, pair)` rows not present in CG's restricted model) instead of
re-solving this subproblem LP at all -- any dual-feasible point tight at `h_hat` gives a valid
cut by LP duality, regardless of which point on a degenerate optimal face is chosen, so CG's own
certified dual suffices without exposure to this LP's own re-solve degeneracy. This is the same
idea as `BendersY`'s `cut_derivation=:zero_completion`
(notes/2026-07-17_restricted_mw_cut_benders_y.md), simplified further for `BendersYZH` since there
is no remaining free y/z/x block left to complete once `h` is fixed -- the certified dual *is*
the whole thing needed for the cut.
"""

"""
    _add_nearest_open_master_h!(master, data, y, physical_pairs, feasible_pairs_by_p, max_walking_distance, allow_walk_only, selector_style)

BendersYZH's master `h`-builder: one continuous `[0,1]` `h[(p,pair)]` per
physical OD pair `p` (not per `(scenario,o,d)`, unlike BendersXY's `x`),
linked to `zp`/`zd` via `_add_nearest_open_endpoint_linked_x!` exactly as
BendersXY's `x` is -- `h` plays the identical collision-blocking role `x`
plays there (`sum(h over pairs)==1` with no diagonal entry unless
walk-only), so BendersYZH's master, like BendersXY's, needs no separate
feasibility-cut branch. Iterating `physical_pairs` (not the flat
per-scenario `requests`) already touches every endpoint any request would,
so this alone populates/reuses `master[:nearest_endpoint_chain_cache]` --
no separate `_add_nearest_open_master_z!` call is needed before this one.
"""
function _add_nearest_open_master_h!(
    master::Model,
    data::StationSelectionData,
    y,
    physical_pairs::Vector{Tuple{Int, Int}},
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    max_walking_distance::Float64,
    allow_walk_only::Bool,
    selector_style::Symbol,
)
    unmet_demand_active = _aggregate_od_route_unmet_demand_active(master)
    h = Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, VariableRef}()
    u = unmet_demand_active ? Dict{Tuple{Int, Int}, VariableRef}() : nothing
    for p in physical_pairs
        o, d = p
        pairs = feasible_pairs_by_p[p]
        for pair in pairs
            h[(p, pair)] = @variable(master, lower_bound = 0.0, upper_bound = 1.0)
        end
        if unmet_demand_active
            u[p] = @variable(master, lower_bound = 0.0, upper_bound = 1.0)
            @constraint(master, sum(h[(p, pair)] for pair in pairs; init=0.0) == u[p])
        else
            @constraint(master, sum(h[(p, pair)] for pair in pairs; init=0.0) == 1.0)
        end
        h_by_pair = Dict(pair => h[(p, pair)] for pair in pairs)
        _add_nearest_open_endpoint_linked_x!(
            master, data, y, o, d, pairs, h_by_pair, max_walking_distance;
            binary=false, allow_walk_only=allow_walk_only, selector_style=selector_style,
        )
    end
    master[:u] = u
    return h
end

"""
    _selected_assignments_from_h(physical_pairs, occurrences, feasible_pairs_by_p, h_hat; unmet_demand_active=false)

Expands a rounded, scenario-compressed `h_hat` back into the flat
`Dict{(s,o,d), (j,k)}` shape `_solve_fixed_route_covering_by_cg` expects --
mirrors `_selected_assignments_from_x`, but each physical pair's selected
station pair is replicated across every scenario in which it occurs.

Under "always feasible" mode (`unmet_demand_active=true`), a physical pair
with no selected `h` (all pairs ~0, i.e. `u[p]≈0`) is genuinely unserved --
skipped entirely rather than thrown, matching `_apply_route_covering_assignments!`'s
tolerance for a missing entry under this mode. Without the mode, a missing
selection is always a real bug.
"""
function _selected_assignments_from_h(
    physical_pairs::Vector{Tuple{Int, Int}},
    occurrences::Dict{Tuple{Int, Int}, Vector{Int}},
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    h_hat::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64};
    unmet_demand_active::Bool=false,
)
    assignments = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
    for p in physical_pairs
        o, d = p
        pairs = feasible_pairs_by_p[p]
        selected_pair = pairs[argmax([get(h_hat, (p, pair), 0.0) for pair in pairs])]
        if get(h_hat, (p, selected_pair), 0.0) < 0.5
            unmet_demand_active && continue
            throw(ArgumentError("BendersYZH master produced no selected assignment for physical pair $(p)"))
        end
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
    model::AnyAggregateODRouteModel,
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

    group_occurrences = Dict{Tuple{Int, Int}, Vector{Int}}()
    for (s, o, d) in group_requests
        push!(get!(group_occurrences, (o, d), Int[]), s)
    end
    group_physical_pairs = collect(keys(group_occurrences))

    h = Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, VariableRef}()
    fix_cons = Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for p in group_physical_pairs, pair in feasible_pairs_by_p[p]
        key = (p, pair)
        h[key] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
        fix_cons[key] = @constraint(m, h[key] == get(h_hat, key, 0.0))
    end

    @variable(m, 0 <= lambda[1:length(columns), 1:n_scenarios(data)] <= 1)
    cover_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for p in group_physical_pairs
        for pair in feasible_pairs_by_p[p]
            # Walk-only and same-station assignments use no vehicle route, so
            # no route column can (or needs to) cover them.
            requires_no_vehicle_route(pair) && continue
            covering = [idx for (idx, column) in enumerate(columns) if pair in column.od_pairs]
            for s in group_occurrences[p]
                con = @constraint(m, sum(lambda[idx, s] for idx in covering; init=0.0) >= h[(p, pair)])
                cover_cons[((s, p[1], p[2]), pair)] = con
            end
        end
    end

    obj = AffExpr(0.0)
    for (idx, column) in enumerate(columns), s in 1:n_scenarios(data)
        add_to_expression!(
            obj,
            aggregate_od_route_column_objective_coefficient(
                model.route_regularization_weight,
                model.repositioning_time,
                column,
            ),
            lambda[idx, s],
        )
    end
    @objective(m, Min, obj)
    return m, fix_cons, cover_cons
end

function _solve_yzh_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    group_requests,
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    h_hat::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64},
    optimizer_env,
    silent::Bool,
)
    m, fix_cons, _cover_cons = _build_yzh_route_subproblem_lp(
        data, model, group_requests, feasible_pairs_by_p, columns, h_hat, optimizer_env, silent
    )
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("BendersYZH route LP subproblem failed with status $(termination_status(m))"))
    return objective_value(m), Dict(key => dual(con) for (key, con) in fix_cons)
end

"""
    _solve_yzh_route_subproblem_lp_with_repricing(data, model, mapping, group_requests, feasible_pairs_by_p, columns, h_hat, optimizer_env, silent; max_reprice_rounds)

Diagnostic/soundness-check companion to [`_solve_yzh_route_subproblem_lp`](@ref), mirroring
`_solve_yz_route_subproblem_lp_with_repricing` exactly: after each LP solve, extracts the
covering-constraint duals (via `_extract_nearest_open_y_subproblem_coverage_duals`, reused
unmodified since `cover_cons` has the identical `(request, pair) => ConstraintRef` shape) and
runs genuine label-setting pricing against them. If pricing finds a column with negative
reduced cost beyond the seeded pool, the pool folds it in and the LP is re-solved, repeating
until pricing finds nothing more or `max_reprice_rounds` is hit -- the same convergence
criterion CG's own pricing uses. Unlike `BendersY`/`BendersYZ`, this is not expected to find
anything (the design argument in this file's module docstring is that CG-priming is already
provably exhaustive for this exact LP since `h` is fixed fully, not merely linked), so this
function exists to make that argument empirically checkable rather than merely assumed.
Re-solving after adding repriced columns must preserve the subproblem objective value; a
change means the original restricted LP value was not certified and the routine throws.
"""
function _solve_yzh_route_subproblem_lp_with_repricing(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    mapping::AggregateODRouteMap,
    group_requests,
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    h_hat::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64},
    optimizer_env,
    silent::Bool;
    max_reprice_rounds::Int=20,
)
    pool = copy(columns)
    v_hat = NaN
    baseline_v_hat = nothing
    max_objective_delta = 0.0
    rho = Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64}()
    n_new_columns_total = 0
    rounds = 0
    fully_exhausted = true
    for round in 1:max_reprice_rounds
        rounds = round
        m, fix_cons, cover_cons = _build_yzh_route_subproblem_lp(
            data, model, group_requests, feasible_pairs_by_p, pool, h_hat, optimizer_env, silent
        )
        optimize!(m)
        primal_status(m) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersYZH repricing subproblem LP failed with status $(termination_status(m))"))
        v_hat = objective_value(m)
        if isnothing(baseline_v_hat)
            baseline_v_hat = v_hat
        else
            objective_delta = abs(v_hat - baseline_v_hat)
            max_objective_delta = max(max_objective_delta, objective_delta)
            objective_delta <= 1e-6 * max(1.0, abs(baseline_v_hat)) || throw(ArgumentError(
                "BendersYZH repricing changed subproblem objective: before=$(baseline_v_hat), " *
                "after=$(v_hat), delta=$(objective_delta). Repricing is expected to certify the " *
                "same LP value, not improve it."
            ))
        end
        rho = Dict(key => dual(con) for (key, con) in fix_cons)

        duals = _extract_nearest_open_y_subproblem_coverage_duals(cover_cons)
        next_column_id = isempty(pool) ? 1 : maximum(column.id for column in pool) + 1
        all_new_columns = AggregateODRouteColumn[]
        pricing_exhausted = true
        for s in 1:n_scenarios(data)
            pricing_duals = _scenario_pricing_duals(duals, s)
            pricing_data = create_aggregate_od_route_pricing_data(model, data, mapping, s, pricing_duals)
            new_columns_s, exhausted_s, _stats = aggregate_od_route_pricing_by_label_setting(
                pricing_data,
                pool,
                pricing_duals;
                next_column_id=next_column_id,
                reduced_cost_tol=model.reduced_cost_tol,
                max_new_columns=model.max_new_columns,
                n_candidates=model.n_candidates,
                time_limit=model.pricing_time_limit_sec,
                max_visits_per_node=model.max_visits_per_node,
            )
            pricing_exhausted &= exhausted_s
            append!(all_new_columns, new_columns_s)
            next_column_id += length(new_columns_s)
        end
        fully_exhausted = pricing_exhausted
        isempty(all_new_columns) && break
        pricing_exhausted ||
            @warn "BendersYZH subproblem repricing: pricing hit its time limit before exhausting the search " *
                "while new columns were still being found -- completeness not fully proven this round" round
        @warn "BendersYZH subproblem repricing found columns beyond the seeded pool -- pool was not complete " *
            "for this subproblem's own dual structure (dual degeneracy or genuine pool gap)" round n_new=length(all_new_columns)
        n_new_columns_total += length(all_new_columns)
        pool = _deduplicate_aggregate_od_route_columns(vcat(pool, all_new_columns))
    end
    return v_hat, rho, pool, n_new_columns_total, rounds, fully_exhausted, max_objective_delta
end

"""
    _run_aggregate_od_route_nearest_open_benders_yzh(data, model, solver)

Benders-YZH (Variant 3): master = `y,z,h`; subproblem = `θ` only. `h` is
scenario-compressed -- one variable per *physical* OD pair `p=(o,d)`
(`_add_nearest_open_master_h!`), weighted in the master objective by
`occurrence_count[p]` (its raw scenario-occurrence count, required -- not
just convenient -- to bit-match `BendersXY`/`BendersYZ`'s flat per-`(s,o,d)`
sum, since neither the canonical objective nor `_assignment_pair_cost`
weight walking cost by demand anywhere). Only `feasibility_cut_style in
(:big_m_nearest, :endpoint_chain)` is supported, same as `BendersYZ`.

Unlike `BendersYZ`, no feasibility-cut branch is needed: `h` is linked to
`zp`/`zd` via the same `_add_nearest_open_endpoint_linked_x!` block `x` uses
in `BendersXY`'s master (`sum(h over pairs)==1`, no diagonal entry unless
walk-only), so it plays the identical collision-blocking role -- every
integer-feasible `(y,z,h)` is already assignment-consistent. Also unlike
`BendersYZ`, no repricing companion is needed either: `h` is fixed *fully* in
the subproblem (`_build_yzh_route_subproblem_lp`), exactly as `BendersXY`'s
`x` is, so CG-priming is always exhaustive for the one LP the cut is drawn
from.
"""
function _run_aggregate_od_route_nearest_open_benders_yzh(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
)
    _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style) ||
        throw(ArgumentError(
            "BendersYZH requires NearestOpenAggregateODAssignmentPolicy(:big_m_nearest) or " *
            "(:endpoint_chain); got :$(model.assignment_policy.feasibility_cut_style) -- :pair_chain has no " *
            "addressable z separate from x, so there is nothing for BendersYZH's master to lift."
        ))
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    mapping = create_map(model, data)
    validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))
    physical_pairs, occurrences, feasible_pairs_by_p = _aggregate_od_route_benders_physical_pairs(mapping)
    occurrence_count = Dict(p => length(occurrences[p]) for p in physical_pairs)
    cut_groups = _benders_cut_groups(requests, solver.cut_mode)
    cut_ids = sort!(collect(keys(cut_groups)))

    unmet_demand_active = !isnothing(model.unmet_demand_penalty)
    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    master[:aggregate_od_route_unmet_demand_penalty] = model.unmet_demand_penalty
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    h = _add_nearest_open_master_h!(
        master, data, y, physical_pairs, feasible_pairs_by_p, model.max_walking_distance, model.allow_walk_only,
        model.assignment_policy.feasibility_cut_style,
    )
    u = unmet_demand_active ? master[:u] : nothing

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
    for cut_id in cut_ids
        add_to_expression!(obj, 1.0, theta[cut_id])
    end
    @objective(master, Min, obj)

    best_result = nothing
    best_ub = Inf
    optimality_cuts = 0
    inner_cg_iters = 0
    total_reprice_columns_found = 0
    total_reprice_rounds = 0
    benders_rows = NamedTuple[]

    for iteration in 1:solver.max_iterations
        master_start = time()
        optimize!(master)
        master_solve_seconds = time() - master_start
        primal_status(master) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersYZH master failed with status $(termination_status(master))"))
        lower_bound = objective_value(master)
        assert_endpoint_chain_near_binary(master)
        assert_service_near_binary(master)

        h_hat = Dict(key => round(value(var)) for (key, var) in h)
        y_hat = [round(value(y[j])) for j in 1:data.n_stations]
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)
        assignments = _selected_assignments_from_h(
            physical_pairs, occurrences, feasible_pairs_by_p, h_hat; unmet_demand_active=unmet_demand_active,
        )

        cg_start = time()
        cg_result = _solve_fixed_route_covering_by_cg(
            data, model, assignments, solver, iteration, _open_station_values(y_hat)
        )
        priming_cg_seconds = time() - cg_start
        inner_cg_iters += cg_result.n_cg_iters
        final_result = cg_result.final_result
        if !isnothing(final_result.objective_value) && final_result.objective_value < best_ub
            best_ub = final_result.objective_value
            best_result = final_result
        end

        iteration_lp_value = 0.0
        cuts_added_this_iteration = 0
        subproblem_lp_seconds = 0.0
        for cut_id in cut_ids
            group_requests = cut_groups[cut_id]
            lp_start = time()
            if solver.reprice_subproblem
                v_hat, rho, _pool, n_new, reprice_rounds, exhausted, _delta = _solve_yzh_route_subproblem_lp_with_repricing(
                    data,
                    model,
                    mapping,
                    group_requests,
                    feasible_pairs_by_p,
                    cg_result.generated_columns,
                    h_hat,
                    optimizer_env,
                    cfg.silent;
                    max_reprice_rounds=solver.max_reprice_rounds,
                )
                total_reprice_columns_found += n_new
                total_reprice_rounds += reprice_rounds
                exhausted ||
                    @warn "BendersYZH subproblem repricing hit max_reprice_rounds without pricing exhaustion" iteration cut_id
            else
                v_hat, rho = _solve_yzh_route_subproblem_lp(
                    data,
                    model,
                    group_requests,
                    feasible_pairs_by_p,
                    cg_result.generated_columns,
                    h_hat,
                    optimizer_env,
                    cfg.silent,
                )
            end
            subproblem_lp_seconds += time() - lp_start
            iteration_lp_value += v_hat
            if theta_hat[cut_id] < v_hat - solver.optimality_tol
                @constraint(master, theta[cut_id] >= v_hat + sum(
                    rho[key] * (h[key] - get(h_hat, key, 0.0)) for key in keys(rho)
                ))
                optimality_cuts += 1
                cuts_added_this_iteration += 1
            end
        end
        push!(benders_rows, (
            iteration=iteration,
            master_status=string(termination_status(master)),
            lower_bound=lower_bound,
            incumbent_objective=isfinite(best_ub) ? best_ub : nothing,
            outer_gap=_outer_gap(lower_bound, best_ub),
            outer_gap_absolute=_outer_gap_absolute(lower_bound, best_ub),
            outer_gap_relative=_outer_gap_relative(lower_bound, best_ub),
            master_solve_seconds=master_solve_seconds,
            priming_cg_seconds=priming_cg_seconds,
            subproblem_lp_seconds=subproblem_lp_seconds,
            cuts_added=cuts_added_this_iteration,
            feasibility_cuts_added=0,
            optimality_cuts_added=optimality_cuts,
            selected_assignment_count=length(assignments),
            generated_column_pool_size=length(cg_result.generated_columns),
            inner_cg_iterations=inner_cg_iters,
        ))
        _flush_benders_iteration_log!(solver, benders_rows)

        if cuts_added_this_iteration == 0
            return _opt_result_from_benders(best_result, Dict{String, Any}(
                "solve_method" => "benders",
                "benders_decomposition" => "BendersYZH",
                "benders_cut_mode" => _benders_cut_mode_name(solver),
                "benders_iterations" => iteration,
                "benders_lower_bound" => lower_bound,
                "benders_incumbent_objective" => best_ub,
                "benders_outer_gap" => _outer_gap(lower_bound, best_ub),
                "benders_outer_gap_absolute" => _outer_gap_absolute(lower_bound, best_ub),
                "benders_outer_gap_relative" => _outer_gap_relative(lower_bound, best_ub),
                "benders_master_solve_time_sec" => master_solve_seconds,
                "benders_priming_cg_time_sec" => priming_cg_seconds,
                "benders_subproblem_lp_time_sec" => subproblem_lp_seconds,
                "feasibility_cuts_added" => 0,
                "optimality_cuts_added" => optimality_cuts,
                "inner_cg_iterations" => inner_cg_iters,
                "benders_lp_value" => iteration_lp_value,
                "best_upper_bound" => best_ub,
                "selected_assignment_count" => length(assignments),
                "generated_column_pool_size" => length(cg_result.generated_columns),
                "feasibility_cut_style" => string(model.assignment_policy.feasibility_cut_style),
                "reprice_subproblem" => solver.reprice_subproblem,
                "total_reprice_columns_found" => total_reprice_columns_found,
                "total_reprice_rounds" => total_reprice_rounds,
            ))
        end
    end
    isnothing(best_result) && throw(ArgumentError("BendersYZH did not find a feasible incumbent"))
    throw(ArgumentError("BendersYZH did not converge within max_iterations=$(solver.max_iterations)"))
end
