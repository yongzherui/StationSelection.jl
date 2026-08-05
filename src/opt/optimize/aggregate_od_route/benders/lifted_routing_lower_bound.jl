"""
Shared machinery for `BendersSolver(lifted_routing_lower_bound=true)`
(`AggregateODRouteModel` + `NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`,
`BendersYZ` + `cut_mode isa MultiCut`, any `cut_derivation`). See `solver_types.jl`'s
`BendersSolver` docstring for the option's contract.

# What this builds, and why it's a valid lower bound

Grounded in the actual pricing semantics (`pricing/labels.jl`): a route has no
capacity/demand-quantity limit at all ("intentionally no onboard passenger count or capacity
resource") and its cost is *purely* `route_regularization_weight * (tau + repositioning_time)`
-- a function of its own total travel time plus a flat per-route fee, independent of how
many/which OD pairs it certifies. Once the exact wait/detour timing is dropped (a further, still
valid, relaxation direction -- it only *removes* a requirement, never adds one), the one
structural requirement left per served pair is that the route's stop sequence visits the pickup
station before the dropoff station: a reachability/precedence condition, not a capacity or
scheduling one.

That is exactly a **multicommodity arc-flow relaxation**: an unlimited-vehicle-count flow network
over the stations relevant to one scenario (with a virtual depot capturing route count and the
flat `repositioning_time` fee -- deliberately uncapped, so this needs no `max_stops`-based route
count formula and is unaffected by `max_stops` being finite or `typemax(Int)`), where every
demand bucket `(s,o,d)` is a unit commodity that must reach from its *fractionally selected*
pickup station to its *fractionally selected* dropoff station, using only arcs the aggregate
route-flow provides (`g <= f` coupling). Each commodity's supply/demand profile is literally the
`zp`/`zd` nearest-open endpoint selectors the master already builds
(`_nearest_open_endpoint_selectors!`, cached in `m[:nearest_endpoint_chain_cache]`) -- reused
here, not rebuilt, so this adds no new station-selection logic, only a new flow layer on top of
the existing `y`/`zp`/`zd` variables.

This is intentionally looser than reality in two further ways on top of dropping capacity/timing:
there is no route/vehicle identity, so a commodity's path can be stitched together from arcs that
in reality belong to *different* vehicles (an implicit free transfer); and it allows as many
vehicles/routes as needed (`sum(f0)` is uncapped). Both are valid relaxation directions -- they
only make it *easier* to satisfy the LP, never harder, so its optimal value is always
`<= ` the true routing cost for any `y`/`z`.

# Objective term + residual cuts, not a floor constraint

Earlier iteration of this feature added `theta[s] >= route_lb_expr[s]` as a static floor
constraint alongside the existing iteratively-added Benders cuts. Measured empirically (n=15
zhuzhou instances) that this floor genuinely works as a bound -- initial outer gap dropped from
~99.8% to ~33% on one case, and stayed tighter than baseline through most of the run -- but wall
time still got *worse* (up to 7.5x), because the master has to jointly re-solve the arc-flow LP's
variables with `y` at every outer iteration regardless of whether `route_lb_expr` enters via a
constraint or the objective; that part doesn't change here either.

This version instead adds `route_lb_expr[s]` directly as an **objective** term and makes
`theta[s]` the nonnegative residual. The routing subproblem and its cuts remain in full-recourse
units. A full cut `alpha_k + b_k' z` is installed as
`theta[s] >= alpha_k + b_k' z - route_lb_expr[s]`, using the live master expression rather than
an incumbent snapshot. Thus `theta[s] + route_lb_expr[s]` globally bounds the full cut. Measured
on n=15 zhuzhou instances (`:standard`+reprice):
(final objective always matches ground truth), 24-38% fewer outer iterations, and the initial
outer gap drops from ~99.8% to 25-36% at iteration 1 -- the bound is doing real, informative work.
Net wall time is still somewhat worse (1.17-2.8x, down from up to 7.5x under the floor-constraint
version) since the master's own size (the arc-flow variables) is unchanged either way -- the
iteration savings partially, not fully, offset the higher per-iteration master-solve cost.

This applies to all three `cut_derivation` values. `:zero_completion` and
`:restricted_mw_fixed_pi` receive the full certified `Q_bar`, so their completion LP remains the
full routing dual. The common cut-assembly step alone converts its full cut to the equivalent
residual row by subtracting the live `route_lb_expr[s]`.

# Interaction with `lifted_walking_objective`

Callers must pass `subproblem_model`, not `model`: under `lifted_walking_objective=true`,
`subproblem_model = _unit_weighted_routing_model(model)` (`route_regularization_weight=1.0`)
because the master's objective separately re-scales `theta` (and, now, `route_lb_expr`) by
`current_beta` (`benders/yz.jl`); under `lifted_walking_objective=false`, `subproblem_model ===
model` and both are denominated in the model's real weight directly. Using
`model.route_regularization_weight` unconditionally here would silently mis-scale the term
whenever `lifted_walking_objective=true`.
"""

"""
    _build_lifted_routing_lower_bound_exprs!(master, data, subproblem_model, y, cut_ids, requests, feasible_pairs) -> Dict{Int, AffExpr}

Builds, for every `cut_id` in `cut_ids` (each a real scenario id under `MultiCut` -- callers must
not use this under `SingleCut`), the multicommodity arc-flow relaxation described above. Mutates
`master` in place (adds the `f`/`f0`/`fj0`/`g` variables and constraints); returns
`route_lb_exprs::Dict{Int, AffExpr}`, one entry per `cut_id`, for the caller to (a) fold into the
master's own objective and (b) subtract as a live expression from every full-routing cut.
"""
function _build_lifted_routing_lower_bound_exprs!(
    master::Model,
    data::StationSelectionData,
    subproblem_model::AggregateODRouteModel,
    y,
    cut_ids::Vector{Int},
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
)::Dict{Int, AffExpr}
    max_walking_distance = subproblem_model.max_walking_distance
    selector_style = subproblem_model.assignment_policy.feasibility_cut_style
    route_lb_exprs = Dict{Int, AffExpr}()

    for cut_id in cut_ids
        requests_s = filter(request -> request[1] == cut_id, requests)
        if isempty(requests_s)
            route_lb_exprs[cut_id] = AffExpr(0.0)
            continue
        end

        stations_s = Set{Int}()
        for (_s, o, d) in requests_s
            union!(stations_s, _nearest_open_endpoint_candidates(data, o, max_walking_distance, :pickup))
            union!(stations_s, _nearest_open_endpoint_candidates(data, d, max_walking_distance, :dropoff))
        end
        stations_list = sort!(collect(stations_s))

        # Aggregate arc-flow + depot arcs (route start/end -- captures route count and the flat
        # repositioning_time fee, deliberately uncapped).
        f = Dict{Tuple{Int, Int}, VariableRef}()
        for j in stations_list, k in stations_list
            j == k && continue
            f[(j, k)] = @variable(master, lower_bound = 0.0)
            @constraint(master, f[(j, k)] <= y[j])
            @constraint(master, f[(j, k)] <= y[k])
        end
        f0 = Dict{Int, VariableRef}()
        fj0 = Dict{Int, VariableRef}()
        for j in stations_list
            f0[j] = @variable(master, lower_bound = 0.0)
            fj0[j] = @variable(master, lower_bound = 0.0)
            @constraint(master, f0[j] <= y[j])
            @constraint(master, fj0[j] <= y[j])
        end

        # Flow conservation: routes are walks, so in = out at every stop (depot arcs close the loop).
        for j in stations_list
            out_flow = sum(f[(j, k)] for k in stations_list if k != j; init = 0.0) + fj0[j]
            in_flow = sum(f[(k, j)] for k in stations_list if k != j; init = 0.0) + f0[j]
            @constraint(master, out_flow == in_flow)
        end

        # Per-commodity reachability, one commodity per (s,o,d) demand bucket in this scenario.
        for request in requests_s
            _s, o, d = request
            pairs = feasible_pairs[request]
            zp, zd, _real_pairs, pickup_rank, dropoff_rank, sorted_pickups, sorted_dropoffs =
                _nearest_open_endpoint_selectors!(
                    master, data, y, o, d, pairs, max_walking_distance;
                    binary = false, allow_walk_only = subproblem_model.allow_walk_only,
                    selector_style = selector_style,
                )

            net_supply = Dict{Int, AffExpr}()
            for j in sorted_pickups
                net_supply[j] = get(net_supply, j, AffExpr(0.0)) + zp[pickup_rank[j]]
            end
            for k in sorted_dropoffs
                net_supply[k] = get(net_supply, k, AffExpr(0.0)) - zd[dropoff_rank[k]]
            end

            g = Dict{Tuple{Int, Int}, VariableRef}()
            for u in stations_list, v in stations_list
                u == v && continue
                g[(u, v)] = @variable(master, lower_bound = 0.0)
                @constraint(master, g[(u, v)] <= f[(u, v)])
            end
            for u in stations_list
                out_flow = sum(g[(u, v)] for v in stations_list if v != u; init = 0.0)
                in_flow = sum(g[(v, u)] for v in stations_list if v != u; init = 0.0)
                @constraint(master, out_flow - in_flow == get(net_supply, u, AffExpr(0.0)))
            end
        end

        # route_lb_expr: route_regularization_weight * (arc travel time + repositioning fee), in
        # subproblem_model's units (see interaction note above).
        route_lb_expr = AffExpr(0.0)
        for ((j, k), var) in f
            add_to_expression!(route_lb_expr, get_routing_cost(data, j, k), var)
        end
        for (_j, var) in fj0
            add_to_expression!(route_lb_expr, subproblem_model.repositioning_time, var)
        end
        route_lb_exprs[cut_id] = subproblem_model.route_regularization_weight * route_lb_expr
    end
    return route_lb_exprs
end

"""Build one common-OD MCF bound and expose it per scenario for residual Benders cuts."""
function _build_common_od_mcf_lower_bound_exprs!(
    master::Model,
    data::StationSelectionData,
    subproblem_model::AggregateODRouteModel,
    y,
    cut_ids::Vector{Int},
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
)::Dict{Int, AffExpr}
    od_sets = [Set((o, d) for (s, o, d) in requests if s == cut_id) for cut_id in cut_ids]
    common_ods = isempty(od_sets) ? Set{Tuple{Int, Int}}() : intersect(od_sets...)
    representative = first(cut_ids)
    common_requests = [
        request for request in requests
        if request[1] == representative && (request[2], request[3]) in common_ods
    ]
    common_expr = isempty(common_requests) ? AffExpr(0.0) :
        _build_lifted_routing_lower_bound_exprs!(
            master, data, subproblem_model, y, [representative], common_requests, feasible_pairs,
        )[representative]
    # L_common(y) <= Q_s(y) for every scenario s. Classical Benders therefore
    # uses theta_s as the residual Q_s-L_common, exactly as the full lifted MCF
    # mode uses theta_s as Q_s-L_s.
    return Dict(cut_id => common_expr for cut_id in cut_ids)
end
