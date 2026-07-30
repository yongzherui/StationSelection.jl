"""
    scripts/diag_passenger_vs_aggregate_solution_dump.jl

Localise the exact `beta * repositioning_time` (= 200 at beta=10, repo=20)
discrepancy between the passenger free-assignment CG optimum and the aggregate
`DirectSolver` optimum found by `diag_passenger_cg_vs_aggregate_direct.jl`.

The passenger model reports the LOWER objective on both tested instances, by
exactly one route's repositioning charge, which points at the passenger model
requiring one fewer route than the aggregate model for the same service pattern.
Rather than argue from arithmetic, this dumps both optimal solutions in full and
recomputes each objective from its parts, so the missing 200 can be attributed to
a specific route / assignment / station.

For each side it prints:
  * open stations;
  * every served passenger / OD and the (j, k) it uses;
  * every selected route column: its assignments (or od_pairs), tau, and cost
    contribution `beta * (tau + repositioning)`;
  * a hand-recomputed objective = walking part + route part, checked against the
    solver's reported objective.

If the recomputed totals match their solvers but the route COUNTS differ, the
discrepancy is structural (coverage semantics). If a recomputation disagrees with
its own solver, the bug is in that side's objective accounting.

Usage:
    julia --project=. scripts/diag_passenger_vs_aggregate_solution_dump.jl [n_stations] [n_pairs] [max_stops]
"""

using Printf, Gurobi, JuMP, StationSelection

const MOI = JuMP.MOI

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const SEED = parse(Int, get(ENV, "PFAC_SEED", "42"))
const ROUTE_WEIGHT = parse(Float64, get(ENV, "PFAC_ROUTE_WEIGHT", "10.0"))
const WALK_WEIGHT = parse(Float64, get(ENV, "PFAC_WALK_WEIGHT", "0.1"))
const REPOSITIONING = 20.0
const MAX_WALK = 600.0
const MAX_WAIT = 900.0
const DETOUR = 2.0
const MAX_VISITS = 2

const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

function shared_model(n_stations::Int, max_stops::Int)
    return AggregateODRouteModel(
        _l_for(n_stations);
        assignment_policy            = FreeAggregateODAssignmentPolicy(),
        route_regularization_weight  = ROUTE_WEIGHT,
        walk_cost_weight             = WALK_WEIGHT,
        repositioning_time           = REPOSITIONING,
        max_walking_distance         = MAX_WALK,
        max_wait_time                = MAX_WAIT,
        detour_factor                = DETOUR,
        max_stops                    = max_stops,
        max_visits_per_node          = MAX_VISITS,
        allow_walk_only              = false,
    )
end

function main()
    n_stations = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 8
    n_pairs = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 6
    max_stops = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 3

    @printf("=== solution dump: n=%d p=%d ms=%d beta=%.3g walk=%.3g repo=%.3g ===\n",
        n_stations, n_pairs, max_stops, ROUTE_WEIGHT, WALK_WEIGHT, REPOSITIONING)
    @printf("beta * repositioning = %.3f  (the observed discrepancy)\n\n",
        ROUTE_WEIGHT * REPOSITIONING)

    data, _meta = generate_zhuzhou_data(DATA_DIR, n_stations, n_pairs; n_scenarios=1, seed=SEED)
    model = shared_model(n_stations, max_stops)
    mapping = create_map(model, data)

    # ─────────────── aggregate DirectSolver ───────────────
    println("############ AGGREGATE (x vars, (j,k) coverage) ############")
    agg = StationSelection.run_opt(
        data, model,
        DirectSolver(
            config=SolverConfig(optimizer_env=GRB_ENV, silent=true, mip_gap=0.0),
            max_enumerated_routes=typemax(Int),
            max_enumeration_time_sec=600.0,
        ),
    )
    @printf("termination=%s objective=%.6f\n", agg.termination_status, agg.objective_value)
    am = agg.model
    amap = agg.mapping
    id_map = amap.array_idx_to_station_id

    y_open = [j for j in 1:data.n_stations if value(am[:y][j]) > 0.5]
    println("open stations (idx): $(y_open)  (ids: $([id_map[j] for j in y_open]))")

    agg_walk = 0.0
    println("assignments (od -> (j,k)):")
    x = am[:x]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(amap.Omega_s[s])
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            for (pair_idx, pair) in enumerate(get_valid_jk_pairs(amap, o, d))
                value(x_od[pair_idx]) > 0.5 || continue
                w = WALK_WEIGHT * StationSelection.od_pair_walking_cost(data, o, d, pair)
                agg_walk += w
                needs_route = !StationSelection.requires_no_vehicle_route(pair)
                @printf("  od=(%d,%d) -> (%d,%d)  walk_cost=%.3f  needs_route=%s\n",
                    o, d, pair[1], pair[2], w, needs_route)
            end
        end
    end

    agg_route = 0.0
    n_agg_routes = 0
    println("selected route columns:")
    theta = am[:theta_compat]
    col_by_id = Dict(c.id => c for c in amap.columns)
    for ((cid, s), tv) in theta
        value(tv) > 0.5 || continue
        c = col_by_id[cid]
        contrib = ROUTE_WEIGHT * (c.tau + REPOSITIONING)
        agg_route += contrib
        n_agg_routes += 1
        @printf("  col=%d s=%d tau=%.3f cost=%.3f od_pairs=%s route=%s\n",
            cid, s, c.tau, contrib, string(c.od_pairs),
            string(get(c.metadata, "route", ())))
    end
    @printf("recomputed: walk=%.6f + route=%.6f = %.6f   (solver %.6f)  n_routes=%d\n",
        agg_walk, agg_route, agg_walk + agg_route, agg.objective_value, n_agg_routes)

    # Decisive check: can the AGGREGATE pool cover several pairs with ONE route? If the
    # passenger model serves the same passengers with one fewer route, either such a
    # column exists here (and the aggregate MIP inexplicably declined it) or it does
    # not (and the two certification logics genuinely disagree -- the real suspect).
    @printf("enumerated aggregate columns: %d\n", length(amap.columns))
    # Full dump. The two SELECTED columns printed `route=()`, i.e. they are the
    # `_singleton_aggregate_od_route_columns` (one per active pair, tau =
    # routing_cost(j,k)) rather than enumerated routes -- so it matters exactly what
    # the enumeration itself produced, and whether any revisiting route is present.
    println("ALL aggregate columns (source / route / od_pairs / tau):")
    for c in sort(collect(amap.columns); by=cc -> cc.tau)
        @printf("   [%-11s] tau=%9.3f route=%-22s od_pairs=%s\n",
            string(get(c.metadata, "initialization", "?")),
            c.tau, string(get(c.metadata, "route", ())), string(c.od_pairs))
    end
    println("active pairs for pricing/enumeration: $(sort(collect(filter(!StationSelection.requires_no_vehicle_route, get(amap.active_jk_s, 1, Tuple{Int,Int}[])))))")
    @printf("travel(3,4)=%.3f travel(4,3)=%.3f  ride_limit(3,4)=%.3f ride_limit(4,3)=%.3f  max_wait=%.1f\n",
        get_routing_cost(data, 3, 4), get_routing_cost(data, 4, 3),
        DETOUR * get_routing_cost(data, 3, 4), DETOUR * get_routing_cost(data, 4, 3), MAX_WAIT)
    multi = [c for c in amap.columns if length(c.od_pairs) >= 2]
    @printf("columns covering >= 2 od_pairs: %d\n", length(multi))
    sort!(multi; by=c -> c.tau)
    for c in first(multi, min(10, length(multi)))
        @printf("   tau=%9.3f cost=%9.3f od_pairs=%s\n",
            c.tau, ROUTE_WEIGHT * (c.tau + REPOSITIONING), string(c.od_pairs))
    end
    # specifically: any single column covering every route-needing assigned pair?
    needed = Set{Tuple{Int, Int}}()
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(amap.Omega_s[s])
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            for (pair_idx, pair) in enumerate(get_valid_jk_pairs(amap, o, d))
                value(x_od[pair_idx]) > 0.5 || continue
                StationSelection.requires_no_vehicle_route(pair) || push!(needed, pair)
            end
        end
    end
    println("route-needing pairs in the aggregate optimum: $(sort(collect(needed)))")
    covering_all = [c for c in amap.columns if needed ⊆ Set(c.od_pairs)]
    if isempty(covering_all)
        println(">>> NO enumerated aggregate column covers all of them at once, so the")
        println("    aggregate model MUST buy >= 2 routes. If the passenger model buys 1,")
        println("    the certification logics disagree.")
    else
        best = minimum(c.tau for c in covering_all)
        @printf(">>> %d enumerated column(s) cover all of them; cheapest tau=%.3f cost=%.3f\n",
            length(covering_all), best, ROUTE_WEIGHT * (best + REPOSITIONING))
        println("    (so a 1-route solution WAS available to the aggregate MIP)")
    end
    println()

    # ─────────────── passenger free-assignment CG ───────────────
    println("############ PASSENGER (columns carry (p,j,k)) ############")
    md = create_passenger_free_assignment_master_data(model, data, mapping)
    cg = run_passenger_free_assignment_column_generation(
        model, data; optimizer_env=GRB_ENV,
        max_cg_iters=5000, n_candidates=20, max_new_columns=20,
        pricing_time_limit_sec=120.0, certification_time_limit_sec=600.0,
        ip_time_limit_sec=600.0, total_time_limit_sec=1800.0, verbose=false,
    )
    @printf("stop=%s certified=%s lp=%.6f mip=%.6f unserved=%d\n",
        cg.cg_stop_reason, cg.lp_bound_certified, cg.lp_bound,
        isnothing(cg.mip_objective) ? NaN : cg.mip_objective,
        length(cg.unserved_passengers))
    println("open stations (ids): $(cg.open_stations)")
    println("passengers: $(length(md.passengers))")
    for p in md.passengers
        @printf("  p%-3d o=%-3d d=%-3d demand=%d\n", p.id, p.origin, p.destination, p.demand)
    end

    # Rebuild + re-solve the passenger MIP so the selected columns can be inspected.
    println("(rebuilding the passenger MIP over its own certified pool to dump selections)")
    # Must be built RELAXED: the pricing loop below reads `dual()`, which is only
    # available for an LP. Binaries are imposed afterwards, once the pool is closed.
    reb = build_passenger_free_assignment_master(md, GRB_ENV; relax_integrality=true)
    set_silent(reb.model)
    # regenerate the pool by exhaustive pricing from scratch
    nid = 1
    for _ in 1:2000
        optimize!(reb.model)
        primal_status(reb.model) == MOI.FEASIBLE_POINT || break
        a, go, gd = extract_passenger_free_assignment_duals(reb)
        added = false
        for s in sort!(collect(keys(md.passengers_by_scenario)))
            cands = passenger_free_assignment_pricing_candidates(md, a, go, gd, s)
            isempty(cands) && continue
            pd = create_passenger_free_assignment_pricing_data(
                s, md.nodes, md.travel_cost, cands;
                route_regularization_weight=md.route_regularization_weight,
                max_wait_time=md.max_wait_time, repositioning_time=md.repositioning_time,
                max_stops=md.max_stops, max_visits_per_node=md.max_visits_per_node,
            )
            isempty(pd.opportunities) && continue
            ex = PassengerFreeAssignmentRouteColumn[
                c for c in values(reb.columns) if get(c.metadata, "scenario", 0) == s]
            cols, _e, _st = passenger_free_assignment_pricing_by_label_setting(
                pd, ex; next_column_id=nid, max_new_columns=10^6, n_candidates=10^6, time_limit=300.0)
            nid += length(cols)
            for c in cols
                _t, act = add_passenger_free_assignment_column!(reb, c)
                act == :added && (added = true)
            end
        end
        added || break
    end
    for yv in reb.y; set_binary(yv); end
    for th in values(reb.theta); set_binary(th); end
    for xv in values(reb.x_same); set_binary(xv); end
    optimize!(reb.model)
    @printf("rebuilt passenger MIP objective=%.6f (CG reported %.6f)\n",
        objective_value(reb.model), isnothing(cg.mip_objective) ? NaN : cg.mip_objective)

    p_walk = 0.0
    p_route = 0.0
    n_p_routes = 0
    println("selected same-station (no-vehicle-route) legs:")
    for ((p, j), var) in reb.x_same
        value(var) > 0.5 || continue
        w = WALK_WEIGHT * md.same_station_walk_cost[(p, j)]
        p_walk += w
        @printf("  p%d -> station %d  walk_cost=%.3f\n", p, j, w)
    end
    println("selected route columns:")
    for (cid, var) in reb.theta
        value(var) > 0.5 || continue
        c = reb.columns[cid]
        rc = ROUTE_WEIGHT * (c.tau + REPOSITIONING)
        wc = WALK_WEIGHT * sum(get(md.assignment_walk_cost, a, 0.0) for a in c.assignments; init=0.0)
        p_route += rc
        p_walk += wc
        n_p_routes += 1
        @printf("  col=%d tau=%.3f route_cost=%.3f walk_cost=%.3f assignments=%s route=%s\n",
            cid, c.tau, rc, wc, string(c.assignments), string(get(c.metadata, "route", ())))
    end
    @printf("recomputed: walk=%.6f + route=%.6f = %.6f   n_routes=%d\n\n",
        p_walk, p_route, p_walk + p_route, n_p_routes)

    # ─────────────── verdict ───────────────
    println("############ COMPARISON ############")
    @printf("aggregate: obj=%.6f  n_routes=%d  walk=%.6f  route=%.6f\n",
        agg.objective_value, n_agg_routes, agg_walk, agg_route)
    @printf("passenger: obj=%.6f  n_routes=%d  walk=%.6f  route=%.6f\n",
        p_walk + p_route, n_p_routes, p_walk, p_route)
    @printf("route-count difference   = %d\n", n_agg_routes - n_p_routes)
    @printf("route-cost  difference   = %.6f\n", agg_route - p_route)
    @printf("walk-cost   difference   = %.6f\n", agg_walk - p_walk)
    @printf("beta*repositioning       = %.6f\n", ROUTE_WEIGHT * REPOSITIONING)
    if n_agg_routes != n_p_routes
        println(">>> route COUNTS differ: the discrepancy is structural (coverage semantics),")
        println("    i.e. the two models disagree about how many vehicle routes the same")
        println("    service pattern requires.")
    else
        println(">>> route counts MATCH: the discrepancy is in cost accounting, not structure.")
    end
end

main()
