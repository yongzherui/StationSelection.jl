"""
    scripts/diag_passenger_cg_vs_aggregate_direct.jl

Compare the passenger free-assignment CG scheme against the repository's own
exact reference: **`DirectSolver` on `AggregateODRouteModel` with
`FreeAggregateODAssignmentPolicy`** -- the formulation that keeps `x` assignment
variables and puts coverage on `(j, k)` pairs only.

This is a stronger test than `diag_passenger_cg_vs_direct_full_milp.jl`. That
script enumerates *my own* column definition and so can only prove my code is
internally consistent. This one checks that my passenger-level formulation
describes the **same optimization problem** as the established aggregate model,
by comparing final MIP objectives from two independently-built models.

# Why the two are expected to agree at the integer optimum

Aggregate model: `x_od[(j,k)]` assigns a distinct OD pair to a station pair, and
one route covering `(j,k)` serves *every* OD assigned to `(j,k)` (unbounded
capacity). Passenger model: a column names the passengers it serves, so a single
route covering `(j,k)` appears as one column listing every passenger at `(j,k)`.
Same feasible set of physical outcomes; different encoding.

The passenger model's linking rows are **disaggregated** (one per `(p,j)`),
so its LP relaxation is at least as tight as the aggregate model's. Only the
**integer** optima must coincide -- the LP bounds need not, and a difference there
is expected rather than a bug.

# Objective comparability -- two real differences that must be controlled

  1. **Walking cost is not demand-weighted in the aggregate model.**
     `set_aggregate_od_route_objective!` adds
     `walk_cost_weight * od_pair_walking_cost(o,d,(j,k)) * x_od` once per DISTINCT
     OD pair, with no `Q_s` multiplier. This script's passenger model uses
     `demand * (walk_o + walk_d)`. The two coincide only when every OD demand is 1,
     so that is asserted rather than assumed. (The zhuzhou generator draws distinct
     pairs, so it holds there -- but it is a latent trap for any instance with
     repeated ODs.)
  2. **The passenger model has an unserved slack `v_p`; the aggregate model has
     none** (its assignment row is an equality). Any run leaving `v_p > 0` carries
     a big-M term and is not comparable, so that is checked too.

Route cost is identical on both sides: `beta * (tau + repositioning_time)`
(`aggregate_od_route_column_objective_coefficient` vs
`passenger_free_assignment_column_cost`), verified by reading both.

Weights default to the repo's established `route100x` convention for this
instance family (beta=10.0, walk=0.1; see zhuzhou_p16_scaling_route100x.jl).

Usage:
    julia --project=. scripts/diag_passenger_cg_vs_aggregate_direct.jl [n_stations] [n_pairs] [max_stops]

Env:
    PFAC_ROUTE_WEIGHT  default 10.0
    PFAC_WALK_WEIGHT   default 0.1
    PFAC_SEED          default 42
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
const TOL = 1e-4

const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

"""
The single shared model object. `FreeAggregateODAssignmentPolicy()` is the default
-- keeping `x` and covering only `(j,k)`, which is exactly the reference we want,
NOT the NearestOpen policy used by run_method_compare_task.jl's `build_model`.
"""
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

    @printf("=== passenger CG vs aggregate DirectSolver: n=%d p=%d ms=%d beta=%.3g walk=%.3g ===\n",
        n_stations, n_pairs, max_stops, ROUTE_WEIGHT, WALK_WEIGHT)
    data, _meta = generate_zhuzhou_data(DATA_DIR, n_stations, n_pairs; n_scenarios=1, seed=SEED)
    model = shared_model(n_stations, max_stops)

    # ── comparability guard 1: demands must all be 1 ──────────────────────────
    mapping = create_map(model, data)
    bad_demand = Tuple{Int, Tuple{Int, Int}, Int}[]
    for (s, q) in mapping.Q_s
        for (od, dem) in q
            dem == 1 || push!(bad_demand, (s, od, dem))
        end
    end
    if !isempty(bad_demand)
        println("ABORT: objectives are not comparable -- the aggregate model does NOT")
        println("       demand-weight walking cost, but the passenger model does.")
        println("       Offending (scenario, od, demand): $(first(bad_demand, 5))")
        exit(2)
    end
    println("comparability: all OD demands == 1  (walking-cost conventions coincide)")

    # ── aggregate direct solve (x variables, coverage on (j,k)) ───────────────
    #
    # CRITICAL: `run_opt(data, ::AggregateODRouteModel, ::DirectSolver)` only
    # enumerates when the assignment policy is NearestOpen (see covering.jl:174-192);
    # under FreeAggregateODAssignmentPolicy it falls through to a plain build-and-solve
    # over whatever columns the map already holds -- which is just the SINGLETON pool
    # (one column per active pair, tau = routing_cost(j,k)), and silently ignores
    # max_enumerated_routes / max_enumeration_time_sec.
    #
    # A singleton-only pool cannot express any route serving two pairs, so it is not a
    # valid "direct enumeration" reference. Enumerate explicitly and seed the pool,
    # mirroring what `_run_direct_enumerated_aggregate_od_route` does internally.
    t0 = time()
    agg_result = nothing
    agg_err = ""
    n_enumerated = 0
    try
        enum_cols = enumerate_aggregate_od_route_columns(
            model, data; max_routes=200_000, time_limit_sec=600.0,
        )
        n_enumerated = length(enum_cols)
        enum_model = AggregateODRouteModel(
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
            initial_columns              = enum_cols,
        )
        agg_result = StationSelection.run_opt(
            data, enum_model,
            DirectSolver(config=SolverConfig(optimizer_env=GRB_ENV, silent=true, mip_gap=0.0)),
        )
    catch err
        agg_err = sprint(showerror, err)
        showerror(stderr, err, catch_backtrace()); println(stderr)
    end
    agg_wall = time() - t0
    @printf("enumerated aggregate route columns: %d\n", n_enumerated)
    z_agg = isnothing(agg_result) ? nothing :
        (agg_result.termination_status == MOI.OPTIMAL ? agg_result.objective_value : nothing)
    @printf("AGGREGATE Direct : term=%s obj=%s wall=%.1fs\n",
        isnothing(agg_result) ? "error" : string(agg_result.termination_status),
        isnothing(z_agg) ? "-" : @sprintf("%.6f", z_agg), agg_wall)
    isempty(agg_err) || println("  error: $agg_err")

    # ── passenger free-assignment CG ──────────────────────────────────────────
    t1 = time()
    cg = run_passenger_free_assignment_column_generation(
        model, data; optimizer_env=GRB_ENV,
        max_cg_iters=5000, n_candidates=20, max_new_columns=20,
        pricing_time_limit_sec=120.0, certification_time_limit_sec=600.0,
        ip_time_limit_sec=600.0, total_time_limit_sec=1800.0, verbose=false,
    )
    cg_wall = time() - t1
    @printf("PASSENGER CG     : stop=%s certified=%s lp=%.6f mip=%s unserved=%d pool=%d wall=%.1fs\n",
        cg.cg_stop_reason, cg.lp_bound_certified, cg.lp_bound,
        isnothing(cg.mip_objective) ? "-" : @sprintf("%.6f", cg.mip_objective),
        length(cg.unserved_passengers), cg.n_columns, cg_wall)
    println()

    # ── comparability guard 2: no slack in use ───────────────────────────────
    ok = true
    if !isempty(cg.unserved_passengers)
        println("ABORT: passenger CG left $(length(cg.unserved_passengers)) passenger(s) on the")
        println("       unserved slack, so its objective contains a big-M term that the")
        println("       aggregate model (equality assignment row) has no counterpart for.")
        println("       Not comparable; raise l or enable more assignment options.")
        exit(2)
    end

    if isnothing(z_agg) || isnothing(cg.mip_objective)
        println("SKIP comparison (a solve did not reach OPTIMAL)")
        ok = false
    else
        gap = cg.mip_objective - z_agg
        rel = 100 * gap / max(abs(z_agg), 1e-12)
        @printf("MIP objectives: aggregate=%.6f  passenger=%.6f  diff=%.3g (%.4f%%)\n",
            z_agg, cg.mip_objective, gap, rel)
        if isapprox(cg.mip_objective, z_agg; atol=TOL, rtol=1e-6)
            println("PASS  the two formulations agree at the integer optimum")
        else
            println("FAIL  integer optima DIFFER -- the passenger formulation is not modelling")
            println("      the same problem as the aggregate model. Investigate before trusting")
            println("      any passenger-CG objective.")
            ok = false
        end
        # The passenger LP is disaggregated, hence >= the aggregate LP; and any
        # certified LP bound must not exceed the shared integer optimum.
        if cg.lp_bound_certified && cg.lp_bound > z_agg + TOL
            @printf("FAIL  certified passenger lp_bound (%.6f) exceeds the integer optimum (%.6f)\n",
                cg.lp_bound, z_agg)
            ok = false
        elseif cg.lp_bound_certified
            @printf("PASS  certified passenger lp_bound (%.6f) <= shared integer optimum (%.6f)\n",
                cg.lp_bound, z_agg)
        end
    end

    println()
    println(ok ? "OVERALL: PASS" : "OVERALL: FAIL")
    ok || exit(1)
end

main()
