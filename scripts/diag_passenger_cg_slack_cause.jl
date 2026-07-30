"""
    scripts/diag_passenger_cg_slack_cause.jl

Why does one passenger land on the unserved slack at n=10 (driving an apparent
92.7% LP/IP "gap" that is really a big-M artifact)?

Three candidate causes, tested in order of increasing specificity:

  A. The passenger has NO `j != k` feasible assignment at all. The master
     formulation deliberately excludes same-station (`j == k`) assignments since
     they need no vehicle route, so such a passenger is *structurally*
     unservable in this formulation -- a modeling gap on our side, not a
     property of the instance.

  B. `l` is too tight: no choice of `l` open stations admits a feasible
     assignment for every passenger, even ignoring routing entirely. Tested by
     solving a pure station-selection + assignment feasibility MIP (no routes,
     no ride limits) that minimizes the number of unserved passengers.

  C. Routing is the binding constraint: assignments exist and `l` suffices, but
     no route within `max_stops` / `max_visits_per_node` can certify the needed
     `(j, k)` within its ride limit. Diagnosed by elimination plus a direct
     check of whether any single 2-stop route `[j, k]` certifies the assignment.

Also reports, for reference, what changes if same-station assignments were
admitted (cause A's fix) -- i.e. how many passengers only have `j == k` options.

Usage:
    julia --project=. scripts/diag_passenger_cg_slack_cause.jl [n_stations] [n_pairs]
"""

using Printf, Gurobi, JuMP, StationSelection

const MOI = JuMP.MOI

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const SEED = 42
const N_SCENARIOS = 1
const MAX_WALK = 600.0
const MAX_STOPS = 4

const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

function build_model_for(n_stations::Int)
    return AggregateODRouteModel(
        _l_for(n_stations);
        route_regularization_weight = 1.0,
        walk_cost_weight            = 0.1,
        repositioning_time          = 20.0,
        max_walking_distance        = MAX_WALK,
        max_wait_time               = 900.0,
        detour_factor               = 2.0,
        max_stops                   = MAX_STOPS,
        max_visits_per_node         = 3,
    )
end

"""
Cause-B test: ignoring routes entirely, can `l` open stations give every
passenger some feasible `(j, k)` with both endpoints open? Minimizes the number
of unserved passengers, so the answer is quantitative rather than just
feasible/infeasible.

`allow_same_station` controls whether `j == k` assignments count as servable,
which is exactly the difference cause A hinges on.
"""
function min_unserved_ignoring_routes(
    data::StationSelectionData,
    passengers,
    n_stations::Int,
    l::Int,
    max_walk::Float64;
    allow_same_station::Bool,
)
    m = Model(() -> Gurobi.Optimizer(GRB_ENV))
    set_silent(m)
    @variable(m, y[1:n_stations], Bin)
    @constraint(m, sum(y) == l)

    unserved = Dict{Int, VariableRef}()
    for p in passengers
        u = @variable(m, binary = true)
        unserved[p.id] = u
        pairs = Tuple{Int, Int}[]
        for j in 1:n_stations
            get_walking_cost(data, p.origin, j) <= max_walk || continue
            for k in 1:n_stations
                (!allow_same_station && j == k) && continue
                get_walking_cost(data, p.destination, k) <= max_walk || continue
                push!(pairs, (j, k))
            end
        end
        if isempty(pairs)
            @constraint(m, u == 1)
            continue
        end
        # x[p,j,k] = 1 if p uses (j,k); needs y[j] and y[k] open.
        xs = VariableRef[]
        for (j, k) in pairs
            x = @variable(m, binary = true)
            @constraint(m, x <= y[j])
            @constraint(m, x <= y[k])
            push!(xs, x)
        end
        @constraint(m, sum(xs) + u >= 1)
    end
    @objective(m, Min, sum(values(unserved)))
    optimize!(m)
    term = termination_status(m)
    if term != MOI.OPTIMAL
        return nothing, Int[], term
    end
    n_uns = round(Int, objective_value(m))
    who = sort([p for (p, u) in unserved if value(u) > 0.5])
    return n_uns, who, term
end

function main()
    n_stations = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 10
    n_pairs = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 16
    l = _l_for(n_stations)

    println("=== slack cause diagnosis: n_stations=$n_stations n_pairs=$n_pairs l=$l ===")
    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, n_pairs; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    model = build_model_for(n_stations)
    mapping = create_map(model, data)
    md = create_passenger_free_assignment_master_data(model, data, mapping)

    println("passengers: $(length(md.passengers))")
    println()

    # ── cause A: passengers with no j != k feasible assignment ────────────────
    println("--- cause A: structurally unservable (no j != k assignment) ---")
    no_pair = Int[]
    same_only = Int[]
    for p in md.passengers
        n_feas = length(md.feasible_assignments[p.id])
        # same-station options this formulation drops
        n_same = 0
        for j in md.nodes
            get_walking_cost(data, p.origin, j) <= MAX_WALK || continue
            get_walking_cost(data, p.destination, j) <= MAX_WALK || continue
            n_same += 1
        end
        n_feas == 0 && push!(no_pair, p.id)
        (n_feas == 0 && n_same > 0) && push!(same_only, p.id)
        @printf("  p%-3d o=%-3d d=%-3d demand=%d  |A_p (j!=k)|=%-4d  same-station options dropped=%d\n",
            p.id, p.origin, p.destination, p.demand, n_feas, n_same)
    end
    println()
    if isempty(no_pair)
        println("  => every passenger has at least one j != k assignment; cause A RULED OUT")
    else
        println("  => passengers with NO j!=k assignment: $no_pair")
        println("     of those, servable only via dropped same-station options: $same_only")
        println("  => cause A CONFIRMED for $(length(no_pair)) passenger(s)")
    end
    println()

    # ── cause B: is l too tight, ignoring routing? ────────────────────────────
    println("--- cause B: station budget, routing ignored ---")
    n_uns_nosame, who_nosame, t1 = min_unserved_ignoring_routes(
        data, md.passengers, n_stations, l, MAX_WALK; allow_same_station=false,
    )
    n_uns_same, who_same, t2 = min_unserved_ignoring_routes(
        data, md.passengers, n_stations, l, MAX_WALK; allow_same_station=true,
    )
    @printf("  min unserved with l=%d, j!=k only        : %s  (%s) unserved=%s\n",
        l, string(n_uns_nosame), string(t1), string(who_nosame))
    @printf("  min unserved with l=%d, same-station too  : %s  (%s) unserved=%s\n",
        l, string(n_uns_same), string(t2), string(who_same))
    println()

    # ── verdict ───────────────────────────────────────────────────────────────
    println("--- verdict ---")
    if !isnothing(n_uns_nosame) && n_uns_nosame > 0
        if !isnothing(n_uns_same) && n_uns_same < n_uns_nosame
            println("  Cause A (same-station exclusion) is the binding limitation:")
            println("  admitting j == k assignments would reduce forced-unserved from " *
                    "$(n_uns_nosame) to $(n_uns_same).")
            println("  => this is OUR formulation gap, not an instance property.")
        else
            println("  Cause B: l=$l is genuinely too tight -- even ignoring routing AND")
            println("  even allowing same-station assignments, $(n_uns_same) passenger(s)")
            println("  cannot be assigned. The slack is a real instance/budget property.")
        end
    else
        println("  Assignments exist and l=$l suffices ignoring routing, yet CG's MIP still")
        println("  left a passenger on slack => cause C: ROUTING is binding (max_stops=" *
                "$MAX_STOPS / ride limits / max_visits_per_node),")
        println("  or the generated column pool lacks a route certifying the needed pair.")
    end
    println()

    # ── which passenger does the real CG run abandon, and why ─────────────────
    println("--- actual CG run: who is left unserved ---")
    result = run_passenger_free_assignment_column_generation(
        model, data;
        optimizer_env=GRB_ENV,
        max_cg_iters=2000,
        n_candidates=20, max_new_columns=20,
        pricing_time_limit_sec=60.0,
        certification_time_limit_sec=600.0,
        ip_time_limit_sec=300.0,
        total_time_limit_sec=900.0,
        verbose=false,
    )
    println("  cg_stop_reason=$(result.cg_stop_reason) certified=$(result.lp_bound_certified)")
    println("  lp_bound=$(result.lp_bound)  mip=$(result.mip_objective)")
    println("  unserved passengers: $(result.unserved_passengers)")
    for p_id in result.unserved_passengers
        p = md.passengers[findfirst(x -> x.id == p_id, md.passengers)]
        feas = md.feasible_assignments[p_id]
        @printf("  p%d (o=%d d=%d): |A_p|=%d\n", p_id, p.origin, p.destination, length(feas))
        # can any single 2-stop route [j,k] certify one of its assignments?
        certifiable = [(j, k) for (j, k) in feas
                       if md.travel_cost[(j, k)] <= md.ride_limit[(p_id, j, k)] + 1e-9]
        @printf("     assignments certifiable by a direct 2-stop route [j,k]: %d of %d\n",
            length(certifiable), length(feas))
        isempty(certifiable) ||
            @printf("     e.g. %s\n", string(first(certifiable, min(5, length(certifiable)))))
        open_set = Set(result.open_stations)
        open_idx = Set(j for j in md.nodes if data.array_idx_to_station_id[j] in open_set)
        within_open = [(j, k) for (j, k) in feas if j in open_idx && k in open_idx]
        @printf("     assignments with BOTH endpoints among the %d chosen stations: %d\n",
            length(open_idx), length(within_open))
    end
end

main()
