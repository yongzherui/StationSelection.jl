"""
    scripts/diag_passenger_free_assignment_pricing_scale.jl

Scale smoke test for the new passenger free-assignment pricer
(src/opt/optimize/aggregate_od_route/pricing/passenger/), which has no
master-problem/CG wiring yet -- there is no real RMP producing duals
(alpha_p, gamma^O_pj, gamma^D_pk, w_pjk) to price against. This script fakes a
"cold start" reward (a flat base value minus a walk_cost_weight-scaled walking
penalty, the same knob AggregateODRouteModel uses) so the label search has a
realistic-sized candidate set to chew on, using the same zhuzhou n_stations/
n_pairs instance family and CFG values as
scripts/zhuzhou_p16_scaling_route100x.jl (n_pairs=16, 3 scenarios,
route_regularization_weight=10.0, max_wait_time=900s, detour_factor=2.0,
repositioning_time=20.0, max_stops=4 == "ms4").

Each request IS one passenger here (unlike the aggregate pricer, which prices
per aggregated station-OD pair): generate_zhuzhou_data gives every request a
globally unique :id, and every request has its own true origin/destination
station, matching the passenger pricer's `PassengerAssignmentCandidate`
(passenger, origin, destination, ride_limit, reward) contract directly.

Usage:
    julia --project=. scripts/diag_passenger_free_assignment_pricing_scale.jl [n_stations ...]

Defaults to n_stations in (10, 15) if no arguments given.
"""

using Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = 16
const SEED = 42
const N_SCENARIOS = 3
const MAX_WALK = 600.0
const ROUTE_REG_WEIGHT = 10.0
const MAX_WAIT_TIME = 900.0
const REPOSITIONING_TIME = 20.0
const DETOUR_FACTOR = 2.0
const WALK_COST_WEIGHT = 0.1
# Calibrated so a route actually has a chance to pay for itself: routing costs
# here run ~30-800s (mean ~400s) and route_regularization_weight=10.0, so even
# a single hop costs ~4000 in the objective -- BASE_VALUE needs to be on that
# same order, not an arbitrary small constant, or every route trivially prices
# out non-improving regardless of whether the search/certification logic works.
const BASE_VALUE = 5000.0
const MAX_STOPS = 4
const MAX_VISITS_PER_NODE = 3
const TIME_LIMIT_SEC = 180.0

function build_scenario_candidates(data::StationSelectionData, n_stations::Int, s::Int)
    candidates = PassengerAssignmentCandidate[]
    requests = data.scenarios[s].requests
    for row in eachrow(requests)
        o, d = row.origin_idx, row.dest_idx
        for j in 1:n_stations
            walk_o = get_walking_cost(data, o, j)
            walk_o <= MAX_WALK || continue
            for k in 1:n_stations
                k == j && continue
                walk_d = get_walking_cost(data, d, k)
                walk_d <= MAX_WALK || continue
                reward = BASE_VALUE - WALK_COST_WEIGHT * (walk_o + walk_d)
                reward > 0 || continue
                ride_limit = DETOUR_FACTOR * get_routing_cost(data, j, k)
                push!(candidates, PassengerAssignmentCandidate(row.id, j, k, ride_limit, reward))
            end
        end
    end
    return candidates
end

function run_case(n_stations::Int)
    println("=== n_stations=$n_stations n_pairs=$N_PAIRS seed=$SEED ===")
    flush(stdout)

    t_build0 = time()
    data, meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    print_zhuzhou_data_summary(data, meta)
    build_time = time() - t_build0
    @printf("instance built in %.2fs\n", build_time)
    flush(stdout)

    nodes = collect(1:n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        travel_cost[(i, j)] = get_routing_cost(data, i, j)
    end

    for s in 1:StationSelection.n_scenarios(data)
        candidates = build_scenario_candidates(data, n_stations, s)
        if isempty(candidates)
            println("  scenario $s: no positive-reward candidates, skipping")
            flush(stdout)
            continue
        end

        pricing_data = create_passenger_free_assignment_pricing_data(
            s, nodes, travel_cost, candidates;
            route_regularization_weight=ROUTE_REG_WEIGHT,
            max_wait_time=MAX_WAIT_TIME,
            repositioning_time=REPOSITIONING_TIME,
            max_stops=MAX_STOPS,
            max_visits_per_node=MAX_VISITS_PER_NODE,
        )
        n_passengers = length(Set(c.passenger for c in candidates))
        @printf("  scenario %d: %d passengers, %d positive candidates, %d opportunities, %d reward layers\n",
            s, n_passengers, length(candidates), length(pricing_data.opportunities), pricing_data.n_layers)
        flush(stdout)

        t0 = time()
        columns, exhausted, stats = passenger_free_assignment_pricing_by_label_setting(
            pricing_data, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=5, n_candidates=5, time_limit=TIME_LIMIT_SEC,
        )
        wall = time() - t0
        @printf(
            "  scenario %d: wall=%.2fs exhausted=%s labels_generated=%d labels_removed_by_dominance=%d max_frontier=%d max_live=%d columns_found=%d\n",
            s, wall, exhausted, stats.labels_generated, stats.labels_removed_by_dominance,
            stats.max_frontier_size, stats.max_live_labels, length(columns),
        )
        for c in columns
            @printf("    route=%s n_assignments=%d tau=%.1f reduced_cost=%.3f\n",
                string(c.route), length(c.assignments), c.tau, c.metadata["reduced_cost"])
        end
        flush(stdout)
    end
    println()
end

function main()
    n_stations_list = isempty(ARGS) ? [10, 15] : parse.(Int, ARGS)
    for n_stations in n_stations_list
        run_case(n_stations)
    end
end

main()
