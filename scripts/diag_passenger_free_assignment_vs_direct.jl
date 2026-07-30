"""
    scripts/diag_passenger_free_assignment_vs_direct.jl

Ground-truth check for the passenger free-assignment label search: does it find
the SAME optimal pricing route that exhaustive direct enumeration finds?

`diag_passenger_free_assignment_pricing_scale.jl` only established that the
search runs fast and returns improving columns -- it stopped early at
`n_candidates`, so it never proved optimality. This script instead runs the
label search to genuine exhaustion (`n_candidates` effectively unbounded) and
compares its best reduced cost against a brute-force enumeration of every
physical route allowed by the same `max_stops`/`max_visits_per_node` limits.

What this actually validates: the label DP's **dominance rule, remaining-reward
bound admissibility, candidate-node filtering, and start-node restriction** --
i.e. every place the search discards work. If dominance were too aggressive or
the bound inadmissible, the search would prune the true optimum and disagree
here.

What it does NOT independently validate: the route-scoring function itself. Both
sides score routes with `_passenger_free_assignment_column_from_route` (replay
from t=0), so a bug in replay would move both numbers together. Replay is
covered separately by the hand-computed unit tests in
test/opt/test_passenger_free_assignment_pricing.jl, and the label DP's own
reward-layer accounting is cross-checked against replay on every accepted route
by the `@assert` inside that same function -- so the three checks are mutually
reinforcing rather than circular.

Brute force deliberately enumerates routes starting from EVERY station, not just
the positive-candidate endpoints the label search seeds from, so the start-node
restriction in `initial_passenger_free_assignment_pricing_labels` is itself
under test.

Usage:
    julia --project=. scripts/diag_passenger_free_assignment_vs_direct.jl [n_stations ...]

Defaults to n_stations in (10, 15). Uses the same synthetic reward calibration
as diag_passenger_free_assignment_pricing_scale.jl.
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
const BASE_VALUE = 5000.0
const MAX_STOPS = 4
const MAX_VISITS_PER_NODE = 3
const TIME_LIMIT_SEC = 600.0
const TOL = 1e-6

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

"""
Exhaustively enumerate every physical route with `route_length <= max_stops`, no
immediate self-loop, and at most `max_visits_per_node` visits to any station
(the same feasible route set the label search explores), scoring each by route
replay. Returns the best (most negative) reduced cost and a witness route.
"""
function brute_force_best(pricing_data, nodes, max_stops::Int, max_visits::Int)
    best_rc = Inf
    best_route = Int[]
    n_scored = 0
    visit_counts = Dict{Int, Int}()

    route = Int[]
    function recurse!()
        if length(route) >= 2
            _assignments, _tau, rc = StationSelection._passenger_free_assignment_column_from_route(
                copy(route), pricing_data,
            )
            n_scored += 1
            if !isempty(_assignments) && rc < best_rc - 1e-12
                best_rc = rc
                best_route = copy(route)
            end
        end
        length(route) >= max_stops && return
        for nd in nodes
            !isempty(route) && nd == route[end] && continue
            get(visit_counts, nd, 0) < max_visits || continue
            push!(route, nd)
            visit_counts[nd] = get(visit_counts, nd, 0) + 1
            recurse!()
            visit_counts[nd] -= 1
            pop!(route)
        end
    end

    for start in nodes
        push!(route, start)
        visit_counts[start] = 1
        recurse!()
        visit_counts[start] = 0
        pop!(route)
    end
    return best_rc, best_route, n_scored
end

function run_case(n_stations::Int)
    println("=== n_stations=$n_stations n_pairs=$N_PAIRS seed=$SEED max_stops=$MAX_STOPS ===")
    flush(stdout)

    data, meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )

    nodes = collect(1:n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        travel_cost[(i, j)] = get_routing_cost(data, i, j)
    end

    all_match = true
    for s in 1:StationSelection.n_scenarios(data)
        candidates = build_scenario_candidates(data, n_stations, s)
        isempty(candidates) && continue

        pricing_data = create_passenger_free_assignment_pricing_data(
            s, nodes, travel_cost, candidates;
            route_regularization_weight=ROUTE_REG_WEIGHT,
            max_wait_time=MAX_WAIT_TIME,
            repositioning_time=REPOSITIONING_TIME,
            max_stops=MAX_STOPS,
            max_visits_per_node=MAX_VISITS_PER_NODE,
        )

        # n_candidates huge => the stop_if early-exit never fires, so the search
        # runs to genuine exhaustion and its best column really is its optimum.
        t0 = time()
        columns, exhausted, stats = passenger_free_assignment_pricing_by_label_setting(
            pricing_data, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1,
            max_new_columns=10^6,
            n_candidates=10^6,
            time_limit=TIME_LIMIT_SEC,
        )
        search_wall = time() - t0
        search_best = isempty(columns) ? Inf : minimum(c.metadata["reduced_cost"] for c in columns)
        search_route = isempty(columns) ? Int[] :
            columns[argmin([c.metadata["reduced_cost"] for c in columns])].route

        t1 = time()
        bf_best, bf_route, n_scored = brute_force_best(pricing_data, nodes, MAX_STOPS, MAX_VISITS_PER_NODE)
        bf_wall = time() - t1

        matched = isapprox(search_best, bf_best; atol=TOL) ||
            (isinf(search_best) && isinf(bf_best))
        all_match &= matched

        @printf("  scenario %d: search exhausted=%s wall=%.2fs labels=%d columns=%d\n",
            s, exhausted, search_wall, stats.labels_generated, length(columns))
        @printf("    search   best_rc=%.6f route=%s\n", search_best, string(search_route))
        @printf("    brute    best_rc=%.6f route=%s (%d routes scored, %.2fs)\n",
            bf_best, string(bf_route), n_scored, bf_wall)
        @printf("    MATCH=%s%s\n", matched,
            matched ? "" : @sprintf("  <-- MISMATCH, gap=%.6g", search_best - bf_best))
        flush(stdout)
    end
    println(all_match ? "  ALL SCENARIOS MATCH" : "  *** MISMATCH DETECTED ***")
    println()
    return all_match
end

function main()
    n_stations_list = isempty(ARGS) ? [10, 15] : parse.(Int, ARGS)
    ok = true
    for n_stations in n_stations_list
        ok &= run_case(n_stations)
    end
    println(ok ? "OVERALL: label search matches direct enumeration everywhere" :
                 "OVERALL: at least one mismatch -- see above")
    ok || exit(1)
end

main()
