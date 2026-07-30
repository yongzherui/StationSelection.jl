"""
    scripts/bench_passenger_free_assignment_labels.jl

Fixed benchmark for the passenger free-assignment label search, used to accept or
reject incremental pruning/dominance changes to
`src/opt/optimize/aggregate_od_route/pricing/passenger/`.

Unlike `diag_passenger_free_assignment_pricing_scale.jl` (which stops at
`n_candidates=5` and therefore measures "time to five columns", a quantity that
moves when acceptance order changes even if nothing got faster), this runs the
search to **genuine exhaustion** and reports both a correctness invariant and a
cost metric:

  - `best_rc` -- the most negative reduced cost found. Every exact change must
    leave this bit-identical; a change in `best_rc` means the pruning removed a
    real optimum, not redundant work. (Non-exact / IP-only options are expected
    to change it, and are flagged as such where they are introduced.)
  - `wall`, `labels_generated`, `labels_removed_by_dominance`, `max_live` --
    the cost side. A change is worth keeping when these drop at unchanged
    `best_rc`.

Output is one `RESULT` line per (n_stations, max_stops, scenario), tab-separated
and stable across runs, so two runs can be diffed directly.

Usage:
    julia --project=. scripts/bench_passenger_free_assignment_labels.jl [--stations 10,12] [--max-stops 4,5]
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
const MAX_VISITS_PER_NODE = 3
const TIME_LIMIT_SEC = 900.0
# Exhaustive: never let the driver's own early-return fire.
const N_CANDIDATES = 1_000_000_000

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

function run_case(n_stations::Int, max_stops::Int)
    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )

    nodes = collect(1:n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        travel_cost[(i, j)] = get_routing_cost(data, i, j)
    end

    for s in 1:StationSelection.n_scenarios(data)
        candidates = build_scenario_candidates(data, n_stations, s)
        isempty(candidates) && continue

        pricing_data = create_passenger_free_assignment_pricing_data(
            s, nodes, travel_cost, candidates;
            route_regularization_weight=ROUTE_REG_WEIGHT,
            max_wait_time=MAX_WAIT_TIME,
            repositioning_time=REPOSITIONING_TIME,
            max_stops=max_stops,
            max_visits_per_node=MAX_VISITS_PER_NODE,
        )

        t0 = time()
        columns, exhausted, stats = passenger_free_assignment_pricing_by_label_setting(
            pricing_data, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=1, n_candidates=N_CANDIDATES,
            time_limit=TIME_LIMIT_SEC, profile=true,
        )
        wall = time() - t0
        best_rc = isempty(columns) ? NaN : columns[1].metadata["reduced_cost"]
        best_route = isempty(columns) ? Int[] : columns[1].route

        @printf(
            "RESULT\tn=%d\tms=%d\ts=%d\texhausted=%s\twall=%.3f\tlabels=%d\trejected=%d\tremoved=%d\tstale=%d\tmax_live=%d\tbest_rc=%.6f\troute=%s\n",
            n_stations, max_stops, s, exhausted, wall,
            stats.labels_generated, stats.labels_rejected_by_dominance,
            stats.labels_removed_by_dominance, stats.stale_pops,
            stats.max_live_labels, best_rc, string(best_route),
        )
        @printf(
            "PROFILE\tn=%d\tms=%d\ts=%d\tdominance=%.3f\tqueue=%.3f\tcandidates=%.3f\textension=%.3f\n",
            n_stations, max_stops, s,
            stats.t_dominance_sec, stats.t_queue_sec,
            stats.t_candidates_sec, stats.t_extension_sec,
        )
        flush(stdout)
    end
end

"""
Force JIT compilation of the whole pricing path on a throwaway tiny instance, so
the first real `RESULT` row measures the search rather than Julia's compiler.
"""
function warmup()
    nodes = collect(1:4)
    travel_cost = Dict((i, j) => 100.0 * abs(i - j) for i in nodes, j in nodes if i != j)
    candidates = [
        PassengerAssignmentCandidate(1, 1, 2, 500.0, 900.0),
        PassengerAssignmentCandidate(1, 1, 3, 500.0, 700.0),
        PassengerAssignmentCandidate(2, 2, 4, 500.0, 800.0),
    ]
    pd = create_passenger_free_assignment_pricing_data(
        1, nodes, travel_cost, candidates;
        route_regularization_weight=1.0, max_wait_time=200.0,
        repositioning_time=0.0, max_stops=3, max_visits_per_node=2,
    )
    passenger_free_assignment_pricing_by_label_setting(
        pd, PassengerFreeAssignmentRouteColumn[];
        next_column_id=1, max_new_columns=1, n_candidates=N_CANDIDATES, time_limit=10.0,
    )
    return nothing
end

"""
The standard comparison grid. Every case here runs to genuine exhaustion inside
`TIME_LIMIT_SEC` at baseline (n=20/ms=6 does not, so it is deliberately absent --
a timed-out case reports whatever it happened to reach and is not comparable
across variants).
"""
const DEFAULT_CASES = [(15, 5), (15, 6), (20, 5)]

function main()
    cases = DEFAULT_CASES
    i = 1
    while i <= length(ARGS)
        if ARGS[i] == "--cases"
            cases = [(parse(Int, first(sp)), parse(Int, last(sp)))
                     for sp in split.(split(ARGS[i + 1], ","), ":")]
            i += 2
        else
            error("unknown argument $(ARGS[i])")
        end
    end

    warmup()
    println("# bench_passenger_free_assignment_labels cases=$(cases)")
    for (n_stations, max_stops) in cases
        run_case(n_stations, max_stops)
    end
end

main()
