"""
    scripts/audit_pfa_dominance_conditions.jl

Census of *which* dominance condition rejects each tested label pair in the
revisit-tolerant PFA pricer.

The dominance scan is ~90% of that pricer's wall time and it is a chain of
short-circuiting conditions, so the order they are evaluated in is a real
parameter: a condition that rejects 60% of pairs for two instructions belongs
ahead of one that rejects 5% for a bitset traversal. This script is where the
"most likely to reject, cheapest first" claim in
`_dominates_passenger_free_assignment_in_bucket`'s docstring comes from, and it
is what to re-run before reordering those conditions again.

Reading the output: `share` is the fraction of all *tested pairs* that this
condition was the first to reject. A condition late in the chain with a large
share is a candidate to move earlier -- but only if it is cheap; the trade is
against what it costs to evaluate on the pairs that reach it. `dominates` is the
pairs that survived every condition.

Instrumentation is a type-parameter specialization, not a runtime flag, so a
production search compiles the counters out entirely. That does mean this script
measures a *differently compiled* pricer than production: use it for the
rejection distribution, never for wall-clock comparisons.

Usage:
    julia --project=. scripts/audit_pfa_dominance_conditions.jl [--cases 15:6:1,20:5:3]
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
const N_CANDIDATES = 1_000_000_000
const TIME_LIMIT_SEC = parse(Float64, get(ENV, "PFA_TIME_LIMIT", "900"))

function build_scenario_candidates(data::StationSelectionData, n_stations::Int, s::Int)
    candidates = PassengerAssignmentCandidate[]
    for row in eachrow(data.scenarios[s].requests)
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
                push!(candidates, PassengerAssignmentCandidate(
                    row.id, j, k, DETOUR_FACTOR * get_routing_cost(data, j, k), reward,
                ))
            end
        end
    end
    return candidates
end

function audit_case(n_stations::Int, raw_max_stops::Int, scenario::Int)
    max_stops = raw_max_stops <= 0 ? typemax(Int) : raw_max_stops
    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    nodes = collect(1:n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j || (travel_cost[(i, j)] = get_routing_cost(data, i, j))
    end
    pricing_data = create_passenger_free_assignment_pricing_data(
        scenario, nodes, travel_cost, build_scenario_candidates(data, n_stations, scenario);
        route_regularization_weight=ROUTE_REG_WEIGHT,
        max_wait_time=MAX_WAIT_TIME,
        repositioning_time=REPOSITIONING_TIME,
        max_stops=max_stops,
        max_visits_per_node=MAX_VISITS_PER_NODE,
    )

    StationSelection.passenger_free_assignment_dominance_rejections(; reset=true)
    columns, exhausted, stats = passenger_free_assignment_pricing_by_label_setting(
        pricing_data, PassengerFreeAssignmentRouteColumn[];
        next_column_id=1, max_new_columns=N_CANDIDATES, n_candidates=N_CANDIDATES,
        time_limit=TIME_LIMIT_SEC, dominance_census=true,
    )
    census = StationSelection.passenger_free_assignment_dominance_rejections()
    tested = sum(last, census)

    @printf("CASE\tn=%d\tms=%d\ts=%d\texhausted=%s\tlabels=%d\ttested_pairs=%d\tpairs_per_label=%.1f\n",
            n_stations, raw_max_stops, scenario, exhausted, stats.labels_generated,
            tested, tested / max(stats.labels_generated, 1))
    for (name, count) in census
        @printf("CENSUS\tn=%d\tms=%d\ts=%d\tcondition=%-18s\tcount=%12d\tshare=%.4f\n",
                n_stations, raw_max_stops, scenario, name, count, count / max(tested, 1))
    end
    flush(stdout)
end

function main()
    cases = [(15, 6, 1), (20, 5, 3)]
    i = 1
    while i <= length(ARGS)
        if ARGS[i] == "--cases"
            cases = Tuple{Int, Int, Int}[]
            for spec in split(ARGS[i + 1], ",")
                parts = split(spec, ":")
                push!(cases, (parse(Int, parts[1]), parse(Int, parts[2]), parse(Int, parts[3])))
            end
            i += 2
        else
            error("unknown argument $(ARGS[i])")
        end
    end
    println("# audit_pfa_dominance_conditions cases=$(cases)")
    for (n, ms, s) in cases
        audit_case(n, ms, s)
    end
end

main()
