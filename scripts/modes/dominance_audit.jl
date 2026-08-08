"""
    diagnose.jl dominance_audit -- census of *which* dominance condition rejects
    each tested label pair in the revisit-tolerant PFA pricer.

The dominance scan is ~90% of that pricer's wall time and it is a chain of
short-circuiting conditions, so the order they are evaluated in is a real
parameter: a condition that rejects 60% of pairs for two instructions belongs
ahead of one that rejects 5% for a bitset traversal. This mode is where the
"most likely to reject, cheapest first" claim in
`_dominates_passenger_free_assignment_in_bucket`'s docstring comes from, and it
is what to re-run before reordering those conditions again.

Reading the output: `share` is the fraction of all *tested pairs* that this
condition was the first to reject. A condition late in the chain with a large
share is a candidate to move earlier -- but only if it is cheap; the trade is
against what it costs to evaluate on the pairs that reach it. `dominates` is
the pairs that survived every condition.

Instrumentation is a type-parameter specialization, not a runtime flag, so a
production search compiles the counters out entirely. That does mean this
mode measures a *differently compiled* pricer than production: use it for the
rejection distribution, never for wall-clock comparisons.

Usage:
    julia --project=. scripts/diagnose.jl dominance_audit [--cases 15:6:1,20:5:3]
"""

const _DOM_AUDIT_N_PAIRS = 16
const _DOM_AUDIT_SEED = 42
const _DOM_AUDIT_N_SCENARIOS = 3
const _DOM_AUDIT_N_CANDIDATES = 1_000_000_000

function _dominance_audit_case(n_stations::Int, raw_max_stops::Int, scenario::Int; time_limit::Float64)
    max_stops = diag_unbounded(raw_max_stops)
    data, _meta = diag_zz_data(n_stations; n_pairs=_DOM_AUDIT_N_PAIRS, n_scenarios=_DOM_AUDIT_N_SCENARIOS, seed=_DOM_AUDIT_SEED)
    travel_cost = diag_travel_cost(data, n_stations)
    candidates = diag_scenario_candidates(data, n_stations, scenario)
    pricing_data = create_passenger_free_assignment_pricing_data(
        scenario, collect(1:n_stations), travel_cost, candidates;
        route_regularization_weight=10.0, max_wait_time=900.0,
        repositioning_time=20.0, max_stops=max_stops, max_visits_per_node=3,
    )

    StationSelection.passenger_free_assignment_dominance_rejections(; reset=true)
    columns, exhausted, stats = passenger_free_assignment_pricing_by_label_setting(
        pricing_data, PassengerFreeAssignmentRouteColumn[];
        next_column_id=1, max_new_columns=_DOM_AUDIT_N_CANDIDATES, n_candidates=_DOM_AUDIT_N_CANDIDATES,
        time_limit=time_limit, dominance_census=true,
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

function run_dominance_audit(args::Vector{String})
    cases = [(15, 6, 1), (20, 5, 3)]
    i = 1
    while i <= length(args)
        if args[i] == "--cases"
            cases = Tuple{Int, Int, Int}[]
            for spec in split(args[i + 1], ",")
                parts = split(spec, ":")
                push!(cases, (parse(Int, parts[1]), parse(Int, parts[2]), parse(Int, parts[3])))
            end
            i += 2
        else
            error("unknown argument $(args[i])")
        end
    end
    time_limit = env_float("PFA_TIME_LIMIT", 900.0)
    println("# dominance_audit cases=$(cases)")
    for (n, ms, s) in cases
        _dominance_audit_case(n, ms, s; time_limit=time_limit)
    end
end

register_mode!("dominance_audit", run_dominance_audit)
