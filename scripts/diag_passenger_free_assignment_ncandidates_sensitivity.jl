"""
    scripts/diag_passenger_free_assignment_ncandidates_sensitivity.jl

How far from the true pricing optimum does the `n_candidates` early stop land?

`passenger_free_assignment_pricing_by_label_setting` stops as soon as
`n_candidates` distinct assignment signatures have been accepted (the `stop_if`
path in search.jl, inherited from the aggregate pricer's identical structure).
That is a speed heuristic, not an optimality-preserving one: acceptance is keyed
on signature NOVELTY, so the first N accepted columns are whatever the frontier
happens to surface first, not the N most negative.

This sweeps n_candidates and reports, per setting:
  - the best (most negative) reduced cost among returned columns,
  - whether the search actually exhausted,
  - the gap to the exhaustive optimum.

Why this matters for a future CG loop:
  * CORRECTNESS: a CG loop may only declare `:optimality_proven` when pricing
    returns NO columns AND `exhausted == true`. Returning no columns after an
    early stop or timeout proves nothing. (The existing aggregate loop in
    pricing/column_generation.jl already makes exactly this distinction --
    `pricing_exhausted ? :optimality_proven : :no_columns_not_exhausted`.)
  * EFFICIENCY: weak columns don't break the LP bound, but they lengthen the
    loop. This quantifies how weak "first 5 signatures" actually is.

Usage:
    julia --project=. scripts/diag_passenger_free_assignment_ncandidates_sensitivity.jl [n_stations]
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
const N_CANDIDATES_SWEEP = [1, 5, 20, 100, 10^6]

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

function main()
    n_stations = isempty(ARGS) ? 10 : parse(Int, ARGS[1])
    println("=== n_candidates sensitivity: n_stations=$n_stations p=$N_PAIRS seed=$SEED max_stops=$MAX_STOPS ===")
    println()

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
            max_stops=MAX_STOPS,
            max_visits_per_node=MAX_VISITS_PER_NODE,
        )

        println("--- scenario $s ---")
        @printf("%12s  %14s  %10s  %8s  %12s  %s\n",
            "n_candidates", "best_rc", "exhausted", "columns", "gap_to_opt", "best_route")

        # Establish the optimum FIRST (largest/exhaustive setting), so every row's
        # gap is printed rather than only the last one's.
        optimum = nothing
        for nc in sort(N_CANDIDATES_SWEEP; rev=true)
            columns, exhausted, _stats = passenger_free_assignment_pricing_by_label_setting(
                pricing_data, PassengerFreeAssignmentRouteColumn[];
                next_column_id=1,
                max_new_columns=nc,
                n_candidates=nc,
                time_limit=TIME_LIMIT_SEC,
            )
            best_rc = isempty(columns) ? Inf : minimum(c.metadata["reduced_cost"] for c in columns)
            best_route = isempty(columns) ? Int[] :
                columns[argmin([c.metadata["reduced_cost"] for c in columns])].route
            # The fully-exhausted setting (last in the sweep) defines the optimum.
            exhausted && (optimum = isnothing(optimum) ? best_rc : min(optimum, best_rc))
            gap = isnothing(optimum) ? NaN : best_rc - optimum
            @printf("%12d  %14.3f  %10s  %8d  %12s  %s\n",
                nc, best_rc, exhausted, length(columns),
                isnan(gap) ? "?" : @sprintf("%.3f", gap), string(best_route))
            flush(stdout)
        end
        println()
    end
end

main()
