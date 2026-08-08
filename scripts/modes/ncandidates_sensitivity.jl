"""
    diagnose.jl ncandidates_sensitivity -- how far from the true pricing optimum
    does the `n_candidates` early stop land?

`passenger_free_assignment_pricing_by_label_setting` stops as soon as
`n_candidates` distinct assignment signatures have been accepted (the `stop_if`
path in search.jl, inherited from the aggregate pricer's identical structure).
That is a speed heuristic, not an optimality-preserving one: acceptance is
keyed on signature NOVELTY, so the first N accepted columns are whatever the
frontier happens to surface first, not the N most negative.

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
    julia --project=. scripts/diagnose.jl ncandidates_sensitivity [n_stations]
"""

const _NCAND_N_PAIRS = 16
const _NCAND_SEED = 42
const _NCAND_N_SCENARIOS = 3
const _NCAND_MAX_STOPS = 4
const _NCAND_TIME_LIMIT = 600.0
const _NCAND_SWEEP = [1, 5, 20, 100, 10^6]

function run_ncandidates_sensitivity(args::Vector{String})
    n_stations = isempty(args) ? 10 : parse(Int, args[1])
    println("=== n_candidates sensitivity: n_stations=$n_stations p=$_NCAND_N_PAIRS seed=$_NCAND_SEED max_stops=$_NCAND_MAX_STOPS ===")
    println()

    data, _meta = diag_zz_data(n_stations; n_pairs=_NCAND_N_PAIRS, n_scenarios=_NCAND_N_SCENARIOS, seed=_NCAND_SEED)
    travel_cost = diag_travel_cost(data, n_stations)

    for s in 1:StationSelection.n_scenarios(data)
        candidates = diag_scenario_candidates(data, n_stations, s)
        isempty(candidates) && continue
        pricing_data = create_passenger_free_assignment_pricing_data(
            s, collect(1:n_stations), travel_cost, candidates;
            route_regularization_weight=10.0, max_wait_time=900.0,
            repositioning_time=20.0, max_stops=_NCAND_MAX_STOPS, max_visits_per_node=3,
        )

        println("--- scenario $s ---")
        @printf("%12s  %14s  %10s  %8s  %12s  %s\n",
            "n_candidates", "best_rc", "exhausted", "columns", "gap_to_opt", "best_route")

        # Establish the optimum FIRST (largest/exhaustive setting), so every row's
        # gap is printed rather than only the last one's.
        optimum = nothing
        for nc in sort(_NCAND_SWEEP; rev=true)
            columns, exhausted, _stats = passenger_free_assignment_pricing_by_label_setting(
                pricing_data, PassengerFreeAssignmentRouteColumn[];
                next_column_id=1, max_new_columns=nc, n_candidates=nc, time_limit=_NCAND_TIME_LIMIT,
            )
            best_rc = isempty(columns) ? Inf : minimum(c.metadata["reduced_cost"] for c in columns)
            best_route = isempty(columns) ? Int[] :
                columns[argmin([c.metadata["reduced_cost"] for c in columns])].route
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

register_mode!("ncandidates_sensitivity", run_ncandidates_sensitivity)
