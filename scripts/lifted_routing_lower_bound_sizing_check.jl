"""
    scripts/lifted_routing_lower_bound_sizing_check.jl

Sizing check for the planned `lifted_routing_lower_bound` BendersYZ feature (see
notes/2026-07-27_branch_and_price_on_y_design_sketch.md and the plan at
`.claude/plans/ticklish-herding-honey.md` for the design). Before writing the
actual JuMP multicommodity arc-flow construction, this reports -- for the real
`zhuzhou_n30_p16_s*` instances that already timed out in
`experiments/zhuzhou_p16_scaling_route100x/` -- per scenario:

  - `n_requests`  : number of (s,o,d) demand buckets (one commodity each)
  - `n_stations_s`: size of the "relevant station set" (union of every
                    pickup/dropoff candidate station across that scenario's
                    requests) -- NOT all n_stations, only the ones the arc-flow
                    LP would actually need arcs over
  - estimated variable count: `n_stations_s^2 * (1 + n_requests)` (aggregate
    arcs + depot arcs, plus one commodity's own O(n_stations_s^2) arc set per
    request)

No `optimize!()` calls anywhere -- this only loads data and counts, matching
this repo's established exception for job-list/sizing scripts (see
CS_DIRECT_TIME_LIMIT-adjacent scripts and generate_*_job_list.jl for
precedent), so it is safe to run directly rather than via sbatch.

Usage:
    julia --project=. scripts/lifted_routing_lower_bound_sizing_check.jl
"""

using StationSelection

include(joinpath(@__DIR__, "zhuzhou_p16_scaling_route100x.jl"))

function report_sizing(n_stations::Int, seed::Int)
    l = _l_for(n_stations)
    max_stops = resolve_max_stops(:ms4, n_stations)
    data, max_walk = build_instance(FAMILY, n_stations, N_PAIRS, seed, DATA_DIR)
    model = build_model(l, max_stops, max_walk, CFG)
    mapping = StationSelection.create_map(model, data)
    requests, _demand, _feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)

    println("=== n_stations=$n_stations seed=$seed (l=$l) ===")
    for s in sort!(collect(keys(mapping.Q_s)))
        requests_s = filter(r -> r[1] == s, requests)
        stations_s = Set{Int}()
        for (_s, o, d) in requests_s
            for j in StationSelection._nearest_open_endpoint_candidates(data, o, max_walk, :pickup)
                push!(stations_s, j)
            end
            for k in StationSelection._nearest_open_endpoint_candidates(data, d, max_walk, :dropoff)
                push!(stations_s, k)
            end
        end
        n_req = length(requests_s)
        n_st = length(stations_s)
        est_vars = n_st^2 * (1 + n_req)
        @printf("  scenario %d: n_requests=%-5d n_stations_s=%-5d estimated_vars=%d\n", s, n_req, n_st, est_vars)
    end
    println()
end

function main()
    for seed in SEEDS
        report_sizing(30, seed)
    end
end

main()
