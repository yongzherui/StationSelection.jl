"""
    scripts/diag_sample09_route_enumeration_size.jl

Quick, solve-free diagnostic: how many routes does `enumerate_aggregate_od_route_columns`
produce for the real sample_09 fixture at a given n_stations/max_stops, and what's the
distribution of route lengths (stop counts)? No Gurobi solve happens here -- this is
pure combinatorial enumeration, used to size the `direct_enumeration_guide`'s master
(one `theta_direct` binary per enumerated route per scenario) before committing to a
max_stops choice.

Usage:
    DEG_N_STATIONS=15 julia --project=. scripts/diag_sample09_route_enumeration_size.jl
"""

using DataFrames, Gurobi, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

n_stations = parse(Int, get(ENV, "DEG_N_STATIONS", "15"))
l = L_FOR[n_stations]
data = load_sample09(n_stations)

for mode in (:ms3, :ms4, :ms5, :uncapped)
    max_stops = resolve_max_stops(mode, n_stations)
    model = build_model(l, max_stops, MAX_WALK, CFG)
    unit_model = StationSelection._unit_weighted_routing_model(model)
    t0 = time()
    routes = try
        StationSelection.enumerate_aggregate_od_route_columns(
            unit_model, data; max_routes=200_000, time_limit_sec=60.0,
        )
    catch err
        println("mode=$mode max_stops=$max_stops -> FAILED: $(sprint(showerror, err))")
        continue
    end
    elapsed = time() - t0
    lengths = [length(r.metadata["route"]) for r in routes]
    hist = Dict{Int, Int}()
    for len in lengths
        hist[len] = get(hist, len, 0) + 1
    end
    println(
        "n_stations=$n_stations mode=$mode max_stops=$max_stops -> ",
        "n_routes=$(length(routes))  elapsed=$(round(elapsed, digits=2))s  ",
        "stop_count_histogram=$(sort(collect(hist)))",
    )
    flush(stdout)
end
