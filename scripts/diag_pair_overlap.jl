"""
    scripts/diag_pair_overlap.jl

Structural (no-solve) overlap analysis of the PFA instances, to test what makes a
scenario's pricing tractable. For each (n, scen, seed) it builds only the master
DATA (no Gurobi, no pricing) and reports overlap metrics of the passengers'
feasible assignments:

  breadth        mean |feasible (j,k)| per passenger
  load_mean/max  passengers sharing a station (as pickup or dropoff endpoint)
  jaccard        mean pairwise Jaccard of passengers' feasible pickup-station sets
                 -- the direct "do pairs want the same stations" overlap measure
  hub_frac       fraction of endpoint stations carrying > half the passengers

Higher load / jaccard / hub_frac == more overlap == (hypothesis) harder pricing.
"""

using Printf, Statistics, StationSelection
include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))

_l_for(n) = max(2, ceil(Int, n / 2))

function metrics_for(n::Int, n_scen::Int, seed::Int)
    data, _ = generate_zhuzhou_data(DATA_DIR, n, 16; n_scenarios=n_scen, seed=seed)
    model = AggregateODRouteModel(
        _l_for(n);
        route_regularization_weight=10.0, walk_cost_weight=0.1, repositioning_time=20.0,
        max_walking_distance=600.0, max_wait_time=900.0, detour_factor=2.0,
        max_stops=typemax(Int), max_visits_per_node=3,
    )
    mapping = create_map(model, data)
    md = create_passenger_free_assignment_master_data(model, data, mapping)

    # per scenario, average the metrics (each scenario's pricing is separate)
    breadths=Float64[]; load_means=Float64[]; load_maxes=Float64[]
    jaccards=Float64[]; hub_fracs=Float64[]
    for (s, pids) in md.passengers_by_scenario
        isempty(pids) && continue
        push!(breadths, mean(length(md.feasible_assignments[p]) for p in pids))
        # station load: passengers listing station j as pickup or dropoff
        load = Dict{Int, Int}()
        pickup_sets = Dict{Int, Set{Int}}()
        for p in pids
            ps = Set(md.feasible_pickups[p]); pickup_sets[p] = ps
            for j in union(Set(md.feasible_pickups[p]), Set(md.feasible_dropoffs[p]))
                load[j] = get(load, j, 0) + 1
            end
        end
        loads = collect(values(load))
        push!(load_means, mean(loads)); push!(load_maxes, maximum(loads))
        push!(hub_fracs, count(>(length(pids) / 2), loads) / length(loads))
        # mean pairwise Jaccard of feasible pickup-station sets
        js=Float64[]
        pv=collect(pids)
        for a in 1:length(pv), b in (a+1):length(pv)
            A=pickup_sets[pv[a]]; B=pickup_sets[pv[b]]
            u=length(union(A,B)); u==0 && continue
            push!(js, length(intersect(A,B))/u)
        end
        push!(jaccards, isempty(js) ? 0.0 : mean(js))
    end
    return (n=n, scen=n_scen, seed=seed, l=_l_for(n),
            breadth=mean(breadths), load_mean=mean(load_means), load_max=mean(load_maxes),
            jaccard=mean(jaccards), hub_frac=mean(hub_fracs))
end

function main()
    println("n  sc seed |  l | breadth | load_mean load_max | jaccard | hub_frac")
    for n in (20,25,30), scen in (1,3), seed in (42,43)
        m = metrics_for(n, scen, seed)
        @printf("%-2d %-2d %-4d | %-2d | %7.1f | %9.1f %8.1f | %7.3f | %7.3f\n",
            m.n, m.scen, m.seed, m.l, m.breadth, m.load_mean, m.load_max, m.jaccard, m.hub_frac)
    end
end

main()
