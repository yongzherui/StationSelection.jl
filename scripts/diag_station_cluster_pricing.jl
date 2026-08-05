# Empirical exact-vs-adaptive-cluster pricing check on Zhuzhou n=10,15.
using Printf, StationSelection
include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS=16; const MAX_WALK=600.0
const BETA=10.0; const W=900.0; const REPOSITION=20.0
const DETOUR=2.0; const WALK_WEIGHT=0.1; const BASE_REWARD=5000.0

function fixture(n,seed,scenario)
    data,_=generate_zhuzhou_data(DATA_DIR,n,N_PAIRS;n_scenarios=3,seed=seed)
    travel=Dict{Tuple{Int,Int},Float64}()
    for i in 1:n,j in 1:n
        i==j || (travel[(i,j)]=get_routing_cost(data,i,j))
    end
    candidates=PassengerAssignmentCandidate[]
    for row in eachrow(data.scenarios[scenario].requests),j in 1:n,k in 1:n
        j==k && continue
        wo=get_walking_cost(data,row.origin_idx,j); wd=get_walking_cost(data,row.dest_idx,k)
        wo<=MAX_WALK && wd<=MAX_WALK || continue
        reward=BASE_REWARD-WALK_WEIGHT*(wo+wd); reward>0 || continue
        push!(candidates,PassengerAssignmentCandidate(row.id,j,k,
            DETOUR*get_routing_cost(data,j,k),reward))
    end
    pd=create_passenger_free_assignment_pricing_data(scenario,collect(1:n),travel,candidates;
        route_regularization_weight=BETA,max_wait_time=W,repositioning_time=REPOSITION,
        max_stops=4,max_visits_per_node=3)
    return pd,candidates,travel
end

function run(n,seed,scenario)
    pd,candidates,travel=fixture(n,seed,scenario)
    println("CASE n=$n seed=$seed scenario=$scenario candidates=$(length(candidates)) passengers=$(length(unique(c.passenger for c in candidates)))")
    exact=price_exact_on_stations(pd,BitSet(pd.nodes);time_limit=300.0,use_reduced_cost_pruning=false)
    @printf("EXACT n=%d seed=%d scenario=%d rc=%.6f certified=%s labels=%d time=%.3f route=%s\n",
        n,seed,scenario,exact.reduced_cost,string(exact.certified),exact.labels_generated,exact.runtime_sec,string(exact.route))

    k0=max(2,ceil(Int,n/3))
    cfg=StationClusteringConfig(initial_num_clusters=k0,max_num_clusters=n,
        max_cluster_size=ceil(Int,n/k0)+1,pricing_tolerance=1e-6,
        numerical_tolerance=1e-6,time_limit=300.0)
    h=initial_station_clustering(travel,n,cfg)
    cache=build_cluster_pricing_cache(h,pd,candidates)
    rows=NamedTuple[]
    initial=solve_cluster_pricer(h,cache)
    @printf("INITIAL n=%d seed=%d scenario=%d K=%d lb=%.6f gap_to_exact=%.6f labels=%d time=%.3f route=%s\n",
        n,seed,scenario,length(h.clusters),initial.lower_bound_reduced_cost,
        exact.reduced_cost-initial.lower_bound_reduced_cost,initial.labels_generated,
        initial.runtime_seconds,string(initial.cluster_route))
    result,reason=solve_adaptive_cluster_lower_bound(h,cache;logger=row->push!(rows,row))
    for row in rows
        @printf("REFINE n=%d seed=%d scenario=%d iter=%d K=%d lb=%.6f split=%s score=%.4f time=%.3f stop=%s\n",
            n,seed,scenario,row.iteration,row.num_clusters,row.lower_bound_reduced_cost,
            string(row.selected_cluster_to_split),row.split_score,row.cluster_pricing_time,string(row.stop_reason))
    end
    valid=result.lower_bound_reduced_cost<=exact.reduced_cost+1e-6
    @printf("FINAL n=%d seed=%d scenario=%d K=%d lb=%.6f exact=%.6f valid=%s certified_nonnegative=%s stop=%s labels=%d time=%.3f\n",
        n,seed,scenario,length(h.clusters),result.lower_bound_reduced_cost,exact.reduced_cost,string(valid),
        string(result.certified_no_negative_column),string(reason),result.labels_generated,result.runtime_seconds)
end

if isempty(ARGS)
    for n in (10,15),seed in (42,314,2718),scenario in 1:3
        run(n,seed,scenario)
    end
else
    length(ARGS)==3 || error("usage: diag_station_cluster_pricing.jl <n> <seed> <scenario>")
    run(parse(Int,ARGS[1]),parse(Int,ARGS[2]),parse(Int,ARGS[3]))
end
