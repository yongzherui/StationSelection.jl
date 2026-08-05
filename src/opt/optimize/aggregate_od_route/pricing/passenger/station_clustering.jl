"""
Adaptive geographic station clustering for passenger free-assignment pricing.

The construction is deliberately componentwise optimistic.  A cluster arc may
use unrelated physical witnesses for time and cost, and a passenger opportunity
may use yet another pair.  Consequently every physical route maps to a feasible
cluster route of no larger reduced cost:

    LB_cluster <= min physical reduced cost.

A negative value is only a lower bound.  Only an exhausted, nonnegative cluster
search is a pricing certificate.  Refinements are nested and only remove hidden
station-switching choices, so their exact optima are monotone nondecreasing.
"""

struct StationCluster
    id::Int
    stations::Vector{Int}
    medoid::Int
    radius::Float64
    diameter::Float64
end

struct ClusterArcData
    min_time::Float64
    min_cost::Float64
    time_witness::Tuple{Int, Int}
    cost_witness::Tuple{Int, Int}
end

struct ClusterRewardData
    reward::Float64
    origin_station_witness::Int
    destination_station_witness::Int
    feasible::Bool
end

Base.@kwdef struct StationClusteringConfig
    initial_num_clusters::Int
    max_num_clusters::Int
    max_cluster_size::Union{Nothing, Int}=nothing
    max_cluster_radius::Union{Nothing, Float64}=nothing
    clustering_method::Symbol=:farthest_medoid
    split_method::Symbol=:farthest_pair
    distance_weight::Float64=1.0
    reward_weight::Float64=0.2
    witness_inconsistency_weight::Float64=0.5
    reset_refinement_each_cg_iteration::Bool=false
    pricing_tolerance::Float64=1e-6
    numerical_tolerance::Float64=1e-8
    time_limit::Float64=30.0
end

@enum ClusterPricingStopReason begin
    CertifiedNonnegative
    ReachedMaximumClusters
    FoundExactNegativeColumn
    NoSplittableCluster
    TimeLimit
end

struct ClusterPricingResult
    lower_bound_reduced_cost::Float64
    cluster_route::Vector{Int}
    selected_cluster_assignments::Vector{Tuple{Int, Int, Int}}
    labels_generated::Int
    labels_dominated::Int
    runtime_seconds::Float64
    certified_no_negative_column::Bool
end

mutable struct StationClusterHierarchy
    clusters::Vector{StationCluster}
    station_to_cluster::Vector{Int}
    initial_clusters::Vector{StationCluster}
    initial_station_to_cluster::Vector{Int}
    next_cluster_id::Int
    config::StationClusteringConfig
    initial_clustering_time::Float64
end

mutable struct ClusterPricingCache
    arcs::Dict{Tuple{Int, Int}, ClusterArcData}
    rewards::Dict{Tuple{Int, Int, Int}, ClusterRewardData}
    candidates::Vector{PassengerAssignmentCandidate}
    physical_candidates::Vector{PassengerAssignmentCandidate}
    travel_time::Dict{Tuple{Int, Int}, Float64}
    travel_cost::Dict{Tuple{Int, Int}, Float64}
    scenario::Int
    route_regularization_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    max_stops::Int
    max_visits_per_node::Int
    aggregate_build_time::Float64
end

_dist(d, i, j) = get(d, (i, j), i == j ? 0.0 : Inf)

function _cluster_geometry(id, stations, distance)
    isempty(stations) && throw(ArgumentError("a station cluster cannot be empty"))
    medoid = argmin(j -> sum(_dist(distance, j, k) for k in stations), stations)
    radius = maximum(_dist(distance, medoid, j) for j in stations)
    diameter = maximum(_dist(distance, j, k) for j in stations, k in stations)
    StationCluster(id, sort!(collect(stations)), medoid, radius, diameter)
end

function _validate_clustering_config(config, n)
    1 <= config.initial_num_clusters <= config.max_num_clusters <= n ||
        throw(ArgumentError("require 1 <= initial_num_clusters <= max_num_clusters <= num_stations"))
    isnothing(config.max_cluster_size) || config.max_cluster_size >= 1 ||
        throw(ArgumentError("max_cluster_size must be positive"))
    if config.distance_weight <= config.reward_weight
        @warn "station splitting is not distance dominated" distance_weight=config.distance_weight reward_weight=config.reward_weight
    end
end

"""Deterministic farthest-first k-medoids with compactness constraints."""
function initial_station_clustering(distance::Dict{Tuple{Int, Int}, Float64},
                                    num_stations::Int, config::StationClusteringConfig)
    t0 = time()
    _validate_clustering_config(config, num_stations)
    config.clustering_method in (:farthest_medoid, :pmedian, :capacitated_pmedian) ||
        throw(ArgumentError("unsupported clustering_method $(config.clustering_method)"))
    stations = collect(1:num_stations)
    medoids = [argmin(j -> sum(_dist(distance, j, k) for k in stations), stations)]
    while length(medoids) < config.initial_num_clusters
        push!(medoids, argmax(j -> (minimum(_dist(distance, j, m) for m in medoids), -j),
                              setdiff(stations, medoids)))
    end
    groups = [Int[] for _ in medoids]
    # Capacity-aware nearest-medoid assignment controls geographic compactness.
    for j in sort(stations; by=j -> -minimum(_dist(distance, j, m) for m in medoids))
        choices = sort!(collect(eachindex(medoids)); by=q -> (_dist(distance, j, medoids[q]), q))
        q = findfirst(q -> (isnothing(config.max_cluster_size) || length(groups[q]) < config.max_cluster_size) &&
                           (isnothing(config.max_cluster_radius) ||
                            _dist(distance, medoids[q], j) <= config.max_cluster_radius + config.numerical_tolerance), choices)
        isnothing(q) && throw(ArgumentError("compactness constraints make the initial partition infeasible"))
        push!(groups[choices[q]], j)
    end
    any(isempty, groups) && throw(ArgumentError("initial medoid assignment produced an empty cluster"))
    clusters = [_cluster_geometry(i, groups[i], distance) for i in eachindex(groups)]
    mapping = zeros(Int, num_stations)
    for c in clusters, j in c.stations mapping[j] = c.id end
    StationClusterHierarchy(clusters, mapping, deepcopy(clusters), copy(mapping),
                            length(clusters) + 1, config, time()-t0)
end

function _arc_data(a::StationCluster, b::StationCluster, tt, tc)
    tpairs = [(j, k) for j in a.stations for k in b.stations]
    tw = argmin(x -> (_dist(tt, x...), x), tpairs)
    cw = argmin(x -> (_dist(tc, x...), x), tpairs)
    # optimistic lower bound on resource consumption
    ClusterArcData(_dist(tt, tw...), _dist(tc, cw...), tw, cw)
end

function _reward_aggregate(p, a, b, candidates)
    represented = filter(c -> c.passenger == p && c.origin in a.stations &&
                              c.destination in b.stations, candidates)
    isempty(represented) && return ClusterRewardData(0.0, 0, 0, false)
    w = argmax(c -> (c.reward, -c.origin, -c.destination), represented)
    ClusterRewardData(w.reward, w.origin, w.destination, true)
end

function build_cluster_pricing_cache(h::StationClusterHierarchy,
        physical::PassengerFreeAssignmentPricingData,
        candidates::Vector{PassengerAssignmentCandidate};
        travel_time::Dict{Tuple{Int, Int}, Float64}=physical.travel_cost,
        travel_cost::Dict{Tuple{Int, Int}, Float64}=physical.travel_cost)
    t0 = time()
    arcs = Dict{Tuple{Int, Int}, ClusterArcData}()
    for a in h.clusters, b in h.clusters arcs[(a.id, b.id)] = _arc_data(a, b, travel_time, travel_cost) end
    rewards = Dict{Tuple{Int, Int, Int}, ClusterRewardData}()
    passengers = unique(c.passenger for c in candidates)
    relaxed = PassengerAssignmentCandidate[]
    for p in passengers, a in h.clusters, b in h.clusters
        rd = _reward_aggregate(p, a, b, candidates)
        rewards[(p, a.id, b.id)] = rd
        rd.feasible || continue
        represented = filter(c -> c.passenger == p && c.origin in a.stations && c.destination in b.stations, candidates)
        # optimistic upper bound on feasibility allowance
        ride_limit = maximum(c.ride_limit for c in represented)
        push!(relaxed, PassengerAssignmentCandidate(p, a.id, b.id, ride_limit, rd.reward))
    end
    ClusterPricingCache(arcs, rewards, relaxed, copy(candidates), travel_time, travel_cost,
        physical.scenario, physical.route_regularization_weight, physical.repositioning_time,
        physical.max_wait_time, physical.max_stops, physical.max_visits_per_node, time()-t0)
end

"""Recompute only dual-dependent reward/feasibility aggregates; keep geography."""
function refresh_cluster_rewards!(h::StationClusterHierarchy, cache::ClusterPricingCache,
                                  candidates::Vector{PassengerAssignmentCandidate})
    cache.physical_candidates = copy(candidates)
    empty!(cache.rewards); empty!(cache.candidates)
    passengers = unique(c.passenger for c in candidates)
    for p in passengers, a in h.clusters, b in h.clusters
        rd = _reward_aggregate(p,a,b,candidates)
        cache.rewards[(p,a.id,b.id)] = rd
        rd.feasible || continue
        represented=filter(c->c.passenger==p && c.origin in a.stations && c.destination in b.stations,candidates)
        # optimistic upper bound on feasibility allowance
        push!(cache.candidates,PassengerAssignmentCandidate(
            p,a.id,b.id,maximum(c.ride_limit for c in represented),rd.reward))
    end
    cache
end

"""Explicitly reset to K0; never called automatically unless configured by the caller."""
function reset_station_cluster_refinement!(h::StationClusterHierarchy, cache::ClusterPricingCache)
    h.clusters=deepcopy(h.initial_clusters)
    h.station_to_cluster=copy(h.initial_station_to_cluster)
    h.next_cluster_id=length(h.clusters)+1
    empty!(cache.arcs)
    for a in h.clusters,b in h.clusters
        cache.arcs[(a.id,b.id)]=_arc_data(a,b,cache.travel_time,cache.travel_cost)
    end
    refresh_cluster_rewards!(h,cache,cache.physical_candidates)
end

function assert_cluster_lower_bound_coefficients(h, cache; atol=1e-9)
    byid = Dict(c.id => c for c in h.clusters)
    for ((a, b), arc) in cache.arcs, j in byid[a].stations, k in byid[b].stations
        @assert arc.min_time <= _dist(cache.travel_time, j, k) + atol
        @assert arc.min_cost <= _dist(cache.travel_cost, j, k) + atol
    end
    for ((p, a, b), rd) in cache.rewards
        rd.feasible || continue
        for c in cache.physical_candidates
            c.passenger == p && c.origin in byid[a].stations && c.destination in byid[b].stations || continue
            @assert rd.reward + atol >= c.reward
        end
    end
    true
end

function _cluster_pricing_data(h, cache)
    ids = sort!(getfield.(h.clusters, :id))
    byid = Dict(c.id => c for c in h.clusters)
    # A non-singleton intra-cluster assignment needs two logical visits even
    # though both endpoints are the same cluster node.  A shadow copy permits
    # that hidden station switch without selecting either physical endpoint.
    shadows = Dict{Int,Int}()
    next_shadow = maximum(ids) + 1
    for id in ids
        if length(byid[id].stations) > 1 && any(c -> c.origin == id && c.destination == id, cache.candidates)
            shadows[id] = next_shadow; next_shadow += 1
        end
    end
    virtual_ids = vcat(ids, collect(values(shadows)))
    node_to_cluster = Dict(id => id for id in ids)
    for (id, shadow) in shadows node_to_cluster[shadow] = id end
    unified = Dict{Tuple{Int, Int}, Float64}()
    for u in virtual_ids, v in virtual_ids
        arc = cache.arcs[(node_to_cluster[u], node_to_cluster[v])]
        # The current exact pricer uses one routing metric for both elapsed time
        # and beta-weighted route cost.  Taking the smaller component is an
        # optimistic lower bound on both resources when callers supply two.
        # optimistic lower bound on resource consumption
        unified[(u, v)] = min(arc.min_time, arc.min_cost)
    end
    virtual_candidates = PassengerAssignmentCandidate[]
    for c in cache.candidates
        destination = c.origin == c.destination && haskey(shadows, c.origin) ? shadows[c.destination] : c.destination
        push!(virtual_candidates, PassengerAssignmentCandidate(
            c.passenger, c.origin, destination, c.ride_limit, c.reward))
    end
    pd = create_passenger_free_assignment_pricing_data(cache.scenario, virtual_ids, unified, virtual_candidates;
        route_regularization_weight=cache.route_regularization_weight,
        repositioning_time=cache.repositioning_time, max_wait_time=cache.max_wait_time,
        max_stops=cache.max_stops, max_visits_per_node=cache.max_visits_per_node)
    return pd, node_to_cluster
end

function solve_cluster_pricer(h::StationClusterHierarchy, cache::ClusterPricingCache;
                              time_limit::Float64=h.config.time_limit)
    t0 = time(); assert_cluster_lower_bound_coefficients(h, cache)
    pd, node_to_cluster = _cluster_pricing_data(h, cache)
    labels, exhausted, stats = _enumerate_passenger_free_assignment_pricing_labels(pd;
        time_limit=time_limit, reduced_cost_tol=h.config.pricing_tolerance,
        max_visits_per_node=pd.max_visits_per_node, use_reduced_cost_pruning=false)
    best = isempty(labels) ? nothing : argmin(l -> l.reduced_cost, labels)
    lb = exhausted ? (isnothing(best) ? pd.route_regularization_weight * pd.repositioning_time : best.reduced_cost) : -Inf
    virtual_route = isnothing(best) ? Int[] : best.route
    virtual_assignments = isnothing(best) ? Tuple{Int,Int,Int}[] :
        first(_passenger_free_assignment_column_from_route(virtual_route, pd; label_reduced_cost=best.reduced_cost))
    route = [node_to_cluster[x] for x in virtual_route]
    assignments = [(p,node_to_cluster[a],node_to_cluster[b]) for (p,a,b) in virtual_assignments]
    ClusterPricingResult(lb, route, assignments, stats.labels_generated,
        stats.labels_rejected_by_dominance + stats.labels_removed_by_dominance,
        time()-t0, exhausted && lb >= -h.config.pricing_tolerance)
end

function assert_cluster_pricing_lower_bound(cluster_result, exact_reduced_cost; atol=1e-6)
    @assert cluster_result.lower_bound_reduced_cost <= exact_reduced_cost + atol (
        "cluster lower bound $(cluster_result.lower_bound_reduced_cost) exceeds exact pricing optimum $(exact_reduced_cost)")
    true
end

function _split_score(c, result, cache, config)
    route_hits = count(==(c.id), result.cluster_route)
    used = filter(a -> a[2] == c.id || a[3] == c.id, result.selected_cluster_assignments)
    dispersion = 0.0
    witnesses = Set{Int}()
    for (p, a, b) in used
        represented = filter(x -> x.passenger == p &&
            (a == c.id ? x.origin in c.stations : true) &&
            (b == c.id ? x.destination in c.stations : true), cache.physical_candidates)
        isempty(represented) || (dispersion = max(dispersion,
            maximum(x.reward for x in represented)-minimum(x.reward for x in represented)))
        rd = cache.rewards[(p,a,b)]
        a == c.id && push!(witnesses, rd.origin_station_witness)
        b == c.id && push!(witnesses, rd.destination_station_witness)
    end
    for i in eachindex(result.cluster_route)
        result.cluster_route[i] == c.id || continue
        i > 1 && push!(witnesses, cache.arcs[(result.cluster_route[i-1],c.id)].time_witness[2])
        i < length(result.cluster_route) && push!(witnesses, cache.arcs[(c.id,result.cluster_route[i+1])].time_witness[1])
    end
    dscale = max(maximum(x.diameter for x in values(Dict(z.id=>z for z in [c]))), 1e-9)
    rscale = max(maximum((x.reward for x in cache.physical_candidates); init=0.0), 1e-9)
    config.distance_weight*c.diameter/dscale + config.reward_weight*dispersion/rscale +
        config.witness_inconsistency_weight*max(0,length(witnesses)-1) + 1e-6*route_hits
end

function _split_cluster!(h, cache, parent_id)
    idx = findfirst(c -> c.id == parent_id, h.clusters); isnothing(idx) && return false
    parent = h.clusters[idx]; length(parent.stations) >= 2 || return false
    pairs = [(j,k) for j in parent.stations for k in parent.stations if j < k]
    seeds = argmax(x -> (_dist(cache.travel_time, x...), x), pairs)
    left = Int[]; right = Int[]
    for j in parent.stations
        (_dist(cache.travel_time,j,seeds[1]),j) <= (_dist(cache.travel_time,j,seeds[2]),j) ? push!(left,j) : push!(right,j)
    end
    (isempty(left) || isempty(right)) && return false
    c1 = _cluster_geometry(h.next_cluster_id, left, cache.travel_time); h.next_cluster_id += 1
    c2 = _cluster_geometry(h.next_cluster_id, right, cache.travel_time); h.next_cluster_id += 1
    splice!(h.clusters, idx:idx, [c1,c2])
    for j in left h.station_to_cluster[j]=c1.id end
    for j in right h.station_to_cluster[j]=c2.id end
    _rebuild_after_split!(h, cache, parent_id, c1, c2)
    true
end

function _rebuild_after_split!(h, cache, parent_id, c1, c2)
    # Preserve all unaffected cluster-pair cache entries.
    for key in collect(keys(cache.arcs))
        parent_id in key && delete!(cache.arcs, key)
    end
    for child in (c1,c2), other in h.clusters
        cache.arcs[(child.id,other.id)] = _arc_data(child,other,cache.travel_time,cache.travel_cost)
        cache.arcs[(other.id,child.id)] = _arc_data(other,child,cache.travel_time,cache.travel_cost)
    end
    for key in collect(keys(cache.rewards))
        parent_id == key[2] || parent_id == key[3] || continue
        delete!(cache.rewards,key)
    end
    passengers = unique(c.passenger for c in cache.physical_candidates)
    for p in passengers, child in (c1,c2), other in h.clusters
        cache.rewards[(p,child.id,other.id)] = _reward_aggregate(p,child,other,cache.physical_candidates)
        cache.rewards[(p,other.id,child.id)] = _reward_aggregate(p,other,child,cache.physical_candidates)
    end
    empty!(cache.candidates)
    byid = Dict(c.id=>c for c in h.clusters)
    for ((p,a,b),rd) in cache.rewards
        rd.feasible || continue
        represented=filter(c->c.passenger==p && c.origin in byid[a].stations && c.destination in byid[b].stations,cache.physical_candidates)
        push!(cache.candidates,PassengerAssignmentCandidate(p,a,b,maximum(c.ride_limit for c in represented),rd.reward))
    end
end

function solve_adaptive_cluster_lower_bound(h::StationClusterHierarchy, cache::ClusterPricingCache;
        exact_pricer=nothing, logger=nothing)
    cfg=h.config; iteration=1; previous=-Inf; stop=TimeLimit
    cfg.reset_refinement_each_cg_iteration && reset_station_cluster_refinement!(h,cache)
    result=solve_cluster_pricer(h,cache)
    exact_pricing_time=0.0
    while true
        if result.certified_no_negative_column stop=CertifiedNonnegative; break end
        isfinite(result.lower_bound_reduced_cost) || (stop=TimeLimit; break)
        if !isnothing(exact_pricer)
            exact_t0=time()
            exact=exact_pricer(result.cluster_route,result.selected_cluster_assignments)
            exact_pricing_time += time()-exact_t0
            if exact.reduced_cost < -cfg.pricing_tolerance stop=FoundExactNegativeColumn; break end
        end
        length(h.clusters) >= cfg.max_num_clusters && (stop=ReachedMaximumClusters; break)
        remaining_cluster_budget=cfg.max_num_clusters-length(h.clusters)
        remaining_cluster_budget == 0 && (stop=ReachedMaximumClusters; break)
        selection_t0=time()
        candidates=filter(c->length(c.stations)>1 &&
            (c.id in result.cluster_route || any(a->a[2]==c.id||a[3]==c.id,result.selected_cluster_assignments)),h.clusters)
        isempty(candidates) && (stop=NoSplittableCluster; break)
        scores=Dict(c.id=>_split_score(c,result,cache,cfg) for c in candidates)
        selected=argmax(c->(scores[c.id],-c.id),candidates)
        split_selection_time=time()-selection_t0
        t0=time(); _split_cluster!(h,cache,selected.id) || (stop=NoSplittableCluster; break)
        split_rebuild_time=time()-t0
        refined=solve_cluster_pricer(h,cache)
        refined.lower_bound_reduced_cost < result.lower_bound_reduced_cost-cfg.numerical_tolerance &&
            @warn "refined cluster lower bound decreased" before=result.lower_bound_reduced_cost after=refined.lower_bound_reduced_cost
        previous=result.lower_bound_reduced_cost; result=refined; iteration+=1
        isnothing(logger) || logger((iteration=iteration,num_clusters=length(h.clusters),
            lower_bound_reduced_cost=result.lower_bound_reduced_cost,best_exact_reduced_cost=NaN,
            selected_cluster_to_split=selected.id,split_score=scores[selected.id],
            cluster_route=result.cluster_route,
            initial_clustering_time=h.initial_clustering_time,
            aggregate_build_time=cache.aggregate_build_time,
            cluster_pricing_time=result.runtime_seconds,
            split_selection_time=split_selection_time,
            split_rebuild_time=split_rebuild_time,
            exact_pricing_time=exact_pricing_time,stop_reason=nothing))
    end
    isnothing(logger) || logger((iteration=iteration,num_clusters=length(h.clusters),
        lower_bound_reduced_cost=result.lower_bound_reduced_cost,best_exact_reduced_cost=NaN,
        selected_cluster_to_split=nothing,split_score=NaN,cluster_route=result.cluster_route,
        initial_clustering_time=h.initial_clustering_time,
        aggregate_build_time=cache.aggregate_build_time,
        cluster_pricing_time=result.runtime_seconds,
        split_selection_time=0.0,split_rebuild_time=0.0,
        exact_pricing_time=exact_pricing_time,stop_reason=stop))
    return result, stop
end
