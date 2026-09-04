"""
Building one scenario's `RelaxedClusterPricingData`: the augmented cluster
graph, the optimistic travel matrix over it, and the per-passenger
per-cluster-pair reward and ride-limit maxima. Equation numbers refer to
`types.jl`'s module docstring, which carries the relaxation argument this file
implements.

The output is an ordinary `JointRoutingAssignmentPricingData` whose "stations"
are cluster-graph nodes, so **every** downstream piece -- seeding, extension,
dominance, the reward bound, route replay -- is the exact pricer's, reused
verbatim. There is no relaxed-specific label type and no relaxed-specific
search: the relaxation lives entirely in what this file puts into the graph.

# The intra-cluster service arc

A passenger whose whole trip fits inside one cluster has no inter-cluster arc
to be certified on. An earlier version handled that by crediting the reward
*unconditionally on arrival*, with zero internal travel -- valid, but measured
to be the dominant source of slack (`benchmarks/diagnostics/
relaxed_cluster_certification_probe.jl`: 960 units of slack at K=12, where only
two cells held more than one station), because one m-station cell then hands a
route every within-cell passenger for free.

The fix is to make the service a real, **optional** arc. Each cluster `C` that
has intra-cluster passengers and at least two members gets a second node `C'`,
reached only by paying

    tau_intra(C) = min over j != k in C of tau(j, k),

and every intra candidate `(p, C, C)` becomes an ordinary `(p, C, C')`.
Visiting `C` opens the pickup clock; paying the arc to `C'` certifies the
passenger.

**Optionality is what keeps this valid, and it is not negotiable.** Charging
`tau_intra(C)` on arrival instead -- i.e. a nonzero `tau(C, C)` with the reward
still automatic -- breaks the bound: a real route whose visit to `C` touches
only one station pays nothing inside `C`, so its image would pay
`tau_intra(C)` for reward it need not have earned, and when
`rho_bar < beta * tau_intra(C)` the image's reduced cost exceeds the real
route's. With the arc optional, that route's image simply never takes it.

Validity of the charge itself: a real route that earns intra-cluster reward in
`C` must visit two distinct stations of `C`, hence pays at least the minimum
within-cell hop, hence at least `tau_intra(C)`. And the certification still
fires, because `tau_intra(C) <= tau(j_p, k_p) <= R_pjk <= R_bar` whenever
`detour_factor >= 1`.
"""

export create_joint_routing_assignment_relaxed_cluster_pricing_data

# -- per-passenger cluster-pair reward/ride-limit maxima (relaxation step 2) --
"""
    _aggregate_relaxed_cluster_candidates(clustering, candidates) -> Vector{PassengerAssignmentCandidate}

Collapse the exact `(p, j, k)` candidates onto cluster pairs, keeping passenger
identity exact: one relaxed candidate per `(p, C, D)` carrying
`rho_bar = max rho` (2) and `R_bar = max R` (3) over the exact pairs that fall
in that cell pair. Both maxima are taken independently, which can only relax
further.

`C = D` entries are produced here like any other; the caller rewrites their
destination to the cluster's service node. It matters that they exist as
candidates rather than as a side table, so their reward joins the passenger's
layer ladder alongside the inter-cluster ones -- otherwise the running
per-passenger maximum could credit an intra and an inter reward for the same
passenger twice.

Output is sorted for reproducibility: the layer ids assigned downstream depend
on candidate order, and two runs of the same instance must not differ in them.
"""
function _aggregate_relaxed_cluster_candidates(
    clustering::StationClustering,
    candidates::AbstractVector{PassengerAssignmentCandidate};
    tol::Float64=1e-9,
)::Vector{PassengerAssignmentCandidate}
    best = Dict{Tuple{Int, Int, Int}, Tuple{Float64, Float64}}()
    for candidate in candidates
        candidate.reward > tol || continue
        origin_cluster = get(clustering.cluster_of, candidate.origin, 0)
        dest_cluster = get(clustering.cluster_of, candidate.destination, 0)
        (origin_cluster == 0 || dest_cluster == 0) && throw(ArgumentError(
            "candidate ($(candidate.p), $(candidate.origin), $(candidate.destination)) " *
            "references a station outside the clustering -- the partition was built over " *
            "a different node set than the one being priced",
        ))
        key = (candidate.p, origin_cluster, dest_cluster)
        current = get(best, key, nothing)
        best[key] = isnothing(current) ?
            (candidate.reward, candidate.ride_limit) :
            (max(current[1], candidate.reward), max(current[2], candidate.ride_limit))
    end

    relaxed = PassengerAssignmentCandidate[]
    sizehint!(relaxed, length(best))
    for key in sort!(collect(keys(best)))
        reward, ride_limit = best[key]
        p, origin_cluster, dest_cluster = key
        push!(relaxed, PassengerAssignmentCandidate(
            p, origin_cluster, dest_cluster, ride_limit, reward,
        ))
    end
    return relaxed
end

# -- intra-cluster service arcs ----------------------------------------------
"""
    _intra_cluster_travel(clustering, travel_cost) -> Dict{Int, Float64}

`tau_intra(C)` for every cluster with at least two members: the cheapest
directed hop between two distinct members. A singleton cell has no such hop and
is absent from the result, which is also why it can never carry intra-cluster
passengers.

This is a lower bound on the travel any real route pays inside `C` on a visit
that serves an intra-cluster passenger, which is exactly what the service arc
needs to charge.
"""
function _intra_cluster_travel(
    clustering::StationClustering, travel_cost::Dict{Tuple{Int, Int}, Float64},
)::Dict{Int, Float64}
    out = Dict{Int, Float64}()
    for (cluster, members) in enumerate(clustering.members)
        length(members) >= 2 || continue
        best = Inf
        for j in members, k in members
            j == k && continue
            cost = get(travel_cost, (j, k), Inf)
            cost < best && (best = cost)
        end
        isfinite(best) && (out[cluster] = best)
    end
    return out
end

# -- optimistic travel matrix over the augmented graph (relaxation step 1) ---
"""
    _relaxed_cluster_travel_cost(clustering, travel_cost, intra_travel, service_node, n_nodes)
        -> Dict{Tuple{Int,Int}, Float64}

`tau_hat` from `types.jl`, over the augmented node set: cluster nodes `1:K`
plus one service node per entry of `service_node`.

Arc costs before closure:

| from | to | cost |
| --- | --- | --- |
| cluster `C` | cluster `D` | `min over j in C, k in D of tau(j,k)` |
| cluster `u` | service `C'` | `min-pair(u, C) + tau_intra(C)` (just `tau_intra(C)` when `u = C`) |
| service `C'` | cluster `v` | `min-pair(C, v)` -- being at `C'` is being in `C` |
| service `C'` | service `D'` | `min-pair(C, D) + tau_intra(D)` |

then a Floyd-Warshall metric closure over the whole thing.

The closure is required, not an optimization -- see `types.jl` for why a
non-metric travel matrix would let station-age pruning drop clocks the
relaxation still needs, turning the bound the wrong way round. It cannot
undercut a service arc (any detour `C -> v -> C'` costs
`min-pair(C,v) + min-pair(v,C) + tau_intra(C) >= tau_intra(C)`), so the
intra-cluster charge survives it intact. It is `O(n_nodes^3)` on the *cluster*
count, which is the whole point of clustering.

Directed costs stay directed: only the k-medoids in `clustering.jl`
symmetrizes, and only to pick the partition.
"""
function _relaxed_cluster_travel_cost(
    clustering::StationClustering,
    travel_cost::Dict{Tuple{Int, Int}, Float64},
    intra_travel::Dict{Int, Float64},
    service_node::Dict{Int, Int},
    n_nodes::Int,
)::Dict{Tuple{Int, Int}, Float64}
    n_clusters = clustering.n_clusters
    # Cluster-to-cluster minimum over member pairs, the raw (pre-closure) backbone.
    pair_min = fill(Inf, n_clusters, n_clusters)
    @inbounds for c in 1:n_clusters
        pair_min[c, c] = 0.0
    end
    for ((u, v), cost) in travel_cost
        cu = get(clustering.cluster_of, u, 0)
        cv = get(clustering.cluster_of, v, 0)
        (cu == 0 || cv == 0 || cu == cv) && continue
        cost < pair_min[cu, cv] && (pair_min[cu, cv] = cost)
    end

    # Which cluster each augmented node sits in, and what it costs to enter it. A service
    # node must be entered through its arc, so arriving there costs the trip to its cluster
    # plus the internal hop.
    cluster_of_node = Vector{Int}(undef, n_nodes)
    @inbounds for c in 1:n_clusters
        cluster_of_node[c] = c
    end
    entry_surcharge = zeros(Float64, n_nodes)
    for (cluster, node) in service_node
        cluster_of_node[node] = cluster
        entry_surcharge[node] = intra_travel[cluster]
    end

    best = fill(Inf, n_nodes, n_nodes)
    @inbounds for u in 1:n_nodes, v in 1:n_nodes
        if u == v
            best[u, v] = 0.0
            continue
        end
        backbone = pair_min[cluster_of_node[u], cluster_of_node[v]]
        isfinite(backbone) || continue
        best[u, v] = backbone + entry_surcharge[v]
    end

    # Metric closure. `w` MUST be the outer loop (that is what makes it Floyd-Warshall
    # rather than a single relaxation pass).
    @inbounds for w in 1:n_nodes, u in 1:n_nodes
        best[u, w] == Inf && continue
        for v in 1:n_nodes
            through = best[u, w] + best[w, v]
            through < best[u, v] && (best[u, v] = through)
        end
    end

    out = Dict{Tuple{Int, Int}, Float64}()
    @inbounds for u in 1:n_nodes, v in 1:n_nodes
        isfinite(best[u, v]) && (out[(u, v)] = best[u, v])
    end
    return out
end

# -- assembly ----------------------------------------------------------------
"""
    create_joint_routing_assignment_relaxed_cluster_pricing_data(
        scenario, clustering, travel_cost, candidates; kwargs...) -> RelaxedClusterPricingData

Build one scenario's relaxed pricing problem from the **same**
`PassengerAssignmentCandidate`s the exact pricer would consume
(`joint_routing_assignment_pricing_candidates`, already carrying the RMP duals)
plus a fixed station partition.

Every keyword is passed through to the inner
`create_joint_routing_assignment_pricing_data` verbatim and must match what the
exact pricer is given: the relaxation bounds the exact pricing problem, so the
two have to be the same problem apart from the clustering.
"""
function create_joint_routing_assignment_relaxed_cluster_pricing_data(
    scenario::Int,
    clustering::StationClustering,
    travel_cost::Dict{Tuple{Int, Int}, Float64},
    candidates::AbstractVector{PassengerAssignmentCandidate};
    route_regularization_weight::Float64,
    max_wait_time::Float64,
    repositioning_time::Float64=0.0,
    max_stops::Int=typemax(Int),
    compensated_dominance::Bool=true,
)::RelaxedClusterPricingData
    relaxed_candidates = _aggregate_relaxed_cluster_candidates(clustering, candidates)
    intra_travel = _intra_cluster_travel(clustering, travel_cost)

    # A service node exists exactly where an intra-cluster candidate does AND the cell has
    # a within-cell hop to charge. (Two distinct members are needed for j != k in the first
    # place, so the second condition never bites -- it is checked rather than assumed.)
    service_clusters = sort!(collect(Set(
        c.origin for c in relaxed_candidates
        if c.origin == c.destination && haskey(intra_travel, c.origin)
    )))
    service_node = Dict{Int, Int}(
        cluster => clustering.n_clusters + i for (i, cluster) in enumerate(service_clusters)
    )
    n_nodes = clustering.n_clusters + length(service_clusters)

    # Rewrite each intra candidate onto its service arc; inter candidates are untouched.
    routed_candidates = PassengerAssignmentCandidate[]
    for candidate in relaxed_candidates
        if candidate.origin == candidate.destination
            haskey(service_node, candidate.origin) || continue
            push!(routed_candidates, PassengerAssignmentCandidate(
                candidate.p, candidate.origin, service_node[candidate.origin],
                candidate.ride_limit, candidate.reward,
            ))
        else
            push!(routed_candidates, candidate)
        end
    end

    cluster_travel = _relaxed_cluster_travel_cost(
        clustering, travel_cost, intra_travel, service_node, n_nodes,
    )
    inner = create_joint_routing_assignment_pricing_data(
        scenario, collect(1:n_nodes), cluster_travel, routed_candidates;
        route_regularization_weight=route_regularization_weight,
        max_wait_time=max_wait_time,
        repositioning_time=repositioning_time,
        max_stops=max_stops,
        compensated_dominance=compensated_dominance,
    )
    return RelaxedClusterPricingData(
        scenario, clustering, inner, service_node, intra_travel, length(routed_candidates),
    )
end
