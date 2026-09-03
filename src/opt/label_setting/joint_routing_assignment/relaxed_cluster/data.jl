"""
Building one scenario's `RelaxedClusterPricingData` from the exact candidates
the RMP duals produced: the optimistic cluster travel matrix (1), the
per-passenger/per-cluster-pair reward and ride-limit maxima (2)/(3), the
intra-cluster credit table (4), and the one post-construction edit to the inner
pricing data that keeps that credit sound. Equation numbers refer to
`types.jl`'s module docstring, which carries the relaxation argument this file
implements.

Everything downstream of `create_joint_routing_assignment_pricing_data` --
reward layers, prefix masks, the endpoint groupings -- is inherited unchanged:
the relaxed candidates are ordinary `PassengerAssignmentCandidate`s whose
"stations" happen to be cluster indices.
"""

export create_joint_routing_assignment_relaxed_cluster_pricing_data

# ── optimistic cluster travel matrix (relaxation step 1) ────────────────────
"""
    _relaxed_cluster_travel_cost(clustering, travel_cost) -> Dict{Tuple{Int,Int}, Float64}

`tau_hat` from `types.jl`: min over member pairs for each ordered cluster pair,
then a Floyd-Warshall metric closure, with a zero diagonal.

The closure is required, not an optimization -- see `types.jl` for why a
non-metric travel matrix would let station-age pruning drop clocks that the
relaxation still needs, turning the bound the wrong way round. It is `O(K^3)`
on the *cluster* count, which is the whole point of clustering, so it costs
nothing at the sizes this runs at.

Directed costs stay directed: only the k-medoids in `clustering.jl` symmetrizes,
and only to pick the partition.
"""
function _relaxed_cluster_travel_cost(
    clustering::StationClustering, travel_cost::Dict{Tuple{Int, Int}, Float64},
)::Dict{Tuple{Int, Int}, Float64}
    n_clusters = clustering.n_clusters
    best = fill(Inf, n_clusters, n_clusters)
    @inbounds for c in 1:n_clusters
        best[c, c] = 0.0
    end
    for ((u, v), cost) in travel_cost
        cu = get(clustering.cluster_of, u, 0)
        cv = get(clustering.cluster_of, v, 0)
        (cu == 0 || cv == 0 || cu == cv) && continue
        cost < best[cu, cv] && (best[cu, cv] = cost)
    end

    # Metric closure. `w` MUST be the outer loop (that is what makes it Floyd-Warshall
    # rather than a single relaxation pass).
    @inbounds for w in 1:n_clusters, u in 1:n_clusters
        best[u, w] == Inf && continue
        for v in 1:n_clusters
            through = best[u, w] + best[w, v]
            through < best[u, v] && (best[u, v] = through)
        end
    end

    out = Dict{Tuple{Int, Int}, Float64}()
    @inbounds for u in 1:n_clusters, v in 1:n_clusters
        isfinite(best[u, v]) && (out[(u, v)] = best[u, v])
    end
    return out
end

# ── per-passenger cluster-pair reward/ride-limit maxima (relaxation step 2) ──
"""
    _aggregate_relaxed_cluster_candidates(clustering, candidates) -> Vector{PassengerAssignmentCandidate}

Collapse the exact `(p, j, k)` candidates onto cluster pairs, keeping passenger
identity exact: one relaxed candidate per `(p, C, D)` carrying
`rho_bar = max rho` (2) and `R_bar = max R` (3) over the exact pairs that fall
in that cell pair. Both maxima are taken independently, which can only relax
further.

`C = D` entries are produced here exactly like any other -- they are the
intra-cluster credit (4), and it is essential that they exist as candidates so
that their reward participates in the passenger's layer ladder alongside the
inter-cluster ones (otherwise the running per-passenger maximum could
double-count an intra and an inter reward for the same passenger). What
`create_joint_routing_assignment_relaxed_cluster_pricing_data` does with them
afterwards is a separate matter -- see `_strip_intra_cluster_opportunities`.

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

# ── the one post-construction edit (see types.jl's docstring) ───────────────
"""
    _strip_intra_cluster_opportunities(pricing_data) -> JointRoutingAssignmentPricingData

Return `pricing_data` with `C = D` opportunities removed from
`assignments_by_origin` and `assignments_by_destination` only. Everything else
-- `opportunities`, `origin_layer_mask`, `destination_layer_mask`, the layer
tables -- is shared with the input, unmodified.

See `RelaxedClusterPricingData`'s docstring for why the split falls exactly
here: the tables that drive *search branching and certification* must not see
intra opportunities (they are credited on arrival instead), while the tables
that drive *reachability bounds* must, or `prune.jl`'s bound stops being
admissible.
"""
function _strip_intra_cluster_opportunities(
    pricing_data::JointRoutingAssignmentPricingData,
)::JointRoutingAssignmentPricingData
    inter_only(opps) = filter(opp -> opp.origin != opp.destination, opps)
    by_destination = Dict{Int, Vector{PassengerAssignmentOpportunity}}()
    for (node, opps) in pricing_data.assignments_by_destination
        kept = inter_only(opps)
        isempty(kept) || (by_destination[node] = kept)
    end
    by_origin = Dict{Int, Vector{PassengerAssignmentOpportunity}}()
    for (node, opps) in pricing_data.assignments_by_origin
        kept = inter_only(opps)
        isempty(kept) || (by_origin[node] = kept)
    end
    return JointRoutingAssignmentPricingData(
        pricing_data.scenario,
        pricing_data.nodes,
        pricing_data.travel_cost,
        pricing_data.route_regularization_weight,
        pricing_data.repositioning_time,
        pricing_data.max_wait_time,
        pricing_data.max_stops,
        pricing_data.bounded_max_stops,
        pricing_data.compensated_dominance,
        pricing_data.n_layers,
        pricing_data.layer_weight,
        pricing_data.assignment_layer_mask,
        by_destination,
        by_origin,
        pricing_data.origin_layer_mask,
        pricing_data.destination_layer_mask,
        pricing_data.opportunities,
    )
end

# ── assembly ────────────────────────────────────────────────────────────────
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
    cluster_nodes = collect(1:clustering.n_clusters)
    cluster_travel = _relaxed_cluster_travel_cost(clustering, travel_cost)

    full = create_joint_routing_assignment_pricing_data(
        scenario, cluster_nodes, cluster_travel, relaxed_candidates;
        route_regularization_weight=route_regularization_weight,
        max_wait_time=max_wait_time,
        repositioning_time=repositioning_time,
        max_stops=max_stops,
        compensated_dominance=compensated_dominance,
    )

    intra_layer_mask = Dict{Int, RewardLayerBitset}()
    for opp in full.opportunities
        opp.origin == opp.destination || continue
        intra_layer_mask[opp.origin] =
            union(get(intra_layer_mask, opp.origin, RewardLayerBitset()), opp.layer_mask)
    end

    return RelaxedClusterPricingData(
        scenario, clustering, _strip_intra_cluster_opportunities(full),
        intra_layer_mask, length(relaxed_candidates),
    )
end
