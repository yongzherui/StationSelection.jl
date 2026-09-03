"""
Geographic station clustering: the `StationClustering` partition every
relaxed-cluster pricing pass is built on top of, and the deterministic
k-medoids that produces one.

The clustering is a *pure input* to the relaxation, not part of its
correctness argument: the bound in `types.jl` holds for **any** partition of
the stations, because it only ever takes minima of travel times and maxima of
rewards over whatever cells it is given. A bad partition costs bound tightness
(and therefore certification rate), never validity. That is exactly why the
partition is computed **once, up front** (at master build time -- see
`optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl`)
and never re-derived per pricing round or per scenario: the cells must be
identical across every CG iteration of a run for `n_clusters` to be a
meaningful swept parameter, and a partition that drifted mid-solve would make
two iterations' relaxed bounds incomparable without buying anything.

Distances come from the pricing graph's own travel-cost table rather than from
lon/lat, since travel time is the quantity the relaxation actually
underestimates; the matrix is symmetrized (`(tau_uv + tau_vu)/2`) purely so
k-medoids has a symmetric distance to work with -- the *pricer's* travel matrix
(`data.jl`) keeps the raw directed costs.
"""

export StationClustering
export cluster_stations_by_travel_cost
export station_cluster_sizes

"""
    StationClustering(n_clusters, nodes, cluster_of, members, medoids)

A partition of `nodes` (station ids) into `n_clusters` non-empty cells.

- `cluster_of[station] -> 1:n_clusters`, total over `nodes`;
- `members[c]` -- that cell's station ids, sorted ascending;
- `medoids[c]` -- the cell's representative station (the member minimizing the
  summed distance to the rest of its own cell). Kept for reporting/diagnostics
  only: nothing in the relaxation reads it, since every cluster-level travel
  cost is a *minimum over member pairs*, not a medoid-to-medoid distance.
"""
struct StationClustering
    n_clusters::Int
    nodes::Vector{Int}
    cluster_of::Dict{Int, Int}
    members::Vector{Vector{Int}}
    medoids::Vector{Int}
end

"""Cell sizes, in cluster order -- the headline diagnostic for how balanced a
partition came out (a partition with one giant cell relaxes almost everything
into a single node and will certify nothing)."""
station_cluster_sizes(clustering::StationClustering) = [length(ms) for ms in clustering.members]

# ── distance matrix ─────────────────────────────────────────────────────────
"""
Symmetrized station-to-station distance over `nodes`, in the pricing graph's own
travel-cost units.

Arcs missing from `travel_cost` in *both* directions are replaced by a sentinel
strictly larger than any observed distance rather than `Inf`, so the k-medoids
objective below stays finite and simply treats an unreachable pair as maximally
far apart. This affects only which partition is chosen, never the relaxation's
validity (see the module docstring).
"""
function _station_distance_matrix(
    nodes::Vector{Int}, travel_cost::Dict{Tuple{Int, Int}, Float64},
)::Matrix{Float64}
    n = length(nodes)
    distance = zeros(Float64, n, n)
    finite_max = 0.0
    @inbounds for i in 1:n, j in (i + 1):n
        forward = get(travel_cost, (nodes[i], nodes[j]), Inf)
        backward = get(travel_cost, (nodes[j], nodes[i]), Inf)
        d = if isfinite(forward) && isfinite(backward)
            0.5 * (forward + backward)
        elseif isfinite(forward)
            forward
        else
            backward
        end
        distance[i, j] = d
        distance[j, i] = d
        isfinite(d) && (finite_max = max(finite_max, d))
    end
    sentinel = finite_max > 0.0 ? 10.0 * finite_max : 1.0
    @inbounds for idx in eachindex(distance)
        isfinite(distance[idx]) || (distance[idx] = sentinel)
    end
    return distance
end

# ── k-medoids ───────────────────────────────────────────────────────────────
"""Farthest-point seeding, fully deterministic: the first medoid is the node
minimizing its summed distance to every other node (the 1-medoid), and each
subsequent one is the node farthest from the medoids chosen so far. Ties break
on the smaller node index, so the same instance always yields the same
partition -- a requirement for `n_clusters` to be a reproducible swept
parameter, and the reason there is no RNG anywhere in this file."""
function _initial_medoid_indices(distance::Matrix{Float64}, k::Int)::Vector{Int}
    n = size(distance, 1)
    total = vec(sum(distance; dims=2))
    first_medoid = argmin(total)
    medoids = Int[first_medoid]
    nearest = distance[:, first_medoid]
    while length(medoids) < k
        best_idx, best_dist = 0, -Inf
        @inbounds for i in 1:n
            i in medoids && continue
            nearest[i] > best_dist + 1e-12 || continue
            best_idx, best_dist = i, nearest[i]
        end
        best_idx == 0 && break  # every remaining node coincides with a medoid
        push!(medoids, best_idx)
        @inbounds for i in 1:n
            nearest[i] = min(nearest[i], distance[i, best_idx])
        end
    end
    return medoids
end

"""One Voronoi assignment pass: every node joins its nearest medoid, ties
broken on the earlier medoid so the pass is deterministic."""
function _assign_to_medoids(distance::Matrix{Float64}, medoids::Vector{Int})::Vector{Int}
    n = size(distance, 1)
    assignment = Vector{Int}(undef, n)
    @inbounds for i in 1:n
        best_c, best_d = 1, Inf
        for (c, medoid) in enumerate(medoids)
            d = distance[i, medoid]
            d < best_d - 1e-12 || continue
            best_c, best_d = c, d
        end
        assignment[i] = best_c
    end
    return assignment
end

"""
    cluster_stations_by_travel_cost(nodes, travel_cost, n_clusters) -> StationClustering

Deterministic k-medoids (farthest-point seeding, then alternating
assign/update until the assignment stops changing or `max_iterations` passes
elapse) over the symmetrized travel-cost distance.

`n_clusters >= length(nodes)` returns the **identity partition** -- one station
per cluster -- rather than erroring. That degenerate case is deliberately
supported and deliberately useful: the relaxation then coincides with the exact
pricing graph arc for arc and reward for reward (there are no intra-cluster
pairs left to over-credit and no inter-cluster minima left to take), so it is
the natural K = n end of a cluster-count sweep and the sanity check that the
relaxed pricer certifies exactly when the exact one does.

A cell can never come out empty: seeding places every medoid on a distinct node
and each node is nearest to (at worst) its own medoid, and the update step below
keeps the previous medoid for any cell that would otherwise vanish.
"""
function cluster_stations_by_travel_cost(
    nodes::AbstractVector{Int},
    travel_cost::Dict{Tuple{Int, Int}, Float64},
    n_clusters::Int;
    max_iterations::Int=50,
)::StationClustering
    n_clusters > 0 || throw(ArgumentError("n_clusters must be positive, got $n_clusters"))
    node_list = collect(Int, nodes)
    isempty(node_list) && throw(ArgumentError("cannot cluster an empty node set"))

    if n_clusters >= length(node_list)
        # Identity partition: cluster c IS station node_list[c].
        return StationClustering(
            length(node_list), node_list,
            Dict(node => c for (c, node) in enumerate(node_list)),
            [[node] for node in node_list],
            copy(node_list),
        )
    end

    distance = _station_distance_matrix(node_list, travel_cost)
    medoids = _initial_medoid_indices(distance, n_clusters)
    assignment = _assign_to_medoids(distance, medoids)

    for _ in 1:max_iterations
        # Update: each cell's new medoid is the member minimizing the summed distance to
        # its own members. A cell that lost every member keeps its previous medoid, which
        # the next assignment pass will then repopulate with at least that node itself.
        members = [Int[] for _ in 1:n_clusters]
        for (i, c) in enumerate(assignment)
            push!(members[c], i)
        end
        new_medoids = copy(medoids)
        for c in 1:n_clusters
            isempty(members[c]) && continue
            best_member, best_cost = members[c][1], Inf
            for candidate in members[c]
                cost = 0.0
                for other in members[c]
                    cost += distance[candidate, other]
                end
                cost < best_cost - 1e-12 || continue
                best_member, best_cost = candidate, cost
            end
            new_medoids[c] = best_member
        end
        new_assignment = _assign_to_medoids(distance, new_medoids)
        converged = new_medoids == medoids && new_assignment == assignment
        medoids, assignment = new_medoids, new_assignment
        converged && break
    end

    # Drop any cell that ended up empty, so `n_clusters` on the returned object is the
    # number of cells that actually exist (the caller reports it, and an empty cell would
    # silently make the swept parameter a lie).
    members_by_cell = [Int[] for _ in 1:n_clusters]
    for (i, c) in enumerate(assignment)
        push!(members_by_cell[c], node_list[i])
    end
    kept = [c for c in 1:n_clusters if !isempty(members_by_cell[c])]
    cluster_of = Dict{Int, Int}()
    members = Vector{Vector{Int}}()
    kept_medoids = Int[]
    for (new_c, c) in enumerate(kept)
        cell = sort!(members_by_cell[c])
        push!(members, cell)
        push!(kept_medoids, node_list[medoids[c]])
        for node in cell
            cluster_of[node] = new_c
        end
    end
    return StationClustering(length(kept), node_list, cluster_of, members, kept_medoids)
end
