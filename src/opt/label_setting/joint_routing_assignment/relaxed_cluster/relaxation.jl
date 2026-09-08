"""
The relaxed-cluster pricer's data type, plus the relaxation argument the whole
directory exists to make good on. See `clustering.jl` for the partition this is
built over, `data.jl` for how an exact scenario's candidates are turned into
the relaxed ones, `cuts.jl` + `types.jl`/`seed.jl`/`extend.jl`/`context.jl`/
`hooks.jl` for the cut-aware search built on it,
`utils/certification/certify.jl` for the no-good-cut loop that certifies, and
`utils/guiding/guide.jl` for the station-subset use.

# The relaxation itself is a graph, not a pricer

`RelaxedClusterPricingData.inner` is an ordinary
`JointRoutingAssignmentPricingData` whose "stations" are cluster-graph nodes,
so an ordinary `JointRoutingAssignmentSearchContext` -- `../exact/`'s,
unmodified -- can run on it directly: seeding, extension, dominance, the
remaining-reward bound and route replay all inherited. Everything the
relaxation *is* lives in `data.jl`, in what goes into that graph.

(An earlier version carried its own label type, seeder, extender, context and
replay in order to credit intra-cluster passengers on arrival as a special
case. Making intra service an explicit optional arc -- see below -- removed the
special case and with it all five files.)

# Two searches run on this graph, and only one can certify

The **cut-aware** search is this directory's own pricer, and it takes the
unprefixed file roles (`types.jl`, `seed.jl`, `extend.jl`, `context.jl`,
`hooks.jl`, plus `cuts.jl` for the cut resource) because it is the only search
here with files of its own. A no-good cut is a condition on the finished route,
so it has to ride on the label, in the search state and in the best-so-far
signature -- none of which `../exact/`'s label can carry. It still borrows
`../exact/`'s dominance, bound and arithmetic verbatim, which is why there is no
`dominate.jl` or `prune.jl`.

The **cut-free** search is `../exact/`'s context handed this graph, with no
files here at all. Its only job is guiding (`utils/guiding/guide.jl`), which
prices it to pick a station subset and needs the argmin to land in the right
neighbourhood, not tightness.

It cannot certify, and nothing asks it to any more. A cut-free certification
round is the cut loop's round 1, and round 1 certified 0 times across ~1130
measured attempts at every size and every K -- because a converged master's exact
minimum reduced cost is exactly 0 and the relaxation's slack overshoots it by
10^2-10^3. A cut-free mode selecting it existed for that
comparison and was removed once the answer was in. Certification is the cut
loop's alone: the cuts are the mechanism, not a refinement of it (`cuts.jl`).

# The relaxation

Fix any partition `C = {C_1, ..., C_K}` of the stations (`clustering.jl`; any
partition is valid -- see that file). The relaxed pricing graph has one node
per cluster, plus one *service* node per cluster that has intra-cluster
passengers, and:

**Travel is optimistic.** For clusters `C != D`,

    tau_hat(C, D) = shortest path, over the graph whose arcs are
                    min{ tau(j, k) : j in C, k in D }

Both steps only ever *decrease* a cost, so for every real arc `(j, k)` with
`j in C`, `k in D`,

    tau_hat(C, D) <= min over the cluster pair <= tau(j, k).                (1)

The shortest-path (metric) closure is not decoration: the label search's
station-age pruning (`../data.jl`'s `_joint_routing_assignment_age_is_useful`)
assumes `travel(current, dest)` is a genuine *lower* bound on the time still
needed to reach `dest`, i.e. that detouring cannot be faster. A raw
min-over-member-pairs matrix violates the triangle inequality (two clusters can
each be near a third at different member stations while being far from each
other), and a clock dropped on that false premise would make the relaxed search
*under*-collect reward -- which is the one direction that breaks the bound.

**Rewards are optimistic, per passenger.** Passenger `p` is served at most
once, so its alternatives are combined by `max`, while distinct passengers are
combined by `+`. For a cluster pair,

    rho_bar(p, C, D) = max{ rho(p, j, k) : (j,k) in A_p, j in C, k in D }    (2)
    R_bar(p, C, D)   = max{ R(p, j, k)   : (j,k) in A_p, j in C, k in D }    (3)

(taking the two maxima independently only relaxes further). The running
per-passenger maximum across cluster pairs is exactly what `../data.jl`'s
reward-layer prefix encoding already does, so the relaxed candidates go through
`create_joint_routing_assignment_pricing_data` unchanged.

**Intra-cluster passengers get an optional service arc.** A real route serving
`j, k in C` collapses to a relaxed route that visits `C` once, and there is no
`C -> C` arc to certify it on. Rather than crediting it for free, cluster `C`
gets a second node `C'` reachable only by paying

    tau_intra(C) = min over j != k in C of tau(j, k),                       (4)

and the intra candidate becomes an ordinary `(p, C, C')`. See `data.jl` for why
the arc must be *optional* -- charging on arrival instead breaks the bound
outright.

# Why every real route maps to a relaxed one that is at least as good

Take any real route `j_1 -> ... -> j_m`, map it to the cluster sequence
`C(j_1), ..., C(j_m)` with consecutive repeats collapsed into blocks, and
insert `C'` after any block that serves an intra-cluster passenger. Write `t_i`
for the real arrival times, `t'_i` for the relaxed ones, and `W_q` for the real
travel inside block `q`.

- *Times only shrink.* Each surviving arc costs at most its real counterpart by
  (1); each collapsed block contributes `tau_intra(C_q) <= W_q` if it took the
  service arc (a block serving an intra passenger visits two distinct stations
  of `C_q`, so `W_q >= tau_intra(C_q)`) and `0 <= W_q` otherwise. So
  `t'_i <= t_i` position by position, and the pickup window is no harder to
  meet.
- *Elapsed spans only shrink.* `t'_D - t'_C` and `t_k - t_j` sum over the same
  contiguous run of arcs with pointwise smaller summands, so
  `t'_D - t'_C <= t_k - t_j <= R(p,j,k) <= R_bar(p,C,D)` by (3). Every
  ride-limit test the real route passes, its image passes. (A revisited cluster
  gives an even fresher clock, which only helps.)
- *Reward only grows.* Whatever `(p, j, k)` the real route certifies, its image
  certifies `(p, C, D)` -- worth at least as much by (2), or `(p, C, C')` by
  (4) -- and per-passenger maxima are taken on both sides.
- *Stop count only shrinks*, except for service arcs, which are only inserted
  for blocks of two or more real stops -- so the same `max_stops` cap is valid.
- *Cost only shrinks*: the same `repositioning_time`, plus a travel sum bounded
  as above.

Hence `rc_relaxed(image) <= rc_exact(route)` for every real route, and

    min over relaxed routes  <=  min over real routes.                       (5)

Note (5) runs over the **full revisit-tolerant** universe, not just the
elementary one.

# What (5) is and is not good for

*One-shot certification* -- an exhausted relaxed search finding nothing below
`-tol` proves no real improving column exists -- is sound but was **measured
useless**: at a converged master the exact minimum is exactly 0 (any column the
master uses has reduced cost 0 at every optimal dual, by complementary slackness
on `theta >= 0`), so certification needs the relaxation tight to within `tol`,
and it is off by 10^2-10^3. See `utils/certification/certify.jl` and
`benchmarks/diagnostics/relaxed_cluster_certification_probe.jl` (0/31).

*No-good-cut certification* recovers it, and is the mode that actually works:
when the relaxation names an improving cluster route, search its cluster support
exhaustively with the exact pricer, and if the support is barren, cut it and ask
again. Each cut only removes supports already proven barren, so the certificate
still covers the full route universe. MEASURED to certify at K=9 and K=12. See
`utils/certification/certify.jl` and `cuts.jl`.

*Guiding* -- taking the winning cluster route's members as a station subset and
running the exact pricer on that subset -- needs only the argmin to land in the
right neighbourhood, not tightness, so it can pay without any certification at
all. See `utils/guiding/guide.jl`.
"""

export RelaxedClusterPricingData
export relaxed_cluster_n_clusters, relaxed_cluster_n_nodes, relaxed_cluster_of_node

"""
    RelaxedClusterPricingData(scenario, clustering, inner, service_node, intra_travel, n_relaxed_candidates)

One scenario's relaxed pricing problem.

`inner` is an ordinary `JointRoutingAssignmentPricingData` over the augmented
node set -- cluster nodes `1:K` first, then one service node per cluster in
`service_node` -- which is what lets `../exact/`'s search machinery run against
it untouched.

- `service_node[C]` -- the node id of cluster `C`'s intra-cluster service node,
  present only for clusters that have intra-cluster passengers;
- `intra_travel[C]` -- `tau_intra(C)` from (4), the cheapest within-cell hop,
  present for every cluster with two or more members (so it can be non-empty
  where `service_node` is not);
- `n_relaxed_candidates` -- how many candidates the aggregation produced, i.e.
  how much the cluster collapse compressed the pricing problem;
- `reward_witness[(p, origin_node, dest_node)]` -- the real `(j, k)` whose reward the
  `rho_bar` maximum actually came from, keyed by the ROUTED nodes so a lookup after route
  replay works directly. This is what makes the relaxation's optimism attributable to a
  specific cell and station: see `utils/refinement/refine.jl`.
"""
struct RelaxedClusterPricingData
    scenario::Int
    clustering::StationClustering
    inner::JointRoutingAssignmentPricingData
    service_node::Dict{Int, Int}
    intra_travel::Dict{Int, Float64}
    n_relaxed_candidates::Int
    reward_witness::Dict{Tuple{Int, Int, Int}, Tuple{Int, Int}}
end

relaxed_cluster_n_clusters(data::RelaxedClusterPricingData) = data.clustering.n_clusters

"""Number of nodes the relaxed search actually runs on: one per cluster plus one
per intra-cluster service arc. This, not `n_clusters`, is what sets the search's
cost."""
relaxed_cluster_n_nodes(data::RelaxedClusterPricingData) = length(data.inner.nodes)

"""
    relaxed_cluster_of_node(data, node) -> Int

The cluster a relaxed graph node belongs to -- itself for a cluster node, and
the served cluster for a service node. The single-node form, for callers that
need one answer; anything that maps a whole route (or every node) back to
clusters should build `_relaxed_cluster_node_clusters` once instead, since this
scans `service_node` linearly.
"""
function relaxed_cluster_of_node(data::RelaxedClusterPricingData, node::Int)::Int
    node <= data.clustering.n_clusters && return node
    for (cluster, service) in data.service_node
        service == node && return cluster
    end
    throw(ArgumentError("node $node is not a node of this relaxed cluster graph"))
end

"""
    _relaxed_cluster_node_clusters(data) -> Vector{Int}

`relaxed_cluster_of_node` for every node at once, as a dense vector: cluster
nodes map to themselves, service nodes to the cluster they serve. Built once by
callers that map many nodes back -- reading a relaxed route as a cluster
sequence (`utils/guiding/guide.jl`), taking a route's cluster support
(`utils/certification/certify.jl`),
compiling the cut masks (`cuts.jl`) -- so none of them pays the single-node
form's linear scan per node.
"""
function _relaxed_cluster_node_clusters(data::RelaxedClusterPricingData)::Vector{Int}
    clusters = collect(1:length(data.inner.nodes))
    for (cluster, node) in data.service_node
        clusters[node] = cluster
    end
    return clusters
end
