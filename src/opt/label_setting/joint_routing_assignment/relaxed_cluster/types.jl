"""
The relaxed-cluster pricer's data type, plus the relaxation argument the whole
directory exists to make good on. See `clustering.jl` for the partition this is
built over, `data.jl` for how an exact scenario's candidates are turned into
the relaxed ones, `seed.jl`/`extend.jl` for the label search's two deviations
from `../exact/`, and `certify.jl` for what a finished search is actually used
for.

# What this pricer is for -- and what it is NOT

**It never produces columns.** Every other pricer under
`joint_routing_assignment/` searches the real route universe and hands the
master real, replayable routes. This one searches a *relaxed* universe whose
"routes" are sequences of station clusters, and its answer is a single bit:

    every relaxed route has reduced cost >= -tol
        =>  every real route has reduced cost >= -tol
        =>  pricing is done, the restricted master LP is optimal for the FULL
            column set.

So it is a **certificate**, not a harvester -- a cheap way to end a CG
iteration without paying for the exhaustive exact search that would otherwise
be needed to prove the same thing. When it fails (some relaxed route prices
negative) it has proved nothing at all, because that route need not correspond
to any real one; the loop then falls back to the exact pricer, which is the
only thing that can find actual columns (`solvers/cg_solver.jl`'s
`certification_pricing_mode`).

That asymmetry is why this pricer is deliberately **not** a
`pricing_mode` value on `AggregateODRouteJointRoutingAssignmentFormulation`:
plugging it into `_run_pricing_round` would ask it to materialize columns out
of cluster routes, which is meaningless. Its round-level hooks
(`hooks.jl`) refuse rather than improvise.

# The relaxation

Fix any partition `C = {C_1, ..., C_K}` of the stations (`clustering.jl`; any
partition is valid -- see that file). The relaxed pricing graph has one node
per cluster, and:

**Travel is optimistic.** For clusters `C != D`,

    tau_hat(C, D) = shortest path, over the graph whose arcs are
                    min{ tau(j, k) : j in C, k in D }

and `tau_hat(C, C) = 0`. Both steps only ever *decrease* a cost, so for every
real arc `(j, k)` with `j in C`, `k in D`,

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
per-passenger maximum across cluster pairs is then exactly what
`../data.jl`'s reward-layer prefix encoding already does, so the relaxed
candidates go through `create_joint_routing_assignment_pricing_data`
unchanged and the "sum over passengers of each one's best" accounting is
inherited rather than rewritten.

**Intra-cluster rewards must not disappear.** A real route serving `j, k in C`
collapses to a relaxed route that visits `C` once, and there is no `C -> C` arc
to certify it on. If that reward vanished, a real route could earn something
its relaxed image cannot and the bound would break -- so a `C = D` candidate is
credited **on arrival at `C`** (`extend.jl`, and at `t = 0` in `seed.jl`),
with no arc and no travel time, i.e.

    rho_bar_intra(p, C) = max{ rho(p, j, k) : (j,k) in A_p, j, k in C }.     (4)

`intra_layer_mask` below is that credit, pre-reduced to the layer prefixes it
activates. Charging zero internal travel is a further relaxation, and a
deliberate one.

# Why every real route maps to a relaxed one that is at least as good

Take any real route `j_1 -> ... -> j_m` the exact pricer could build, and map
it to the cluster sequence `C(j_1), ..., C(j_m)` with consecutive repeats
collapsed. Write `t_i` for the real arrival times and `t'_i` for the relaxed
ones.

- *Times only shrink.* Each surviving arc costs at most its real counterpart by
  (1), and each collapsed arc drops a non-negative cost entirely, so `t'_i <=
  t_i` position by position. The pickup window is therefore no harder to meet.
- *Elapsed spans only shrink.* This is the part that needs the position-by-
  position statement rather than the endpoint one: `t'_D - t'_C` and `t_k -
  t_j` sum over the *same* contiguous run of arcs, and the relaxed summands are
  pointwise smaller, so `t'_D - t'_C <= t_k - t_j <= R(p,j,k) <= R_bar(p,C,D)`
  by (3). Every ride-limit test the real route passes, the relaxed image passes
  too. (If `C` is revisited, the relaxed clock is even fresher, which only
  helps.)
- *Reward only grows.* Whatever `(p, j, k)` the real route certifies, its image
  certifies `(p, C, D)` -- worth at least as much by (2), or by (4) when
  `j, k in C` -- and per-passenger maxima are taken on both sides.
- *Stop count only shrinks*, so the same `max_stops` cap is valid.
- *Cost only shrinks*: the same `repositioning_time`, plus a travel sum bounded
  by (1).

Hence `rc_relaxed(image) <= rc_exact(route)` for every real route, and

    min over relaxed routes  <=  min over real routes.                       (5)

An exhausted relaxed search that found nothing below `-tol` therefore certifies
that nothing real is below `-tol` either. Note (5) runs over the **full
revisit-tolerant** universe, not just the elementary one -- so a relaxed
certificate is a full-universe certificate even in a run whose column-finding
pricer is `:station_simple`.

# Reuse

Labels, bitsets, dominance filters, the dominance predicate and the
remaining-reward bound are `../exact/`'s, verbatim: the relaxed search is the
same search over a different graph. Only three things are this pricer's own --
the data construction (`data.jl`), the intra-cluster credit in
`seed.jl`/`extend.jl`, and the certification driver (`certify.jl`).
"""

export RelaxedClusterPricingData
export relaxed_cluster_n_clusters

"""
    RelaxedClusterPricingData(scenario, clustering, inner, intra_layer_mask, n_relaxed_candidates)

One scenario's relaxed pricing problem.

`inner` is an ordinary `JointRoutingAssignmentPricingData` whose "stations" are
cluster indices `1:K` -- that is what lets every piece of `../exact/`'s search
machinery run against it untouched. It carries the relaxed candidates from (2)
and (3) above, with **one deliberate edit** made by `data.jl` after
construction: intra-cluster (`C = D`) opportunities are removed from
`assignments_by_origin`/`assignments_by_destination` while being *kept* in
`opportunities`/`origin_layer_mask`/the layer tables.

That split is what keeps the intra credit both sound and cheap:

- kept in `origin_layer_mask`, so candidate generation still proposes visiting
  `C` for an intra-only reward, and in `opportunities`, so `prune.jl`'s
  remaining-reward bound (which reads the search index built from
  `opportunities`) still counts intra reward as reachable -- an admissible
  bound must not miss it;
- removed from `assignments_by_destination`, so the ordinary
  destination-certification path cannot also grant intra reward on a *revisit*
  past the pickup window (harmless for validity, since over-crediting only
  loosens the bound, but pointless);
- removed from `assignments_by_origin`, so a live clock at `C` does not make
  the search propose revisiting `C` purely to certify a reward it already
  banked on arrival -- that is pure branching factor with nothing to gain.

`intra_layer_mask[C]` is the union of the layer prefixes every intra candidate
at `C` activates; `seed.jl`/`extend.jl` union it into a label's activated set
on arrival, within the pickup window.
"""
struct RelaxedClusterPricingData
    scenario::Int
    clustering::StationClustering
    inner::JointRoutingAssignmentPricingData
    intra_layer_mask::Dict{Int, RewardLayerBitset}
    n_relaxed_candidates::Int
end

relaxed_cluster_n_clusters(data::RelaxedClusterPricingData) = data.clustering.n_clusters

"""Shared empty mask, so the common "this cluster has no intra-cluster
passenger" case allocates nothing on the extension hot path. Never mutated --
`extend.jl`/`seed.jl` only ever read it and `union` (non-mutating) from it."""
const EMPTY_RELAXED_CLUSTER_LAYER_MASK = RewardLayerBitset()

"""
    _relaxed_cluster_intra_mask(data, node) -> RewardLayerBitset

The intra-cluster credit available at cluster `node`, or the shared empty mask.
"""
_relaxed_cluster_intra_mask(data::RelaxedClusterPricingData, node::Int) =
    get(data.intra_layer_mask, node, EMPTY_RELAXED_CLUSTER_LAYER_MASK)

"""
    _relaxed_cluster_collect_intra(data, node, arrival_time, activated) -> (activated', reward)

Grant cluster `node`'s intra-cluster reward (4) to a label arriving at
`arrival_time` with `activated` already banked, returning the widened layer set
and the incremental reward.

Gated on the pickup window: a real intra-cluster assignment needs its pickup
`j` inside `max_wait_time`, and the relaxed image arrives at `C` no later than
that pickup (`t'_C <= t_j`), so a route that reaches `C` only after the cutoff
is never the image of a real route that earns this. Past the cutoff the credit
is therefore withheld -- which the remaining-reward bound agrees with, since
past the cutoff its refreshable-origins branch (the only one that counts intra
layers) is switched off too.
"""
function _relaxed_cluster_collect_intra(
    data::RelaxedClusterPricingData,
    node::Int,
    arrival_time::Float64,
    activated::RewardLayerBitset,
)
    arrival_time <= data.inner.max_wait_time + 1e-9 || return activated, 0.0
    mask = _relaxed_cluster_intra_mask(data, node)
    isempty(mask) && return activated, 0.0
    new_layers = setdiff(mask, activated)
    isempty(new_layers) && return activated, 0.0
    return union(activated, new_layers), _sum_layer_weights(data.inner, new_layers)
end
