"""
No-good cuts on cluster sets, and the relaxed search that respects them.

This is the resource behind the no-good-cut certification loop in
`certify.jl`. The loop's step is:

    relaxed search  ->  best improving cluster route, support T
    exact search over stations(T)
        found an improving real column  ->  refuted, stop
        exhausted with nothing          ->  T is BARREN; cut it and search again

# The cut, and why it is shaped this way

Exhausting the exact pricer over `stations(T)` proves exactly one thing: *no
real route confined to those stations is improving*. So the sound cut is

    every relaxed route must visit at least one cluster OUTSIDE T             (C)

**not** "at most |T|-1 clusters of T". The weaker-looking form is the correct
one; the stronger one is invalid. Take a real improving route `R` touching
clusters `A, B, C, D` after `T = {A,B,C}` was cut. `R` was never examined by
step 3 (it uses a station in `D`), yet a cut of the form
`|route ∩ T| <= 2` deletes its image, and the relaxed search would then certify
with `R` still out there -- a false certificate.

Under (C) that cannot happen: if a real improving `R` exists then
`stations(R) ⊄ stations(T)` (or step 3 would have found it), so `R` touches a
station whose cluster is not in `T`, so its image visits a cluster outside `T`
and survives every cut. Validity is preserved cut after cut, so the loop can
only ever end in a true certificate.

(C) is also stronger than it looks operationally: it kills every route confined
to *any subset* of `T` at once, so each cut removes a downward-closed family
and the loop cannot revisit ground it has covered. With at most `2^K` cluster
supports, it terminates.

**A cut may only be added after an EXHAUSTED exact search.** One that merely ran
out of time has proved nothing about `stations(T)`, and cutting on it would be
the same false certificate by a different route.

# Why the mask has to live inside the search

Satisfying (C) is a property of the *finished* route -- a partial route can sit
inside `T` now and leave later -- so it cannot be enforced by pruning during
extension, and it must not be applied as a filter over the search's results
either. `_run_label_setting` keeps only the best label per signature; if that
label violates a cut while a slightly worse one satisfies it, the satisfying one
is already gone and filtering afterwards silently certifies. So the
satisfied-cuts mask is part of the label, part of the search *state* (labels
with different masks never dominate one another) and part of the
best-so-far signature.

The mask is monotone -- bits only ever turn on -- and one `UInt64` covers 64
simultaneous cuts, which is also the loop's round cap.
"""

export RelaxedClusterNoGoodCuts

"""
    RelaxedClusterNoGoodCuts(cluster_sets, node_mask, all_satisfied)

The active cuts, compiled for the hot path.

`node_mask[v]` has bit `c` set when relaxed-graph node `v` lies OUTSIDE cut
`c`'s cluster set -- i.e. visiting `v` satisfies cut `c`. A label's satisfied
mask is then just an `OR` along its route, and "obeys every cut" is one
equality against `all_satisfied`.
"""
struct RelaxedClusterNoGoodCuts
    cluster_sets::Vector{Set{Int}}
    node_mask::Vector{UInt64}
    all_satisfied::UInt64
end

"""Cap on simultaneously active cuts, set by the `UInt64` mask. The
certification loop stops adding cuts here and reports itself inconclusive
rather than silently dropping one (which would re-admit an already-refuted
cluster support and could loop forever)."""
const RELAXED_CLUSTER_MAX_CUTS = 64

"""Cluster of every relaxed-graph node, as a dense vector -- cluster nodes map to
themselves, service nodes to the cluster they serve. Built once so cut compilation
is not `O(nodes x cuts x service nodes)`."""
function _relaxed_cluster_node_clusters(data::RelaxedClusterPricingData)::Vector{Int}
    clusters = collect(1:length(data.inner.nodes))
    for (cluster, node) in data.service_node
        clusters[node] = cluster
    end
    return clusters
end

function _relaxed_cluster_cuts(
    data::RelaxedClusterPricingData, cluster_sets::AbstractVector{Set{Int}},
)::RelaxedClusterNoGoodCuts
    length(cluster_sets) <= RELAXED_CLUSTER_MAX_CUTS || throw(ArgumentError(
        "at most $RELAXED_CLUSTER_MAX_CUTS cuts fit the UInt64 satisfied-mask, got " *
        "$(length(cluster_sets))",
    ))
    node_clusters = _relaxed_cluster_node_clusters(data)
    node_mask = zeros(UInt64, length(node_clusters))
    for (c, forbidden) in enumerate(cluster_sets)
        bit = UInt64(1) << (c - 1)
        for node in eachindex(node_clusters)
            node_clusters[node] in forbidden || (node_mask[node] |= bit)
        end
    end
    all_satisfied = isempty(cluster_sets) ? UInt64(0) :
        (length(cluster_sets) == 64 ? typemax(UInt64) :
         (UInt64(1) << length(cluster_sets)) - UInt64(1))
    return RelaxedClusterNoGoodCuts(collect(Set{Int}, cluster_sets), node_mask, all_satisfied)
end

# ── label ───────────────────────────────────────────────────────────────────
"""
`JointRoutingAssignmentPricingLabel` plus the monotone satisfied-cuts mask. The
fields are duplicated rather than nested because `_run_label_setting` reads
`label.tau` directly off whatever label type it is given, and a wrapper would
need `getproperty` forwarding to satisfy that. `_relaxed_cluster_base_label`
projects back for the extension/candidate machinery, which is reused verbatim
from `../exact/` so the aging, certification and reduced-cost arithmetic have a
single definition and cannot drift.
"""
struct RelaxedClusterCutLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    activated_reward_layers::RewardLayerBitset
    tau::Float64
    reduced_cost::Float64
    route_length::Int
    satisfied::UInt64
end

_relaxed_cluster_base_label(label::RelaxedClusterCutLabel) =
    JointRoutingAssignmentPricingLabel(
        label.current, label.route, label.time, label.station_age,
        label.activated_reward_layers, label.tau, label.reduced_cost, label.route_length,
    )

PricingLabelEntry(
    id::Int, label::RelaxedClusterCutLabel, bitsets::JointRoutingAssignmentLabelBitsets,
) = PricingLabelEntry(
    JointRoutingAssignmentDominanceFilters(
        label.reduced_cost, label.time, bitsets.age_mask,
        Int32(label.route_length), Int32(length(bitsets.age_idx)),
    ),
    id, label, bitsets,
)

# ── context ─────────────────────────────────────────────────────────────────
"""
The cut-aware relaxed search. Same graph and same dominance predicate as the
plain relaxed search; what differs is that the state key carries the
satisfied-cuts mask, so labels that have escaped different cut sets never
compete, and that only a fully-satisfying label can be an answer.

Keying the state on the mask is the conservative choice: the sharper rule is
that `a` may dominate `b` when `satisfied(a) ⊇ satisfied(b)` (a has escaped
everything `b` has), which would let more labels compete. That is a strict
improvement to make later if the live-label population turns out to be the
bottleneck -- it is not needed for correctness.
"""
struct RelaxedClusterCutSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentDominanceFilters, RelaxedClusterCutLabel,
    JointRoutingAssignmentLabelBitsets, Tuple{Int, UInt64}, Tuple{RewardLayerBitset, UInt64},
}
    pricing_data::JointRoutingAssignmentPricingData
    cuts::RelaxedClusterNoGoodCuts
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    n_nodes::Int
end

function RelaxedClusterCutSearchContext(
    data::RelaxedClusterPricingData, cluster_sets::AbstractVector{Set{Int}},
)
    inner = data.inner
    cuts = _relaxed_cluster_cuts(data, cluster_sets)
    search_index = _build_joint_routing_assignment_search_index(inner)
    rules = _joint_routing_assignment_dominance_rules(
        inner.bounded_max_stops, inner.compensated_dominance, false,
    )
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.bitsets, y.filters, y.bitsets, inner.layer_weight, rules,
    )
    return RelaxedClusterCutSearchContext(
        inner, cuts, dominates, search_index,
        _create_joint_routing_assignment_bound_workspace(), length(inner.nodes),
    )
end

# ── AbstractPricingSearchContext hooks ──────────────────────────────────────
function _pricing_initial_labels(ctx::RelaxedClusterCutSearchContext)
    return RelaxedClusterCutLabel[
        RelaxedClusterCutLabel(
            base.current, base.route, base.time, base.station_age,
            base.activated_reward_layers, base.tau, base.reduced_cost, base.route_length,
            ctx.cuts.node_mask[base.current],
        )
        for base in initial_joint_routing_assignment_pricing_labels(ctx.pricing_data)
    ]
end

function _pricing_make_bitsets(ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel)
    age_idx, age_val, age_mask =
        _make_sparse_station_ages(label.station_age, ctx.search_index.node_index)
    return JointRoutingAssignmentLabelBitsets(
        label.activated_reward_layers, age_idx, age_val, age_mask,
    )
end

# The mask is part of the STATE, not just the signature: two labels that have escaped
# different cuts have genuinely different futures and must not dominate one another.
_pricing_state(
    ::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel,
    ::JointRoutingAssignmentLabelBitsets,
) = (label.current, label.satisfied)

# The bound reads only current/time/activated layers off the label, which this type
# exposes directly, so `../exact/prune.jl` applies unchanged.
_pricing_label_priority(
    ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel,
    label_bs::JointRoutingAssignmentLabelBitsets,
)::Float64 = label.reduced_cost - _joint_routing_assignment_remaining_reward_bound(
    label, label_bs, ctx.pricing_data, ctx.search_index, ctx.bound_workspace,
)

"""A label is an answer only once it has escaped EVERY active cut. Labels that have
not are still extended -- they may escape later -- they simply cannot be reported.
This is the filter, and it has to live here rather than over the returned labels:
the search keeps one best label per signature, so a post-hoc filter would silently
discard a cut-satisfying route in favour of a cut-violating one and certify."""
_pricing_best_signature(
    ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel,
) = (isempty(label.activated_reward_layers) || label.satisfied != ctx.cuts.all_satisfied) ?
    nothing : (label.activated_reward_layers, label.satisfied)

_pricing_route_length(::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel) =
    label.route_length

_pricing_max_route_length(ctx::RelaxedClusterCutSearchContext) = ctx.pricing_data.max_stops

"""
The exact pricer's candidate rule, **widened by every node that would newly satisfy an
outstanding cut**.

That widening is not an optimization, it is required for completeness.
`_joint_routing_assignment_candidate_next_nodes` is reward-driven: it proposes a node only
if visiting it can unlock a not-yet-activated reward layer. Under a cut, a route may have
to visit a cluster *purely to leave the cut set*, collecting nothing there -- and the
reward-driven rule would never propose it, so the search could not build the route at all.

MEASURED: without this, a cut of `{1,2}` made the search report a minimum of -114.6 where
exhaustive enumeration over cut-satisfying routes found -250.5. The winning route was
`[2, 1, 4, 3]`, whose last stop (cluster 3) opened no reward and existed only to escape the
cut. Under-reporting here is the false-certificate failure mode: the loop sees "nothing
improving survives" and certifies while an improving route exists.

Only labels that still owe a cut pay for the widening; once `satisfied` is complete the
rule is the exact pricer's untouched.
"""
function _pricing_candidate_next_nodes(
    ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel,
)
    base = _joint_routing_assignment_candidate_next_nodes(
        _relaxed_cluster_base_label(label), ctx.pricing_data,
    )
    label.satisfied == ctx.cuts.all_satisfied && return base

    escapes = Int[]
    @inbounds for v in eachindex(ctx.cuts.node_mask)
        v == label.current && continue
        # Would visiting `v` turn on a cut bit this label does not yet have?
        (label.satisfied | ctx.cuts.node_mask[v]) == label.satisfied && continue
        v in base && continue
        push!(escapes, v)
    end
    isempty(escapes) && return base
    return sort!(vcat(base, escapes))
end

function _pricing_extend_label(
    ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel, next_node::Int,
)
    child = _extend_joint_routing_assignment_pricing_label(
        _relaxed_cluster_base_label(label), next_node, ctx.pricing_data,
    )
    return RelaxedClusterCutLabel(
        child.current, child.route, child.time, child.station_age,
        child.activated_reward_layers, child.tau, child.reduced_cost, child.route_length,
        label.satisfied | ctx.cuts.node_mask[next_node],
    )
end

_pricing_dominates_fn(ctx::RelaxedClusterCutSearchContext) = ctx.dominates
