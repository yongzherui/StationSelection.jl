"""
The cut-aware relaxed search's label, and the projection back to the plain one.
See `cuts.jl` for what a cut is and why the satisfied-mask has to ride on the
label rather than being applied as a filter afterwards, and `cut_seed.jl` /
`cut_extend.jl` / `cut_context.jl` / `cut_hooks.jl` for the label-setting
functionality built on the type below.

There is no `cut_dominate.jl`: this pricer reuses `../exact/`'s
`JointRoutingAssignmentDominanceFilters`, `JointRoutingAssignmentLabelBitsets`
and dominance predicate unchanged. A cut restricts which finished routes may be
*reported*, not which partial label is better at a given state -- and the state
itself carries the mask (`cut_hooks.jl`), so labels that have escaped different
cut sets are never compared in the first place.
"""

"""
`JointRoutingAssignmentPricingLabel` plus the monotone satisfied-cuts mask. The
fields are duplicated rather than nested because `_run_label_setting` reads
`label.tau` directly off whatever label type it is given, and a wrapper would
need `getproperty` forwarding to satisfy that.
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

"""
    _relaxed_cluster_base_label(label) -> JointRoutingAssignmentPricingLabel

Drop the mask. `cut_seed.jl`/`cut_extend.jl` project through this so the aging,
certification and reduced-cost arithmetic have a single definition in
`../exact/` and cannot drift from the pricer this one relaxes.
"""
_relaxed_cluster_base_label(label::RelaxedClusterCutLabel) =
    JointRoutingAssignmentPricingLabel(
        label.current, label.route, label.time, label.station_age,
        label.activated_reward_layers, label.tau, label.reduced_cost, label.route_length,
    )

"""
The dominance-filter mirror, built straight off the cut label's own fields --
identical to `../exact/dominate.jl`'s, since the filters carry nothing a cut
affects. (Projecting through `_relaxed_cluster_base_label` first would
materialize a base label per insertion for no gain.)
"""
PricingLabelEntry(
    id::Int, label::RelaxedClusterCutLabel, bitsets::JointRoutingAssignmentLabelBitsets,
) = PricingLabelEntry(
    JointRoutingAssignmentDominanceFilters(
        label.reduced_cost, label.time, bitsets.age_mask,
        Int32(label.route_length), Int32(length(bitsets.age_idx)),
    ),
    id, label, bitsets,
)
