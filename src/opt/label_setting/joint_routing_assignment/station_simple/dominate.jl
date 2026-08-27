"""
When is one label strictly better than another? Bitsets construction and the
dominance predicates themselves (`_pricing_dominates_fn`, wired in
`hooks.jl`) live here. Reuses `../exact/dominate.jl`'s
`_joint_routing_assignment_compensation` directly for the reward-layer
sub-test rather than redefining it.
"""

# ── ages construction (hot-path dominance mirror) ────────────────────────────
"""
Delegates to the shared `_make_sparse_station_ages` (`label_setting/utils.jl`), which runs
the identical insertion sort used by the revisit-tolerant pricer's twin in
`../exact/dominate.jl` -- only the return type differs (wrapped here in
`JointRoutingAssignmentStationSimpleAges`).
"""
function _make_joint_routing_assignment_station_simple_ages(
    label::JointRoutingAssignmentStationSimpleLabel,
    node_index::Dict{Int, Int},
)::JointRoutingAssignmentStationSimpleAges
    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)
    return JointRoutingAssignmentStationSimpleAges(age_idx, age_val, age_mask)
end

JointRoutingAssignmentStationSimpleDominanceFilters(
    label::JointRoutingAssignmentStationSimpleLabel, ages::JointRoutingAssignmentStationSimpleAges,
) = JointRoutingAssignmentStationSimpleDominanceFilters(
    label.reduced_cost, label.time, Int32(label.route_length), Int32(length(ages.age_idx)),
)

PricingLabelEntry(
    id::JointRoutingAssignmentLabelId,
    label::JointRoutingAssignmentStationSimpleLabel,
    ages::JointRoutingAssignmentStationSimpleAges,
) = PricingLabelEntry(JointRoutingAssignmentStationSimpleDominanceFilters(label, ages), id, label, ages)

# ── dominance predicates ──────────────────────────────────────────────────────
"""
    _dominates_joint_routing_assignment_station_simple_label(a, b, abs, bbs, layer_weight)

`a` dominates `b`: every completion of `b` has a counterpart from `a` at least as
good. Callers only ever compare labels drawn from the same `current` state, so
`a.current == b.current` is re-checked only as a cheap guard.

The visited resource is a **subset** test, `U_a ⊆ U_b`, not equality. For an
elementary route `visited` is the set of forbidden future stations, so if `a` has
visited a subset of what `b` has, every station `b` may still visit `a` may visit
too -- hence every completion feasible for `b` is feasible from `a`. This is
strictly stronger than the exact `(current, visited)` state it replaced: a
"lean" label (visited a subset) can now kill a "wandered" one that forbade itself
extra stations for no gain, which the exact rule structurally could not, and which
was measured letting the live-label population balloon 3-6x (see the note). Because
`U_a ⊆ U_b` implies `route_length_a <= route_length_b`, the `max_stops` resource is
subsumed and needs no separate check. The remaining conditions are the
revisit-tolerant pricer's:

  - `time_a <= time_b`;
  - the compensated reward-layer budget `rc_a + w(A_a ∖ A_b) <= rc_b` (see
    `_joint_routing_assignment_compensation` and the dominance docstring in
    `../exact/dominate.jl` for why this, not `issubset` on layers, is the sound test);
  - every live station age in `a` is no larger than `b`'s (sparse merge walk).

Conditions are ordered cheapest-and-likeliest-to-reject first, exactly as in
`_pricing_dominates_at_state` (passenger method): scalars, then the word-wise
`visited` subset, then the `O(#live)` age walk, and only last the reward-layer
compensation, which is the one test that has to sum weights. `a.current ==
b.current` is *not* checked -- both states (`current` under `:subset`,
`(current, visited)` under `:exact`) already include it, so it was a
guaranteed-true compare in the hot loop.
"""
function _dominates_joint_routing_assignment_station_simple_label(
    a::JointRoutingAssignmentStationSimpleLabel,
    b::JointRoutingAssignmentStationSimpleLabel,
    a_ages::JointRoutingAssignmentStationSimpleAges,
    b_ages::JointRoutingAssignmentStationSimpleAges,
    layer_weight::Vector{Float64},
)::Bool
    return _pricing_dominates_at_state(
        JointRoutingAssignmentStationSimpleDominanceFilters(a, a_ages), a, a_ages,
        JointRoutingAssignmentStationSimpleDominanceFilters(b, b_ages), b, b_ages,
        layer_weight, JointRoutingAssignmentStationSimpleDominanceRules(),
    )
end

"""
The form the state's label-list scan calls: the scalars come from
`JointRoutingAssignmentStationSimpleDominanceFilters`, so an entry rejected on
time, live-clock count or reduced cost is never dereferenced into its label at
all. `visited`/`activated_reward_layers` are read off `a`/`b` directly (see
`JointRoutingAssignmentStationSimpleDominanceFilters`'s docstring for why they
are not mirrored into the filters/ages, unlike the revisit-tolerant pricer).
"""
@inline function _pricing_dominates_at_state(
    af::JointRoutingAssignmentStationSimpleDominanceFilters,
    a::JointRoutingAssignmentStationSimpleLabel, a_ages::JointRoutingAssignmentStationSimpleAges,
    bf::JointRoutingAssignmentStationSimpleDominanceFilters,
    b::JointRoutingAssignmentStationSimpleLabel, b_ages::JointRoutingAssignmentStationSimpleAges,
    layer_weight::Vector{Float64},
    ::JointRoutingAssignmentStationSimpleDominanceRules,
)::Bool
    af.time <= bf.time + 1e-9 || return false
    # `dom(age_b) ⊆ dom(age_a)` is required below, so `a` cannot have fewer live
    # clocks than `b`, nor a mask missing any of `b`'s -- both cheap, ahead of
    # anything that reads set contents. Shared with the revisit-tolerant bitset
    # dominance via `label_setting/utils.jl`.
    _sparse_station_age_support_rejection(
        a_ages.age_idx, a_ages.age_mask, b_ages.age_idx, b_ages.age_mask,
    ) == 0 || return false
    budget = bf.reduced_cost - af.reduced_cost + 1e-9
    budget >= 0.0 || return false
    # `visited` and `activated_reward_layers` are read straight off the labels --
    # both are `BitSet`s, so `issubset`/compensation are word-wise with no per-label
    # bitset reconstruction.
    issubset(a.visited, b.visited) || return false
    # `dom(age_b) ⊆ dom(age_a)` and `age_a(j) <= age_b(j)` for j in dom(age_b) --
    # the same O(#live) merge as the revisit-tolerant bitset dominance, shared via
    # `label_setting/utils.jl`.
    _sparse_station_age_values_dominate(
        a_ages.age_idx, a_ages.age_val, b_ages.age_idx, b_ages.age_val,
    ) || return false
    _joint_routing_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, layer_weight, budget,
    ) <= budget || return false
    return true
end
