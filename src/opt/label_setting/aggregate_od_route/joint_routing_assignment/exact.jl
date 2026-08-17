"""
`JointRoutingAssignmentSearchContext`: the revisit-tolerant passenger
free-assignment pricer's plug into the shared search loop
(`_run_pricing_label_search`, `engine.jl`), plus route replay
(`_joint_routing_assignment_column_from_route`) to turn a finished label's
physical route into concrete per-passenger assignments -- the only place
concrete `(j, k)` pairs are recovered, since the label itself only tracks a
cheap reward-layer proxy during search (section 16). `round.jl`'s
`_pricing_candidate_from_label` hook is what invokes replay, once per
surviving label offered to `_run_pricing_round`'s accept/dedupe test.
"""

# ── remaining-reward bound (drives frontier priority + pop-time pruning) ────
"""
Admissible bound on the additional *net* gain (reward minus the route
regularization cost of the travel needed to collect it) still available to
`label`. `label.reduced_cost - bound` is therefore a lower bound on the reduced
cost of every completion of `label`, used both to order the frontier and to
prune at pop time.

Rewritten once from an `O(|opportunities|)` scan (every opportunity re-tested for
every label) to `O(#live origins' opportunities + n_nodes)`: `|opportunities| ~
P * n^2`, so the old form made per-label cost grow with `n^2` and drove measured
time to ~`n^5.5-7.6` while label counts only grew ~`n^3.4`.

That rewrite is why this is **no longer** a hot spot, and the claim that it is
"the single biggest cost in the search" -- which this docstring used to make --
has been false ever since. Profiling on 2026-07-30 put this bound at ~0.6% of
wall time against ~90% in the dominance scan. Do not spend effort tightening it
for speed; see `notes/2026-07-30_passenger_pricing_label_search_optimizations.md`.

Two structurally different sources of future reward, handled separately:

  - **live origins** -- an origin with a live clock can still certify its own
    opportunities, but only those whose ride limit survives `age + travel(current, k)`.
    That test needs the actual age, so it stays per-opportunity -- but only over
    opportunities of origins that are *actually live*, which pruning keeps small.
    Such an opportunity is booked against its *destination* `k`, the node the
    vehicle must reach to collect it.

  - **refreshable origins** -- if still inside the pickup window, any origin
    reachable before the cutoff could be visited to open a fresh clock. The old
    code tested this per opportunity but its condition depends only on the
    *origin*, so the whole origin's union mask (`origin_union_mask`) can be OR'd
    in at once. Such a mask is booked against the *origin* `j`, the node the
    vehicle must reach first.

# Why the travel discount is valid

The previous version returned the raw reward `R` of everything still reachable,
silently pretending it were free. But collecting a layer booked against node `x`
requires physically reaching `x`, which costs at least `travel(current, x)` --
the same minimum-additional-time argument `_joint_routing_assignment_age_is_useful`
already relies on, and which holds because routing costs are shortest-path
distances and so obey the triangle inequality (detouring only adds more).

So for any completion that collects the layers booked against a node set `S`,
`travel >= max_{x in S} travel(current, x)`, giving net gain
`R(S) - beta * max_{x in S} travel(current, x)`. For a fixed travel budget the
best `S` is "every node within that radius", so scanning nodes in increasing
distance from `current` and taking the running maximum of
`R(prefix) - beta * travel(current, node)` maximises over all `S` in one pass --
no subset enumeration. `max(0.0, ...)` covers "stop here and collect nothing",
which is always available.

`R(prefix)` is the weight of the *union* of the prefix's masks minus `activated`,
accumulated incrementally in `acc`, so layers shared between nodes (the same
passenger certifiable at several destinations) are counted once rather than
summed -- a sum would still be a valid over-estimate, but a needlessly loose one.

Nodes are walked via `nodes_by_travel[current_idx]`, precomputed once per pricing
call, so no per-label sorting is needed.

MEASURED: **no effect** -- label counts moved by under 0.2% and wall time was flat.
At cold-start duals the bound sums 10+ passenger rewards of ~5000 each while one
hop costs `beta * travel ~ 4000`, so discounting one hop essentially never flips
the pruning test. Retained because it is exact, strictly tighter, and should bite
under converged CG duals where most `rho_pjk` are near zero and `beta * travel` is
comparable to the entire remaining reward -- a regime this benchmark could not
reproduce. Do not assume it helps without measuring on the target duals.
"""
# `label`/`label_bs` are intentionally untyped: this bound is shared by the
# revisit-tolerant pricer (`JointRoutingAssignmentPricingLabel` /
# `JointRoutingAssignmentLabelBitsets`) and the elementary station-simple pricer
# (`station_simple.jl`), whose label/bitset types differ but expose the same
# `current`/`time`/`activated_reward_layers` and `age_idx`/`age_val` fields the
# bound reads. Julia still specializes per concrete call site, so there is no
# dispatch or performance cost to dropping the annotations.
function _joint_routing_assignment_remaining_reward_bound(
    label,
    label_bs,
    pricing_data::JointRoutingAssignmentPricingData,
    index::JointRoutingAssignmentSearchIndex,
    workspace::JointRoutingAssignmentBoundWorkspace,
)::Float64
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    current_idx = index.node_index[label.current]
    activated = label.activated_reward_layers
    beta = pricing_data.route_regularization_weight
    layer_weight = pricing_data.layer_weight

    empty!(workspace.touched_nodes)

    # live origins: age-dependent, so still per-opportunity -- but only theirs.
    # Booked against the destination that would certify them.
    @inbounds for t in eachindex(label_bs.age_idx)
        origin_idx = Int(label_bs.age_idx[t])
        age = label_bs.age_val[t]
        for i in index.opps_by_origin_idx[origin_idx]
            issubset(index.opp_layer_mask[i], activated) && continue
            dest_idx = index.opp_dest_idx[i]
            age + index.travel_matrix[current_idx, dest_idx] <= index.opp_ride_limit[i] + 1e-9 || continue
            isempty(workspace.node_mask[dest_idx]) && push!(workspace.touched_nodes, dest_idx)
            union!(workspace.node_mask[dest_idx], index.opp_layer_mask[i])
        end
    end

    # refreshable origins: condition depends only on the origin, so OR whole masks.
    # Booked against the origin, which must be reached before any of its
    # opportunities can even start.
    if !past_pickup_cutoff
        @inbounds for origin_idx in eachindex(index.origin_union_mask)
            isempty(index.origin_union_mask[origin_idx]) && continue
            label.time + index.travel_matrix[current_idx, origin_idx] <=
                pricing_data.max_wait_time + 1e-9 || continue
            isempty(workspace.node_mask[origin_idx]) && push!(workspace.touched_nodes, origin_idx)
            union!(workspace.node_mask[origin_idx], index.origin_union_mask[origin_idx])
        end
    end

    best = 0.0
    if !isempty(workspace.touched_nodes)
        empty!(workspace.layer_scratch)
        acc_weight = 0.0
        remaining = length(workspace.touched_nodes)
        @inbounds for x in index.nodes_by_travel[current_idx]
            mask = workspace.node_mask[x]
            isempty(mask) && continue
            for layer in mask
                (layer in activated || layer in workspace.layer_scratch) && continue
                push!(workspace.layer_scratch, layer)
                acc_weight += layer_weight[layer]
            end
            gain = acc_weight - beta * index.travel_matrix[current_idx, x]
            gain > best && (best = gain)
            remaining -= 1
            remaining == 0 && break
        end
    end

    @inbounds for x in workspace.touched_nodes
        empty!(workspace.node_mask[x])
    end
    return best
end

# ── search context: struct + constructor ────────────────────────────────────
"""
Context for the revisit-tolerant `JointRoutingAssignmentCG` search: bundles
`pricing_data`, the once-built `dominates` closure, the precomputed
`search_index`/`bound_workspace` `_pricing_label_priority`'s remaining-reward
bound needs, and one optional production knob:

  - `label_observer` -- an optional diagnostic callback invoked via
    `_pricing_on_label_inserted`.

(The post-`W` exact completion bound this context used to also support has been
removed along with `post_w_completion.jl` -- peripheral to the base search, not
part of it.)
"""
struct JointRoutingAssignmentSearchContext{D<:Function, O} <: AbstractPricingSearchContext{
    JointRoutingAssignmentDominanceFilters, JointRoutingAssignmentPricingLabel, JointRoutingAssignmentLabelBitsets,
    Int, RewardLayerBitset,
}
    pricing_data::JointRoutingAssignmentPricingData
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    n_nodes::Int
    label_observer::O
end

function JointRoutingAssignmentSearchContext(
    pricing_data::JointRoutingAssignmentPricingData;
    # Count which dominance condition rejected each tested pair, into
    # `PFA_DOMINANCE_REJECTIONS`. Off in production: it selects an instrumented
    # specialization of the dominance predicate, so the counters cost nothing at
    # all when this is `false`. See `julia scripts/diagnose.jl dominance_audit`.
    dominance_census::Bool=false,
    # Diagnostic hook: called once per label that survives dominance and enters the
    # frontier. `nothing` (the default) costs one branch per insertion and nothing
    # else -- production pricing never sets it. Used by
    # `julia scripts/diagnose.jl split_census` to census the live-label population
    # (live-clock support, pickup-phase membership) without duplicating this loop.
    label_observer=nothing,
)
    n_nodes = length(pricing_data.nodes)
    search_index = _build_joint_routing_assignment_search_index(pricing_data)
    bound_workspace = _create_joint_routing_assignment_bound_workspace(n_nodes)
    # Built once per search: the dominance switches live in the type, so the scan
    # compiles down to only the conditions this configuration actually uses.
    dominance_rules = _joint_routing_assignment_dominance_rules(
        pricing_data.bounded_max_stops,
        pricing_data.compensated_dominance,
        dominance_census,
    )
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.bitsets, y.filters, y.bitsets, pricing_data.layer_weight, dominance_rules,
    )
    return JointRoutingAssignmentSearchContext(
        pricing_data, dominates, search_index, bound_workspace, n_nodes, label_observer,
    )
end

# ── context hooks (AbstractPricingSearchContext contract) ───────────────────
_pricing_initial_labels(ctx::JointRoutingAssignmentSearchContext) =
    initial_joint_routing_assignment_pricing_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel) =
    _make_joint_routing_assignment_label_bitsets(label, ctx.search_index.node_index, ctx.n_nodes)

_pricing_state(::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel, ::JointRoutingAssignmentLabelBitsets) =
    _joint_routing_assignment_state(label)

function _pricing_label_priority(
    ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel, label_bs::JointRoutingAssignmentLabelBitsets,
)::Float64
    return label.reduced_cost -
        _joint_routing_assignment_remaining_reward_bound(label, label_bs, ctx.pricing_data, ctx.search_index, ctx.bound_workspace)
end

_pricing_best_signature(::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel) =
    isempty(label.activated_reward_layers) ? nothing : _joint_routing_assignment_layer_signature(label)

_pricing_route_length(::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel) = label.route_length

_pricing_max_route_length(ctx::JointRoutingAssignmentSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel) =
    _joint_routing_assignment_candidate_next_nodes(label, ctx.pricing_data)

# One child per stop, returned directly: see `_extend_joint_routing_assignment_pricing_label`.
_pricing_extend_label(ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel, next_node::Int) =
    _extend_joint_routing_assignment_pricing_label(label, next_node, ctx.pricing_data)

_pricing_dominates_fn(ctx::JointRoutingAssignmentSearchContext) = ctx.dominates

function _pricing_search_started!(::JointRoutingAssignmentSearchContext, ::Float64, ::Float64)
    return nothing
end

function _pricing_on_label_inserted(ctx::JointRoutingAssignmentSearchContext, label)
    isnothing(ctx.label_observer) || ctx.label_observer(label)
    return nothing
end

# ── post-search: route replay → candidate → column → master verification ───
"""
    _replay_joint_routing_assignment_route(route, pricing_data) -> Dict{Int, Tuple{Int,Int,Float64}}

Replay a finished physical route from `t = 0`, independently of any label's
(possibly dominance-pruned) station-age history, and return each passenger's
best certified assignment as `passenger => (origin, destination, reward)`.
This is the only place concrete `(j, k)` pairs are recovered -- label expansion
never materializes them (section 16), since only a small number of finished
candidate routes ever need this, not every intermediate label.

Ties on reward are broken lexicographically by `(origin, destination)` for
determinism.

Clocks are held as **absolute pickup times**, not ages. Ageing every live clock by
the same `travel_time` at each stop used to be written as a comprehension, which
built a brand-new `Dict` per stop of every route replayed -- and replay runs once
per improving route, which in a harvesting configuration is once per accepted
column. Storing `pickup_time[j]` and deriving `age = elapsed_time - pickup_time[j]`
at the point of use is the same arithmetic with no rebuild: a station never seen
reads back as `-Inf`, so its age is `Inf`, exactly as the missing-key default was.
"""
function _replay_joint_routing_assignment_route(
    route::Vector{Int},
    pricing_data::JointRoutingAssignmentPricingData,
)::Dict{Int, Tuple{Int, Int, Float64}}
    best = Dict{Int, Tuple{Int, Int, Float64}}()
    isempty(route) && return best

    pickup_time = Dict{Int, Float64}()
    current = route[1]
    elapsed_time = 0.0
    pickup_time[current] = 0.0  # t = 0 is always within the (non-negative) pickup window

    for idx in 2:length(route)
        next_node = route[idx]
        travel_time = _joint_routing_assignment_travel(pricing_data, current, next_node)
        elapsed_time += travel_time

        for opp in get(pricing_data.assignments_by_destination, next_node, PassengerAssignmentOpportunity[])
            origin_age = elapsed_time - get(pickup_time, opp.origin, -Inf)
            origin_age <= opp.ride_limit + 1e-9 || continue
            current_best = get(best, opp.p, nothing)
            if isnothing(current_best) || opp.reward > current_best[3] + 1e-9 ||
                    (abs(opp.reward - current_best[3]) <= 1e-9 && (opp.origin, opp.destination) < (current_best[1], current_best[2]))
                best[opp.p] = (opp.origin, opp.destination, opp.reward)
            end
        end

        if elapsed_time <= pricing_data.max_wait_time + 1e-9
            pickup_time[next_node] = elapsed_time  # a fresh clock, i.e. age 0 from here
        end
        current = next_node
    end

    return best
end

"""
    _joint_routing_assignment_column_from_route(route, pricing_data)

Route replay plus per-passenger argmax selection (spec section 13): returns
`(assignments, tau, reduced_cost)` where `assignments` is
`[(p, j_p*, k_p*), ...]` for every passenger with a positive certified reward,
`tau` is the route's physical travel time, and `reduced_cost` is recomputed
directly from the selected assignments' rewards (not copied from any label).
When `label_reduced_cost` is supplied, asserts the two agree within tolerance
-- the correctness invariant from spec section 7/13, checked on every finished
route rather than only in tests, since replay is cheap relative to the search
that produced the route in the first place.
"""
function _joint_routing_assignment_column_from_route(
    route::Vector{Int},
    pricing_data::JointRoutingAssignmentPricingData;
    label_reduced_cost::Union{Float64, Nothing}=nothing,
)
    best = _replay_joint_routing_assignment_route(route, pricing_data)
    assignments = Tuple{Int, Int, Int}[(p, o, d) for (p, (o, d, _r)) in best]
    reward_sum = sum((r for (_p, (_o, _d, r)) in best); init=0.0)
    tau = length(route) < 2 ? 0.0 :
        sum(_joint_routing_assignment_travel(pricing_data, route[i], route[i + 1]) for i in 1:(length(route) - 1))
    reduced_cost = pricing_data.route_regularization_weight * (tau + pricing_data.repositioning_time) - reward_sum

    if !isnothing(label_reduced_cost)
        @assert isapprox(reduced_cost, label_reduced_cost; atol=1e-6) (
            "reconstructed reduced cost $(reduced_cost) does not match the searched label's " *
            "$(label_reduced_cost) for route $(route) -- reward-layer accounting is inconsistent " *
            "with a direct passenger-by-passenger recomputation"
        )
    end

    return assignments, tau, reduced_cost
end

"""
Route-replay is what makes this pricer's `_pricing_candidate_from_label`
different from the trivial projection every other pricer uses: the label only
tracks a cheap reward-layer proxy during search (section 16), so the real,
concrete `(p, j, k)` assignments -- and the real dedup signature over them,
not the search's proxy signature -- only exist after replaying the finished
route. `signature`/`tau`/`reduced_cost` below are the *replayed*, exact values;
`payload` carries what `_pricing_make_column` needs to build the column.
"""
function _pricing_candidate_from_label(ctx::JointRoutingAssignmentSearchContext, label::JointRoutingAssignmentPricingLabel)
    assignments, tau, reduced_cost = _joint_routing_assignment_column_from_route(
        label.route, ctx.pricing_data; label_reduced_cost=label.reduced_cost,
    )
    isempty(assignments) && return nothing
    return (
        signature=_joint_routing_assignment_column_signature(assignments),
        tau=tau, reduced_cost=reduced_cost, payload=(route=label.route, assignments=assignments),
    )
end

_pricing_pool_signature(::JointRoutingAssignmentSearchContext, existing_column::JointRoutingAssignmentRouteColumn) =
    _joint_routing_assignment_column_signature(existing_column)

_pricing_make_column(ctx::JointRoutingAssignmentSearchContext, column_id::Int, candidate) =
    JointRoutingAssignmentRouteColumn(
        column_id, candidate.payload.route, candidate.payload.assignments, candidate.tau;
        metadata=Dict{String, Any}(
            "scenario" => ctx.pricing_data.scenario,
            "route" => Tuple(candidate.payload.route),
            "reduced_cost" => candidate.reduced_cost,
        ),
    )

"""
Unlike Base's single-constraint-family check (`aggregate_od_route/exact.jl`),
Joint's master reduced cost sums the coverage row plus both linking-row
families per assignment (`_verify_joint_routing_assignment_master_reduced_cost`,
`duals.jl`) -- so this hook, unlike the aggregate pricers' twins, genuinely
needs `m`/`mapping`/`duals`, not just `ctx`."""
function _pricing_verify_column(::JointRoutingAssignmentSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end
