"""
Orchestrates `labels.jl`'s primitives into a full label-setting pricing pass for
the passenger free-assignment subproblem: a priority-queue search over live
labels (`_enumerate_passenger_free_assignment_pricing_labels`), route replay to
turn a finished label's physical route into concrete per-passenger assignments
(`_passenger_free_assignment_column_from_route`), and the driver that turns
accepted routes into candidate columns
(`passenger_free_assignment_pricing_by_label_setting`).
"""

export passenger_free_assignment_pricing_by_label_setting

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
the same minimum-additional-time argument `_passenger_free_assignment_age_is_useful`
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
# revisit-tolerant pricer (`PassengerFreeAssignmentPricingLabel` /
# `PassengerFreeAssignmentLabelBitsets`) and the elementary station-simple pricer
# (`station_simple.jl`), whose label/bitset types differ but expose the same
# `current`/`time`/`activated_reward_layers` and `age_idx`/`age_val` fields the
# bound reads. Julia still specializes per concrete call site, so there is no
# dispatch or performance cost to dropping the annotations.
function _passenger_free_assignment_remaining_reward_bound(
    label,
    label_bs,
    pricing_data::PassengerFreeAssignmentPricingData,
    index::PassengerFreeAssignmentSearchIndex,
    workspace::PassengerFreeAssignmentBoundWorkspace,
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

"""
Context for the revisit-tolerant `PassengerFreeAssignmentCG` search: bundles
`pricing_data`, the once-built `dominates` closure, the precomputed
`search_index`/`bound_workspace` `_pricing_label_priority`'s remaining-reward
bound needs, and the two optional production knobs that have no counterpart on
the aggregate side:

  - `use_post_w_completion_bound` -- when set, `_pricing_label_priority` tries
    an exact completion oracle once a label is past the pickup window, which
    needs the search's own remaining time budget. Since hooks only ever see
    `ctx` and the label, that budget is stashed into the mutable `t_start`/
    `time_limit` fields by `_pricing_search_started!` (called once, before the
    loop starts) rather than threaded through every hook's signature.
  - `label_observer` -- an optional diagnostic callback invoked via
    `_pricing_on_label_inserted`.

The three post-`W` bound counters (`post_w_bound_calls`, `post_w_bound_states`,
`t_post_w_bound`) are mutable fields for the same reason: `_pricing_label_priority`
is the only place that produces them, and the shared engine's returned `stats`
has no slot for pricer-specific extras, so the wrapper below reads them back
out of `ctx` after the search returns and merges them in.
"""
struct PassengerFreeAssignmentSearchContext{D<:Function, O} <: AbstractPricingSearchContext{
    PassengerFreeAssignmentDominanceFilters, PassengerFreeAssignmentPricingLabel, PassengerFreeAssignmentLabelBitsets,
    Int, RewardLayerBitset,
}
    pricing_data::PassengerFreeAssignmentPricingData
    max_visits_per_node::Int
    use_post_w_completion_bound::Bool
    dominates::D
    search_index::PassengerFreeAssignmentSearchIndex
    bound_workspace::PassengerFreeAssignmentBoundWorkspace
    n_nodes::Int
    label_observer::O
    t_start::Base.RefValue{Float64}
    time_limit::Base.RefValue{Float64}
    post_w_bound_calls::Base.RefValue{Int}
    post_w_bound_states::Base.RefValue{Int}
    t_post_w_bound::Base.RefValue{Float64}
end

function PassengerFreeAssignmentSearchContext(
    pricing_data::PassengerFreeAssignmentPricingData;
    max_visits_per_node::Int,
    use_post_w_completion_bound::Bool=false,
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
    search_index = _build_passenger_free_assignment_search_index(pricing_data)
    bound_workspace = _create_passenger_free_assignment_bound_workspace(n_nodes)
    # Built once per search: the dominance switches live in the type, so the scan
    # compiles down to only the conditions this configuration actually uses.
    dominance_rules = _passenger_free_assignment_dominance_rules(
        pricing_data.bounded_max_stops,
        pricing_data.bounded_distinct_stations,
        pricing_data.compensated_dominance,
        dominance_census,
    )
    dominates(x::PricingBucketEntry, y::PricingBucketEntry) = _pricing_dominates_in_bucket(
        x.filters, x.bitsets, y.filters, y.bitsets, pricing_data.layer_weight, dominance_rules,
    )
    return PassengerFreeAssignmentSearchContext(
        pricing_data, max_visits_per_node, use_post_w_completion_bound, dominates,
        search_index, bound_workspace, n_nodes, label_observer,
        Ref(0.0), Ref(0.0), Ref(0), Ref(0), Ref(0.0),
    )
end

_pricing_initial_labels(ctx::PassengerFreeAssignmentSearchContext) =
    initial_passenger_free_assignment_pricing_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::PassengerFreeAssignmentSearchContext, label::PassengerFreeAssignmentPricingLabel) =
    _make_passenger_free_assignment_label_bitsets(label, ctx.search_index.node_index, ctx.n_nodes)

_pricing_bucket_signature(::PassengerFreeAssignmentSearchContext, label::PassengerFreeAssignmentPricingLabel, ::PassengerFreeAssignmentLabelBitsets) =
    _passenger_free_assignment_dominance_signature(label)

function _pricing_label_priority(
    ctx::PassengerFreeAssignmentSearchContext, label::PassengerFreeAssignmentPricingLabel, label_bs::PassengerFreeAssignmentLabelBitsets,
)::Float64
    if ctx.use_post_w_completion_bound && label.time + 1e-9 >= ctx.pricing_data.max_wait_time
        t0 = time()
        completion, completion_exhausted, completion_stats =
            passenger_free_assignment_post_w_completion(
                label, ctx.pricing_data; time_limit=max(1e-3, ctx.time_limit[] - (time() - ctx.t_start[])),
            )
        ctx.t_post_w_bound[] += time() - t0
        ctx.post_w_bound_calls[] += 1
        ctx.post_w_bound_states[] += completion_stats.states
        completion_exhausted && return completion.reduced_cost
    end
    return label.reduced_cost -
        _passenger_free_assignment_remaining_reward_bound(label, label_bs, ctx.pricing_data, ctx.search_index, ctx.bound_workspace)
end

_pricing_best_signature(::PassengerFreeAssignmentSearchContext, label::PassengerFreeAssignmentPricingLabel) =
    isempty(label.activated_reward_layers) ? nothing : _passenger_free_assignment_layer_signature(label)

_pricing_route_length(::PassengerFreeAssignmentSearchContext, label::PassengerFreeAssignmentPricingLabel) = label.route_length

_pricing_max_route_length(ctx::PassengerFreeAssignmentSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::PassengerFreeAssignmentSearchContext, label::PassengerFreeAssignmentPricingLabel) =
    _passenger_free_assignment_candidate_next_nodes(label, ctx.pricing_data; max_visits_per_node=ctx.max_visits_per_node)

# One child per stop, returned directly: see `_extend_passenger_free_assignment_pricing_label`.
_pricing_extend_label(ctx::PassengerFreeAssignmentSearchContext, label::PassengerFreeAssignmentPricingLabel, next_node::Int) =
    _extend_passenger_free_assignment_pricing_label(label, next_node, ctx.pricing_data)

_pricing_dominates_fn(ctx::PassengerFreeAssignmentSearchContext) = ctx.dominates

function _pricing_search_started!(ctx::PassengerFreeAssignmentSearchContext, t_start::Float64, time_limit::Float64)
    ctx.t_start[] = t_start
    ctx.time_limit[] = time_limit
    return nothing
end

function _pricing_on_label_inserted(ctx::PassengerFreeAssignmentSearchContext, label)
    isnothing(ctx.label_observer) || ctx.label_observer(label)
    return nothing
end

function _enumerate_passenger_free_assignment_pricing_labels(
    pricing_data::PassengerFreeAssignmentPricingData;
    time_limit::Float64,
    reduced_cost_tol::Float64,
    max_visits_per_node::Int,
    use_reduced_cost_pruning::Bool=true,
    use_post_w_completion_bound::Bool=false,
    profile::Bool=false,
    dominance_census::Bool=false,
    stop_if=label -> false,
    label_observer=nothing,
)
    ctx = PassengerFreeAssignmentSearchContext(
        pricing_data;
        max_visits_per_node=max_visits_per_node,
        use_post_w_completion_bound=use_post_w_completion_bound,
        dominance_census=dominance_census,
        label_observer=label_observer,
    )
    labels, exhausted, stats = _run_pricing_label_search(
        ctx;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        use_reduced_cost_pruning=use_reduced_cost_pruning,
        profile=profile,
        stop_if=stop_if,
    )
    stats = merge(stats, (
        post_w_bound_calls=ctx.post_w_bound_calls[],
        post_w_bound_states=ctx.post_w_bound_states[],
        t_post_w_bound_sec=ctx.t_post_w_bound[],
    ))
    return labels, exhausted, stats
end

"""
    _replay_passenger_free_assignment_route(route, pricing_data) -> Dict{Int, Tuple{Int,Int,Float64}}

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
function _replay_passenger_free_assignment_route(
    route::Vector{Int},
    pricing_data::PassengerFreeAssignmentPricingData,
)::Dict{Int, Tuple{Int, Int, Float64}}
    best = Dict{Int, Tuple{Int, Int, Float64}}()
    isempty(route) && return best

    pickup_time = Dict{Int, Float64}()
    current = route[1]
    elapsed_time = 0.0
    pickup_time[current] = 0.0  # t = 0 is always within the (non-negative) pickup window

    for idx in 2:length(route)
        next_node = route[idx]
        travel_time = _passenger_free_assignment_travel(pricing_data, current, next_node)
        elapsed_time += travel_time

        for opp in get(pricing_data.assignments_by_destination, next_node, PassengerAssignmentOpportunity[])
            origin_age = elapsed_time - get(pickup_time, opp.origin, -Inf)
            origin_age <= opp.ride_limit + 1e-9 || continue
            current_best = get(best, opp.passenger, nothing)
            if isnothing(current_best) || opp.reward > current_best[3] + 1e-9 ||
                    (abs(opp.reward - current_best[3]) <= 1e-9 && (opp.origin, opp.destination) < (current_best[1], current_best[2]))
                best[opp.passenger] = (opp.origin, opp.destination, opp.reward)
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
    _passenger_free_assignment_column_from_route(route, pricing_data)

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
function _passenger_free_assignment_column_from_route(
    route::Vector{Int},
    pricing_data::PassengerFreeAssignmentPricingData;
    label_reduced_cost::Union{Float64, Nothing}=nothing,
)
    best = _replay_passenger_free_assignment_route(route, pricing_data)
    assignments = Tuple{Int, Int, Int}[(p, o, d) for (p, (o, d, _r)) in best]
    reward_sum = sum((r for (_p, (_o, _d, r)) in best); init=0.0)
    tau = length(route) < 2 ? 0.0 :
        sum(_passenger_free_assignment_travel(pricing_data, route[i], route[i + 1]) for i in 1:(length(route) - 1))
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
    passenger_free_assignment_pricing_by_label_setting(pricing_data, existing_columns; kwargs...)

Top-level driver: runs the label search, replays every finished candidate route
to recover concrete assignments, and returns up to `max_new_columns` improving,
pool-novel `PassengerFreeAssignmentRouteColumn`s. Dedup/acceptance operates on
the *real* assignment signature (section 13), not the label search's cheap
reward-layer signature (section 12) -- two different physical routes that
happen to reach the same running per-passenger maxima are different candidate
columns here.
"""
function passenger_free_assignment_pricing_by_label_setting(
    pricing_data::PassengerFreeAssignmentPricingData,
    existing_columns::Vector{PassengerFreeAssignmentRouteColumn};
    next_column_id::Int,
    reduced_cost_tol::Float64=1e-6,
    max_new_columns::Int=1,
    n_candidates::Int=max_new_columns,
    time_limit::Float64=30.0,
    max_visits_per_node::Int=pricing_data.max_visits_per_node,
    profile::Bool=false,
    use_post_w_completion_bound::Bool=false,
    dominance_census::Bool=false,
)
    max_new_columns > 0 || throw(ArgumentError("max_new_columns must be positive"))
    n_candidates >= max_new_columns || throw(ArgumentError("n_candidates must be >= max_new_columns"))
    time_limit > 0 || throw(ArgumentError("time_limit must be positive"))

    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        signature = _passenger_free_assignment_column_signature(column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    scored_by_signature = Dict{Any, NamedTuple}()

    function try_accept_route!(route::Vector{Int}, label_reduced_cost::Float64)::Bool
        assignments, tau, reduced_cost = _passenger_free_assignment_column_from_route(
            route, pricing_data; label_reduced_cost=label_reduced_cost,
        )
        isempty(assignments) && return false
        reduced_cost < -reduced_cost_tol || return false
        signature = _passenger_free_assignment_column_signature(assignments)
        tau < get(best_pool_tau, signature, Inf) - 1e-9 || return false
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                reduced_cost < current.reduced_cost - 1e-9 ||
                (abs(reduced_cost - current.reduced_cost) <= 1e-9 && tau < current.tau - 1e-9)
            scored_by_signature[signature] = (reduced_cost=reduced_cost, route=route, assignments=assignments, tau=tau)
        end
        return length(scored_by_signature) >= n_candidates
    end

    labels, exhausted, stats = _enumerate_passenger_free_assignment_pricing_labels(
        pricing_data;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        max_visits_per_node=max_visits_per_node,
        profile=profile,
        use_post_w_completion_bound=use_post_w_completion_bound,
        dominance_census=dominance_census,
        stop_if=label -> try_accept_route!(label.route, label.reduced_cost),
    )

    for label in labels
        try_accept_route!(label.route, label.reduced_cost)
    end

    # Decorate-sort-undecorate. `sort!(...; by=f)` calls `f` inside the comparison,
    # so building the route's string form there cost one `string(::Vector{Int})`
    # per *comparison* rather than per column -- roughly 17x more at 10^5 harvested
    # columns, and measured at ~15% of this pricer's working time (array-show
    # machinery, `_typeinfo_implicit`, showing up in the flame graph). Each key is
    # now built exactly once; the ordering is unchanged.
    scored = collect(values(scored_by_signature))
    scored = _sort_pricing_results_by_route(scored,
        entry -> (entry.reduced_cost, entry.tau, string(entry.route)))
    scored = scored[1:min(length(scored), n_candidates)]
    scored = scored[1:min(length(scored), max_new_columns)]

    columns = PassengerFreeAssignmentRouteColumn[]
    column_id = next_column_id
    for entry in scored
        push!(columns, PassengerFreeAssignmentRouteColumn(
            column_id,
            entry.route,
            entry.assignments,
            entry.tau;
            metadata=Dict{String, Any}(
                "scenario" => pricing_data.scenario,
                "route" => Tuple(entry.route),
                "reduced_cost" => entry.reduced_cost,
            ),
        ))
        column_id += 1
    end
    return columns, exhausted, stats
end
