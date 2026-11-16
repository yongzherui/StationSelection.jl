"""
Context and hook wiring for `:exact_capacitated` (logic lives in `search.jl`), plus the loop
around the label search (`_run_label_setting` below): it tracks the best capacitated column,
tightens the pruning threshold to it, and grows the critical set `S` when `S` started empty.
The remaining-reward bound is `search.jl`'s capacity-aware `_capacitated_remaining_bound`.
"""

struct JointRoutingAssignmentCapacitatedSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentCapacitatedDominanceFilters, JointRoutingAssignmentCapacitatedPricingLabel,
    JointRoutingAssignmentCapacitatedLabelBitsets, Int, CapacitatedBestSignature,
}
    pricing_data::JointRoutingAssignmentCapacitatedPricingData
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    # route => its capacitated optimum (`_capacitated_route_optimum`); one search per
    # context, single-threaded, so a plain Dict.
    route_cache::Dict{Vector{Int}, Any}
    # the capacity-aware remaining-reward bound's per-pass index (rebuilt when S grows)
    cap_bound::CapacitatedBoundIndex
    cap_scan::CapacitatedScan
    # bitsets already computed for a child (handed to the engine's insert) and the bound of each
    # inserted label (read back when it is popped), so neither is computed twice
    bs_cache::IdDict{JointRoutingAssignmentCapacitatedPricingLabel, JointRoutingAssignmentCapacitatedLabelBitsets}
    bound_of::IdDict{JointRoutingAssignmentCapacitatedPricingLabel, Float64}
    # Labels whose priority (reduced cost minus the remaining bound) is at least `thr[]` are
    # neither inserted nor extended: `-tol`, tightened to `rho * (best capacitated reduced
    # cost found)` as columns turn up (`rho = 0` keeps `-tol`).
    thr::Base.RefValue{Float64}
    rho::Float64
    n_pruned::Base.RefValue{Int}
end

function JointRoutingAssignmentCapacitatedSearchContext(pd::JointRoutingAssignmentCapacitatedPricingData; rho::Float64=1.0)
    rules = JointRoutingAssignmentCapacitatedDominanceRules{pd.base.bounded_max_stops, pd.base.compensated_dominance}()
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) =
        _capacitated_dominates(x.filters, x.bitsets, y.filters, y.bitsets, pd, rules)
    index = _build_joint_routing_assignment_search_index(pd.base)
    return JointRoutingAssignmentCapacitatedSearchContext(
        pd, dominates, index, _create_joint_routing_assignment_bound_workspace(), Dict{Vector{Int}, Any}(),
        _build_capacitated_bound_index(pd, index), CapacitatedScan(),
        IdDict{JointRoutingAssignmentCapacitatedPricingLabel, JointRoutingAssignmentCapacitatedLabelBitsets}(),
        IdDict{JointRoutingAssignmentCapacitatedPricingLabel, Float64}(),
        Ref(-1e-6), rho, Ref(0),
    )
end

const _CapCtx = JointRoutingAssignmentCapacitatedSearchContext
const _CapLabel = JointRoutingAssignmentCapacitatedPricingLabel

_pricing_initial_labels(ctx::_CapCtx) = initial_joint_routing_assignment_capacitated_pricing_labels(ctx.pricing_data)
function _pricing_make_bitsets(ctx::_CapCtx, label::_CapLabel)
    bs = pop!(ctx.bs_cache, label, nothing)
    bs === nothing && (bs = _make_joint_routing_assignment_capacitated_label_bitsets(label, ctx))
    ctx.bound_of[label] = bs.bound
    return bs
end
_pricing_state(::_CapCtx, label::_CapLabel, ::JointRoutingAssignmentCapacitatedLabelBitsets) = label.current
_pricing_label_priority(ctx::_CapCtx, label::_CapLabel, bs::JointRoutingAssignmentCapacitatedLabelBitsets)::Float64 =
    label.reduced_cost - bs.bound
function _pricing_best_signature(::_CapCtx, label::_CapLabel)
    isempty(label.activated_reward_layers) && return nothing
    v = Int[]
    for (p, j, k, _, _) in label.credited
        push!(v, p, j, k)
    end
    return (label.activated_reward_layers, v)
end
_pricing_route_length(::_CapCtx, label::_CapLabel) = label.route_length
_pricing_max_route_length(ctx::_CapCtx) = ctx.pricing_data.base.max_stops
"""The children worth inserting: every action's child whose priority (a lower bound on any
completion's reduced cost, the child's own included) is below the threshold `ctx.thr[]`.
Returned as the engine's "next nodes"; `_pricing_extend_label` passes each through. The
parent itself is re-tested first, since the threshold may have tightened since its push."""
function _pricing_candidate_next_nodes(ctx::_CapCtx, label::_CapLabel)
    pd = ctx.pricing_data
    thr = ctx.thr[]
    out = _CapLabel[]
    bnd = pop!(ctx.bound_of, label, NaN)
    isnan(bnd) && (bnd = _make_joint_routing_assignment_capacitated_label_bitsets(label, ctx).bound)
    label.reduced_cost - bnd < thr || return out
    for a in _capacitated_candidate_actions(label, pd; ctx = ctx, thr = thr)
        child = a.child === nothing ? _extend_joint_routing_assignment_capacitated_label(label, a, pd) : a.child
        cbs = _make_joint_routing_assignment_capacitated_label_bitsets(child, ctx)
        pri = child.route_length >= pd.base.max_stops ? child.reduced_cost : child.reduced_cost - cbs.bound
        if pri < thr
            ctx.bs_cache[child] = cbs
            push!(out, child)
        else
            ctx.n_pruned[] += 1
        end
    end
    return out
end
_pricing_extend_label(::_CapCtx, ::_CapLabel, child::_CapLabel) = child
_pricing_dominates_fn(ctx::_CapCtx) = ctx.dominates

"""The capacitated optimum of `label`'s route, cached per route."""
_capacitated_route_result(ctx::_CapCtx, route::Vector{Int}) =
    get!(() -> _capacitated_route_optimum(route, ctx.pricing_data), ctx.route_cache, route)

function _pricing_candidate_from_label(ctx::_CapCtx, label::_CapLabel)
    # Not comparable to `label.reduced_cost` route by route: the label's critical credits are
    # one choice among many, and the route's capacitated optimum may credit a critical rider
    # the label skipped. The relaxation bound holds over all labels, not per label.
    assignments, tau, rc, positions, max_load = _capacitated_route_result(ctx, label.route)
    isempty(assignments) && return nothing
    return (
        signature = Tuple(sort(assignments)), tau = tau, reduced_cost = rc,
        payload = (route = label.route, assignments = assignments, positions = positions, max_load = max_load),
    )
end

_pricing_pool_signature(::_CapCtx, existing_column::JointRoutingAssignmentRouteColumn) =
    _joint_routing_assignment_column_signature(existing_column)

_pricing_make_column(ctx::_CapCtx, column_id::Int, candidate) =
    JointRoutingAssignmentRouteColumn(
        column_id, candidate.payload.route, candidate.payload.assignments, candidate.tau;
        metadata = Dict{String, Any}(
            "scenario" => ctx.pricing_data.base.scenario,
            "route" => Tuple(candidate.payload.route),
            "reduced_cost" => candidate.reduced_cost,
            "assignment_positions" => candidate.payload.positions,
            "max_load" => candidate.payload.max_load,
            "vehicle_capacity" => ctx.pricing_data.capacity,
        ),
    )

function _pricing_verify_column(::_CapCtx, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    return _verify_joint_routing_assignment_master_reduced_cost(
        column, m, m[:joint_routing_assignment_data], mapping, alpha, gamma_o, gamma_d)
end

# ── the loop that grows S ────────────────────────────────────────────────────
"""
    _run_label_setting(ctx::JointRoutingAssignmentCapacitatedSearchContext; kwargs...)

Run the relaxed label search (the generic engine), then decide from its finished labels:

  - some route's capacitated optimum prices below `-tol` and `stop_at_first_column`:
    done -- the caller's `stop_if` has already harvested those columns. Reported as not
    exhausted, since the relaxed search proved nothing about the capacitated optimum.
  - the best relaxed reduced cost is at least `min(best capacitated, -tol)`: done and
    exhausted -- no route can do better than what was found (or than `-tol`).
  - otherwise the best relaxed routes overload some arc; their riders on overloaded arcs
    join `S` and the search reruns with the time left.

The returned labels are those of the last pass; `stats` sums every pass and adds
`dssr_passes`, `n_critical` and `best_capacitated_rc`.
"""
function _run_label_setting(
    ctx::_CapCtx;
    time_limit::Float64,
    reduced_cost_tol::Float64,
    use_reduced_cost_pruning::Bool=true,
    profile::Bool=false,
    stop_if=label -> false,
)
    pd = ctx.pricing_data
    deadline = time() + time_limit
    passes = 0
    totals = Dict{Symbol, Float64}()
    local labels, exhausted, stats
    best_true = Inf
    ctx.thr[] = -reduced_cost_tol
    ctx.n_pruned[] = 0
    settle() = (ctx.thr[] = min(-reduced_cost_tol, isfinite(best_true) ? ctx.rho * best_true : -reduced_cost_tol))
    while true
        passes += 1
        _refresh_capacitated_bound_index!(ctx.cap_bound, pd, ctx.search_index)
        empty!(ctx.bs_cache); empty!(ctx.bound_of)
        found_column = false
        stopped_by_caller = false
        function wrapped(label)
            r = _capacitated_route_result(ctx, label.route)
            r[3] < best_true && !isempty(r[1]) && (best_true = r[3]; settle())
            r[3] < -reduced_cost_tol && !isempty(r[1]) && (found_column = true)
            s = stop_if(label)
            s && (stopped_by_caller = true)
            return s
        end
        labels, exhausted, stats = invoke(
            _run_label_setting, Tuple{AbstractPricingSearchContext}, ctx;
            time_limit = max(0.0, deadline - time()), reduced_cost_tol = reduced_cost_tol,
            use_reduced_cost_pruning = use_reduced_cost_pruning, profile = profile, stop_if = wrapped,
        )
        for (k, v) in pairs(stats)
            totals[k] = k === :max_live_labels || k === :max_frontier_size ? max(get(totals, k, 0.0), v) : get(totals, k, 0.0) + v
        end
        # every finished label's route counts towards the capacitated best, not only the
        # incumbents offered to `stop_if`
        for l in labels
            r = _capacitated_route_result(ctx, l.route)
            !isempty(r[1]) && r[3] < best_true && (best_true = r[3])
        end
        settle()
        relaxed_min = minimum((l.reduced_cost for l in labels); init = Inf)
        @debug "exact_capacitated pass" scenario = pd.base.scenario pass = passes critical = count(pd.critical) labels = stats.labels_generated max_live = stats.max_live_labels relaxed_min best_true thr = ctx.thr[] exhausted
        (stopped_by_caller || !exhausted) && (exhausted = false; break)
        # proved: no column below `threshold` (labels at or above the pruning threshold were
        # cut by an admissible bound, so the returned labels' minimum is all that is left)
        threshold = min(best_true, ctx.thr[])
        relaxed_min >= threshold - 1e-9 && break
        if found_column && pd.stop_at_first_column
            exhausted = false
            break
        end
        # grow S from the best relaxed routes that could still beat `threshold`
        cand = sort!([l for l in labels if l.reduced_cost < threshold - 1e-9]; by = l -> l.reduced_cost)
        added = 0
        for (i, l) in enumerate(cand)
            i > pd.growth_routes && added > 0 && break
            for p in _capacitated_overload_riders(l, pd)
                pd.critical[p] || (pd.critical[p] = true; added += 1)
            end
        end
        added > 0 || error("exact_capacitated: relaxed optimum below the capacitated best, but no " *
                           "rider to add to the critical set (relaxed $relaxed_min, capacitated $best_true)")
        time() < deadline || (exhausted = false; break)
    end
    stats = (; (k => (k in (:labels_generated, :labels_rejected_by_dominance, :labels_removed_by_dominance,
                               :stale_pops, :max_frontier_size, :max_live_labels) ? Int(v) : v) for (k, v) in totals)...,
             dssr_passes = passes, pruned_by_bound = ctx.n_pruned[], threshold = ctx.thr[], n_critical = count(pd.critical), best_capacitated_rc = best_true)
    return labels, exhausted, stats
end
