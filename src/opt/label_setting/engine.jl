"""
The label-setting search loop itself. See `types.jl` for the per-state label
list and the `AbstractPricingSearchContext` hook contract this loop is
written against, and `utils.jl` for the reward-model-independent math those
hooks lean on.
"""

"""
    _run_pricing_label_search(ctx; time_limit, reduced_cost_tol, use_reduced_cost_pruning=true, profile=false, stop_if=label->false)

The priority-queue label-setting search shared by every pricer's enumerate
function: seed the frontier from `_pricing_initial_labels`, then repeatedly pop
the most promising live label, record it as a best-so-far candidate under
`_pricing_best_signature` (offering it to `stop_if` for early exit), and -- if
it is not at the stops cap and its priority still beats `reduced_cost_tol` --
extend it along every `_pricing_candidate_next_nodes` result, inserting each
child through the shared `_add_pricing_label_to_state!`.

This function is the generalization of what were four independently written,
near-identical copies of this exact loop (frontier/live-labels/per-state-label-list
bookkeeping, the same `stats` shape, the same profiling timers, the same
stale-pop handling) -- one per pricer × route-universe combination. Everything
that differs between them (candidate generation, label extension, the reward
bound behind the priority, what counts as a "finished" label worth tracking) is
a hook on `ctx`; this loop knows none of it. See any of the four concrete
`AbstractPricingSearchContext` subtypes and their `_enumerate_*` wrapper for
the pattern.
"""
function _run_pricing_label_search(
    ctx::AbstractPricingSearchContext{F, L, B, State, BestSig};
    time_limit::Float64,
    reduced_cost_tol::Float64,
    use_reduced_cost_pruning::Bool=true,
    profile::Bool=false,
    stop_if=label -> false,
) where {F, L, B, State, BestSig}
    # ── setup: frontier, live-label table, per-state buckets, stats counters ──
    frontier = PriorityQueue{Int, Float64}()
    live_labels = Union{Nothing, L}[]
    n_live_labels = 0
    labels_by_state = Dict{State, PricingStateLabels{F, L, B}}()
    best_by_signature = Dict{BestSig, L}()
    dominated_scratch = Int[]
    dominates = _pricing_dominates_fn(ctx)

    exhausted = true
    t_start = time()
    _pricing_search_started!(ctx, t_start, time_limit)
    next_label_id = 1
    labels_generated = 0
    labels_rejected_by_dominance = 0
    labels_removed_by_dominance = 0
    stale_pops = 0
    max_frontier_size = 0
    max_live_labels = 0
    t_queue = UInt64(0)
    t_candidates = UInt64(0)
    t_extension = UInt64(0)
    t_dominance = UInt64(0)

    # ── add_label!: shared insert-into-state + dominance-prune + frontier-push ──
    # Local closure, not a top-level function: it mutates ~10 locals above
    # (live_labels, n_live_labels, labels_by_state, frontier, next_label_id,
    # the stats counters) directly by capture. Julia has no out-parameters for
    # plain locals, so a top-level function would need everything boxed into
    # Refs or a mutable struct just to be mutated from outside -- more
    # indirection, not less. Called from both the initial-seeding loop below
    # and the extend loop inside the `while`, so this also avoids duplicating
    # the insert-and-update-frontier logic between those two call sites.
    function add_label!(label::L)
        label_id = next_label_id
        next_label_id += 1
        labels_generated += 1
        push!(live_labels, label)
        n_live_labels += 1
        label_bs = _pricing_make_bitsets(ctx, label)
        state = _pricing_state(ctx, label, label_bs)
        state_labels = get!(() -> PricingStateLabels{F, L, B}(), labels_by_state, state)

        t0 = profile ? time_ns() : UInt64(0)
        inserted, removed = _add_pricing_label_to_state!(
            state_labels, live_labels, label, label_id, label_bs, dominates, dominated_scratch,
        )
        profile && (t_dominance += time_ns() - t0)
        labels_removed_by_dominance += removed
        n_live_labels -= removed

        if !inserted
            live_labels[label_id] = nothing
            n_live_labels -= 1
            labels_rejected_by_dominance += 1
            return nothing
        end

        t0 = profile ? time_ns() : UInt64(0)
        push!(frontier, label_id => _pricing_label_priority(ctx, label, label_bs))
        profile && (t_queue += time_ns() - t0)
        max_frontier_size = max(max_frontier_size, length(frontier))
        max_live_labels = max(max_live_labels, n_live_labels)
        _pricing_on_label_inserted(ctx, label)
        return nothing
    end

    # ── seed the frontier from depth-1 labels ──
    for label in _pricing_initial_labels(ctx)
        add_label!(label)
    end

    # ── main loop: pop cheapest, record incumbent (offer to stop_if), prune, extend ──
    while !isempty(frontier)
        if time() - t_start > time_limit
            exhausted = false
            break
        end

        t0 = profile ? time_ns() : UInt64(0)
        label_id, popped_priority = popfirst!(frontier)
        profile && (t_queue += time_ns() - t0)
        maybe_label = live_labels[label_id]
        if isnothing(maybe_label)
            stale_pops += 1
            continue
        end
        label = maybe_label::L

        maybe_signature = _pricing_best_signature(ctx, label)
        if !isnothing(maybe_signature)
            signature = maybe_signature::BestSig
            incumbent = get(best_by_signature, signature, nothing)
            if isnothing(incumbent) || label.tau < incumbent.tau - 1e-9
                best_by_signature[signature] = label
                if stop_if(label)
                    exhausted = false
                    break
                end
            end
        end

        _pricing_route_length(ctx, label) >= _pricing_max_route_length(ctx) && continue
        if use_reduced_cost_pruning
            popped_priority >= -reduced_cost_tol && continue
        end

        t0 = profile ? time_ns() : UInt64(0)
        next_nodes = _pricing_candidate_next_nodes(ctx, label)
        profile && (t_candidates += time_ns() - t0)

        for next_node in next_nodes
            t0 = profile ? time_ns() : UInt64(0)
            child = _pricing_extend_label(ctx, label, next_node)
            profile && (t_extension += time_ns() - t0)
            add_label!(child)
        end
    end

    # ── stats ──
    stats = (
        labels_generated=labels_generated,
        labels_rejected_by_dominance=labels_rejected_by_dominance,
        labels_removed_by_dominance=labels_removed_by_dominance,
        stale_pops=stale_pops,
        max_frontier_size=max_frontier_size,
        max_live_labels=max_live_labels,
        t_queue_sec=t_queue * 1e-9,
        t_candidates_sec=t_candidates * 1e-9,
        t_extension_sec=t_extension * 1e-9,
        t_dominance_sec=t_dominance * 1e-9,
    )
    return collect(values(best_by_signature)), exhausted, stats
end
