"""
The label-setting search loop itself. See `types.jl` for the per-state label
list and the `AbstractPricingSearchContext` hook contract this loop is
written against, and `utils.jl` for the reward-model-independent math those
hooks lean on.
"""

"""
    _run_label_setting(ctx; time_limit, reduced_cost_tol, use_reduced_cost_pruning=true, profile=false, stop_if=label->false)

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
function _run_label_setting(
    ctx::AbstractPricingSearchContext{Filters, Label, Bitsets, State, BestSig};
    time_limit::Float64,
    reduced_cost_tol::Float64,
    use_reduced_cost_pruning::Bool=true,
    profile::Bool=false,
    stop_if=label -> false,
) where {Filters, Label, Bitsets, State, BestSig}
    # ── setup: frontier, live-label table, per-state buckets, stats counters ──
    frontier = PriorityQueue{Int, Float64}()  # label_id => priority; pop = most promising live label next
    live_labels = Union{Nothing, Label}[]  # index = label_id; entry set to `nothing` once dominated/stale (see stale_pops below)
    n_live_labels = 0  # count of non-`nothing` entries in live_labels right now (for stats only)
    labels_by_state = Dict{State, PricingStateLabels{Filters, Label, Bitsets}}()  # one sorted dominance list per search state
    best_by_signature = Dict{BestSig, Label}()  # best-so-far finished label per `_pricing_best_signature`; this dict *is* the search's answer
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
    #
    # What it does, in order: (1) give `label` an id and register it as live;
    # (2) look up (or create) the dominance list for the state `label`
    # occupies; (3) ask `_add_pricing_label_to_state!` to insert it into that
    # list, which also evicts any existing entries `label` newly dominates;
    # (4) if `label` itself got dominated by something already there, discard
    # it and stop -- it will never be popped; (5) otherwise push it onto the
    # frontier so the main loop can visit it later.
    function add_label!(label::Label)
        # Register: this id is `label`'s permanent index into `live_labels`.
        # `live_labels[label_id]` stays this label until (if ever) dominance
        # evicts it, at which point it's overwritten with `nothing` --
        # that's how a stale frontier entry for an evicted label is detected
        # in the main loop below, without having to touch the frontier itself.
        label_id = next_label_id
        next_label_id += 1
        labels_generated += 1
        push!(live_labels, label)
        n_live_labels += 1

        # Find this label's dominance list: labels only ever compete against
        # others in the same search state (see `PricingLabelEntry`'s
        # docstring in types.jl), so each state gets its own sorted list.
        label_bs = _pricing_make_bitsets(ctx, label)
        state = _pricing_state(ctx, label, label_bs)
        state_labels = get!(() -> PricingStateLabels{Filters, Label, Bitsets}(), labels_by_state, state)

        # Try to insert `label` into that list. This single call both checks
        # whether an existing entry already dominates `label` (in which case
        # `inserted` comes back false) and evicts any existing entries that
        # `label` itself dominates (`removed`, folded into the live-label
        # count and stats below either way -- eviction happens regardless of
        # whether `label` survives).
        t0 = profile ? time_ns() : UInt64(0)
        inserted, removed = _add_pricing_label_to_state!(
            state_labels, live_labels, label, label_id, label_bs, dominates, dominated_scratch,
        )
        profile && (t_dominance += time_ns() - t0)
        labels_removed_by_dominance += removed
        n_live_labels -= removed

        # `label` lost to an existing entry: mark it stale immediately (never
        # goes on the frontier, so the main loop will never see it) and bail.
        if !inserted
            live_labels[label_id] = nothing
            n_live_labels -= 1
            labels_rejected_by_dominance += 1
            return nothing
        end

        # `label` survived: it becomes a live frontier entry, ordered by its
        # priority (a lower bound on remaining reward -- see
        # `_pricing_label_priority`'s docstring in types.jl). The optional
        # diagnostic hook is the only place a concrete context can observe
        # every label that actually enters the search (see its docstring).
        t0 = profile ? time_ns() : UInt64(0)
        push!(frontier, label_id => _pricing_label_priority(ctx, label, label_bs))
        profile && (t_queue += time_ns() - t0)
        max_frontier_size = max(max_frontier_size, length(frontier))
        max_live_labels = max(max_live_labels, n_live_labels)
        _pricing_on_label_inserted(ctx, label)
        return nothing
    end

    # ── seed the frontier from depth-1 labels ──
    # Every search starts here: each of `_pricing_initial_labels`'s labels
    # goes through the exact same insert/dominance/frontier-push path as a
    # label produced by extension below -- there is no separate "first step"
    # logic, just an initial supply of labels to run through `add_label!`.
    for label in _pricing_initial_labels(ctx)
        add_label!(label)
    end

    # ── main loop: pop cheapest, record incumbent (offer to stop_if), prune, extend ──
    while !isempty(frontier)
        # Wall-clock budget check: give up on whatever's left in the
        # frontier and return what's been found so far. `exhausted = false`
        # tells the caller the search space wasn't fully explored (as
        # opposed to running dry, i.e. `isempty(frontier)`).
        if time() - t_start > time_limit
            exhausted = false
            break
        end

        # Pop the single most promising live label. `popped_priority` is that
        # label's priority *at the time it was pushed* -- still valid here
        # because priorities never improve as a label collects more
        # dominance/reward-bound state after being pushed.
        t0 = profile ? time_ns() : UInt64(0)
        label_id, popped_priority = popfirst!(frontier)
        profile && (t_queue += time_ns() - t0)

        # This id may belong to a label that add_label! later evicted via
        # dominance (see the `live_labels[label_id] = nothing` writes above);
        # the frontier itself is never edited when that happens, so a stale
        # entry can still surface here. Skip it and move on.
        maybe_label = live_labels[label_id]
        if isnothing(maybe_label)
            stale_pops += 1
            continue
        end
        label = maybe_label::Label

        # Record `label` as a candidate answer if it's "finished" in the
        # sense `_pricing_best_signature` cares about (returns `nothing`
        # otherwise, e.g. a label that hasn't certified any reward yet).
        # `best_by_signature` is competed for by signature -- lower `tau`
        # (this pricer's own notion of cost/time) wins -- and this dict is
        # exactly what gets returned as the search's result at the bottom of
        # this function. `stop_if` (built by the caller, e.g. round.jl's
        # accept/dedupe closure) gets first look at every new incumbent and
        # can end the whole search early, e.g. once enough candidates for
        # this pricing round have been found.
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

        # Two reasons `label` might not be worth extending further, checked
        # *after* it's already been offered as a candidate above (a
        # maxed-out or reduced-cost-exhausted label can still be the best
        # answer for its signature, it just can't produce useful children):
        # it's already at the stops cap, or its priority -- a lower bound on
        # everything it could still earn -- no longer beats the tolerance,
        # meaning no extension of it could ever be profitable.
        _pricing_route_length(ctx, label) >= _pricing_max_route_length(ctx) && continue
        if use_reduced_cost_pruning
            popped_priority >= -reduced_cost_tol && continue
        end

        # Extend `label` along every node it's legally allowed to visit next,
        # running each resulting child straight through add_label! (insert,
        # dominance-check, frontier-push) exactly as the seed labels above
        # did.
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
