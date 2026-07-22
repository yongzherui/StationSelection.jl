"""
Orchestrates the `labels.jl` primitives into a full label-setting pricing
pass: a priority-queue search over live labels (`_enumerate_aggregate_od_route_pricing_labels`),
and the per-request driver that turns surviving labels into candidate columns
(`aggregate_od_route_pricing_by_label_setting`).
"""

export aggregate_od_route_pricing_by_label_setting

function _enumerate_aggregate_od_route_pricing_labels(
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals;
    time_limit::Float64,
    reduced_cost_tol::Float64,
    max_visits_per_node::Int,
    use_reduced_cost_pruning::Bool=true,
    profile::Bool=false,
    stop_if=label -> false,
)
    frontier = PriorityQueue{Int, Float64}()
    live_labels = Dict{Int, AggregateODRoutePricingLabel}()
    dominance_buckets = Dict{Int, SortedDict{AggregateODRouteLabelOrderKey, AggregateODRouteLabelId}}()
    best_by_signature = Dict{Any, AggregateODRoutePricingLabel}()

    n_pairs = length(pricing_data.active_pairs)
    pair_index = Dict(pair => i for (i, pair) in enumerate(pricing_data.active_pairs))
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))
    n_nodes = length(pricing_data.nodes)
    pair_origin_idx = [node_index[pair[1]] for pair in pricing_data.active_pairs]
    pair_dest_idx = [node_index[pair[2]] for pair in pricing_data.active_pairs]
    pair_ride_limit = [_direct_ride_limit(pricing_data, pair) for pair in pricing_data.active_pairs]
    travel_matrix = fill(Inf, n_nodes, n_nodes)
    for (i, u) in enumerate(pricing_data.nodes), (j, v) in enumerate(pricing_data.nodes)
        i == j && (travel_matrix[i, j] = 0.0; continue)
        haskey(pricing_data.travel_cost, (u, v)) &&
            (travel_matrix[i, j] = pricing_data.travel_cost[(u, v)])
    end
    label_bitsets = Dict{Int, AggregateODRouteLabelBitsets}()

    exhausted = true
    t_start = time()
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
    positive_pair_rewards = Float64[
        max(0.0, get(duals.sigma, pair, 0.0)) for pair in pricing_data.active_pairs
    ]

    function remaining_reward_bound(label::AggregateODRoutePricingLabel, label_bs::AggregateODRouteLabelBitsets)
        past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
        current_idx = node_index[label.current]
        ub = 0.0
        @inbounds for i in 1:n_pairs
            positive_pair_rewards[i] > 0 || continue
            i in label_bs.served_bits && continue
            origin_age = label_bs.station_age[pair_origin_idx[i]]
            can_claim_current = isfinite(origin_age) &&
                origin_age + travel_matrix[current_idx, pair_dest_idx[i]] <= pair_ride_limit[i] + 1e-9
            can_refresh = !past_pickup_cutoff &&
                label.time + travel_matrix[current_idx, pair_origin_idx[i]] <= pricing_data.max_wait_time + 1e-9
            can_claim_current || can_refresh || continue
            ub += positive_pair_rewards[i]
        end
        return ub
    end

    label_priority(label::AggregateODRoutePricingLabel, label_bs::AggregateODRouteLabelBitsets) =
        label.reduced_cost - remaining_reward_bound(label, label_bs)

    for label in initial_aggregate_od_route_pricing_labels(pricing_data, duals)
        label_id = next_label_id
        next_label_id += 1
        labels_generated += 1
        live_labels[label_id] = label
        label_bs = _make_aggregate_od_route_label_bitsets(label, pair_index, n_pairs, node_index, n_nodes)
        bucket = get!(() -> _create_aggregate_od_route_dominance_bucket(), dominance_buckets, _aggregate_od_route_dominance_signature(label))
        t0 = profile ? time_ns() : UInt64(0)
        inserted, removed = _add_aggregate_od_route_label_to_bucket!(
            bucket, live_labels, label_bitsets, label, label_id, label_bs,
            pricing_data.bounded_max_stops,
        )
        profile && (t_dominance += time_ns() - t0)
        labels_removed_by_dominance += removed
        if inserted
            t0 = profile ? time_ns() : UInt64(0)
            enqueue!(frontier, label_id => label_priority(label, label_bs))
            profile && (t_queue += time_ns() - t0)
            max_frontier_size = max(max_frontier_size, length(frontier))
            max_live_labels = max(max_live_labels, length(live_labels))
        else
            delete!(live_labels, label_id)
            labels_rejected_by_dominance += 1
        end
    end

    while !isempty(frontier)
        if time() - t_start > time_limit
            exhausted = false
            break
        end

        t0 = profile ? time_ns() : UInt64(0)
        label_id = dequeue!(frontier)
        profile && (t_queue += time_ns() - t0)
        if !haskey(live_labels, label_id)
            stale_pops += 1
            continue
        end
        label = live_labels[label_id]
        label_bs = label_bitsets[label_id]

        if !isempty(label.served_pairs)
            signature = _aggregate_od_route_column_signature(label.served_pairs)
            incumbent = get(best_by_signature, signature, nothing)
            if isnothing(incumbent) || label.tau < incumbent.tau - 1e-9
                best_by_signature[signature] = label
                if stop_if(label)
                    exhausted = false
                    break
                end
            end
        end

        label.route_length >= pricing_data.max_stops && continue
        if use_reduced_cost_pruning
            label_priority(label, label_bs) >= -reduced_cost_tol && continue
        end

        t0 = profile ? time_ns() : UInt64(0)
        next_nodes = _aggregate_od_route_candidate_next_nodes(
            label,
            pricing_data,
            duals;
            max_visits_per_node=max_visits_per_node,
        )
        profile && (t_candidates += time_ns() - t0)

        for next_node in next_nodes
            t0 = profile ? time_ns() : UInt64(0)
            children = extend_aggregate_od_route_pricing_label(label, next_node, pricing_data, duals)
            profile && (t_extension += time_ns() - t0)

            for child in children
                child_id = next_label_id
                next_label_id += 1
                labels_generated += 1
                live_labels[child_id] = child
                child_bs = _make_aggregate_od_route_label_bitsets(child, pair_index, n_pairs, node_index, n_nodes)
                bucket = get!(() -> _create_aggregate_od_route_dominance_bucket(), dominance_buckets, _aggregate_od_route_dominance_signature(child))
                t0 = profile ? time_ns() : UInt64(0)
                inserted, removed = _add_aggregate_od_route_label_to_bucket!(
                    bucket, live_labels, label_bitsets, child, child_id, child_bs,
                    pricing_data.bounded_max_stops,
                )
                profile && (t_dominance += time_ns() - t0)
                labels_removed_by_dominance += removed
                if inserted
                    t0 = profile ? time_ns() : UInt64(0)
                    enqueue!(frontier, child_id => label_priority(child, child_bs))
                    profile && (t_queue += time_ns() - t0)
                    max_frontier_size = max(max_frontier_size, length(frontier))
                    max_live_labels = max(max_live_labels, length(live_labels))
                else
                    delete!(live_labels, child_id)
                    labels_rejected_by_dominance += 1
                end
            end
        end
    end

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

function aggregate_od_route_pricing_by_label_setting(
    pricing_data::AggregateODRoutePricingData,
    existing_columns::Vector{AggregateODRouteColumn},
    duals::AggregateODRoutePricingDuals;
    next_column_id::Int,
    reduced_cost_tol::Float64=1e-6,
    max_new_columns::Int=1,
    n_candidates::Int=max_new_columns,
    time_limit::Float64=30.0,
    max_visits_per_node::Int=pricing_data.max_visits_per_node,
    profile::Bool=false,
)
    max_new_columns > 0 || throw(ArgumentError("max_new_columns must be positive"))
    n_candidates >= max_new_columns || throw(ArgumentError("n_candidates must be >= max_new_columns"))
    time_limit > 0 || throw(ArgumentError("time_limit must be positive"))

    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        signature = _aggregate_od_route_column_signature(column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    scored_by_signature = Dict{Any, Tuple{Float64, AggregateODRoutePricingLabel}}()

    function accept_pricing_label!(label::AggregateODRoutePricingLabel)
        isempty(label.served_pairs) && return false
        label.reduced_cost < -reduced_cost_tol || return false
        signature = _aggregate_od_route_column_signature(label.served_pairs)
        # This assumes `duals` came from the optimal RMP over `existing_columns`.
        # Under that condition, an existing column cannot have negative reduced
        # cost, so subset-served dominance cannot hide a missing improving column
        # behind a non-improving duplicate signature.
        label.tau < get(best_pool_tau, signature, Inf) - 1e-9 || return false
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                label.reduced_cost < current[1] - 1e-9 ||
                (abs(label.reduced_cost - current[1]) <= 1e-9 && label.tau < current[2].tau - 1e-9)
            scored_by_signature[signature] = (label.reduced_cost, label)
        end
        return length(scored_by_signature) >= n_candidates
    end

    labels, exhausted, stats = _enumerate_aggregate_od_route_pricing_labels(
        pricing_data,
        duals;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        max_visits_per_node=max_visits_per_node,
        profile=profile,
        stop_if=accept_pricing_label!,
    )

    for label in labels
        isempty(label.served_pairs) && continue
        label.reduced_cost < -reduced_cost_tol || continue
        signature = _aggregate_od_route_column_signature(label.served_pairs)
        label.tau < get(best_pool_tau, signature, Inf) - 1e-9 || continue
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                label.reduced_cost < current[1] - 1e-9 ||
                (abs(label.reduced_cost - current[1]) <= 1e-9 && label.tau < current[2].tau - 1e-9)
            scored_by_signature[signature] = (label.reduced_cost, label)
        end
    end

    scored = collect(values(scored_by_signature))
    sort!(scored, by=entry -> (entry[1], entry[2].tau, string(entry[2].route)))
    scored = scored[1:min(length(scored), n_candidates)]
    scored = scored[1:min(length(scored), max_new_columns)]

    columns = AggregateODRouteColumn[]
    column_id = next_column_id
    for (_, label) in scored
        push!(columns, _aggregate_od_route_column_from_label(label, column_id, pricing_data.scenario))
        column_id += 1
    end
    return columns, exhausted, stats
end
