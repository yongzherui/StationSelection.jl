"""
Column-generation helpers for AggregateODRouteModel.
"""

export AggregateODRouteCoverageDuals
export AggregateODRouteColumnGenerationResult
export AggregateODRouteCGLogger
export AggregateODRouteCGIterationLog
export AggregateODRouteCGTerminationLog
export AggregateODRoutePricingData
export AggregateODRoutePricingDuals
export AggregateODRoutePricingLabel
export extract_aggregate_od_route_coverage_duals
export aggregate_od_route_coverage_sigma
export create_aggregate_od_route_pricing_data
export initial_aggregate_od_route_pricing_labels
export extend_aggregate_od_route_pricing_label
export aggregate_od_route_pricing_by_label_setting
export generate_aggregate_od_route_columns
export run_aggregate_od_route_column_generation

struct AggregateODRouteCoverageDuals
    raw_duals::Dict{Any, Float64}
    sigma::Dict{NTuple{3, Int}, Float64}
end

struct AggregateODRoutePricingData
    scenario::Int
    nodes::Vector{Int}
    travel_cost::Dict{Tuple{Int, Int}, Float64}
    active_pairs::Vector{Tuple{Int, Int}}
    route_regularization_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int
    max_visits_per_node::Int
    bounded_max_stops::Bool
end

struct AggregateODRoutePricingDuals
    sigma::Dict{Tuple{Int, Int}, Float64}
end

struct AggregateODRoutePricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    served_pairs::Set{Tuple{Int, Int}}
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

struct AggregateODRouteColumnGenerationResult
    status::Symbol
    final_result::OptResult
    lp_bound::Float64
    n_cg_iters::Int
    cg_stop_reason::Symbol
    generated_columns::Vector{AggregateODRouteColumn}
    selected_column_ids::Vector{Int}
    coverage::Dict{NTuple{3, Int}, Int}
    iteration_rows::Vector{NamedTuple}
    column_log_rows::Vector{NamedTuple}
    dual_log_rows::Vector{NamedTuple}
end

mutable struct AggregateODRouteCGLogger
    verbose::Bool
    cg_log_path::Union{Nothing, String}
    iteration_rows::Vector{NamedTuple}
end

struct AggregateODRouteCGIterationLog
    iteration::Int
    columns_before::Int
    columns_after::Int
    lp_status::Symbol
    lp_objective::Union{Nothing, Float64}
    lp_solve_seconds::Float64
    pricing_seconds::Union{Nothing, Float64}
    iteration_seconds::Float64
    new_columns_returned::Int
    columns_added::Int
    columns_replaced::Int
    best_reduced_cost::Union{Nothing, Float64}
    pricing_exhausted::Bool
    stop_reason::Symbol
    dual_min::Union{Nothing, Float64}
    dual_max::Union{Nothing, Float64}
    dual_mean::Union{Nothing, Float64}
    dual_std::Union{Nothing, Float64}
    labels_generated::Union{Nothing, Int}
    labels_rejected_by_dominance::Union{Nothing, Int}
    labels_removed_by_dominance::Union{Nothing, Int}
    stale_pops::Union{Nothing, Int}
    max_frontier_size::Union{Nothing, Int}
    max_live_labels::Union{Nothing, Int}
    t_queue_sec::Union{Nothing, Float64}
    t_candidates_sec::Union{Nothing, Float64}
    t_extension_sec::Union{Nothing, Float64}
    t_dominance_sec::Union{Nothing, Float64}
end

struct AggregateODRouteCGTerminationLog
    reason::Symbol
    iteration::Int
    final_pool_size::Int
end

const AggregateODRouteLabelId = Int
const AggregateODRouteLabelOrderKey = Tuple{Float64, Float64, Int, Int}

struct AggregateODRouteLabelBitsets
    served_bits::BitSet
    station_age::Vector{Float64}
end

"""
    aggregate_od_route_coverage_sigma(raw_dual) -> Float64

Coverage rows are stored as:

    sum(theta[c,s] for c covering (j,k)) - u[j,k,s] >= 0

For a minimization RMP, a new column with coefficient +1 in this row has reduced
cost `mu * (tau + rho) - raw_dual`. Pricing therefore uses
`profit(c,s) = sum(sigma[j,k,s]) - mu * (tau[c] + rho)` with `sigma = raw_dual`.
"""
aggregate_od_route_coverage_sigma(raw_dual::Real)::Float64 = Float64(raw_dual)

function extract_aggregate_od_route_coverage_duals(m::Model)::AggregateODRouteCoverageDuals
    coverage = m[:aggregate_od_route_coverage_constraints]
    raw = Dict{Any, Float64}()
    sigma = Dict{NTuple{3, Int}, Float64}()
    for (key, con) in coverage
        raw_dual = dual(con)
        raw[key] = raw_dual
        pair_s = (Int(key[1]), Int(key[2]), Int(key[3]))
        sigma[pair_s] = get(sigma, pair_s, 0.0) + aggregate_od_route_coverage_sigma(raw_dual)
    end
    return AggregateODRouteCoverageDuals(raw, sigma)
end

extract_aggregate_od_route_coverage_duals(build_result::BuildResult)::AggregateODRouteCoverageDuals =
    extract_aggregate_od_route_coverage_duals(build_result.model)

function _aggregate_od_route_travel(pricing_data::AggregateODRoutePricingData, u::Int, v::Int)::Float64
    cost = get(pricing_data.travel_cost, (u, v), Inf)
    isfinite(cost) || throw(ArgumentError("missing finite routing cost for station arc $((u, v))"))
    return cost
end

function _direct_ride_limit(
    pricing_data::AggregateODRoutePricingData,
    pair::Tuple{Int, Int},
)::Float64
    return pricing_data.detour_factor * _aggregate_od_route_travel(pricing_data, pair[1], pair[2])
end

function _aggregate_od_route_label_reduced_cost(
    tau::Float64,
    served_pairs,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Float64
    dual_credit = sum(get(duals.sigma, pair, 0.0) for pair in served_pairs; init=0.0)
    return pricing_data.route_regularization_weight * (tau + pricing_data.repositioning_time) - dual_credit
end

function create_aggregate_od_route_pricing_data(
    model::AnyAggregateODRouteModel,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    scenario::Int,
    pricing_duals::Union{AggregateODRoutePricingDuals, Nothing}=nothing,
)::AggregateODRoutePricingData
    1 <= scenario <= n_scenarios(data) ||
        throw(ArgumentError("scenario index $scenario is out of range"))
    has_routing_costs(data) ||
        throw(ArgumentError("AggregateODRouteModel pricing requires routing_costs"))

    nodes = collect(1:data.n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    missing_arcs = Tuple{Int, Int}[]
    for i in nodes, j in nodes
        i == j && continue
        cost = get_routing_cost(data, i, j)
        if isfinite(cost)
            travel_cost[(i, j)] = cost
        else
            push!(missing_arcs, (i, j))
        end
    end
    isempty(missing_arcs) ||
        throw(ArgumentError("missing finite routing costs for aggregate OD route pricing arcs: $(missing_arcs)"))

    all_pairs = filter(!is_walk_only_pair, get(mapping.active_jk_s, scenario, Tuple{Int, Int}[]))
    active_pairs = if isnothing(pricing_duals)
        all_pairs
    else
        filter(pair -> get(pricing_duals.sigma, pair, 0.0) > 1e-9, all_pairs)
    end

    bounded_max_stops = model.max_stops != typemax(Int)
    max_stops = model.max_stops == typemax(Int) ? length(nodes) : model.max_stops
    return AggregateODRoutePricingData(
        scenario,
        nodes,
        travel_cost,
        active_pairs,
        model.route_regularization_weight,
        model.repositioning_time,
        model.max_wait_time,
        model.detour_factor,
        max_stops,
        model.max_visits_per_node,
        bounded_max_stops,
    )
end

function _scenario_pricing_duals(
    duals::AggregateODRouteCoverageDuals,
    scenario::Int,
)::AggregateODRoutePricingDuals
    sigma = Dict{Tuple{Int, Int}, Float64}()
    for ((j, k, s), value) in duals.sigma
        s == scenario || continue
        sigma[(j, k)] = value
    end
    return AggregateODRoutePricingDuals(sigma)
end

function _certify_aggregate_od_route_pairs_at_node(
    node::Int,
    station_age::Dict{Int, Float64},
    travel_time::Float64,
    served_pairs::Set{Tuple{Int, Int}},
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)
    certified_pairs = copy(served_pairs)
    reward = 0.0
    for pair in pricing_data.active_pairs
        pair[2] == node || continue
        pair ∈ certified_pairs && continue
        pair_reward = get(duals.sigma, pair, 0.0)
        pair_reward > 1e-9 || continue
        origin_age = get(station_age, pair[1], Inf)
        origin_age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        push!(certified_pairs, pair)
        reward += pair_reward
    end
    return certified_pairs, reward
end

function _prune_irrelevant_aggregate_od_route_station_ages(
    station_age::Dict{Int, Float64},
    served_pairs::Set{Tuple{Int, Int}},
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
    current::Int,
)
    remaining = Dict{Int, Float64}()
    for (station, age) in station_age
        useful = false
        for pair in pricing_data.active_pairs
            pair[1] == station || continue
            pair ∈ served_pairs && continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            t_to_dest = pair[2] == current ? 0.0 : _aggregate_od_route_travel(pricing_data, current, pair[2])
            age + t_to_dest <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            useful = true
            break
        end
        useful && (remaining[station] = age)
    end
    return remaining
end

function initial_aggregate_od_route_pricing_labels(
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Vector{AggregateODRoutePricingLabel}
    endpoints = Set{Int}()
    for (j, k) in pricing_data.active_pairs
        push!(endpoints, j)
        push!(endpoints, k)
    end

    labels = AggregateODRoutePricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, AggregateODRoutePricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            Set{Tuple{Int, Int}}(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
            1,
        ))
    end
    return labels
end

function _has_useful_live_aggregate_od_route_origin(
    label::AggregateODRoutePricingLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Bool
    for (station, age) in label.station_age
        for pair in pricing_data.active_pairs
            pair[1] == station || continue
            pair ∈ label.served_pairs && continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            t_to_dest = pair[2] == label.current ? 0.0 : _aggregate_od_route_travel(pricing_data, label.current, pair[2])
            age + t_to_dest <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            return true
        end
    end
    return false
end

function _aggregate_od_route_candidate_next_nodes(
    label::AggregateODRoutePricingLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals;
    max_visits_per_node::Int=pricing_data.max_visits_per_node,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_aggregate_od_route_origin(label, pricing_data, duals)
        return Int[]
    end
    for pair in pricing_data.active_pairs
        get(duals.sigma, pair, 0.0) > 1e-9 || continue
        origin, destination = pair
        remembered = pair ∈ label.served_pairs

        if !remembered && origin != label.current
            arrival_time = label.time + _aggregate_od_route_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end

        remembered && continue
        destination == label.current && continue
        origin_age = get(label.station_age, origin, Inf)
        origin_age + _aggregate_od_route_travel(pricing_data, label.current, destination) <=
            _direct_ride_limit(pricing_data, pair) + 1e-9 && push!(candidate_nodes, destination)
    end

    if max_visits_per_node < typemax(Int)
        visit_counts = Dict{Int, Int}()
        for node in label.route
            visit_counts[node] = get(visit_counts, node, 0) + 1
        end
        filter!(node -> get(visit_counts, node, 0) < max_visits_per_node, candidate_nodes)
    end

    return sort!(collect(candidate_nodes))
end

function extend_aggregate_od_route_pricing_label(
    label::AggregateODRoutePricingLabel,
    next_node::Int,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Vector{AggregateODRoutePricingLabel}
    travel_time = _aggregate_od_route_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)
    aged_station = Dict(station => age + travel_time for (station, age) in label.station_age)

    certified_pairs, reward =
        _certify_aggregate_od_route_pairs_at_node(
            next_node,
            label.station_age,
            travel_time,
            label.served_pairs,
            pricing_data,
            duals,
        )
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        aged_station[next_node] = 0.0
    end
    aged_station = _prune_irrelevant_aggregate_od_route_station_ages(
        aged_station,
        certified_pairs,
        pricing_data,
        duals,
        next_node,
    )

    child = AggregateODRoutePricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified_pairs,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )

    return AggregateODRoutePricingLabel[child]
end

_aggregate_od_route_dominance_signature(label::AggregateODRoutePricingLabel) = label.current

function _aggregate_od_route_label_order_key(
    label::AggregateODRoutePricingLabel,
    label_id::AggregateODRouteLabelId,
)::AggregateODRouteLabelOrderKey
    return (
        label.reduced_cost,
        label.time,
        label.route_length,
        label_id,
    )
end

_create_aggregate_od_route_dominance_bucket() = SortedDict{AggregateODRouteLabelOrderKey, AggregateODRouteLabelId}()

function _make_aggregate_od_route_label_bitsets(
    label::AggregateODRoutePricingLabel,
    pair_index::Dict{Tuple{Int, Int}, Int},
    n_pairs::Int,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::AggregateODRouteLabelBitsets
    served_bits = BitSet()
    for pair in label.served_pairs
        push!(served_bits, pair_index[pair])
    end

    station_age = fill(Inf, n_nodes)
    for (station, age) in label.station_age
        station_age[node_index[station]] = age
    end

    return AggregateODRouteLabelBitsets(served_bits, station_age)
end

function _dominates_aggregate_od_route_label(
    a::AggregateODRoutePricingLabel,
    b::AggregateODRoutePricingLabel,
    bounded_max_stops::Bool,
)::Bool
    _aggregate_od_route_dominance_signature(a) == _aggregate_od_route_dominance_signature(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    issubset(a.served_pairs, b.served_pairs) || return false
    all_stations = union(keys(a.station_age), keys(b.station_age), (a.current, b.current))
    for station in all_stations
        get(a.station_age, station, Inf) <= get(b.station_age, station, Inf) + 1e-9 || return false
    end
    return true
end

function _dominates_aggregate_od_route_label(
    a::AggregateODRoutePricingLabel,
    b::AggregateODRoutePricingLabel,
    abs::AggregateODRouteLabelBitsets,
    bbs::AggregateODRouteLabelBitsets,
    bounded_max_stops::Bool,
)::Bool
    _aggregate_od_route_dominance_signature(a) == _aggregate_od_route_dominance_signature(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    issubset(abs.served_bits, bbs.served_bits) || return false
    @inbounds for i in eachindex(abs.station_age)
        abs.station_age[i] <= bbs.station_age[i] + 1e-9 || return false
    end
    return true
end

function _add_aggregate_od_route_label_to_bucket!(
    bucket::SortedDict{AggregateODRouteLabelOrderKey, AggregateODRouteLabelId},
    live_labels::Dict{Int, AggregateODRoutePricingLabel},
    label_bitsets::Dict{Int, AggregateODRouteLabelBitsets},
    label::AggregateODRoutePricingLabel,
    label_id::Int,
    label_bs::AggregateODRouteLabelBitsets,
    bounded_max_stops::Bool,
)
    inserted = true
    dominated_ids = Int[]
    switched = false

    for (_existing_key, existing_id) in pairs(bucket)
        existing_label = live_labels[existing_id]
        existing_bs = label_bitsets[existing_id]

        if !switched && label.reduced_cost > existing_label.reduced_cost + 1e-9
            if _dominates_aggregate_od_route_label(existing_label, label, existing_bs, label_bs, bounded_max_stops)
                inserted = false
                break
            end
            continue
        end

        switched = true
        if _dominates_aggregate_od_route_label(label, existing_label, label_bs, existing_bs, bounded_max_stops)
            push!(dominated_ids, existing_id)
        end
    end

    if inserted
        for id in dominated_ids
            delete!(bucket, _aggregate_od_route_label_order_key(live_labels[id], id))
            delete!(live_labels, id)
            delete!(label_bitsets, id)
        end
        bucket[_aggregate_od_route_label_order_key(label, label_id)] = label_id
        label_bitsets[label_id] = label_bs
    end
    return inserted, length(dominated_ids)
end

function _aggregate_od_route_label_priority(
    label::AggregateODRoutePricingLabel,
    duals::AggregateODRoutePricingDuals,
)::Float64
    return label.reduced_cost
end

function _aggregate_od_route_column_signature(pairs)::Tuple{Vararg{Tuple{Int, Int}}}
    return Tuple(sort!(collect(pairs)))
end

_aggregate_od_route_column_signature(column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature(column.od_pairs)

function _aggregate_od_route_column_from_label(
    label::AggregateODRoutePricingLabel,
    column_id::Int,
    scenario::Int,
)::AggregateODRouteColumn
    return AggregateODRouteColumn(
        column_id,
        collect(label.served_pairs),
        label.tau;
        metadata=Dict{String, Any}(
            "scenario" => scenario,
            "route" => Tuple(label.route),
            "reduced_cost" => label.reduced_cost,
        ),
    )
end

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

function _write_aggregate_od_route_cg_log_csv(path::AbstractString, rows; headers::Union{Nothing, Vector{Symbol}}=nothing)
    dir = dirname(path)
    isempty(dir) || mkpath(dir)
    if isempty(rows)
        isnothing(headers) && return path
        open(path, "w") do io
            println(io, join(string.(headers), ","))
        end
        return path
    end
    headers = isnothing(headers) ? collect(keys(first(rows))) : headers
    open(path, "w") do io
        println(io, join(string.(headers), ","))
        for row in rows
            values = [
                begin
                    value = getproperty(row, header)
                    if value === nothing
                        ""
                    elseif value isa AbstractString
                        "\"" * replace(value, "\"" => "\"\"") * "\""
                    elseif value isa Symbol
                        string(value)
                    elseif value isa Bool
                        value ? "true" : "false"
                    else
                        string(value)
                    end
                end
                for header in headers
            ]
            println(io, join(values, ","))
        end
    end
    return path
end

_create_aggregate_od_route_cg_logger(; verbose::Bool, cg_log_path::Union{Nothing, AbstractString}) =
    AggregateODRouteCGLogger(verbose, isnothing(cg_log_path) ? nothing : String(cg_log_path), NamedTuple[])

function _aggregate_od_route_log_header!(
    logger::AggregateODRouteCGLogger,
    n_active_pairs::Int,
    initial_pool_size::Int,
    max_cg_iters::Int,
    pricing_time_limit_sec::Float64,
    max_new_columns::Int,
)
    logger.verbose || return nothing
    println("=" ^ 60)
    println("AggregateODRouteModel — Column Generation")
    println("=" ^ 60)
    @printf("  Active station OD pairs : %d\n", n_active_pairs)
    @printf("  Initial RMP cols        : %d\n", initial_pool_size)
    @printf("  Max CG iterations       : %d\n", max_cg_iters)
    @printf("  Pricing time limit      : %.2f sec\n", pricing_time_limit_sec)
    @printf("  Max new columns/iter    : %d\n", max_new_columns)
    println("=" ^ 60)
    return nothing
end

function _to_named_tuple(log::AggregateODRouteCGIterationLog)
    return (
        iteration=log.iteration,
        columns_before=log.columns_before,
        columns_after=log.columns_after,
        lp_status=string(log.lp_status),
        lp_objective=log.lp_objective,
        lp_solve_seconds=log.lp_solve_seconds,
        pricing_seconds=log.pricing_seconds,
        iteration_seconds=log.iteration_seconds,
        new_columns_returned=log.new_columns_returned,
        columns_added=log.columns_added,
        columns_replaced=log.columns_replaced,
        best_reduced_cost=log.best_reduced_cost,
        pricing_exhausted=log.pricing_exhausted,
        stop_reason=string(log.stop_reason),
        dual_min=log.dual_min,
        dual_max=log.dual_max,
        dual_mean=log.dual_mean,
        dual_std=log.dual_std,
        labels_generated=log.labels_generated,
        labels_rejected_by_dominance=log.labels_rejected_by_dominance,
        labels_removed_by_dominance=log.labels_removed_by_dominance,
        stale_pops=log.stale_pops,
        max_frontier_size=log.max_frontier_size,
        max_live_labels=log.max_live_labels,
        t_queue_sec=log.t_queue_sec,
        t_candidates_sec=log.t_candidates_sec,
        t_extension_sec=log.t_extension_sec,
        t_dominance_sec=log.t_dominance_sec,
    )
end

function _record_aggregate_od_route_cg_iteration!(
    logger::AggregateODRouteCGLogger,
    log::AggregateODRouteCGIterationLog,
)
    push!(logger.iteration_rows, _to_named_tuple(log))
    logger.verbose || return nothing
    println()
    @printf("CG iteration %d\n", log.iteration)
    @printf("  RMP cols before pricing : %d\n", log.columns_before)
    @printf("  LP status               : %s\n", log.lp_status)
    isnothing(log.lp_objective) ? println("  LP objective            : unavailable") :
        @printf("  LP objective            : %.6f\n", log.lp_objective)
    @printf("  LP runtime              : %.3f sec\n", log.lp_solve_seconds)
    isnothing(log.pricing_seconds) ? println("  Pricing runtime         : unavailable") :
        @printf("  Pricing runtime         : %.3f sec\n", log.pricing_seconds)
    @printf("  Iteration runtime       : %.3f sec\n", log.iteration_seconds)
    @printf("  New columns returned    : %d\n", log.new_columns_returned)
    @printf("  Columns added           : %d\n", log.columns_added)
    @printf("  Columns replaced        : %d\n", log.columns_replaced)
    isnothing(log.best_reduced_cost) ? println("  Best reduced cost       : n/a") :
        @printf("  Best reduced cost       : %.6f\n", log.best_reduced_cost)
    @printf("  Pricing exhausted       : %s\n", log.pricing_exhausted)
    @printf("  RMP cols after pricing  : %d\n", log.columns_after)
    if !isnothing(log.dual_min)
        @printf("  Duals [min/max/mean/std]: %.4f / %.4f / %.4f / %.4f\n",
            log.dual_min, log.dual_max, log.dual_mean, log.dual_std)
    end
    if !isnothing(log.labels_generated)
        @printf("  Labels generated        : %d  (rejected=%d  removed=%d  stale=%d)\n",
            log.labels_generated, log.labels_rejected_by_dominance,
            log.labels_removed_by_dominance, log.stale_pops)
        @printf("  Max frontier / live     : %d / %d\n",
            log.max_frontier_size, log.max_live_labels)
    end
    if !isnothing(log.t_queue_sec)
        total_accounted = log.t_queue_sec + log.t_candidates_sec + log.t_extension_sec + log.t_dominance_sec
        @printf("  Phase timing (s)        : queue=%.2f  candidates=%.2f  extension=%.2f  dominance=%.2f  (total=%.2f)\n",
            log.t_queue_sec, log.t_candidates_sec, log.t_extension_sec, log.t_dominance_sec, total_accounted)
    end
    return nothing
end

function _record_aggregate_od_route_cg_termination!(
    logger::AggregateODRouteCGLogger,
    log::AggregateODRouteCGTerminationLog,
)
    logger.verbose || return nothing
    println()
    println("=" ^ 60)
    println("Aggregate OD Route Column Generation Terminated")
    println("=" ^ 60)
    @printf("  Iterations completed : %d\n", log.iteration)
    @printf("  Final RMP cols       : %d\n", log.final_pool_size)
    @printf("  Reason               : %s\n", log.reason)
    println("=" ^ 60)
    return nothing
end

function _flush_aggregate_od_route_cg_log!(logger::AggregateODRouteCGLogger)
    isnothing(logger.cg_log_path) && return nothing
    _write_aggregate_od_route_cg_log_csv(logger.cg_log_path, logger.iteration_rows)
    return nothing
end

function _aggregate_od_route_dual_stats(duals::AggregateODRouteCoverageDuals)
    vals = collect(values(duals.sigma))
    isempty(vals) && return (nothing, nothing, nothing, nothing)
    mean = sum(vals) / length(vals)
    std = length(vals) > 1 ? sqrt(sum((v - mean)^2 for v in vals) / (length(vals) - 1)) : 0.0
    return (minimum(vals), maximum(vals), mean, std)
end

function _merge_pricing_stats(stats)
    isempty(stats) && return (
        labels_generated=0,
        labels_rejected_by_dominance=0,
        labels_removed_by_dominance=0,
        stale_pops=0,
        max_frontier_size=0,
        max_live_labels=0,
        t_queue_sec=0.0,
        t_candidates_sec=0.0,
        t_extension_sec=0.0,
        t_dominance_sec=0.0,
    )
    return (
        labels_generated=sum(s.labels_generated for s in stats; init=0),
        labels_rejected_by_dominance=sum(s.labels_rejected_by_dominance for s in stats; init=0),
        labels_removed_by_dominance=sum(s.labels_removed_by_dominance for s in stats; init=0),
        stale_pops=sum(s.stale_pops for s in stats; init=0),
        max_frontier_size=maximum(s.max_frontier_size for s in stats; init=0),
        max_live_labels=maximum(s.max_live_labels for s in stats; init=0),
        t_queue_sec=sum(s.t_queue_sec for s in stats; init=0.0),
        t_candidates_sec=sum(s.t_candidates_sec for s in stats; init=0.0),
        t_extension_sec=sum(s.t_extension_sec for s in stats; init=0.0),
        t_dominance_sec=sum(s.t_dominance_sec for s in stats; init=0.0),
    )
end

function _aggregate_od_route_coverage_summary(result::OptResult)::Dict{NTuple{3, Int}, Int}
    result.termination_status == MOI.OPTIMAL || return Dict{NTuple{3, Int}, Int}()
    mapping = result.mapping
    theta = result.model[:theta_compat]
    coverage = Dict{NTuple{3, Int}, Int}()
    for s in 1:length(mapping.scenarios)
        for (j, k) in get(mapping.active_jk_s, s, Tuple{Int, Int}[])
            count = 0
            for column_id in get(mapping.columns_by_pair, (j, k), Int[])
                theta_var = get(theta, (column_id, s), nothing)
                theta_var === nothing && continue
                value(theta_var) > 0.5 && (count += 1)
            end
            coverage[(j, k, s)] = count
        end
    end
    return coverage
end

function _selected_aggregate_od_route_column_ids(result::OptResult)::Vector{Int}
    result.termination_status == MOI.OPTIMAL || return Int[]
    theta = result.model[:theta_compat]
    ids = Set{Int}()
    for ((column_id, _s), theta_var) in theta
        value(theta_var) > 0.5 && push!(ids, column_id)
    end
    return sort!(collect(ids))
end

function generate_aggregate_od_route_columns(
    master_state::BuildResult,
    duals::AggregateODRouteCoverageDuals,
    data::StationSelectionData,
)
    m = master_state.model
    mapping = master_state.mapping
    model = AggregateODRouteModel(
        m[:aggregate_od_route_station_budget];
        route_regularization_weight=Float64(m[:aggregate_od_route_route_regularization_weight]),
        repositioning_time=Float64(m[:aggregate_od_route_repositioning_time]),
        max_walking_distance=mapping.max_walking_distance,
        max_wait_time=Float64(m[:aggregate_od_route_max_wait_time]),
        detour_factor=Float64(m[:aggregate_od_route_detour_factor]),
        max_stops=Int(m[:aggregate_od_route_max_stops]),
        max_visits_per_node=Int(m[:aggregate_od_route_max_visits_per_node]),
        max_new_columns=Int(m[:aggregate_od_route_max_new_columns]),
        n_candidates=Int(m[:aggregate_od_route_n_candidates]),
        pricing_time_limit_sec=Float64(m[:aggregate_od_route_pricing_time_limit_sec]),
        reduced_cost_tol=Float64(m[:aggregate_od_route_reduced_cost_tol]),
        relax_integrality=Bool(m[:aggregate_od_route_relax_integrality]),
    )

    next_column_id = isempty(mapping.column_ids) ? 1 : maximum(mapping.column_ids) + 1
    all_columns = AggregateODRouteColumn[]
    for s in 1:n_scenarios(data)
        pricing_duals = _scenario_pricing_duals(duals, s)
        pricing_data = create_aggregate_od_route_pricing_data(model, data, mapping, s, pricing_duals)
        new_columns, _exhausted, _stats = aggregate_od_route_pricing_by_label_setting(
            pricing_data,
            mapping.columns,
            pricing_duals;
            next_column_id=next_column_id,
            reduced_cost_tol=model.reduced_cost_tol,
            max_new_columns=model.max_new_columns,
            n_candidates=model.n_candidates,
            time_limit=model.pricing_time_limit_sec,
        )
        append!(all_columns, new_columns)
        next_column_id += length(new_columns)
    end

    dedup = Dict{Any, AggregateODRouteColumn}()
    for column in all_columns
        signature = _aggregate_od_route_column_signature(column)
        incumbent = get(dedup, signature, nothing)
        if isnothing(incumbent) || column.tau < incumbent.tau - 1e-9
            dedup[signature] = column
        end
    end
    columns = collect(values(dedup))
    sort!(columns, by=column -> (column.tau, string(column.od_pairs)))
    return columns
end

function _clone_for_final_mip(model::AggregateODRouteModel, columns::Vector{AggregateODRouteColumn})
    return AggregateODRouteModel(
        model.l;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time,
        max_walking_distance        = model.max_walking_distance,
        max_wait_time               = model.max_wait_time,
        detour_factor               = model.detour_factor,
        max_stops                   = model.max_stops,
        max_visits_per_node         = model.max_visits_per_node,
        max_new_columns             = model.max_new_columns,
        n_candidates                = model.n_candidates,
        pricing_time_limit_sec      = model.pricing_time_limit_sec,
        reduced_cost_tol            = model.reduced_cost_tol,
        initial_columns             = columns,
        relax_integrality           = false,
        assignment_policy           = model.assignment_policy,
        allow_walk_only             = model.allow_walk_only,
    )
end

function _clone_for_final_mip(model::RouteCoveringProblem, columns::Vector{AggregateODRouteColumn})
    return RouteCoveringProblem(
        model.l,
        model.open_stations,
        model.fixed_assignments;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time,
        max_walking_distance        = model.max_walking_distance,
        max_wait_time               = model.max_wait_time,
        detour_factor               = model.detour_factor,
        max_stops                   = model.max_stops,
        max_visits_per_node         = model.max_visits_per_node,
        max_new_columns             = model.max_new_columns,
        n_candidates                = model.n_candidates,
        pricing_time_limit_sec      = model.pricing_time_limit_sec,
        reduced_cost_tol            = model.reduced_cost_tol,
        initial_columns             = columns,
        relax_integrality           = false,
        assignment_policy           = model.assignment_policy,
        allow_walk_only             = model.allow_walk_only,
    )
end

function run_aggregate_od_route_column_generation(
    model::AnyAggregateODRouteModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    verbose::Bool=true,
    cg_log_path::Union{Nothing, AbstractString}=nothing,
    column_log_path::Union{Nothing, AbstractString}=nothing,
    dual_log_path::Union{Nothing, AbstractString}=nothing,
    max_cg_iters::Int=10_000,
    max_iterations::Union{Nothing, Int}=nothing,
    max_new_columns::Int=model.max_new_columns,
    n_candidates::Int=max(model.n_candidates, max_new_columns),
    max_visits_per_node::Int=model.max_visits_per_node,
    reduced_cost_tol::Float64=model.reduced_cost_tol,
    pricing_time_limit_sec::Float64=model.pricing_time_limit_sec,
    pricing_initial_sec::Float64=pricing_time_limit_sec,
    pricing_ramp_factor::Float64=1.0,
    profile_pricing::Bool=false,
    ip_time_limit_sec::Float64=3600.0,
    mip_gap::Union{Float64, Nothing}=nothing,
    silent::Bool=!verbose,
)::AggregateODRouteColumnGenerationResult
    isnothing(max_iterations) || (max_cg_iters = max_iterations)
    max_cg_iters > 0 || throw(ArgumentError("max_cg_iters must be positive"))
    max_new_columns > 0 || throw(ArgumentError("max_new_columns must be positive"))
    n_candidates >= max_new_columns || throw(ArgumentError("n_candidates must be >= max_new_columns"))
    pricing_time_limit_sec > 0 || throw(ArgumentError("pricing_time_limit_sec must be positive"))
    pricing_initial_sec > 0 || throw(ArgumentError("pricing_initial_sec must be positive"))
    pricing_ramp_factor > 0 || throw(ArgumentError("pricing_ramp_factor must be positive"))
    ip_time_limit_sec > 0 || throw(ArgumentError("ip_time_limit_sec must be positive"))

    start_time = time()
    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    build_result = build_model(
        model,
        data;
        optimizer_env=optimizer_env,
        relax_integrality=true,
    )
    m = build_result.model
    silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    mapping = build_result.mapping
    logger = _create_aggregate_od_route_cg_logger(verbose=verbose, cg_log_path=cg_log_path)
    column_log_rows = NamedTuple[]
    dual_log_rows = NamedTuple[]
    generated_columns = AggregateODRouteColumn[]
    initial_pool_size = length(mapping.columns)
    n_active_pairs = sum(length(mapping.active_jk_s[s]) for s in 1:n_scenarios(data); init=0)
    _aggregate_od_route_log_header!(logger, n_active_pairs, initial_pool_size, max_cg_iters, pricing_time_limit_sec, max_new_columns)

    lp_bound = NaN
    cg_stop_reason = :max_cg_iters
    cg_iterations = 0
    last_status = :error

    for iteration in 1:max_cg_iters
        cg_iterations = iteration
        columns_before = length(mapping.columns)
        lp_start = time()
        optimize!(m)
        lp_solve_seconds = time() - lp_start
        term_status = termination_status(m)
        last_status = term_status == MOI.OPTIMAL ? :optimal :
            term_status == MOI.INFEASIBLE ? :infeasible :
            term_status == MOI.TIME_LIMIT ? :timeout : :error

        if primal_status(m) != MOI.FEASIBLE_POINT
            cg_stop_reason = :no_primal_solution
            _record_aggregate_od_route_cg_iteration!(logger, AggregateODRouteCGIterationLog(
                iteration, columns_before, length(mapping.columns), last_status,
                nothing, lp_solve_seconds, nothing, lp_solve_seconds,
                0, 0, 0, nothing, false, cg_stop_reason,
                nothing, nothing, nothing, nothing,
                nothing, nothing, nothing, nothing, nothing, nothing,
                nothing, nothing, nothing, nothing,
            ))
            break
        end

        lp_bound = objective_value(m)
        duals = extract_aggregate_od_route_coverage_duals(m)
        if !isnothing(dual_log_path)
            for ((j, k, s), val) in duals.sigma
                push!(dual_log_rows, (iteration=iteration, scenario=s, pickup=j, dropoff=k, sigma=val))
            end
        end
        dual_min, dual_max, dual_mean, dual_std = _aggregate_od_route_dual_stats(duals)

        pricing_started = time()
        iter_pricing_sec = min(
            pricing_time_limit_sec,
            pricing_initial_sec * (pricing_ramp_factor ^ (iteration - 1)),
        )

        next_column_id = isempty(mapping.column_ids) ? 1 : maximum(mapping.column_ids) + 1
        all_new_columns = AggregateODRouteColumn[]
        pricing_exhausted = true
        pricing_stats_by_scenario = []
        for s in 1:n_scenarios(data)
            pricing_duals = _scenario_pricing_duals(duals, s)
            pricing_data = create_aggregate_od_route_pricing_data(model, data, mapping, s, pricing_duals)
            pricing_data = AggregateODRoutePricingData(
                pricing_data.scenario,
                pricing_data.nodes,
                pricing_data.travel_cost,
                pricing_data.active_pairs,
                pricing_data.route_regularization_weight,
                pricing_data.repositioning_time,
                pricing_data.max_wait_time,
                pricing_data.detour_factor,
                pricing_data.max_stops,
                max_visits_per_node,
                pricing_data.bounded_max_stops,
            )
            new_columns_s, exhausted_s, stats_s = aggregate_od_route_pricing_by_label_setting(
                pricing_data,
                mapping.columns,
                pricing_duals;
                next_column_id=next_column_id,
                reduced_cost_tol=reduced_cost_tol,
                max_new_columns=max_new_columns,
                n_candidates=n_candidates,
                time_limit=iter_pricing_sec,
                max_visits_per_node=max_visits_per_node,
                profile=profile_pricing,
            )
            pricing_exhausted &= exhausted_s
            push!(pricing_stats_by_scenario, stats_s)
            append!(all_new_columns, new_columns_s)
            next_column_id += length(new_columns_s)
        end

        dedup = Dict{Any, AggregateODRouteColumn}()
        for column in all_new_columns
            signature = _aggregate_od_route_column_signature(column)
            incumbent = get(dedup, signature, nothing)
            if isnothing(incumbent) || column.tau < incumbent.tau - 1e-9
                dedup[signature] = column
            end
        end
        new_columns = collect(values(dedup))
        sort!(new_columns, by=column -> (
            get(column.metadata, "reduced_cost", Inf),
            column.tau,
            string(get(column.metadata, "route", ())),
        ))
        new_columns = new_columns[1:min(length(new_columns), max_new_columns)]
        pricing_seconds = time() - pricing_started
        iteration_seconds = lp_solve_seconds + pricing_seconds
        best_reduced_cost = isempty(new_columns) ? nothing :
            minimum(Float64(get(column.metadata, "reduced_cost", Inf)) for column in new_columns)

        if isempty(new_columns)
            cg_stop_reason = pricing_exhausted ? :optimality_proven : :no_columns_not_exhausted
            stats = _merge_pricing_stats(pricing_stats_by_scenario)
            _record_aggregate_od_route_cg_iteration!(logger, AggregateODRouteCGIterationLog(
                iteration, columns_before, length(mapping.columns), last_status,
                lp_bound, lp_solve_seconds, pricing_seconds, iteration_seconds,
                0, 0, 0, best_reduced_cost, pricing_exhausted, cg_stop_reason,
                dual_min, dual_max, dual_mean, dual_std,
                stats.labels_generated, stats.labels_rejected_by_dominance,
                stats.labels_removed_by_dominance, stats.stale_pops,
                stats.max_frontier_size, stats.max_live_labels,
                profile_pricing ? stats.t_queue_sec : nothing,
                profile_pricing ? stats.t_candidates_sec : nothing,
                profile_pricing ? stats.t_extension_sec : nothing,
                profile_pricing ? stats.t_dominance_sec : nothing,
            ))
            break
        end

        columns_added = 0
        columns_replaced = 0
        for column in new_columns
            _theta, action = add_or_update_aggregate_od_route_column!(build_result, column)
            action == :added && (columns_added += 1)
            action == :replaced && (columns_replaced += 1)
            action in (:added, :replaced) && push!(generated_columns, column)
            if !isnothing(column_log_path)
                route = get(column.metadata, "route", ())
                push!(column_log_rows, (
                    iteration=iteration,
                    action=string(action),
                    scenario=get(column.metadata, "scenario", missing),
                    column_id=column.id,
                    n_pairs=length(column.od_pairs),
                    tau=column.tau,
                    reduced_cost=get(column.metadata, "reduced_cost", missing),
                    route_length=length(route),
                    route=string(route),
                    pairs=string(Tuple(column.od_pairs)),
                ))
            end
        end

        stats = _merge_pricing_stats(pricing_stats_by_scenario)
        _record_aggregate_od_route_cg_iteration!(logger, AggregateODRouteCGIterationLog(
            iteration, columns_before, length(mapping.columns), last_status,
            lp_bound, lp_solve_seconds, pricing_seconds, iteration_seconds,
            length(new_columns), columns_added, columns_replaced,
            best_reduced_cost, pricing_exhausted, :continue,
            dual_min, dual_max, dual_mean, dual_std,
            stats.labels_generated, stats.labels_rejected_by_dominance,
            stats.labels_removed_by_dominance, stats.stale_pops,
            stats.max_frontier_size, stats.max_live_labels,
            profile_pricing ? stats.t_queue_sec : nothing,
            profile_pricing ? stats.t_candidates_sec : nothing,
            profile_pricing ? stats.t_extension_sec : nothing,
            profile_pricing ? stats.t_dominance_sec : nothing,
        ))
    end

    _flush_aggregate_od_route_cg_log!(logger)
    !isnothing(column_log_path) && _write_aggregate_od_route_cg_log_csv(
        String(column_log_path),
        column_log_rows;
        headers=[:iteration, :action, :scenario, :column_id, :n_pairs, :tau, :reduced_cost, :route_length, :route, :pairs],
    )
    !isnothing(dual_log_path) && _write_aggregate_od_route_cg_log_csv(
        String(dual_log_path),
        dual_log_rows;
        headers=[:iteration, :scenario, :pickup, :dropoff, :sigma],
    )
    _record_aggregate_od_route_cg_termination!(
        logger,
        AggregateODRouteCGTerminationLog(cg_stop_reason, cg_iterations, length(mapping.columns)),
    )

    final_model = _clone_for_final_mip(model, copy(mapping.columns))
    final_build_start = time()
    final_build = build_model(final_model, data; optimizer_env=optimizer_env)
    final_m = final_build.model
    silent && set_silent(final_m)
    set_optimizer_attribute(final_m, "TimeLimit", ip_time_limit_sec)
    isnothing(mip_gap) || set_optimizer_attribute(final_m, "MIPGap", mip_gap)
    final_build_time_sec = time() - final_build_start

    final_solve_start = time()
    optimize!(final_m)
    final_solve_time_sec = time() - final_solve_start
    final_term = termination_status(final_m)
    final_obj = final_term == MOI.OPTIMAL ? objective_value(final_m) : nothing
    final_solution = final_term == MOI.OPTIMAL ?
        (_value_recursive(final_m[:x]), _value_recursive(final_m[:y])) :
        nothing
    # No-op unless an endpoint nearest-open style built zp/zd indicators.
    final_term == MOI.OPTIMAL && assert_endpoint_chain_near_binary(final_m)
    final_result = OptResult(
        final_term,
        final_obj,
        final_solution,
        time() - start_time,
        final_m,
        final_build.mapping,
        final_build.detour_combos,
        final_build.counts,
        nothing,
        Dict{String, Any}(
            "build_time_sec" => final_build_time_sec,
            "solve_time_sec" => final_solve_time_sec,
            "cg_time_sec" => final_build_time_sec + final_solve_time_sec,
        ),
    )

    status = final_result.termination_status == MOI.OPTIMAL ? :optimal :
        final_result.termination_status == MOI.INFEASIBLE ? :infeasible :
        final_result.termination_status == MOI.TIME_LIMIT ? :timeout : :error
    cg_stop_reason == :optimality_proven || status != :optimal || (status = :feasible)

    return AggregateODRouteColumnGenerationResult(
        status,
        final_result,
        lp_bound,
        cg_iterations,
        cg_stop_reason,
        copy(mapping.columns),
        _selected_aggregate_od_route_column_ids(final_result),
        _aggregate_od_route_coverage_summary(final_result),
        copy(logger.iteration_rows),
        column_log_rows,
        dual_log_rows,
    )
end

function _aggregate_od_route_cg_log_path(solver::ColumnGenerationSolver, filename::String)
    isnothing(solver.log_dir) && return nothing
    return joinpath(solver.log_dir, filename)
end

function _aggregate_od_route_cg_log_path(solver::BendersSolver, filename::String)
    isnothing(solver.log_dir) && return nothing
    return joinpath(solver.log_dir, filename)
end

function run_opt(
    instance::StationSelectionData,
    formulation::AnyAggregateODRouteModel,
    solver::ColumnGenerationSolver,
)
    cfg = solver.config
    result = run_aggregate_od_route_column_generation(
        formulation,
        instance;
        optimizer_env=cfg.optimizer_env,
        verbose=!cfg.silent,
        cg_log_path=_aggregate_od_route_cg_log_path(solver, "aggregate_od_route_cg_iterations.csv"),
        column_log_path=_aggregate_od_route_cg_log_path(solver, "aggregate_od_route_cg_columns.csv"),
        dual_log_path=_aggregate_od_route_cg_log_path(solver, "aggregate_od_route_cg_duals.csv"),
        max_cg_iters=solver.max_iterations,
        max_new_columns=solver.max_columns_per_iteration,
        n_candidates=solver.n_candidates,
        reduced_cost_tol=solver.reduced_cost_tol,
        pricing_time_limit_sec=solver.pricing_time_limit_sec,
        ip_time_limit_sec=solver.final_ip_time_limit_sec,
        mip_gap=cfg.mip_gap,
        silent=cfg.silent,
    )
    return result.final_result
end

function run_opt(
    instance::StationSelectionData,
    formulation::RouteCoveringProblem,
    solver::ColumnGenerationSolver,
)
    cfg = solver.config
    result = run_aggregate_od_route_column_generation(
        formulation,
        instance;
        optimizer_env=cfg.optimizer_env,
        verbose=!cfg.silent,
        cg_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_iterations.csv"),
        column_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_columns.csv"),
        dual_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_duals.csv"),
        max_cg_iters=solver.max_iterations,
        max_new_columns=solver.max_columns_per_iteration,
        n_candidates=solver.n_candidates,
        reduced_cost_tol=solver.reduced_cost_tol,
        pricing_time_limit_sec=solver.pricing_time_limit_sec,
        ip_time_limit_sec=solver.final_ip_time_limit_sec,
        mip_gap=cfg.mip_gap,
        silent=cfg.silent,
    )
    return result.final_result
end
