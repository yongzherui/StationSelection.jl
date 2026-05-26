export compute_qbar
export candidate_next_stations
export dropoff
export enumerate_pickup_vectors
export extend_label
export dominates
export price_scenario
export solve_alpha_route_pricing

"""
Exact bucketed pricing for the current AlphaRoute column-generation path.

The active restricted master is still indexed by `(scenario, time bucket)` and uses
support profiles `alpha[j,k]` inside a bucket rather than scenario-level
`alpha[j,k,t]`. Pricing is therefore solved exactly per bucket `(s, t_id)`, with a
single ordered route sequence and an aggregate simultaneously feasible loaded profile
for that bucket.

Reduced cost for a bucket-local column is:

    c̄(V, α) = μ * travel_time(V) - Σ_{j,k} π_{jkts} * α_{jk}

Capacity is enforced through the onboard profile `b[j,k]`, not by independent bounds
on the completed support profile `α[j,k]`.
"""

function _request_quantity(row)::Int
    if :pax_num in propertynames(row)
        return Int(row.pax_num)
    end
    return 1
end

function _bucket_request_time_id(
    scenario::ScenarioData,
    row,
    time_window_sec::Int
)::Int
    isnothing(scenario.start_time) && error("Scenario '$(scenario.label)' must have start_time for pricing")
    req_time = row.request_time isa AbstractString ?
        DateTime(row.request_time, "yyyy-mm-dd HH:MM:SS") :
        row.request_time
    t_diff_sec = (req_time - scenario.start_time) / Dates.Second(1)
    return floor(Int, t_diff_sec / time_window_sec)
end

function compute_qbar(
    scenario::ScenarioData,
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    time_window_sec::Int
)::Dict{Int, AlphaRouteBucketDemandCaps}
    _require_indexed_request_columns(scenario.requests)

    by_bucket = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
    for row in eachrow(scenario.requests)
        o = row.origin_idx
        d = row.dest_idx
        t_id = _bucket_request_time_id(scenario, row, time_window_sec)
        bucket_caps = get!(by_bucket, t_id, Dict{Tuple{Int, Int}, Int}())
        qty = _request_quantity(row)
        for (j_idx, k_idx) in get(valid_jk_pairs, (o, d), Tuple{Int, Int}[])
            bucket_caps[(j_idx, k_idx)] = get(bucket_caps, (j_idx, k_idx), 0) + qty
        end
    end

    return Dict(
        t_id => AlphaRouteBucketDemandCaps(0, t_id, caps)
        for (t_id, caps) in by_bucket
    )
end

function _bucket_duals(
    duals::AlphaRouteCGDuals,
    scenario_idx::Int,
    time_id::Int
)::Dict{Tuple{Int, Int}, Float64}
    bucket_duals = Dict{Tuple{Int, Int}, Float64}()
    for ((s, t_id, j_idx, k_idx), value) in duals.route_capacity
        s == scenario_idx || continue
        t_id == time_id || continue
        bucket_duals[(j_idx, k_idx)] = value
    end
    return bucket_duals
end

function _candidate_station_set(
    qbar::AlphaRouteBucketDemandCaps,
    duals::Dict{Tuple{Int, Int}, Float64}
)::Vector{Int}
    stations = Set{Int}()
    for ((j_idx, k_idx), cap) in qbar.caps
        cap > 0 || continue
        push!(stations, j_idx)
        push!(stations, k_idx)
    end
    for ((j_idx, k_idx), dual) in duals
        dual > 0 || continue
        push!(stations, j_idx)
        push!(stations, k_idx)
    end
    return sort!(collect(stations))
end

function _build_bucket_pricing_data(
    model::AlphaRouteModel,
    data::StationSelectionData,
    scenario_idx::Int,
    time_id::Int,
    qbar::AlphaRouteBucketDemandCaps,
    duals::Dict{Tuple{Int, Int}, Float64},
)::AlphaRouteBucketPricingData
    return AlphaRouteBucketPricingData(
        scenario_idx,
        time_id,
        _candidate_station_set(qbar, duals),
        AlphaRouteBucketDemandCaps(scenario_idx, time_id, copy(qbar.caps)),
        copy(duals),
        model.vehicle_capacity,
        model.max_route_length,
        model.stop_dwell_time,
        model.route_regularization_weight,
    )
end

function _profile_sorted_pairs(profile::Dict{Tuple{Int, Int}, Int})
    return sort!(collect(profile), by=x -> (x.first[1], x.first[2]))
end

function _profile_signature(profile::Dict{Tuple{Int, Int}, Int})::String
    isempty(profile) && return ""
    parts = String[]
    for ((j_idx, k_idx), value) in _profile_sorted_pairs(profile)
        push!(parts, "$(j_idx)>$(k_idx)=$(value)")
    end
    return join(parts, ";")
end

function _visited_signature(visited::BitSet)::String
    isempty(visited) && return ""
    return join(sort!(collect(visited)), "|")
end

function _label_signature(label::AlphaRoutePricingLabel)
    return (
        label.current_station,
        _visited_signature(label.visited),
        _profile_signature(label.onboard),
        _profile_signature(label.alpha),
    )
end

function _current_load(label::AlphaRoutePricingLabel)::Int
    return sum(values(label.onboard); init=0)
end

function dropoff(label::AlphaRoutePricingLabel, station_idx::Int)::Dict{Tuple{Int, Int}, Int}
    onboard_after = copy(label.onboard)
    for ((j_idx, k_idx), load) in collect(onboard_after)
        k_idx == station_idx || continue
        load > 0 || continue
        delete!(onboard_after, (j_idx, k_idx))
    end
    return onboard_after
end

function candidate_next_stations(
    label::AlphaRoutePricingLabel,
    pricing_data::AlphaRouteBucketPricingData,
    data::StationSelectionData
)::Vector{Int}
    length(label.route_sequence) >= pricing_data.max_route_length && return Int[]
    can_add_future_dropoff = length(label.route_sequence) + 1 < pricing_data.max_route_length

    candidates = Int[]
    for h in pricing_data.candidate_stations
        h == label.current_station && continue
        h in label.visited && continue
        isfinite(get_routing_cost(data, label.current_station, h)) || continue

        needed_dropoff = any(k_idx == h && load > 0 for ((_, k_idx), load) in label.onboard)
        can_pickup = false
        if can_add_future_dropoff
            for ((j_idx, k_idx), cap) in pricing_data.demand_caps.caps
                j_idx == h || continue
                k_idx == h && continue
                k_idx in label.visited && continue
                cap > get(label.alpha, (j_idx, k_idx), 0) || continue
                get(pricing_data.duals, (j_idx, k_idx), 0.0) > 0.0 || continue
                can_pickup = true
                break
            end
        end

        (needed_dropoff || can_pickup) && push!(candidates, h)
    end

    return sort!(candidates)
end

function enumerate_pickup_vectors(
    label_after_dropoff::AlphaRoutePricingLabel,
    station_idx::Int,
    pricing_data::AlphaRouteBucketPricingData,
    residual_capacity::Int
)::Vector{Dict{Tuple{Int, Int}, Int}}
    can_add_future_dropoff = length(label_after_dropoff.route_sequence) < pricing_data.max_route_length - 1
    admissible = Tuple{Int, Int}[]
    upper_bounds = Int[]
    if can_add_future_dropoff
        for ((j_idx, k_idx), cap) in sort!(collect(pricing_data.demand_caps.caps); by=x -> (x[1][1], x[1][2]))
            j_idx == station_idx || continue
            k_idx == station_idx && continue
            k_idx in label_after_dropoff.visited && continue
            dual = get(pricing_data.duals, (j_idx, k_idx), 0.0)
            dual > 0.0 || continue
            residual_cap = cap - get(label_after_dropoff.alpha, (j_idx, k_idx), 0)
            residual_cap > 0 || continue
            push!(admissible, (j_idx, k_idx))
            push!(upper_bounds, residual_cap)
        end
    end

    results = Dict{Tuple{Int, Int}, Int}[]
    current = Dict{Tuple{Int, Int}, Int}()

    function rec(idx::Int, remaining::Int)
        if idx > length(admissible)
            push!(results, copy(current))
            return
        end

        pair = admissible[idx]
        ub = min(upper_bounds[idx], remaining)
        for amount in 0:ub
            if amount == 0
                delete!(current, pair)
            else
                current[pair] = amount
            end
            rec(idx + 1, remaining - amount)
        end
        delete!(current, pair)
    end

    rec(1, residual_capacity)
    return results
end

function _marginal_extension_travel_time(
    label::AlphaRoutePricingLabel,
    next_station::Int,
    pricing_data::AlphaRouteBucketPricingData,
    data::StationSelectionData
)::Float64
    travel = get_routing_cost(data, label.current_station, next_station)
    isfinite(travel) || return Inf
    dwell = length(label.route_sequence) >= 2 ? pricing_data.stop_dwell_time : 0.0
    return dwell + travel
end

function extend_label(
    label::AlphaRoutePricingLabel,
    next_station::Int,
    pickup_vector::Dict{Tuple{Int, Int}, Int},
    pricing_data::AlphaRouteBucketPricingData,
    data::StationSelectionData
)::Union{AlphaRoutePricingLabel, Nothing}
    marginal_tt = _marginal_extension_travel_time(label, next_station, pricing_data, data)
    isfinite(marginal_tt) || return nothing

    onboard_after_dropoff = dropoff(label, next_station)
    residual_capacity = pricing_data.vehicle_capacity - sum(values(onboard_after_dropoff); init=0)
    pickup_total = sum(values(pickup_vector); init=0)
    pickup_total <= residual_capacity || return nothing

    new_onboard = copy(onboard_after_dropoff)
    new_alpha = copy(label.alpha)
    dual_reward = 0.0
    for ((j_idx, k_idx), amount) in pickup_vector
        amount >= 0 || return nothing
        amount == 0 && continue
        if j_idx != next_station
            return nothing
        end
        cap = get(pricing_data.demand_caps.caps, (j_idx, k_idx), 0)
        amount <= cap - get(new_alpha, (j_idx, k_idx), 0) || return nothing
        get(pricing_data.duals, (j_idx, k_idx), 0.0) > 0.0 || return nothing
        new_onboard[(j_idx, k_idx)] = get(new_onboard, (j_idx, k_idx), 0) + amount
        new_alpha[(j_idx, k_idx)] = get(new_alpha, (j_idx, k_idx), 0) + amount
        dual_reward += pricing_data.duals[(j_idx, k_idx)] * amount
    end

    new_visited = copy(label.visited)
    push!(new_visited, next_station)
    return AlphaRoutePricingLabel(
        next_station,
        new_visited,
        vcat(label.route_sequence, next_station),
        label.resource_tau + marginal_tt,
        new_onboard,
        new_alpha,
        label.reduced_cost + pricing_data.route_regularization_weight * marginal_tt - dual_reward,
    )
end

function dominates(label1::AlphaRoutePricingLabel, label2::AlphaRoutePricingLabel)::Bool
    label1.current_station == label2.current_station || return false
    label1.visited == label2.visited || return false
    label1.onboard == label2.onboard || return false
    label1.alpha == label2.alpha || return false
    label1.resource_tau <= label2.resource_tau + 1e-9 || return false
    label1.reduced_cost <= label2.reduced_cost + 1e-9 || return false
    return true
end

function _push_if_nondominated!(
    store::Dict,
    open_stack::Vector{AlphaRoutePricingLabel},
    label::AlphaRoutePricingLabel,
    dominated_counter::Base.RefValue{Int}
)
    signature = _label_signature(label)
    bucket = get!(store, signature, AlphaRoutePricingLabel[])
    for existing in bucket
        if dominates(existing, label)
            dominated_counter[] += 1
            return false
        end
    end
    filter!(existing -> !dominates(label, existing), bucket)
    push!(bucket, label)
    push!(open_stack, label)
    return true
end

function _route_from_label(
    label::AlphaRoutePricingLabel,
    scenario_idx::Int,
    time_id::Int,
)::AlphaRoutePricedColumn
    route_id = 1
    station_indices = copy(label.route_sequence)
    detour_feasible_legs = sort!(collect(keys(label.alpha)); by=x -> (x[1], x[2]))
    route = RouteData(route_id, station_indices, label.resource_tau, detour_feasible_legs)
    alpha_profile = Dict{NTuple{3, Int}, Float64}()
    for ((j_idx, k_idx), value) in label.alpha
        value > 0 || continue
        alpha_profile[(route_id, j_idx, k_idx)] = Float64(value)
    end
    return AlphaRoutePricedColumn(
        scenario_idx,
        time_id,
        route,
        alpha_profile,
        label.reduced_cost,
    )
end

function _positive_origin_stations(pricing_data::AlphaRouteBucketPricingData)::Vector{Int}
    origins = Set{Int}()
    for ((j_idx, k_idx), dual) in pricing_data.duals
        dual > 0.0 || continue
        get(pricing_data.demand_caps.caps, (j_idx, k_idx), 0) > 0 || continue
        push!(origins, j_idx)
    end
    return sort!(collect(origins))
end

function _initial_labels(
    pricing_data::AlphaRouteBucketPricingData,
    rc_tolerance::Float64
)::Tuple{Vector{AlphaRoutePricingLabel}, Int}
    labels = AlphaRoutePricingLabel[]
    initialized = 0
    for start_station in _positive_origin_stations(pricing_data)
        visited = BitSet([start_station])
        admissible = Tuple{Int, Int}[]
        upper_bounds = Int[]
        for ((j_idx, k_idx), cap) in sort!(collect(pricing_data.demand_caps.caps); by=x -> (x[1][1], x[1][2]))
            j_idx == start_station || continue
            k_idx == start_station && continue
            k_idx in visited && continue
            dual = get(pricing_data.duals, (j_idx, k_idx), 0.0)
            dual > 0.0 || continue
            cap > 0 || continue
            push!(admissible, (j_idx, k_idx))
            push!(upper_bounds, cap)
        end

        pickup_vectors = Dict{Tuple{Int, Int}, Int}[]
        current = Dict{Tuple{Int, Int}, Int}()
        function rec(idx::Int, remaining::Int)
            if idx > length(admissible)
                push!(pickup_vectors, copy(current))
                return
            end
            pair = admissible[idx]
            ub = min(upper_bounds[idx], remaining)
            for amount in 0:ub
                if amount == 0
                    delete!(current, pair)
                else
                    current[pair] = amount
                end
                rec(idx + 1, remaining - amount)
            end
            delete!(current, pair)
        end
        rec(1, pricing_data.vehicle_capacity)

        for pickup_vector in pickup_vectors
            isempty(pickup_vector) && continue
            onboard = Dict{Tuple{Int, Int}, Int}()
            alpha = Dict{Tuple{Int, Int}, Int}()
            reward = 0.0
            for ((j_idx, k_idx), amount) in pickup_vector
                amount > 0 || continue
                onboard[(j_idx, k_idx)] = amount
                alpha[(j_idx, k_idx)] = amount
                reward += pricing_data.duals[(j_idx, k_idx)] * amount
            end
            isempty(onboard) && continue
            push!(labels, AlphaRoutePricingLabel(
                start_station,
                visited,
                [start_station],
                0.0,
                onboard,
                alpha,
                -reward,
            ))
            initialized += 1
        end
    end
    return labels, initialized
end

function _price_bucket(
    model::AlphaRouteModel,
    data::StationSelectionData,
    scenario_idx::Int,
    pricing_data::AlphaRouteBucketPricingData;
    rc_tolerance::Float64=-1e-6,
    max_columns::Int=10,
    time_limit_sec::Float64=60.0,
    return_all_negative::Bool=false,
)::NamedTuple
    initial_labels, initialized = _initial_labels(pricing_data, rc_tolerance)
    isempty(initial_labels) && return (
        columns=AlphaRoutePricedColumn[],
        status=:optimal,
        message="no profitable root pickups",
        metadata=Dict{String, Any}(
            "scenario_idx" => scenario_idx,
            "time_id" => pricing_data.time_id,
            "labels_initialized" => initialized,
            "labels_expanded" => 0,
            "labels_dominated" => 0,
            "completed_labels" => 0,
        ),
    )

    open_stack = copy(initial_labels)
    nondominated = Dict{Any, Vector{AlphaRoutePricingLabel}}()
    for label in initial_labels
        push!(get!(nondominated, _label_signature(label), AlphaRoutePricingLabel[]), label)
    end

    dominated_count = Ref(0)
    expanded_count = 0
    completed_labels = AlphaRoutePricingLabel[]
    best_rc = Inf
    t0 = time()
    status = :optimal
    message = "exact pricing exhausted search"

    while !isempty(open_stack)
        if time() - t0 > time_limit_sec
            status = :time_limit
            message = "bucket pricing hit time limit"
            break
        end

        label = pop!(open_stack)
        expanded_count += 1

        if isempty(label.onboard) && label.reduced_cost < rc_tolerance && length(label.route_sequence) >= 2
            push!(completed_labels, label)
            best_rc = min(best_rc, label.reduced_cost)
        end

        for next_station in candidate_next_stations(label, pricing_data, data)
            onboard_after_dropoff = dropoff(label, next_station)
            residual_capacity = pricing_data.vehicle_capacity - sum(values(onboard_after_dropoff); init=0)
            after_dropoff = AlphaRoutePricingLabel(
                label.current_station,
                label.visited,
                label.route_sequence,
                label.resource_tau,
                onboard_after_dropoff,
                label.alpha,
                label.reduced_cost,
            )
            pickup_vectors = enumerate_pickup_vectors(after_dropoff, next_station, pricing_data, residual_capacity)
            for pickup_vector in pickup_vectors
                new_label = extend_label(label, next_station, pickup_vector, pricing_data, data)
                isnothing(new_label) && continue
                _push_if_nondominated!(nondominated, open_stack, new_label, dominated_count)
            end
        end
    end

    columns = AlphaRoutePricedColumn[]
    for label in completed_labels
        push!(columns, _route_from_label(label, scenario_idx, pricing_data.time_id))
    end
    sort!(columns, by=col -> col.reduced_cost)
    columns = return_all_negative ? columns : columns[1:min(length(columns), max_columns)]
    if !return_all_negative && length(columns) > max_columns
        columns = columns[1:max_columns]
    end
    if isempty(columns) && status == :optimal
        message = "no negative reduced-cost column"
    end

    return (
        columns=columns,
        status=status,
        message=message,
        metadata=Dict{String, Any}(
            "scenario_idx" => scenario_idx,
            "time_id" => pricing_data.time_id,
            "labels_initialized" => initialized,
            "labels_expanded" => expanded_count,
            "labels_dominated" => dominated_count[],
            "completed_labels" => length(completed_labels),
            "best_reduced_cost" => isfinite(best_rc) ? best_rc : nothing,
        ),
    )
end

function price_scenario(
    scenario_idx::Int,
    model::AlphaRouteModel,
    data::StationSelectionData,
    bucket_duals::Dict{Int, Dict{Tuple{Int, Int}, Float64}},
    qbar_by_bucket::Dict{Int, AlphaRouteBucketDemandCaps};
    rc_tolerance::Float64=-1e-6,
    max_columns::Int=10,
    time_limit_sec::Float64=60.0,
    return_all_negative::Bool=false,
)::AlphaRoutePricingResult
    all_columns = AlphaRoutePricedColumn[]
    bucket_metadata = Dict{String, Any}()
    status = :optimal
    messages = String[]
    n_buckets = max(length(bucket_duals), 1)
    per_bucket_limit = max(time_limit_sec / n_buckets, 1e-6)

    for time_id in sort!(collect(keys(bucket_duals)))
        duals_t = bucket_duals[time_id]
        qbar_t = get(qbar_by_bucket, time_id, AlphaRouteBucketDemandCaps(scenario_idx, time_id, Dict{Tuple{Int, Int}, Int}()))
        pricing_data = _build_bucket_pricing_data(model, data, scenario_idx, time_id, qbar_t, duals_t)
        bucket_result = _price_bucket(
            model,
            data,
            scenario_idx,
            pricing_data;
            rc_tolerance=rc_tolerance,
            max_columns=max_columns,
            time_limit_sec=per_bucket_limit,
            return_all_negative=return_all_negative,
        )
        append!(all_columns, bucket_result.columns)
        bucket_metadata["bucket_$(time_id)"] = bucket_result.metadata
        push!(messages, "t=$(time_id): $(bucket_result.message)")
        bucket_result.status == :time_limit && (status = :time_limit)
    end

    sort!(all_columns, by=col -> col.reduced_cost)
    !return_all_negative && (all_columns = all_columns[1:min(length(all_columns), max_columns)])
    if status == :optimal && isempty(all_columns)
        status = :no_improving_column
    end

    return AlphaRoutePricingResult(
        all_columns,
        status,
        join(messages, " | "),
        Dict{String, Any}(
            "scenario_idx" => scenario_idx,
            "bucket_metadata" => bucket_metadata,
        ),
    )
end

function solve_alpha_route_pricing(
    model::AlphaRouteModel,
    data::StationSelectionData,
    state::AlphaRouteColumnGenerationState,
    duals::AlphaRouteCGDuals;
    rc_tolerance::Float64=-1e-6,
    max_columns::Int=10,
    time_limit_sec::Float64=60.0,
)::AlphaRoutePricingResult
    base = _build_alpha_route_base(model, data)

    scenario_duals = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Float64}}}()
    for ((s, t_id, j_idx, k_idx), value) in duals.route_capacity
        value > 0.0 || continue
        by_time = get!(scenario_duals, s, Dict{Int, Dict{Tuple{Int, Int}, Float64}}())
        bucket_duals = get!(by_time, t_id, Dict{Tuple{Int, Int}, Float64}())
        bucket_duals[(j_idx, k_idx)] = value
    end

    scenario_results = AlphaRoutePricingResult[]
    all_columns = AlphaRoutePricedColumn[]
    metadata = Dict{String, Any}()
    status = :optimal
    n_scenarios = max(length(scenario_duals), 1)
    per_scenario_limit = max(time_limit_sec / n_scenarios, 1e-6)

    for scenario_idx in sort!(collect(keys(scenario_duals)))
        qbar_by_bucket = compute_qbar(
            data.scenarios[scenario_idx],
            base.valid_jk_pairs,
            model.time_window_sec,
        )
        scenario_result = price_scenario(
            scenario_idx,
            model,
            data,
            scenario_duals[scenario_idx],
            qbar_by_bucket;
            rc_tolerance=rc_tolerance,
            max_columns=max_columns,
            time_limit_sec=per_scenario_limit,
            return_all_negative=true,
        )
        push!(scenario_results, scenario_result)
        append!(all_columns, scenario_result.columns)
        metadata["scenario_$(scenario_idx)"] = scenario_result.metadata
        scenario_result.status == :time_limit && (status = :time_limit)
    end

    sort!(all_columns, by=col -> col.reduced_cost)

    novel_columns = AlphaRoutePricedColumn[]
    for column in all_columns
        bucket_state = get(state.route_pool.bucket_states, (column.scenario_idx, column.time_id), nothing)
        isnothing(bucket_state) && continue
        signature = _route_alpha_signature(column.route, column.alpha_profile)
        haskey(bucket_state.signature_to_route_id, signature) && continue
        push!(novel_columns, column)
        length(novel_columns) >= max_columns && break
    end

    if status == :optimal && isempty(novel_columns)
        status = :no_improving_column
    end

    return AlphaRoutePricingResult(
        novel_columns,
        status,
        isempty(novel_columns) ? "no novel negative reduced-cost column found" : "priced $(length(novel_columns)) novel negative reduced-cost columns",
        merge(metadata, Dict{String, Any}(
            "total_negative_columns" => length(all_columns),
            "novel_negative_columns" => length(novel_columns),
        )),
    )
end
