"""
Cross-window temporal route generation for TwoStageRouteModel.

Generates vehicle routes that correctly chain orders across time windows.
A route can serve order o1 at time t1 and order o2 at time t2 > t1 iff the vehicle
can travel from o1's pickup station to o2's pickup station and arrive within
`[t2_start, t2_start + max_wait_time]` (vehicle cannot wait).

Implements a BFS label-setting DP where each label represents a partial route
(vehicle at some station at some absolute time, some passengers on board).
"""

export TimedRouteData
export generate_routes_from_timed_orders


"""
    TimedRouteData

A vehicle route with per-(time_window, VBS-leg) actual passenger counts.

# Fields
- `route::RouteData`: station sequence and total routing travel time
- `alpha::Dict{Tuple{Int,Tuple{Int,Int}},Int}`: (t_id, (pickup_idx, dropoff_idx)) → passengers

`alpha[(t_id, (j_idx, k_idx))]` is the total passengers this route carries from
VBS j to VBS k for orders in time window t_id. Used directly in the covering constraint.
"""
struct TimedRouteData
    route  :: RouteData
    alpha  :: Dict{Tuple{Int, Tuple{Int,Int}}, Int}
end


# ─────────────────────────────────────────────────────────────────────────────
# Internal types
# ─────────────────────────────────────────────────────────────────────────────

"""
    _TimedOrder

Internal: one demand aggregate — a unique (o_id, d_id, t_id) combination in a scenario.

# Fields
- `o_id`, `d_id`:    geographic origin/destination station IDs
- `t_id`:            time-window index (t_start = t_id * time_window_sec)
- `demand`:          number of passengers in this group
- `feasible_vbs`:    (pickup_station_id, dropoff_station_id) VBS pairs this order can use
"""
struct _TimedOrder
    o_id         :: Int
    d_id         :: Int
    t_id         :: Int
    demand       :: Int
    feasible_vbs :: Vector{Tuple{Int,Int}}
end


"""
    TimedRouteLabel

BFS label for temporal cross-window route generation.

# Fields
- `station`:        current station (last stop visited)
- `abs_time`:       seconds since scenario start
- `passengers`:     current vehicle load (sum of demands of on-board orders)
- `picked`:         bit j set ⟺ order j+1 has been picked up
- `dropped`:        bit j set ⟺ order j+1 has been dropped off
- `parent`:         1-based index into labels array; 0 = root
- `board_abstime`:  length-n; entry j = abs_time when order j was picked up (Inf = not yet)
- `chosen_pickup`:  length-n; entry j = pickup VBS station ID for order j (0 = not yet)
- `chosen_dropoff`: length-n; entry j = dropoff VBS station ID for order j (0 = not yet)
"""
struct TimedRouteLabel
    station        :: Int
    abs_time       :: Float64
    passengers     :: Int
    picked         :: UInt64
    dropped        :: UInt64
    parent         :: Int
    board_abstime  :: Vector{Float64}
    chosen_pickup  :: Vector{Int}
    chosen_dropoff :: Vector{Int}
end


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

"""
    generate_routes_from_timed_orders(
        orders::Vector{_TimedOrder},
        data::StationSelectionData,
        station_id_to_array_idx::Dict{Int,Int};
        vehicle_capacity::Int = 4,
        max_wait_time::Float64,
        max_delay_time::Union{Float64,Nothing} = nothing,
        max_delay_ratio::Union{Float64,Nothing} = nothing,
        time_window_sec::Int = 120
    ) -> Vector{TimedRouteData}

Generate temporally-valid vehicle routes for a single scenario.

Routes can chain orders from different time windows: order j at t_id_j and order k at
t_id_k > t_id_j can share a route if the vehicle travels from j's pickup to k's pickup
and arrives within `[t_k_start, t_k_start + max_wait_time]`. No vehicle waiting is allowed.

# Arguments
- `orders`: demand aggregates (unique (o,d,t_id) combinations) with feasible VBS pairs
- `data`: problem data with routing costs
- `station_id_to_array_idx`: station ID → 1-based array index mapping
- `vehicle_capacity`: max total passengers on board at any time
- `max_wait_time`: seconds after t_id * time_window_sec that vehicle can arrive at pickup
- `max_delay_time`: max extra in-vehicle time vs direct trip (use `Inf` for no limit)
- `max_delay_ratio`: max ratio `in_vehicle/direct - 1` (use `Inf` for no limit)
- `time_window_sec`: seconds per time window (for t_start = t_id * time_window_sec)

# Returns
`Vector{TimedRouteData}` — distinct feasible routes found by the BFS.
Routes with identical (station_sequence, served_order_set) are deduplicated.

# Limits
Supports at most 63 orders per call (UInt64 bitmask). Throws an error if exceeded.
"""
function generate_routes_from_timed_orders(
    orders               :: Vector{_TimedOrder},
    data                 :: StationSelectionData,
    station_id_to_array_idx :: Dict{Int,Int};
    vehicle_capacity     :: Int = 4,
    max_wait_time        :: Float64,
    max_delay_time       :: Float64,
    max_delay_ratio      :: Float64,
    time_window_sec      :: Int = 120,
    max_labels           :: Int = 400_000
)::Vector{TimedRouteData}
    has_routing_costs(data) || error(
        "generate_routes_from_timed_orders requires routing costs " *
        "(data.routing_costs must not be nothing)"
    )
    vehicle_capacity > 0 || throw(ArgumentError("vehicle_capacity must be positive"))
    isempty(orders) && return TimedRouteData[]

    n = length(orders)
    n <= 63 || error(
        "generate_routes_from_timed_orders supports at most 63 orders per scenario; " *
        "got $n. Consider increasing time_window_sec or reducing scenario duration."
    )

    # (station_sequence, picked_bitmask) → TimedRouteData
    routes_map = Dict{Tuple{Vector{Int},UInt64}, TimedRouteData}()
    next_id    = Ref(0)

    labels = TimedRouteLabel[]
    sizehint!(labels, max(200, 20 * n))

    # ── Initialise: one root label per (order j, feasible VBS pair (p, d)) ────
    for j in 1:n
        order = orders[j]
        isempty(order.feasible_vbs) && continue

        t_start = Float64(order.t_id * time_window_sec)
        bit_j   = UInt64(1) << (j - 1)

        for (p_id, d_id) in order.feasible_vbs
            bct = fill(Inf, n);  bct[j] = t_start
            cp  = zeros(Int, n); cp[j]  = p_id
            cd  = zeros(Int, n); cd[j]  = d_id
            push!(labels, TimedRouteLabel(
                p_id, t_start, order.demand,
                bit_j, UInt64(0), 0,
                bct, cp, cd
            ))
        end
    end

    _timed_run_label_setting!(
        labels, orders, data, station_id_to_array_idx,
        vehicle_capacity, max_wait_time,
        max_delay_time, max_delay_ratio, time_window_sec,
        routes_map, next_id, max_labels
    )

    routes_sorted = sort!(collect(values(routes_map)), by = r -> r.route.id)
    return [TimedRouteData(
                RouteData(i, r.route.station_ids, r.route.travel_time, r.route.od_capacity),
                r.alpha)
            for (i, r) in enumerate(routes_sorted)]
end


# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ─────────────────────────────────────────────────────────────────────────────

"""
BFS label-setting main loop. Processes labels in push order; extends each by one
dropoff or pickup action. Calls `_timed_record_route!` for complete labels.
"""
function _timed_run_label_setting!(
    labels            :: Vector{TimedRouteLabel},
    orders            :: Vector{_TimedOrder},
    data              :: StationSelectionData,
    station_id_to_idx :: Dict{Int,Int},
    vehicle_capacity  :: Int,
    max_wait_time     :: Float64,
    max_delay_time    :: Union{Float64,Nothing},
    max_delay_ratio   :: Union{Float64,Nothing},
    time_window_sec   :: Int,
    routes_map        :: Dict{Tuple{Vector{Int},UInt64}, TimedRouteData},
    next_id           :: Ref{Int},
    max_labels        :: Int
)
    n = length(orders)
    ε = 1e-9

    idx = 1
    while idx <= length(labels)
        if idx > max_labels
            println("    BFS: label limit ($max_labels explored) reached; stopping early with $(length(routes_map)) routes")
            flush(stdout)
            break
        end
        if idx % 50_000 == 1 && idx > 1
            println("    BFS: processed $idx / $(length(labels)) labels, $(length(routes_map)) routes so far")
            flush(stdout)
        end

        lbl = labels[idx]
        idx += 1  # always advance first so any `continue` below does not stall the loop

        # ── Stage 1: drop off on-board passengers ─────────────────────────────
        for j in 0:(n - 1)
            (lbl.picked  >> j) & UInt64(1) == UInt64(0) && continue  # not picked up
            (lbl.dropped >> j) & UInt64(1) == UInt64(1) && continue  # already dropped

            d_id = lbl.chosen_dropoff[j + 1]
            arr  = lbl.abs_time + get_routing_cost(data, lbl.station, d_id)
            p_id = lbl.chosen_pickup[j + 1]

            in_vehicle = arr - lbl.board_abstime[j + 1]
            direct     = get_routing_cost(data, p_id, d_id)

            if in_vehicle - direct > max_delay_time + ε
                continue
            end
            if direct > 0.0 && in_vehicle / direct > 1.0 + max_delay_ratio + ε
                continue
            end

            new_dropped = lbl.dropped | (UInt64(1) << j)
            # Dropoff: reuse parent's vectors (not modified by dropoff)
            child = TimedRouteLabel(
                d_id, arr, lbl.passengers - orders[j + 1].demand,
                lbl.picked, new_dropped, idx - 1,
                lbl.board_abstime, lbl.chosen_pickup, lbl.chosen_dropoff
            )
            push!(labels, child)

            if child.picked == child.dropped
                _timed_record_route!(
                    labels, length(labels), orders,
                    data, station_id_to_idx, vehicle_capacity,
                    routes_map, next_id
                )
            end
        end

        # ── Stage 2: pick up new orders ───────────────────────────────────────
        lbl.passengers == 0 && continue  # vehicle is empty; continuing produces interleaved routes that are always dominated in the MILP
        for k in 0:(n - 1)
            (lbl.picked >> k) & UInt64(1) == UInt64(1) && continue  # already picked

            order_k = orders[k + 1]
            lbl.passengers + order_k.demand > vehicle_capacity && continue

            t_k_start = Float64(order_k.t_id * time_window_sec)

            for (p_id, d_id) in order_k.feasible_vbs
                arr = lbl.abs_time + get_routing_cost(data, lbl.station, p_id)

                # Vehicle arrives before order is ready (no waiting allowed)
                arr < t_k_start - ε && continue
                # Vehicle arrives too late
                arr > t_k_start + max_wait_time + ε && continue

                # Forward delay feasibility: check on-board orders can still make their dropoffs
                feasible = true
                for j in 0:(n - 1)
                    (lbl.picked  >> j) & UInt64(1) == UInt64(0) && continue  # not on board
                    (lbl.dropped >> j) & UInt64(1) == UInt64(1) && continue  # already dropped

                    j_dropoff  = lbl.chosen_dropoff[j + 1]
                    j_pickup   = lbl.chosen_pickup[j + 1]
                    min_invehi = (arr + get_routing_cost(data, p_id, j_dropoff)) - lbl.board_abstime[j + 1]
                    direct_j   = get_routing_cost(data, j_pickup, j_dropoff)

                    if min_invehi - direct_j > max_delay_time + ε
                        feasible = false; break
                    end
                    if direct_j > 0.0 && min_invehi / direct_j > 1.0 + max_delay_ratio + ε
                        feasible = false; break
                    end
                end
                feasible || continue

                new_bct      = copy(lbl.board_abstime);  new_bct[k + 1] = arr
                new_cp       = copy(lbl.chosen_pickup);  new_cp[k + 1]  = p_id
                new_cd       = copy(lbl.chosen_dropoff); new_cd[k + 1]  = d_id

                child = TimedRouteLabel(
                    p_id, arr, lbl.passengers + order_k.demand,
                    lbl.picked | (UInt64(1) << k), lbl.dropped, idx - 1,
                    new_bct, new_cp, new_cd
                )
                push!(labels, child)
            end
        end
    end
end


"""
Reconstruct the station sequence for a complete label and merge into `routes_map`.
Collapses consecutive same-station stops; rejects routes with repeated stations.
"""
function _timed_record_route!(
    labels            :: Vector{TimedRouteLabel},
    terminal_idx      :: Int,
    orders            :: Vector{_TimedOrder},
    data              :: StationSelectionData,
    station_id_to_idx :: Dict{Int,Int},
    capacity          :: Int,
    routes_map        :: Dict{Tuple{Vector{Int},UInt64}, TimedRouteData},
    next_id           :: Ref{Int}
)
    # Follow parent chain to reconstruct stop sequence
    path = Int[]
    cur = terminal_idx
    while cur != 0
        push!(path, cur)
        cur = labels[cur].parent
    end
    reverse!(path)

    # Build station sequence; collapse consecutive same-station stops
    raw = [labels[i].station for i in path]
    stations = [raw[1]]
    for i in 2:length(raw)
        raw[i] != stations[end] && push!(stations, raw[i])
    end

    # Reject routes that revisit any station
    length(unique(stations)) == length(stations) || return

    # Travel time = sum of consecutive routing costs (pure driving time)
    travel_time = length(stations) < 2 ? 0.0 :
        sum(get_routing_cost(data, stations[i], stations[i + 1])
            for i in 1:length(stations) - 1)

    lbl_term = labels[terminal_idx]
    picked   = lbl_term.picked
    n        = length(orders)

    # od_capacity: served (pickup_id, dropoff_id) → vehicle capacity C
    od_cap = Dict{Tuple{Int,Int}, Int}()
    for j in 0:(n - 1)
        (picked >> j) & UInt64(1) == UInt64(1) || continue
        p_id = lbl_term.chosen_pickup[j + 1]
        d_id = lbl_term.chosen_dropoff[j + 1]
        od_cap[(p_id, d_id)] = capacity
    end

    # alpha: (t_id, (j_idx, k_idx)) → actual passengers carried on this leg
    alpha = Dict{Tuple{Int, Tuple{Int,Int}}, Int}()
    for j in 0:(n - 1)
        (picked >> j) & UInt64(1) == UInt64(1) || continue
        order_j = orders[j + 1]
        p_id = lbl_term.chosen_pickup[j + 1]
        d_id = lbl_term.chosen_dropoff[j + 1]
        j_idx = station_id_to_idx[p_id]
        k_idx = station_id_to_idx[d_id]
        key = (order_j.t_id, (j_idx, k_idx))
        alpha[key] = get(alpha, key, 0) + order_j.demand
    end

    # Deduplication: (station sequence, served order set)
    dup_key = (stations, picked)
    haskey(routes_map, dup_key) && return

    next_id[] += 1
    route = RouteData(next_id[], stations, travel_time, od_cap)
    routes_map[dup_key] = TimedRouteData(route, alpha)
end
