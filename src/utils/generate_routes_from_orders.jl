"""
Route generation from actual demand OD pairs for TwoStageRouteModel.

Unlike `generate_routes`, which enumerates all station-pair combinations regardless of
demand, this function only generates routes that serve the observed VBS-level OD pairs
and can enforce per-passenger detour limits.

Implements a BFS label-setting DP: each label represents a partial route (vehicle at some
station, some passengers on board, some dropped off). Extensions — pickup or dropoff of one
order — are checked eagerly so infeasible branches are pruned immediately.
"""

export generate_routes_from_orders

"""
    RouteLabel

Internal label for BFS label-setting route enumeration.

Fields:
- `station`:       current station (last stop visited)
- `cum_time`:      cumulative travel time from route start
- `passengers`:    current vehicle load
- `picked`:        bit j set ⟺ order j+1 has been picked up
- `dropped`:       bit j set ⟺ order j+1 has been dropped off
- `parent`:        1-based index into labels array; 0 = root
- `board_cumtime`: length-n vector; entry j = cum_time when order j was picked up (Inf if not yet)
"""
struct RouteLabel
    station       :: Int
    cum_time      :: Float64
    passengers    :: Int
    picked        :: UInt64
    dropped       :: UInt64
    parent        :: Int
    board_cumtime :: Vector{Float64}
end

"""
    generate_routes_from_orders(
        od_pairs::Vector{Tuple{Int,Int}},
        data::StationSelectionData;
        vehicle_capacity::Int = 4,
        max_detour_time::Union{Float64,Nothing} = nothing,
        max_detour_ratio::Union{Float64,Nothing} = nothing,
        max_route_travel_time::Union{Float64,Nothing} = nothing
    ) -> Vector{RouteData}

Enumerate feasible vehicle routes that serve subsets of the given VBS-level OD pairs.

Uses a BFS label-setting DP over partial routes. All OD pairs are considered simultaneously;
infeasible branches are pruned eagerly at each extension. Routes with repeated station visits
are discarded. Each distinct stop sequence that passes the optional detour and route-time
filters is returned as a `RouteData`.

Routes with identical stop sequences (arising from different OD-pair subsets) are
merged: their `od_capacity` dicts are unioned.

# Detour filters
- `max_detour_time`: reject if `in_route_time - direct_time > max_detour_time` for any passenger
- `max_detour_ratio`: reject if `in_route_time / direct_time > 1 + max_detour_ratio` for any passenger

Requires routing costs (`has_routing_costs(data)` must be true).
"""
function generate_routes_from_orders(
    od_pairs::Vector{Tuple{Int,Int}},
    data::StationSelectionData;
    vehicle_capacity::Int = 4,
    max_detour_time::Union{Float64,Nothing} = nothing,
    max_detour_ratio::Union{Float64,Nothing} = nothing,
    max_route_travel_time::Union{Float64,Nothing} = nothing
)::Vector{RouteData}
    has_routing_costs(data) || error(
        "generate_routes_from_orders requires routing costs (data.routing_costs must not be nothing)"
    )
    vehicle_capacity > 0 || throw(ArgumentError("vehicle_capacity must be positive"))
    isempty(od_pairs) && return RouteData[]

    n = length(od_pairs)
    routes_map = Dict{Vector{Int}, RouteData}()
    next_id = Ref(0)

    # Initialize: one root label per order — vehicle starts at pickup station with that passenger on board
    labels = RouteLabel[]
    sizehint!(labels, max(100, 10 * n))
    for j in 1:n
        o_j, _ = od_pairs[j]
        bct = fill(Inf, n)
        bct[j] = 0.0
        push!(labels, RouteLabel(o_j, 0.0, 1, UInt64(1) << (j - 1), UInt64(0), 0, bct))
    end

    _grf_run_label_setting!(
        labels, od_pairs, data, vehicle_capacity,
        max_detour_time, max_detour_ratio, max_route_travel_time,
        routes_map, next_id
    )

    routes_sorted = sort!(collect(values(routes_map)), by = r -> r.id)
    return [RouteData(i, r.station_ids, r.travel_time, r.od_capacity)
            for (i, r) in enumerate(routes_sorted)]
end

# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers (not exported)
# ─────────────────────────────────────────────────────────────────────────────

"""
BFS label-setting main loop. Processes labels in push order; extends each by one
pickup or dropoff action. Calls `_grf_record_route!` whenever a complete label is found.
"""
function _grf_run_label_setting!(
    labels::Vector{RouteLabel},
    od_pairs::Vector{Tuple{Int,Int}},
    data::StationSelectionData,
    vehicle_capacity::Int,
    max_detour_time::Union{Float64,Nothing},
    max_detour_ratio::Union{Float64,Nothing},
    max_route_travel_time::Union{Float64,Nothing},
    routes_map::Dict{Vector{Int}, RouteData},
    next_id::Ref{Int}
)
    n = length(od_pairs)
    ε = 1e-9

    idx = 1
    while idx <= length(labels)
        lbl = labels[idx]

        # ── Stage 1: try dropping off each on-board passenger ─────────────────
        for j in 0:(n - 1)
            (lbl.picked  >> j) & UInt64(1) == UInt64(0) && continue   # not picked up
            (lbl.dropped >> j) & UInt64(1) == UInt64(1) && continue   # already dropped

            o_j, d_j = od_pairs[j + 1]
            travel = get_routing_cost(data, lbl.station, d_j)
            arr    = lbl.cum_time + travel

            !isnothing(max_route_travel_time) && arr > max_route_travel_time && continue

            in_route = arr - lbl.board_cumtime[j + 1]
            direct   = get_routing_cost(data, o_j, d_j)

            if !isnothing(max_detour_time) && in_route - direct > max_detour_time + ε
                continue
            end
            if !isnothing(max_detour_ratio) && direct > 0.0 &&
               in_route / direct > 1.0 + max_detour_ratio + ε
                continue
            end

            new_dropped = lbl.dropped | (UInt64(1) << j)
            child = RouteLabel(d_j, arr, lbl.passengers - 1,
                               lbl.picked, new_dropped, idx, lbl.board_cumtime)
            push!(labels, child)

            # Complete route: all picked-up passengers have been dropped off
            if child.picked == child.dropped
                _grf_record_route!(labels, length(labels), od_pairs,
                                   vehicle_capacity, routes_map, next_id)
            end
        end

        # ── Stage 2: try picking up each unassigned order ─────────────────────
        if lbl.passengers < vehicle_capacity
            for j in 0:(n - 1)
                (lbl.picked >> j) & UInt64(1) == UInt64(1) && continue  # already picked up

                o_j, _ = od_pairs[j + 1]
                travel = get_routing_cost(data, lbl.station, o_j)
                arr    = lbl.cum_time + travel

                !isnothing(max_route_travel_time) && arr > max_route_travel_time && continue

                new_bct = copy(lbl.board_cumtime)
                new_bct[j + 1] = arr
                child = RouteLabel(o_j, arr, lbl.passengers + 1,
                                   lbl.picked | (UInt64(1) << j), lbl.dropped, idx, new_bct)
                push!(labels, child)
            end
        end

        idx += 1
    end
end

"""
Reconstruct the station sequence for a complete label, then merge into `routes_map`.
Collapses consecutive same-station stops; rejects routes with repeated stations.
"""
function _grf_record_route!(
    labels::Vector{RouteLabel},
    terminal_idx::Int,
    od_pairs::Vector{Tuple{Int,Int}},
    capacity::Int,
    routes_map::Dict{Vector{Int}, RouteData},
    next_id::Ref{Int}
)
    # Follow parent chain to reconstruct the stop sequence
    path = Int[]
    cur = terminal_idx
    while cur != 0
        push!(path, cur)
        cur = labels[cur].parent
    end
    reverse!(path)

    # Build station sequence and collapse consecutive same-station stops
    raw = [labels[i].station for i in path]
    stations = [raw[1]]
    for i in 2:length(raw)
        raw[i] != stations[end] && push!(stations, raw[i])
    end

    # Reject routes that revisit any station (e.g. [L,T,R,T])
    length(unique(stations)) == length(stations) || return

    total_time = labels[terminal_idx].cum_time

    # od_capacity: every OD pair whose order was picked up in this route
    picked = labels[terminal_idx].picked
    n = length(od_pairs)
    od_cap = Dict{Tuple{Int,Int},Int}()
    for j in 0:(n - 1)
        (picked >> j) & UInt64(1) == UInt64(1) || continue
        od_cap[od_pairs[j + 1]] = capacity
    end

    # Insert or merge into routes_map
    if haskey(routes_map, stations)
        existing = routes_map[stations]
        routes_map[stations] = RouteData(
            existing.id,
            existing.station_ids,
            existing.travel_time,
            merge(existing.od_capacity, od_cap)
        )
    else
        next_id[] += 1
        routes_map[stations] = RouteData(next_id[], stations, total_time, od_cap)
    end
end
