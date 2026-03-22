"""
Non-temporal route generation for RouteAlphaCapacityModel and RouteVehicleCapacityModel.

Generates vehicle routes without time windows: orders are grouped only by (origin, destination)
within a scenario. A route can serve multiple OD pairs in any order, subject to vehicle
capacity and detour constraints.

Implements a BFS label-setting DP where each label represents a partial route
(vehicle at some station after some cumulative travel time, some passengers on board).
No time-window gate is applied when picking up orders.
"""

export NonTimedRouteData
export generate_routes_from_orders
export generate_simple_routes_from_orders


"""
    NonTimedRouteData

A vehicle route with per-VBS-leg actual passenger counts (no time-window index).

# Fields
- `route::RouteData`: station sequence and total routing travel time
- `alpha::Dict{Tuple{Int,Int},Int}`: (pickup_idx, dropoff_idx) → actual passengers on that leg

`alpha[(j_idx, k_idx)]` is the total passengers this route carries from VBS j to VBS k.
Used directly in the route-covering constraint for RouteAlphaCapacityModel.
"""
struct NonTimedRouteData
    route  :: RouteData
    alpha  :: Dict{Tuple{Int,Int}, Int}
end


# ─────────────────────────────────────────────────────────────────────────────
# Internal types
# ─────────────────────────────────────────────────────────────────────────────

"""
    _NonTimedOrder

Internal: one demand aggregate — a unique (o_id, d_id) combination in a scenario.

# Fields
- `o_id`, `d_id`:    geographic origin/destination station IDs
- `demand`:          number of passengers in this group
- `feasible_vbs`:    (pickup_station_id, dropoff_station_id) VBS pairs this order can use
"""
struct _NonTimedOrder
    o_id         :: Int
    d_id         :: Int
    demand       :: Int
    feasible_vbs :: Vector{Tuple{Int,Int}}
end


"""
    _NonTimedLabel

BFS label for non-temporal route generation.

# Fields
- `station`:         current station (last stop visited)
- `cum_time`:        cumulative travel time from route start (seconds)
- `passengers`:      current vehicle load (sum of demands of on-board orders)
- `picked`:          bit j set ⟺ order j+1 has been picked up
- `dropped`:         bit j set ⟺ order j+1 has been dropped off
- `parent`:          1-based index into labels array; 0 = root
- `n_stations`:      number of distinct sequential stops visited so far (including start)
- `board_cumtime`:   length-n; entry j = cum_time when order j was picked up (Inf = not yet)
- `chosen_pickup`:   length-n; entry j = pickup VBS station ID for order j (0 = not yet)
- `chosen_dropoff`:  length-n; entry j = dropoff VBS station ID for order j (0 = not yet)
"""
struct _NonTimedLabel
    station       :: Int
    cum_time      :: Float64
    passengers    :: Int
    picked        :: UInt64
    dropped       :: UInt64
    parent        :: Int
    n_stations    :: Int
    board_cumtime :: Vector{Float64}
    chosen_pickup :: Vector{Int}
    chosen_dropoff:: Vector{Int}
end


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

"""
    generate_routes_from_orders(
        orders::Vector{_NonTimedOrder},
        data::StationSelectionData,
        station_id_to_array_idx::Dict{Int,Int};
        vehicle_capacity::Int = 4,
        max_detour_time::Float64,
        max_detour_ratio::Float64,
        max_stations_visited::Int = typemax(Int),
        max_labels::Int = 400_000
    ) -> Vector{NonTimedRouteData}

Generate feasible vehicle routes for a single scenario (no time windows).

Routes can serve orders in any sequence subject to vehicle capacity and detour constraints.
Unlike temporal BFS, there is no time-window gate when picking up orders.

# Arguments
- `orders`: demand aggregates (unique (o,d) combinations) with feasible VBS pairs
- `data`: problem data with routing costs
- `station_id_to_array_idx`: station ID → 1-based array index mapping
- `vehicle_capacity`: max total passengers on board at any time
- `max_detour_time`: max extra in-vehicle seconds vs direct trip; always enforced
- `max_detour_ratio`: max ratio `in_vehicle/direct - 1`; always enforced

# Returns
`Vector{NonTimedRouteData}` — distinct feasible routes found by the BFS.
Routes with identical (station_sequence, served_order_set) are deduplicated.

# Limits
Supports at most 63 orders per call (UInt64 bitmask). Throws an error if exceeded.
"""
function generate_routes_from_orders(
    orders                  :: Vector{_NonTimedOrder},
    data                    :: StationSelectionData,
    station_id_to_array_idx :: Dict{Int,Int};
    vehicle_capacity        :: Int = 4,
    max_detour_time         :: Float64,
    max_detour_ratio        :: Float64,
    max_stations_visited    :: Int = typemax(Int),
    max_labels              :: Int = 400_000
)::Vector{NonTimedRouteData}
    has_routing_costs(data) || error(
        "generate_routes_from_orders requires routing costs " *
        "(data.routing_costs must not be nothing)"
    )
    vehicle_capacity > 0 || throw(ArgumentError("vehicle_capacity must be positive"))
    isempty(orders) && return NonTimedRouteData[]

    n = length(orders)
    n <= 63 || error(
        "generate_routes_from_orders supports at most 63 orders per scenario; " *
        "got $n. Consider reducing scenario duration."
    )

    # (station_sequence, picked_bitmask) → NonTimedRouteData
    routes_map = Dict{Tuple{Vector{Int},UInt64}, NonTimedRouteData}()
    next_id    = Ref(0)

    labels = _NonTimedLabel[]
    sizehint!(labels, max(200, 20 * n))

    # ── Initialise: one root label per (order j, feasible VBS pair (p, d)) ────
    for j in 1:n
        order = orders[j]
        isempty(order.feasible_vbs) && continue

        bit_j = UInt64(1) << (j - 1)

        for (p_id, d_id) in order.feasible_vbs
            bct = fill(Inf, n);  bct[j] = 0.0
            cp  = zeros(Int, n); cp[j]  = p_id
            cd  = zeros(Int, n); cd[j]  = d_id
            push!(labels, _NonTimedLabel(
                p_id, 0.0, order.demand,
                bit_j, UInt64(0), 0, 1,
                bct, cp, cd
            ))
        end
    end

    record_fn! = (lbls, tidx) -> _nontimed_record_route!(
        lbls, tidx, orders, data, station_id_to_array_idx, vehicle_capacity,
        routes_map, next_id
    )

    _nontimed_bfs_core!(
        labels, orders, data, station_id_to_array_idx,
        vehicle_capacity, max_detour_time, max_detour_ratio,
        record_fn!, max_labels, max_stations_visited
    )

    routes_sorted = sort!(collect(values(routes_map)), by = r -> r.route.id)
    return [NonTimedRouteData(
                RouteData(i, r.route.station_ids, r.route.travel_time, r.route.od_capacity),
                r.alpha)
            for (i, r) in enumerate(routes_sorted)]
end


"""
    generate_simple_routes_from_orders(
        orders::Vector{_NonTimedOrder},
        data::StationSelectionData,
        station_id_to_array_idx::Dict{Int,Int};
        vehicle_capacity::Int = 4,
        max_detour_time::Float64,
        max_detour_ratio::Float64,
        max_stations_visited::Int = typemax(Int),
        max_labels::Int = 400_000
    ) -> Vector{RouteData}

Generate feasible vehicle routes (plain RouteData, no alpha dict) for a single scenario.

Used by RouteVehicleCapacityModel where route loading is handled by explicit integer
variables (d, α, θ) rather than pre-computed alpha counts.

Same BFS logic as `generate_routes_from_orders` but records only the station sequence
and travel time; `od_capacity` is stored as an empty dict.

# Limits
Supports at most 63 orders per call (UInt64 bitmask). Throws an error if exceeded.
"""
function generate_simple_routes_from_orders(
    orders                  :: Vector{_NonTimedOrder},
    data                    :: StationSelectionData,
    station_id_to_array_idx :: Dict{Int,Int};
    vehicle_capacity        :: Int = 4,
    max_detour_time         :: Float64,
    max_detour_ratio        :: Float64,
    max_stations_visited    :: Int = typemax(Int),
    max_labels              :: Int = 400_000
)::Vector{RouteData}
    has_routing_costs(data) || error(
        "generate_simple_routes_from_orders requires routing costs " *
        "(data.routing_costs must not be nothing)"
    )
    vehicle_capacity > 0 || throw(ArgumentError("vehicle_capacity must be positive"))
    isempty(orders) && return RouteData[]

    n = length(orders)
    n <= 63 || error(
        "generate_simple_routes_from_orders supports at most 63 orders per scenario; " *
        "got $n. Consider reducing scenario duration."
    )

    # (station_sequence, picked_bitmask) → RouteData
    routes_map = Dict{Tuple{Vector{Int},UInt64}, RouteData}()
    next_id    = Ref(0)

    labels = _NonTimedLabel[]
    sizehint!(labels, max(200, 20 * n))

    # ── Initialise: one root label per (order j, feasible VBS pair (p, d)) ────
    for j in 1:n
        order = orders[j]
        isempty(order.feasible_vbs) && continue

        bit_j = UInt64(1) << (j - 1)

        for (p_id, d_id) in order.feasible_vbs
            bct = fill(Inf, n);  bct[j] = 0.0
            cp  = zeros(Int, n); cp[j]  = p_id
            cd  = zeros(Int, n); cd[j]  = d_id
            push!(labels, _NonTimedLabel(
                p_id, 0.0, order.demand,
                bit_j, UInt64(0), 0, 1,
                bct, cp, cd
            ))
        end
    end

    record_fn! = (lbls, tidx) -> _nontimed_record_simple_route!(
        lbls, tidx, data, routes_map, next_id
    )

    _nontimed_bfs_core!(
        labels, orders, data, station_id_to_array_idx,
        vehicle_capacity, max_detour_time, max_detour_ratio,
        record_fn!, max_labels, max_stations_visited
    )

    routes_sorted = sort!(collect(values(routes_map)), by = r -> r.id)
    return [RouteData(i, r.station_ids, r.travel_time, r.od_capacity)
            for (i, r) in enumerate(routes_sorted)]
end


# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ─────────────────────────────────────────────────────────────────────────────

"""
Returns the dominance key for a non-timed label: same key ⟺ same partial-route state
(same location, same orders served/on-board, same VBS assignments).
"""
function _nt_dom_key(lbl::_NonTimedLabel)
    return (lbl.station, lbl.picked, lbl.dropped,
            Tuple(lbl.chosen_pickup), Tuple(lbl.chosen_dropoff))
end

"""
Returns true if `lbl1` dominates `lbl2` for the same partial-route state.
lbl1 dominates lbl2 iff lbl1 arrives no later (cum_time) AND has no more accumulated
in-vehicle time for every on-board passenger.
"""
function _nt_dominates(lbl1::_NonTimedLabel, lbl2::_NonTimedLabel, n::Int, ε::Float64)
    lbl1.cum_time > lbl2.cum_time + ε && return false
    lbl1.n_stations > lbl2.n_stations && return false
    on_board = lbl1.picked & ~lbl1.dropped
    for j in 0:(n - 1)
        (on_board >> j) & UInt64(1) == UInt64(0) && continue
        tov1 = lbl1.cum_time - lbl1.board_cumtime[j + 1]
        tov2 = lbl2.cum_time - lbl2.board_cumtime[j + 1]
        tov1 > tov2 + ε && return false
    end
    return true
end

"""
Push `child` into `labels` unless it is dominated by an existing live label with the
same key. Also marks any existing labels that child dominates as dead.
"""
function _nt_push_if_not_dominated!(
    child    :: _NonTimedLabel,
    labels   :: Vector{_NonTimedLabel},
    alive    :: BitVector,
    dom_dict :: Dict,
    n        :: Int,
    ε        :: Float64
) :: Bool
    key      = _nt_dom_key(child)
    existing = get(dom_dict, key, nothing)

    if existing !== nothing
        for ex_idx in existing
            !alive[ex_idx] && continue
            _nt_dominates(labels[ex_idx], child, n, ε) && return false
        end
        for ex_idx in existing
            !alive[ex_idx] && continue
            if _nt_dominates(child, labels[ex_idx], n, ε)
                alive[ex_idx] = false
            end
        end
        filter!(i -> alive[i], existing)
        push!(existing, length(labels) + 1)
    else
        dom_dict[key] = [length(labels) + 1]
    end

    push!(labels, child)
    push!(alive, true)
    return true
end


"""
BFS label-setting core. Processes labels in push order; extends each by one
dropoff or pickup action. Calls `record_fn!(labels, terminal_idx)` for complete labels.

Shared by `generate_routes_from_orders` and `generate_simple_routes_from_orders`.
"""
function _nontimed_bfs_core!(
    labels               :: Vector{_NonTimedLabel},
    orders               :: Vector{_NonTimedOrder},
    data                 :: StationSelectionData,
    station_id_to_idx    :: Dict{Int,Int},
    vehicle_capacity     :: Int,
    max_detour_time      :: Float64,
    max_detour_ratio     :: Float64,
    record_fn!           :: Function,
    max_labels           :: Int,
    max_stations_visited :: Int
)
    n = length(orders)
    ε = 1e-9

    alive    = trues(length(labels))
    dom_dict = Dict{Any, Vector{Int}}()
    for i in eachindex(labels)
        key = _nt_dom_key(labels[i])
        existing = get(dom_dict, key, nothing)
        if existing === nothing
            dom_dict[key] = [i]
        else
            push!(existing, i)
        end
    end

    idx = 1
    while idx <= length(labels)
        if idx > max_labels
            println("    BFS: label limit ($max_labels explored) reached; stopping early")
            flush(stdout)
            break
        end
        if idx % 50_000 == 1 && idx > 1
            println("    BFS: processed $idx / $(length(labels)) labels so far")
            flush(stdout)
        end

        lbl = labels[idx]
        idx += 1
        !alive[idx - 1] && continue

        # ── Stage 1: drop off on-board passengers ─────────────────────────────
        for j in 0:(n - 1)
            (lbl.picked  >> j) & UInt64(1) == UInt64(0) && continue
            (lbl.dropped >> j) & UInt64(1) == UInt64(1) && continue

            d_id = lbl.chosen_dropoff[j + 1]
            p_id = lbl.chosen_pickup[j + 1]
            arr  = lbl.cum_time + get_routing_cost(data, lbl.station, d_id)

            in_vehicle = arr - lbl.board_cumtime[j + 1]
            direct     = get_routing_cost(data, p_id, d_id)

            if in_vehicle - direct > max_detour_time + ε
                continue
            end
            if direct > 0.0 && in_vehicle / direct > 1.0 + max_detour_ratio + ε
                continue
            end

            new_dropped = lbl.dropped | (UInt64(1) << j)
            new_ns_drop = (d_id != lbl.station) ? lbl.n_stations + 1 : lbl.n_stations
            new_ns_drop > max_stations_visited && continue
            child = _NonTimedLabel(
                d_id, arr, lbl.passengers - orders[j + 1].demand,
                lbl.picked, new_dropped, idx - 1, new_ns_drop,
                lbl.board_cumtime, lbl.chosen_pickup, lbl.chosen_dropoff
            )
            pushed = _nt_push_if_not_dominated!(child, labels, alive, dom_dict, n, ε)

            if pushed && child.picked == child.dropped
                record_fn!(labels, length(labels))
            end
        end

        # ── Stage 2: pick up new orders ───────────────────────────────────────
        lbl.passengers == 0 && continue
        for k in 0:(n - 1)
            (lbl.picked >> k) & UInt64(1) == UInt64(1) && continue

            order_k = orders[k + 1]
            lbl.passengers + order_k.demand > vehicle_capacity && continue

            for (p_id, d_id) in order_k.feasible_vbs
                arr = lbl.cum_time + get_routing_cost(data, lbl.station, p_id)

                # Forward delay feasibility: check on-board orders can still make their dropoffs
                feasible = true
                for j in 0:(n - 1)
                    (lbl.picked  >> j) & UInt64(1) == UInt64(0) && continue
                    (lbl.dropped >> j) & UInt64(1) == UInt64(1) && continue

                    j_dropoff  = lbl.chosen_dropoff[j + 1]
                    j_pickup   = lbl.chosen_pickup[j + 1]
                    min_invehi = (arr + get_routing_cost(data, p_id, j_dropoff)) - lbl.board_cumtime[j + 1]
                    direct_j   = get_routing_cost(data, j_pickup, j_dropoff)

                    if min_invehi - direct_j > max_detour_time + ε
                        feasible = false; break
                    end
                    if direct_j > 0.0 && min_invehi / direct_j > 1.0 + max_detour_ratio + ε
                        feasible = false; break
                    end
                end
                feasible || continue

                new_ns_pick = (p_id != lbl.station) ? lbl.n_stations + 1 : lbl.n_stations
                new_ns_pick > max_stations_visited && continue

                new_bct = copy(lbl.board_cumtime);  new_bct[k + 1] = arr
                new_cp  = copy(lbl.chosen_pickup);  new_cp[k + 1]  = p_id
                new_cd  = copy(lbl.chosen_dropoff); new_cd[k + 1]  = d_id

                child = _NonTimedLabel(
                    p_id, arr, lbl.passengers + order_k.demand,
                    lbl.picked | (UInt64(1) << k), lbl.dropped, idx - 1, new_ns_pick,
                    new_bct, new_cp, new_cd
                )
                _nt_push_if_not_dominated!(child, labels, alive, dom_dict, n, ε)
            end
        end
    end
end


"""
Reconstruct the station sequence for a complete non-timed label and merge into `routes_map`.
Collapses consecutive same-station stops; rejects routes with repeated stations.
Records a NonTimedRouteData (with alpha passenger counts).
"""
function _nontimed_record_route!(
    labels            :: Vector{_NonTimedLabel},
    terminal_idx      :: Int,
    orders            :: Vector{_NonTimedOrder},
    data              :: StationSelectionData,
    station_id_to_idx :: Dict{Int,Int},
    capacity          :: Int,
    routes_map        :: Dict{Tuple{Vector{Int},UInt64}, NonTimedRouteData},
    next_id           :: Ref{Int}
)
    path = Int[]
    cur = terminal_idx
    while cur != 0
        push!(path, cur)
        cur = labels[cur].parent
    end
    reverse!(path)

    raw = [labels[i].station for i in path]
    stations = [raw[1]]
    for i in 2:length(raw)
        raw[i] != stations[end] && push!(stations, raw[i])
    end

    length(unique(stations)) == length(stations) || return

    travel_time = length(stations) < 2 ? 0.0 :
        sum(get_routing_cost(data, stations[i], stations[i + 1])
            for i in 1:length(stations) - 1)

    lbl_term = labels[terminal_idx]
    picked   = lbl_term.picked
    n        = length(orders)

    od_cap = Dict{Tuple{Int,Int}, Int}()
    for j in 0:(n - 1)
        (picked >> j) & UInt64(1) == UInt64(1) || continue
        p_id = lbl_term.chosen_pickup[j + 1]
        d_id = lbl_term.chosen_dropoff[j + 1]
        od_cap[(p_id, d_id)] = capacity
    end

    # alpha: (j_idx, k_idx) → actual passengers carried on this leg
    alpha = Dict{Tuple{Int,Int}, Int}()
    for j in 0:(n - 1)
        (picked >> j) & UInt64(1) == UInt64(1) || continue
        order_j = orders[j + 1]
        p_id = lbl_term.chosen_pickup[j + 1]
        d_id = lbl_term.chosen_dropoff[j + 1]
        j_idx = station_id_to_idx[p_id]
        k_idx = station_id_to_idx[d_id]
        key = (j_idx, k_idx)
        alpha[key] = get(alpha, key, 0) + order_j.demand
    end

    dup_key = (stations, picked)
    haskey(routes_map, dup_key) && return

    next_id[] += 1
    route = RouteData(next_id[], stations, travel_time, od_cap)
    routes_map[dup_key] = NonTimedRouteData(route, alpha)
end


"""
Reconstruct the station sequence for a complete non-timed label and merge into `routes_map`.
Records a plain RouteData with empty od_capacity (used by RouteVehicleCapacityModel).
"""
function _nontimed_record_simple_route!(
    labels       :: Vector{_NonTimedLabel},
    terminal_idx :: Int,
    data         :: StationSelectionData,
    routes_map   :: Dict{Tuple{Vector{Int},UInt64}, RouteData},
    next_id      :: Ref{Int}
)
    path = Int[]
    cur = terminal_idx
    while cur != 0
        push!(path, cur)
        cur = labels[cur].parent
    end
    reverse!(path)

    raw = [labels[i].station for i in path]
    stations = [raw[1]]
    for i in 2:length(raw)
        raw[i] != stations[end] && push!(stations, raw[i])
    end

    length(unique(stations)) == length(stations) || return

    travel_time = length(stations) < 2 ? 0.0 :
        sum(get_routing_cost(data, stations[i], stations[i + 1])
            for i in 1:length(stations) - 1)

    lbl_term = labels[terminal_idx]
    picked   = lbl_term.picked

    dup_key = (stations, picked)
    haskey(routes_map, dup_key) && return

    next_id[] += 1
    route = RouteData(next_id[], stations, travel_time, Dict{Tuple{Int,Int},Int}())
    routes_map[dup_key] = route
end
