"""
Shared helpers for the `test_cases/` synthetic benchmark generator family
(`base_middle_zone.jl`, `test1_vehicle.jl` .. `test6_bidirectional.jl`).

These generators were ported from standalone scripts in the sibling
`scripts/test_case_generation/` repo that each independently redefined an
identical set of constants/functions (coordinate conversion, Poisson draw,
segment-DataFrame construction, ...). Everything here is prefixed `tc_`/`TC_`
so it can never collide with a per-test-case file's own `T{N}_`-prefixed
symbols, and stays unexported (internal implementation detail).
"""

# ---------------------------------------------------------------------------
# Coordinate conversion — identical reference point across every source script
# ---------------------------------------------------------------------------

const TC_REF_LON        = 113.0
const TC_REF_LAT        = 28.0
const TC_LON_KM_PER_DEG = 98.29
const TC_LAT_KM_PER_DEG = 110.57

tc_to_lon(x_km::Real) = TC_REF_LON + x_km / TC_LON_KM_PER_DEG
tc_to_lat(y_km::Real) = TC_REF_LAT + y_km / TC_LAT_KM_PER_DEG

# ---------------------------------------------------------------------------
# Distance / time helpers
# ---------------------------------------------------------------------------

tc_euclid_km(x1, y1, x2, y2) = sqrt((x1 - x2)^2 + (y1 - y2)^2)
tc_euclid_m(x1, y1, x2, y2)  = 1000.0 * tc_euclid_km(x1, y1, x2, y2)
tc_veh_time_s(x1, y1, x2, y2; veh_speed::Float64 = 8.0)   = tc_euclid_m(x1, y1, x2, y2) / veh_speed
tc_walk_time_s(x1, y1, x2, y2; walk_speed::Float64 = 1.4) = tc_euclid_m(x1, y1, x2, y2) / walk_speed

# ---------------------------------------------------------------------------
# Poisson draw — same approximation algorithm in every source script; using
# test6's slightly more defensive `lam <= 0 → 0` guard as the canonical form.
# ---------------------------------------------------------------------------

function tc_poisson_draw(lam::Real, rng::AbstractRNG)::Int
    lam <= 0.0 && return 0
    L = exp(-Float64(lam))
    k, p = 0, 1.0
    while p > L
        k += 1
        p *= rand(rng)
    end
    return k - 1
end

# ---------------------------------------------------------------------------
# Shared zone-origin offsets — byte-identical in tests 2-6
# ---------------------------------------------------------------------------

const TC_ZONE_OFFSETS_KM = [
    (dx = -0.20, dy = +0.15, name = "p1"),
    (dx = +0.20, dy = +0.15, name = "p2"),
    (dx = -0.15, dy = -0.15, name = "p3"),
    (dx = +0.15, dy = -0.15, name = "p4"),
]

# ---------------------------------------------------------------------------
# station.csv / segment.csv builders — identical shape in every source script
# ---------------------------------------------------------------------------

"""
    tc_build_station_df(all_stations) -> DataFrame

Note: unlike the original scripts (which never persisted `role` to
station.csv), this port carries a `:role` column through — this is what
lets hypothesis tests look up "the station currently tagged M" instead of
hardcoding IDs, since IDs shift between variants.
"""
function tc_build_station_df(all_stations)::DataFrame
    DataFrame(
        station_id   = [s.id           for s in all_stations],
        station_name = [s.name         for s in all_stations],
        station_lon  = [tc_to_lon(s.x_km) for s in all_stations],
        station_lat  = [tc_to_lat(s.y_km) for s in all_stations],
        role         = [s.role         for s in all_stations],
    )
end

function tc_build_segment_df(all_stations; veh_speed::Float64 = 8.0)::DataFrame
    rows = NamedTuple[]
    seg_id = 1
    for si in all_stations, sj in all_stations
        si.id == sj.id && continue
        dist_m = tc_euclid_m(si.x_km, si.y_km, sj.x_km, sj.y_km)
        push!(rows, (
            id           = seg_id,
            from_station = si.id,
            to_station   = sj.id,
            seg_dist     = round(dist_m, digits = 2),
            seg_time     = round(dist_m / veh_speed, digits = 3),
        ))
        seg_id += 1
    end
    DataFrame(rows)
end

# ---------------------------------------------------------------------------
# Demand-generation helpers
# ---------------------------------------------------------------------------

function tc_rand_timestamps(n::Int, rng::AbstractRNG, window_start_dt::DateTime, window_sec::Int)::Vector{DateTime}
    offsets = sort(rand(rng, n) .* window_sec)
    return [window_start_dt + Millisecond(round(Int, t * 1000)) for t in offsets]
end

"""
    tc_push_order!(order_rows, order_id, origin_id, dest_id, ts; region_id=1, pax_num=1)

Appends one order row (in the shared 12-column order.csv schema) to
`order_rows` and increments the mutable `order_id` counter (a `Ref{Int}`).
"""
function tc_push_order!(
    order_rows::Vector,
    order_id::Base.RefValue{Int},
    origin_id::Int,
    dest_id::Int,
    ts::DateTime;
    region_id::Int = 1,
    pax_num::Int = 1,
)
    push!(order_rows, (
        order_id                = order_id[],
        region_id               = region_id,
        pax_num                 = pax_num,
        order_time               = Dates.format(ts, "yyyy-mm-dd HH:MM:SS"),
        origin_station_id       = origin_id,
        destination_station_id  = dest_id,
        status                  = 1,
        vehicle_id               = "",
        pick_up_time             = "",
        drop_off_time            = "",
        pick_up_early            = "",
        drop_off_early           = "",
    ))
    order_id[] += 1
    return nothing
end

# ---------------------------------------------------------------------------
# StationSelectionData conversion — identical recipe for every generator
# ---------------------------------------------------------------------------

"""
    tc_stations_frame(stations::DataFrame) -> DataFrame

Renames a generator's `station_id/station_name/station_lon/station_lat[/role]`
DataFrame to the `:id/:lon/:lat` contract required by
`create_station_selection_data`, keeping `name`/`role` as pass-through extra
columns (only `:id/:lon/:lat` are required — see `src/data/core/struct.jl`).
"""
function tc_stations_frame(stations::DataFrame)::DataFrame
    df = DataFrame(
        id  = stations.station_id,
        lon = stations.station_lon,
        lat = stations.station_lat,
        name = stations.station_name,
    )
    if hasproperty(stations, :role)
        df.role = stations.role
    end
    return df
end

"""
    tc_requests_frame(orders::DataFrame) -> DataFrame

Renames a generator's `order_id/order_time/origin_station_id/destination_station_id/...`
DataFrame to the `:id/:origin_station_id/:destination_station_id/:request_time`
contract required by `create_station_selection_data`. Simulator-only fields
(`region_id`, `pax_num`, `status`, `vehicle_id`, pick-up/drop-off timestamps)
are intentionally dropped — they have no analog in `ScenarioData.requests`.
"""
function tc_requests_frame(orders::DataFrame)::DataFrame
    DataFrame(
        id = orders.order_id,
        origin_station_id = orders.origin_station_id,
        destination_station_id = orders.destination_station_id,
        request_time = DateTime.(orders.order_time, "yyyy-mm-dd HH:MM:SS"),
    )
end

"""
    tc_routing_costs(segments::DataFrame) -> Dict{Tuple{Int,Int},Float64}

Builds a routing-cost dictionary directly from a generator's segment.csv
DataFrame. No shortest-path computation is needed since these generators
always emit a complete pairwise graph.
"""
function tc_routing_costs(segments::DataFrame)::Dict{Tuple{Int,Int},Float64}
    return Dict((row.from_station, row.to_station) => row.seg_time for row in eachrow(segments))
end

"""
    tc_problem_data(stations, orders, segments; max_walking_distance,
                    walking_speed=1.4, walking_cost_scale=1.0, routing_cost_scale=1.0)
        -> StationSelectionData

Shared conversion recipe used by every `create_test{N}_problem_data`: renames
columns to the `StationSelectionData` contract, computes Haversine walking
costs via `compute_station_pairwise_costs`, and routing costs directly from
the segment DataFrame. `max_walking_distance` is accepted for interface
consistency with `create_grid_problem_data`/`create_zhuzhou_problem_data` but
is not used to prune costs here (unlike those two converters) — walking-pair
pruning happens on the model-build side (e.g. `TwoStageODPolicy`'s
`max_walking_distance`), so returning the full cost dict is simpler and
correct.
"""
function tc_problem_data(
    stations::DataFrame,
    orders::DataFrame,
    segments::DataFrame;
    max_walking_distance::Float64,
    walking_speed::Float64 = 1.4,
    walking_cost_scale::Float64 = 1.0,
    routing_cost_scale::Float64 = 1.0,
    scenarios::Union{Vector{Tuple{String,String}},Nothing} = nothing,
)::StationSelectionData
    stations_df = tc_stations_frame(stations)
    requests_df = tc_requests_frame(orders)

    walking_costs = compute_station_pairwise_costs(stations_df, walking_speed)
    walking_costs = Dict(key => walking_cost_scale * value for (key, value) in walking_costs)

    routing_costs = tc_routing_costs(segments)
    routing_costs = Dict(key => routing_cost_scale * value for (key, value) in routing_costs)

    return create_station_selection_data(
        stations_df, requests_df, walking_costs;
        routing_costs = routing_costs, scenarios = scenarios,
    )
end
