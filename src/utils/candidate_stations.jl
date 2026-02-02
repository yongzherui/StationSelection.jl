"""
    select_top_used_candidate_stations(stations, orders, n_candidate_stations) -> (stations, orders, n_candidates)

Filter stations and orders to the top-used candidate station set.
"""
function select_top_used_candidate_stations(
    stations::DataFrame,
    orders::DataFrame,
    n_candidate_stations::Int
)
    if n_candidate_stations <= 0
        @warn "n_candidate_stations is non-positive; skipping filter" n_candidate_stations
        return stations, orders, nrow(stations)
    end
    if n_candidate_stations > nrow(stations)
        @warn "n_candidate_stations exceeds available stations; skipping filter" n_candidate_stations n_stations=nrow(stations)
        return stations, orders, nrow(stations)
    end
    if nrow(orders) == 0
        @warn "No orders in specified time period; skipping filter" n_candidate_stations n_stations=nrow(stations)
        return stations, orders, nrow(stations)
    end

    station_counts = combine(
        groupby(
            vcat(
                select(orders, :start_station_id => :station_id),
                select(orders, :end_station_id => :station_id)
            ),
            :station_id
        ),
        nrow => :request_count
    )
    station_counts = sort(station_counts, :request_count, rev=true)
    top_station_ids = Set(first(
        station_counts.station_id,
        min(n_candidate_stations, nrow(station_counts))
    ))

    if length(top_station_ids) < n_candidate_stations
        remaining_ids = [id for id in stations.id if !(id in top_station_ids)]
        needed = min(n_candidate_stations - length(top_station_ids), length(remaining_ids))
        if needed > 0
            union!(top_station_ids, first(remaining_ids, needed))
        end
    end

    stations_filtered = stations[in.(stations.id, Ref(top_station_ids)), :]
    station_ids = Set(stations_filtered.id)
    orders_filtered = orders[
        in.(orders.start_station_id, Ref(station_ids)) .&
        in.(orders.end_station_id, Ref(station_ids)),
        :
    ]

    return stations_filtered, orders_filtered, length(station_ids)
end
