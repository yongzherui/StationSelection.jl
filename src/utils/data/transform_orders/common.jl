"""
    parse_station_list(list_str::String) -> Vector{Int}

Parse a string like "[1,2,3]" or "[1 2 3]" into a vector of integers.
"""
function parse_station_list(list_str)
    cleaned = strip(replace(string(list_str), "[" => "", "]" => "", "," => " "))
    if isempty(cleaned) || cleaned == "missing"
        return Int[]
    end
    return parse.(Int, split(cleaned))
end

function _row_station_id(row, side::Symbol)::Int
    columns = propertynames(row)
    candidates = side == :origin ?
        (:origin_station_id, :start_station_id, :origin_id) :
        (:destination_station_id, :end_station_id, :target_id, :dest_station_id)

    for col in candidates
        if col in columns && !ismissing(row[col])
            station_id = Int(row[col])
            _warn_if_legacy_station_disagrees(row, side, station_id, col)
            return station_id
        end
    end

    legacy_col = side == :origin ? :available_pickup_station_list : :available_dropoff_station_list
    legacy_col in columns || return 0
    stations = parse_station_list(string(row[legacy_col]))
    return isempty(stations) ? 0 : first(stations)
end

function _warn_if_legacy_station_disagrees(row, side::Symbol, station_id::Int, scalar_col::Symbol)
    legacy_col = side == :origin ? :available_pickup_station_list : :available_dropoff_station_list
    legacy_col in propertynames(row) || return
    ismissing(row[legacy_col]) && return

    stations = parse_station_list(string(row[legacy_col]))
    isempty(stations) && return
    legacy_id = first(stations)
    if legacy_id != station_id
        @warn "Scalar station column disagrees with legacy station list; using scalar column" side scalar_col station_id legacy_col legacy_id
    end
end

_row_origin_station_id(row)::Int = _row_station_id(row, :origin)
_row_destination_station_id(row)::Int = _row_station_id(row, :destination)

function _row_value_or_missing(row, col::Symbol)
    return col in propertynames(row) ? row[col] : missing
end

"""
    precompute_distances(stations::DataFrame) -> Dict{Tuple{Int,Int}, Float64}

Precompute all pairwise distances between stations using Haversine distance.
"""
function precompute_distances(stations::DataFrame)
    distances = Dict{Tuple{Int,Int}, Float64}()
    dist_func = Haversine()

    for i in 1:nrow(stations)
        for j in 1:nrow(stations)
            if i == j
                distances[(stations[i, :id], stations[j, :id])] = 0.0
            else
                p1 = [stations[i, :lat], stations[i, :lon]]
                p2 = [stations[j, :lat], stations[j, :lon]]
                distance = evaluate(dist_func, p1, p2)
                distances[(stations[i, :id], stations[j, :id])] = distance
            end
        end
    end

    return distances
end

"""
    find_closest_selected_station(candidate_station_id::Int,
                                  selected_station_ids::Vector{Int},
                                  distance_matrix::Dict{Tuple{Int,Int}, Float64}) -> Int

Find the closest selected station to a given candidate station using precomputed distances.
"""
function find_closest_selected_station(candidate_station_id::Int,
                                       selected_station_ids::Vector{Int},
                                       distance_matrix::Dict{Tuple{Int,Int}, Float64})
    if candidate_station_id == 0 || isempty(selected_station_ids)
        return 0
    end

    min_distance = Inf
    closest_selected_id = 0

    for selected_id in selected_station_ids
        distance = get(distance_matrix, (candidate_station_id, selected_id), Inf)

        if distance < min_distance
            min_distance = distance
            closest_selected_id = selected_id
        end
    end

    return closest_selected_id
end
