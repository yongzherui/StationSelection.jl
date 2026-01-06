module StationCosts

using Distances
using DataFrames
using CSV
using Graphs
using SimpleWeightedGraphs

export compute_station_pairwise_costs, read_routing_costs_from_segments

"""
compute_station_pairwise_costs(candidate_stations::DataFrame) -> Dict::Tuple{Int, Int}, Float64}
matrix between candidate stations based on Haversine distance.
"""
function compute_station_pairwise_costs(candidate_stations::DataFrame)::Dict{Tuple{Int, Int}, Float64}
    n = nrow(candidate_stations)
    costs = Dict{Tuple{Int, Int}, Float64}()

    dist_func = Haversine()
    for i in 1:n
        for j in 1:n
            i_id = candidate_stations[i, :id]
            j_id = candidate_stations[j, :id]
            if i != j
                p1 = [candidate_stations[i, :lat], candidate_stations[i, :lon]]
                p2 = [candidate_stations[j, :lat], candidate_stations[j, :lon]]
                costs[(i_id, j_id)] = evaluate(dist_func, p1, p2)
            elseif i == j
                costs[(i_id, j_id)] = 0.0
            end
        end
    end

    return costs
end


"""
read_routing_costs_from_segments(segment_file::String, candidate_stations::DataFrame) -> Dict{Tuple{Int, Int}, Float64}

Read routing costs from a segment CSV file and compute all-pairs shortest paths using Floyd-Warshall.

The segment file should have columns:
- from_station: origin station ID
- to_station: destination station ID
- seg_dist: segment distance (used as routing cost)

For station pairs not directly connected, computes shortest paths through the network.
"""
function read_routing_costs_from_segments(segment_file::String, candidate_stations::DataFrame)::Dict{Tuple{Int, Int}, Float64}
    # Read segment data
    segments = CSV.read(segment_file, DataFrame)

    # Get list of all station IDs
    station_ids = sort(unique(candidate_stations.id))
    n_stations = length(station_ids)

    # Create mapping from station ID to index
    id_to_idx = Dict(id => i for (i, id) in enumerate(station_ids))

    # Create weighted directed graph
    g = SimpleWeightedDiGraph(n_stations)

    # Add edges from segment data
    for row in eachrow(segments)
        from_id = row.from_station
        to_id = row.to_station

        # Only use segments between stations in our candidate set
        if haskey(id_to_idx, from_id) && haskey(id_to_idx, to_id)
            i = id_to_idx[from_id]
            j = id_to_idx[to_id]
            add_edge!(g, i, j, row.seg_dist)
        end
    end

    # Compute all-pairs shortest paths using Floyd-Warshall
    dist_matrix = floyd_warshall_shortest_paths(g).dists

    # Convert matrix to dictionary format
    routing_costs = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:n_stations
        for j in 1:n_stations
            from_id = station_ids[i]
            to_id = station_ids[j]
            routing_costs[(from_id, to_id)] = dist_matrix[i, j]
        end
    end

    return routing_costs
end

end # module