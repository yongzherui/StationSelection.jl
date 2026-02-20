gurobi_available = try
    using Gurobi
    env = Gurobi.Env()
    true
catch
    false
end

if !gurobi_available
    @warn "Gurobi not available, skipping corridor clustering tests"
    @test true
    return
end

env = Gurobi.Env()

@testset "Corridor Clustering" begin
    # Create test data with routing costs
    stations = DataFrame(
        id = [1, 2, 3, 4, 5],
        lon = [113.0, 113.1, 113.2, 113.3, 113.4],
        lat = [28.0, 28.1, 28.2, 28.3, 28.4]
    )

    requests = DataFrame(
        id = [1, 2],
        start_station_id = [1, 2],
        end_station_id = [2, 3],
        request_time = [
            DateTime(2024, 1, 1, 8, 0, 0),
            DateTime(2024, 1, 1, 8, 1, 0)
        ]
    )

    walking_costs = Dict{Tuple{Int,Int}, Float64}()
    routing_costs = Dict{Tuple{Int,Int}, Float64}()
    for i in 1:5, j in 1:5
        walking_costs[(i, j)] = abs(i - j) * 100.0
        routing_costs[(i, j)] = abs(i - j) * 50.0
    end

    scenarios = [("2024-01-01 08:00:00", "2024-01-01 09:00:00")]
    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )

    array_idx_to_station_id = [1, 2, 3, 4, 5]

    @testset "compute_cluster_diameter" begin
        # Single station has diameter 0
        @test compute_cluster_diameter([1], data, array_idx_to_station_id) == 0.0

        # Two stations: routing distance between them
        diam = compute_cluster_diameter([1, 2], data, array_idx_to_station_id)
        @test diam == 50.0  # abs(1-2) * 50

        # Three stations: max pairwise distance
        diam = compute_cluster_diameter([1, 2, 3], data, array_idx_to_station_id)
        @test diam == 100.0  # abs(1-3) * 50

        # All stations
        diam = compute_cluster_diameter([1, 2, 3, 4, 5], data, array_idx_to_station_id)
        @test diam == 200.0  # abs(1-5) * 50
    end

    @testset "cluster_stations_by_diameter" begin
        # Large diameter: should result in 1 cluster
        labels, medoids, n_clusters = cluster_stations_by_diameter(
            data, array_idx_to_station_id, 1000.0; optimizer_env=env
        )
        @test n_clusters == 1
        @test length(labels) == 5
        @test all(l -> l == 1, labels)
        @test length(medoids) == 1

        # Small diameter: should need more clusters
        labels, medoids, n_clusters = cluster_stations_by_diameter(
            data, array_idx_to_station_id, 50.0; optimizer_env=env
        )
        @test n_clusters >= 2
        @test length(labels) == 5
        @test all(l -> 1 <= l <= n_clusters, labels)

        # Verify diameter constraint is satisfied
        for c in 1:n_clusters
            members = findall(==(c), labels)
            diam = compute_cluster_diameter(members, data, array_idx_to_station_id)
            @test diam <= 50.0
        end

        # Very small diameter: each station is its own cluster
        labels, medoids, n_clusters = cluster_stations_by_diameter(
            data, array_idx_to_station_id, 0.0; optimizer_env=env
        )
        @test n_clusters == 5
    end

    @testset "cluster_stations_by_diameter with 2 clusters" begin
        # 4 stations with two natural clusters: {1,2} close together, {3,4} close together
        # d(1,2)=10, d(3,4)=10, cross-cluster distances ≥ 100
        stations_4 = DataFrame(
            id = [1, 2, 3, 4],
            lon = [113.0, 113.01, 113.1, 113.11],
            lat = [28.0, 28.01, 28.1, 28.11]
        )
        requests_4 = DataFrame(
            id = [1],
            start_station_id = [1],
            end_station_id = [3],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)]
        )
        walking_4 = Dict{Tuple{Int,Int}, Float64}()
        routing_4 = Dict{Tuple{Int,Int}, Float64}()
        dists = [0.0 10.0 100.0 110.0; 10.0 0.0 90.0 100.0; 100.0 90.0 0.0 10.0; 110.0 100.0 10.0 0.0]
        for i in 1:4, j in 1:4
            walking_4[(i, j)] = 1000.0
            routing_4[(i, j)] = dists[i, j]
        end
        data_4 = StationSelection.create_station_selection_data(
            stations_4, requests_4, walking_4;
            routing_costs=routing_4,
            scenarios=[("2024-01-01 08:00:00", "2024-01-01 09:00:00")]
        )
        arr_4 = [1, 2, 3, 4]

        # D=50: {1,2} (diam 10) and {3,4} (diam 10) should form 2 clusters
        labels, medoids, n_clusters = cluster_stations_by_diameter(
            data_4, arr_4, 50.0; optimizer_env=env
        )
        @test n_clusters == 2
        @test length(labels) == 4
        @test length(medoids) == 2

        # Stations 1,2 should be in the same cluster; stations 3,4 in the same cluster
        @test labels[1] == labels[2]
        @test labels[3] == labels[4]
        @test labels[1] != labels[3]

        # Each medoid must be in its own cluster
        for (c, med) in enumerate(medoids)
            @test labels[med] == c
        end

        # Verify diameter constraint
        for c in 1:n_clusters
            members = findall(==(c), labels)
            diam = compute_cluster_diameter(members, data_4, arr_4)
            @test diam <= 50.0
        end
    end

    @testset "cluster_stations_by_count" begin
        # n_clusters=1: all stations in one cluster
        labels, medoids, nc = cluster_stations_by_count(
            data, array_idx_to_station_id, 1; optimizer_env=env
        )
        @test nc == 1
        @test length(labels) == 5
        @test all(l -> l == 1, labels)
        @test length(medoids) == 1

        # n_clusters=5: each station its own cluster
        labels, medoids, nc = cluster_stations_by_count(
            data, array_idx_to_station_id, 5; optimizer_env=env
        )
        @test nc == 5
        @test length(labels) == 5
        @test length(medoids) == 5
        @test length(unique(labels)) == 5

        # Error: n_clusters=0
        @test_throws ArgumentError cluster_stations_by_count(
            data, array_idx_to_station_id, 0; optimizer_env=env
        )

        # Error: n_clusters > n_stations
        @test_throws ArgumentError cluster_stations_by_count(
            data, array_idx_to_station_id, 6; optimizer_env=env
        )
    end

    @testset "cluster_stations_by_count with 2 natural clusters" begin
        # Reuse the 4-station data with two natural clusters: {1,2} and {3,4}
        stations_4 = DataFrame(
            id = [1, 2, 3, 4],
            lon = [113.0, 113.01, 113.1, 113.11],
            lat = [28.0, 28.01, 28.1, 28.11]
        )
        requests_4 = DataFrame(
            id = [1],
            start_station_id = [1],
            end_station_id = [3],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)]
        )
        walking_4 = Dict{Tuple{Int,Int}, Float64}()
        routing_4 = Dict{Tuple{Int,Int}, Float64}()
        dists = [0.0 10.0 100.0 110.0; 10.0 0.0 90.0 100.0; 100.0 90.0 0.0 10.0; 110.0 100.0 10.0 0.0]
        for i in 1:4, j in 1:4
            walking_4[(i, j)] = 1000.0
            routing_4[(i, j)] = dists[i, j]
        end
        data_4 = StationSelection.create_station_selection_data(
            stations_4, requests_4, walking_4;
            routing_costs=routing_4,
            scenarios=[("2024-01-01 08:00:00", "2024-01-01 09:00:00")]
        )
        arr_4 = [1, 2, 3, 4]

        labels, medoids, nc = cluster_stations_by_count(
            data_4, arr_4, 2; optimizer_env=env
        )
        @test nc == 2
        @test length(labels) == 4
        @test length(medoids) == 2

        # Stations 1,2 should be in the same cluster; stations 3,4 in the same cluster
        @test labels[1] == labels[2]
        @test labels[3] == labels[4]
        @test labels[1] != labels[3]
    end

    @testset "compute_corridor_data" begin
        labels = [1, 1, 2, 2, 2]
        medoids = [1, 3]  # station 1 is medoid of cluster 1, station 3 of cluster 2
        n_clusters = 2

        corridor_indices, cluster_station_sets, corridor_costs = compute_corridor_data(
            labels, medoids, n_clusters, data, array_idx_to_station_id
        )

        # Should have n_clusters² = 4 corridors
        @test length(corridor_indices) == 4
        @test length(corridor_costs) == 4

        # Check cluster station sets
        @test length(cluster_station_sets) == 2
        @test Set(cluster_station_sets[1]) == Set([1, 2])
        @test Set(cluster_station_sets[2]) == Set([3, 4, 5])

        # Check corridor indices
        @test (1, 1) in corridor_indices
        @test (1, 2) in corridor_indices
        @test (2, 1) in corridor_indices
        @test (2, 2) in corridor_indices

        # Check self-corridor cost is 0
        self_idx_1 = findfirst(==((1, 1)), corridor_indices)
        @test corridor_costs[self_idx_1] == 0.0

        self_idx_2 = findfirst(==((2, 2)), corridor_indices)
        @test corridor_costs[self_idx_2] == 0.0

        # Cross-corridor cost = routing distance between medoids
        cross_idx = findfirst(==((1, 2)), corridor_indices)
        @test corridor_costs[cross_idx] == get_routing_cost(data, 1, 3)  # 100.0
    end
end
