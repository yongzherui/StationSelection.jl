@testset "Adaptive station-cluster pricing lower bound" begin
    nodes = collect(1:4)
    travel = Dict((i,j) => Float64(abs(i-j)) for i in nodes for j in nodes)
    candidates = PassengerAssignmentCandidate[
        PassengerAssignmentCandidate(1, 1, 4, 5.0, 5.0),
        PassengerAssignmentCandidate(1, 2, 3, 5.0, 8.0),
        PassengerAssignmentCandidate(2, 1, 3, 4.0, 4.0),
    ]
    physical = create_passenger_free_assignment_pricing_data(
        1, nodes, travel, candidates; route_regularization_weight=1.0,
        max_wait_time=2.0, max_stops=4, max_visits_per_node=2)

    function exact_optimum(pd)
        labels, exhausted, _ = StationSelection._enumerate_passenger_free_assignment_pricing_labels(
            pd; time_limit=10.0, reduced_cost_tol=1e-9,
            max_visits_per_node=pd.max_visits_per_node, use_reduced_cost_pruning=false)
        @test exhausted
        minimum(l.reduced_cost for l in labels)
    end
    exact = exact_optimum(physical)

    @testset "configuration and compact initial partition" begin
        @test_throws ArgumentError initial_station_clustering(travel, 4,
            StationClusteringConfig(initial_num_clusters=0, max_num_clusters=2))
        cfg = StationClusteringConfig(initial_num_clusters=2, max_num_clusters=3,
                                      max_cluster_size=2)
        h = initial_station_clustering(travel, 4, cfg)
        @test length(h.clusters) == 2
        @test all(length(c.stations) <= 2 for c in h.clusters)
        @test sort(vcat(getfield.(h.clusters, :stations)...)) == nodes
        @test all(h.station_to_cluster[j] == c.id for c in h.clusters for j in c.stations)
    end

    @testset "singleton clusters coincide with exact pricing" begin
        cfg = StationClusteringConfig(initial_num_clusters=4, max_num_clusters=4,
                                      pricing_tolerance=1e-9)
        h = initial_station_clustering(travel, 4, cfg)
        cache = build_cluster_pricing_cache(h, physical, candidates)
        result = solve_cluster_pricer(h, cache)
        @test result.lower_bound_reduced_cost ≈ exact atol=1e-8
        @test assert_cluster_pricing_lower_bound(result, exact)
    end

    @testset "one cluster is valid and keeps inconsistent witnesses" begin
        cfg = StationClusteringConfig(initial_num_clusters=1, max_num_clusters=2,
                                      pricing_tolerance=1e-9)
        h = initial_station_clustering(travel, 4, cfg)
        cache = build_cluster_pricing_cache(h, physical, candidates)
        @test assert_cluster_lower_bound_coefficients(h, cache)
        arc = cache.arcs[(1,1)]
        reward = cache.rewards[(1,1,1)]
        @test arc.min_time == 0.0
        @test reward.reward == 8.0
        @test arc.time_witness != (reward.origin_station_witness,
                                   reward.destination_station_witness)
        @test arc.min_time <= travel[(h.clusters[1].medoid,h.clusters[1].medoid)]
        result = solve_cluster_pricer(h, cache)
        @test result.lower_bound_reduced_cost <= exact + 1e-8
        @test !result.certified_no_negative_column
    end

    @testset "nested split tightens and respects hard cap" begin
        cfg = StationClusteringConfig(initial_num_clusters=1, max_num_clusters=2,
                                      pricing_tolerance=1e-9)
        h = initial_station_clustering(travel, 4, cfg)
        cache = build_cluster_pricing_cache(h, physical, candidates)
        before = solve_cluster_pricer(h, cache)
        after, reason = solve_adaptive_cluster_lower_bound(h, cache)
        @test length(h.clusters) <= cfg.max_num_clusters
        @test length(h.clusters) == 2
        @test after.lower_bound_reduced_cost + 1e-8 >= before.lower_bound_reduced_cost
        @test after.lower_bound_reduced_cost <= exact + 1e-8
        @test reason in (CertifiedNonnegative, ReachedMaximumClusters, NoSplittableCluster)
        @test all(count(c -> j in c.stations, h.clusters) == 1 for j in nodes)
    end

    @testset "thirty-station requested configuration" begin
        d30 = Dict((i,j) => Float64(abs(i-j)) for i in 1:30 for j in 1:30)
        cfg = StationClusteringConfig(initial_num_clusters=10, max_num_clusters=15,
                                      max_cluster_size=4)
        h = initial_station_clustering(d30, 30, cfg)
        @test length(h.clusters) == 10
        @test all(length(c.stations) <= 4 for c in h.clusters)
    end
end
