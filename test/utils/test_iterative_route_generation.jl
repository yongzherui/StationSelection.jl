using Test
using DataFrames
using Dates

@testset "Iterative Route Generation" begin
    function make_route_test_data()
        stations = DataFrame(
            id = [1, 2, 3, 4],
            lon = [0.0, 1.0, 2.0, 3.0],
            lat = [0.0, 0.0, 0.0, 0.0],
        )
        requests = DataFrame(
            order_id = [1, 2, 3],
            pax_num = [1, 1, 1],
            start_station_id = [1, 1, 2],
            end_station_id = [3, 4, 4],
            request_time = [
                DateTime(2024, 1, 1, 8, 0, 0),
                DateTime(2024, 1, 1, 8, 10, 0),
                DateTime(2024, 1, 1, 8, 20, 0),
            ],
        )

        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:4, j in 1:4
            walking_costs[(i, j)] = abs(i - j) * 60.0
            routing_costs[(i, j)] = abs(i - j) * 10.0
        end

        StationSelection.create_station_selection_data(
            stations,
            requests,
            walking_costs;
            routing_costs=routing_costs,
            scenarios=[("2024-01-01 08:00:00", "2024-01-01 09:00:00")],
        )
    end

    data = make_route_test_data()
    valid_jk_pairs = Set([(1, 3), (1, 4), (2, 4), (2, 3), (3, 4)])

    @testset "generator keeps direct routes and respects dedup/max-length" begin
        cfg = StationSelection.IterativeRouteGenerationConfig(
            max_route_length=3,
            max_iterations=2,
            max_new_routes_per_iter=20,
            max_routes_total=100,
            verbose=false,
        )
        routes = StationSelection.generate_iterative_routes(
            valid_jk_pairs,
            data;
            config=cfg,
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            stop_dwell_time=10.0,
        )
        route_keys = map(r -> Tuple(r.station_indices), routes)
        @test length(route_keys) == length(unique(route_keys))
        @test all(length(r.station_indices) <= 3 for r in routes)
        @test all(length(Set(r.station_indices)) == length(r.station_indices) for r in routes)
        @test all(((j, k) in route_keys) for (j, k) in valid_jk_pairs)
    end

    @testset "geometry insertion respects arc condition" begin
        route = StationSelection.RouteData(1, [1, 3], 20.0, [(1, 3)])
        insertions = StationSelection._plausible_insertions(route, [1, 2, 3, 4], data, 0.0)
        @test (0.0, 1, 2) in insertions
        @test all(candidate[3] != 4 for candidate in insertions)
    end

    @testset "coverage counting tracks feasible leg multiplicity" begin
        routes = [
            StationSelection.RouteData(1, [1, 3], 20.0, [(1, 3)]),
            StationSelection.RouteData(2, [1, 2, 4], 40.0, [(1, 2), (1, 4), (2, 4)]),
        ]
        coverage = StationSelection._route_coverage_count(routes)
        @test coverage[(1, 3)] == 1
        @test coverage[(1, 4)] == 1
        @test coverage[(2, 4)] == 1
    end

    @testset "interior replacement preserves endpoints" begin
        cfg = StationSelection.IterativeRouteGenerationConfig(
            max_route_length=3,
            knn_replacement=2,
            min_new_feasible_legs=0,
        )
        routes = [StationSelection.RouteData(1, [1, 2, 4], 40.0, [(1, 2), (1, 4), (2, 4)])]
        coverage = StationSelection._route_coverage_count(routes)
        candidates = StationSelection._interior_replacement_candidates(
            routes,
            [1, 2, 3, 4],
            data,
            valid_jk_pairs,
            coverage,
            cfg;
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            stop_dwell_time=10.0,
        )
        @test !isempty(candidates)
        @test all(c.route.station_indices[1] == 1 && c.route.station_indices[end] == 4 for c in candidates)
    end

    @testset "reverse mutation produces reversed candidate when valid" begin
        cfg = StationSelection.IterativeRouteGenerationConfig(
            max_route_length=3,
            min_new_feasible_legs=0,
        )
        routes = [StationSelection.RouteData(1, [1, 2, 3], 30.0, [(1, 2), (1, 3), (2, 3)])]
        coverage = StationSelection._route_coverage_count(routes)
        _, reverse_candidates = StationSelection._endpoint_and_reverse_candidates(
            routes,
            [1, 2, 3, 4],
            data,
            Set([(1, 2), (1, 3), (2, 3), (3, 2), (3, 1), (2, 1)]),
            coverage,
            cfg;
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            stop_dwell_time=10.0,
        )
        @test any(c.route.station_indices == [3, 2, 1] for c in reverse_candidates)
    end

    @testset "fixed rng seed gives deterministic routes" begin
        cfg = StationSelection.IterativeRouteGenerationConfig(
            max_route_length=3,
            max_iterations=2,
            rng_seed=42,
        )
        routes_a = StationSelection.generate_iterative_routes(
            valid_jk_pairs,
            data;
            config=cfg,
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            stop_dwell_time=10.0,
        )
        routes_b = StationSelection.generate_iterative_routes(
            valid_jk_pairs,
            data;
            config=cfg,
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            stop_dwell_time=10.0,
        )
        @test map(r -> r.station_indices, routes_a) == map(r -> r.station_indices, routes_b)
    end
end
