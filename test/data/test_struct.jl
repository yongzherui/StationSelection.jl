@testset "Data Structures" begin
    using DataFrames
    using Dates

    @testset "ScenarioData creation" begin
        requests = DataFrame(
            id = [1, 2, 3],
            start_station_id = [1, 1, 2],
            end_station_id = [2, 3, 3],
            request_time = [
                DateTime(2024, 1, 1, 8, 0, 0),
                DateTime(2024, 1, 1, 8, 30, 0),
                DateTime(2024, 1, 1, 9, 0, 0)
            ]
        )

        # Basic creation without time bounds
        scenario = StationSelection.create_scenario_data(requests, "test_scenario")
        @test scenario.label == "test_scenario"
        @test isnothing(scenario.start_time)
        @test isnothing(scenario.end_time)
        @test nrow(scenario.requests) == 3

        # Creation with time bounds
        start_dt = DateTime(2024, 1, 1, 8, 0, 0)
        end_dt = DateTime(2024, 1, 1, 10, 0, 0)
        scenario_with_times = StationSelection.create_scenario_data(
            requests, "bounded_scenario";
            start_time=start_dt, end_time=end_dt
        )
        @test scenario_with_times.start_time == start_dt
        @test scenario_with_times.end_time == end_dt
    end

    @testset "StationSelectionData creation" begin
        stations = DataFrame(
            id = [1, 2, 3],
            lon = [113.0, 113.1, 113.2],
            lat = [28.0, 28.1, 28.2]
        )
        requests = DataFrame(
            id = [1, 2],
            start_station_id = [1, 2],
            end_station_id = [2, 3],
            request_time = [
                DateTime(2024, 1, 1, 8, 0, 0),
                DateTime(2024, 1, 1, 9, 0, 0)
            ]
        )
        walking_costs = Dict{Tuple{Int,Int}, Float64}(
            (1, 2) => 100.0, (2, 1) => 100.0,
            (2, 3) => 150.0, (3, 2) => 150.0,
            (1, 3) => 200.0, (3, 1) => 200.0,
            (1, 1) => 0.0, (2, 2) => 0.0, (3, 3) => 0.0
        )

        # Single scenario (no scenario time windows)
        data = StationSelection.create_station_selection_data(stations, requests, walking_costs)
        @test data.n_stations == 3
        @test StationSelection.n_scenarios(data) == 1
        @test data.scenarios[1].label == "all_requests"
        @test !StationSelection.has_routing_costs(data)

        # With routing costs
        routing_costs = Dict{Tuple{Int,Int}, Float64}(
            (1, 2) => 10.0, (2, 1) => 10.0,
            (2, 3) => 15.0, (3, 2) => 15.0,
            (1, 3) => 20.0, (3, 1) => 20.0,
            (1, 1) => 0.0, (2, 2) => 0.0, (3, 3) => 0.0
        )
        data_with_routing = StationSelection.create_station_selection_data(
            stations, requests, walking_costs;
            routing_costs=routing_costs
        )
        @test StationSelection.has_routing_costs(data_with_routing)
        @test StationSelection.get_routing_cost(data_with_routing, 1, 2) == 10.0
    end

    @testset "Walking and routing cost accessors" begin
        stations = DataFrame(id = [1, 2], lon = [0.0, 0.0], lat = [0.0, 0.0])
        requests = DataFrame(
            id = [1],
            start_station_id = [1],
            end_station_id = [2],
            request_time = [DateTime(2024, 1, 1)]
        )
        walking_costs = Dict{Tuple{Int,Int}, Float64}((1, 2) => 100.0, (2, 1) => 100.0, (1, 1) => 0.0, (2, 2) => 0.0)
        routing_costs = Dict{Tuple{Int,Int}, Float64}((1, 2) => 10.0, (2, 1) => 10.0, (1, 1) => 0.0, (2, 2) => 0.0)

        data = StationSelection.create_station_selection_data(
            stations, requests, walking_costs;
            routing_costs=routing_costs
        )

        @test StationSelection.get_walking_cost(data, 1, 2) == 100.0
        @test StationSelection.get_routing_cost(data, 1, 2) == 10.0

        # Test error when routing costs not available
        data_no_routing = StationSelection.create_station_selection_data(stations, requests, walking_costs)
        @test_throws ErrorException StationSelection.get_routing_cost(data_no_routing, 1, 2)
    end

    @testset "Station ID mappings" begin
        station_ids = [10, 20, 30, 40]

        id_to_idx, idx_to_id = StationSelection.create_station_id_mappings(station_ids)

        @test length(id_to_idx) == 4
        @test length(idx_to_id) == 4

        # Verify bidirectional mapping
        for (idx, id) in enumerate(station_ids)
            @test id_to_idx[id] == idx
            @test idx_to_id[idx] == id
        end

        # Test with non-sequential IDs
        @test id_to_idx[10] == 1
        @test id_to_idx[40] == 4
        @test idx_to_id[2] == 20
    end

    @testset "Scenario label mappings" begin
        requests = DataFrame(
            id = Int[], start_station_id = Int[], end_station_id = Int[], request_time = DateTime[]
        )

        scenarios = [
            StationSelection.ScenarioData("morning", DateTime(2024, 1, 1, 6), DateTime(2024, 1, 1, 12), requests),
            StationSelection.ScenarioData("afternoon", DateTime(2024, 1, 1, 12), DateTime(2024, 1, 1, 18), requests),
            StationSelection.ScenarioData("evening", DateTime(2024, 1, 1, 18), DateTime(2024, 1, 1, 24), requests)
        ]

        label_to_idx, idx_to_label = StationSelection.create_scenario_label_mappings(scenarios)

        @test length(label_to_idx) == 3
        @test length(idx_to_label) == 3

        @test label_to_idx["morning"] == 1
        @test label_to_idx["afternoon"] == 2
        @test label_to_idx["evening"] == 3

        @test idx_to_label[1] == "morning"
        @test idx_to_label[2] == "afternoon"
        @test idx_to_label[3] == "evening"
    end

    @testset "Time to OD count mapping" begin
        start_time = DateTime(2024, 1, 1, 8, 0, 0)
        requests = DataFrame(
            id = [1, 2, 3, 4, 5, 6],
            start_station_id = [1, 1, 2, 2, 1, 1],
            end_station_id = [2, 2, 3, 1, 2, 3],
            request_time = [
                DateTime(2024, 1, 1, 8, 0, 0),   # time_id = 0, OD (1,2)
                DateTime(2024, 1, 1, 8, 0, 30),  # time_id = 0, OD (1,2) - duplicate
                DateTime(2024, 1, 1, 8, 1, 0),   # time_id = 1, OD (2,3)
                DateTime(2024, 1, 1, 8, 2, 0),   # time_id = 2, OD (2,1)
                DateTime(2024, 1, 1, 8, 0, 45),  # time_id = 0, OD (1,2) - third occurrence
                DateTime(2024, 1, 1, 8, 0, 15)   # time_id = 0, OD (1,3)
            ]
        )

        scenario = StationSelection.ScenarioData("test", start_time, DateTime(2024, 1, 1, 9, 0, 0), requests)
        time_window = 60  # 60 seconds

        time_to_od_count = StationSelection.compute_time_to_od_count_mapping(scenario, time_window)

        # Check time_id 0: should have (1,2) with count 3 and (1,3) with count 1
        @test haskey(time_to_od_count, 0)
        @test time_to_od_count[0][(1, 2)] == 3
        @test time_to_od_count[0][(1, 3)] == 1
        @test length(time_to_od_count[0]) == 2  # Only 2 unique OD pairs

        # Check time_id 1: should have (2,3) with count 1
        @test haskey(time_to_od_count, 1)
        @test time_to_od_count[1][(2, 3)] == 1

        # Check time_id 2: should have (2,1) with count 1
        @test haskey(time_to_od_count, 2)
        @test time_to_od_count[2][(2, 1)] == 1
    end

    @testset "Time to OD count mapping with string times" begin
        start_time = DateTime(2024, 1, 1, 8, 0, 0)
        requests = DataFrame(
            id = [1, 2],
            start_station_id = [1, 2],
            end_station_id = [2, 3],
            request_time = ["2024-01-01 08:00:00", "2024-01-01 08:05:00"]
        )

        scenario = StationSelection.ScenarioData("test", start_time, DateTime(2024, 1, 1, 9, 0, 0), requests)
        time_window = 300  # 5 minutes

        time_to_od_count = StationSelection.compute_time_to_od_count_mapping(scenario, time_window)

        @test haskey(time_to_od_count, 0)
        @test haskey(time_to_od_count, 1)
        @test haskey(time_to_od_count[0], (1, 2))
        @test haskey(time_to_od_count[1], (2, 3))
        @test time_to_od_count[0][(1, 2)] == 1
        @test time_to_od_count[1][(2, 3)] == 1
    end

    @testset "TwoStageSingleDetourModel construction" begin
        # Valid construction with max_walking_distance
        model = TwoStageSingleDetourModel(3, 5, 0.5, 300.0, 60.0; max_walking_distance=500.0)
        @test model.k == 3
        @test model.l == 5
        @test model.routing_weight == 0.5
        @test model.time_window == 300.0
        @test model.routing_delay == 60.0
        @test model.max_walking_distance == 500.0

        # Test validation: k must be positive
        @test_throws ArgumentError TwoStageSingleDetourModel(0, 5, 0.5, 300.0, 60.0; max_walking_distance=500.0)
        @test_throws ArgumentError TwoStageSingleDetourModel(-1, 5, 0.5, 300.0, 60.0; max_walking_distance=500.0)

        # Test validation: l must be >= k
        @test_throws ArgumentError TwoStageSingleDetourModel(5, 3, 0.5, 300.0, 60.0; max_walking_distance=500.0)

        # Test validation: routing_weight must be non-negative
        @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, -0.5, 300.0, 60.0; max_walking_distance=500.0)
        # 0.0 is valid for routing_weight
        model_zero_rw = TwoStageSingleDetourModel(3, 5, 0.0, 300.0, 60.0; max_walking_distance=500.0)
        @test model_zero_rw.routing_weight == 0.0

        # Test validation: time_window must be positive
        @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, 0.5, 0.0, 60.0; max_walking_distance=500.0)

        # Test validation: routing_delay must be non-negative
        @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, 0.5, 300.0, -1.0; max_walking_distance=500.0)
        # 0.0 is valid for routing_delay
        model_zero_rd = TwoStageSingleDetourModel(3, 5, 0.5, 300.0, 0.0; max_walking_distance=500.0)
        @test model_zero_rd.routing_delay == 0.0

        # Test validation: max_walking_distance must be non-negative
        @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, 0.5, 300.0, 60.0; max_walking_distance=-1.0)
        # 0.0 is valid for max_walking_distance (though not practical)
        model_zero_wd = TwoStageSingleDetourModel(3, 5, 0.5, 300.0, 60.0; max_walking_distance=0.0)
        @test model_zero_wd.max_walking_distance == 0.0
    end

    @testset "PoolingScenarioOriginDestTimeMap structure" begin
        # Create test data
        stations = DataFrame(id = [1, 2, 3], lon = [0.0, 0.1, 0.2], lat = [0.0, 0.1, 0.2])
        start_time = DateTime(2024, 1, 1, 8, 0, 0)
        end_time = DateTime(2024, 1, 1, 9, 0, 0)

        requests = DataFrame(
            id = [1, 2],
            start_station_id = [1, 2],
            end_station_id = [2, 3],
            request_time = [
                DateTime(2024, 1, 1, 8, 0, 0),
                DateTime(2024, 1, 1, 8, 5, 0)
            ]
        )

        walking_costs = Dict{Tuple{Int,Int}, Float64}()
        routing_costs = Dict{Tuple{Int,Int}, Float64}()
        for i in [1,2,3], j in [1,2,3]
            walking_costs[(i, j)] = abs(i - j) * 100.0
            routing_costs[(i, j)] = abs(i - j) * 10.0
        end

        scenarios = [
            StationSelection.ScenarioData("morning", start_time, end_time, requests)
        ]

        data = StationSelection.StationSelectionData(
            stations, 3, walking_costs, routing_costs, scenarios
        )

        model = TwoStageSingleDetourModel(2, 3, 1.0, 300.0, 60.0; max_walking_distance=500.0)

        pooling_map = StationSelection.create_pooling_scenario_origin_dest_time_map(model, data)

        # Check structure
        @test length(pooling_map.station_id_to_array_idx) == 3
        @test length(pooling_map.array_idx_to_station_id) == 3
        @test pooling_map.time_window == 300
        @test length(pooling_map.scenarios) == 1
        @test haskey(pooling_map.Omega_s_t, 1)  # scenario_id = 1
        @test haskey(pooling_map.Q_s_t, 1)  # demand counts for scenario_id = 1
    end

    @testset "PoolingScenarioOriginDestTimeMap demand counts" begin
        # Create test data with duplicate OD pairs
        stations = DataFrame(id = [1, 2, 3], lon = [0.0, 0.1, 0.2], lat = [0.0, 0.1, 0.2])
        start_time = DateTime(2024, 1, 1, 8, 0, 0)
        end_time = DateTime(2024, 1, 1, 9, 0, 0)

        # Multiple requests with same OD pair at same time
        requests = DataFrame(
            id = [1, 2, 3, 4],
            start_station_id = [1, 1, 1, 2],
            end_station_id = [2, 2, 3, 3],
            request_time = [
                DateTime(2024, 1, 1, 8, 0, 0),   # OD (1,2), time_id = 0
                DateTime(2024, 1, 1, 8, 0, 30),  # OD (1,2), time_id = 0 - same as above
                DateTime(2024, 1, 1, 8, 0, 45),  # OD (1,3), time_id = 0
                DateTime(2024, 1, 1, 8, 5, 0)    # OD (2,3), time_id = 1
            ]
        )

        walking_costs = Dict{Tuple{Int,Int}, Float64}()
        routing_costs = Dict{Tuple{Int,Int}, Float64}()
        for i in [1,2,3], j in [1,2,3]
            walking_costs[(i, j)] = abs(i - j) * 100.0
            routing_costs[(i, j)] = abs(i - j) * 10.0
        end

        scenarios = [
            StationSelection.ScenarioData("test", start_time, end_time, requests)
        ]

        data = StationSelection.StationSelectionData(
            stations, 3, walking_costs, routing_costs, scenarios
        )

        model = TwoStageSingleDetourModel(2, 3, 1.0, 60.0, 30.0; max_walking_distance=500.0)  # time_window = 60s

        pooling_map = StationSelection.create_pooling_scenario_origin_dest_time_map(model, data)

        # Check demand counts
        @test pooling_map.Q_s_t[1][0][(1, 2)] == 2  # Two requests for (1,2) at time 0
        @test pooling_map.Q_s_t[1][0][(1, 3)] == 1  # One request for (1,3) at time 0
        @test pooling_map.Q_s_t[1][5][(2, 3)] == 1  # One request for (2,3) at time 5 (300s / 60s = 5)

        # Check that Omega_s_t still has unique OD pairs
        @test length(pooling_map.Omega_s_t[1][0]) == 2  # (1,2) and (1,3)
        @test (1, 2) in pooling_map.Omega_s_t[1][0]
        @test (1, 3) in pooling_map.Omega_s_t[1][0]
    end
end
