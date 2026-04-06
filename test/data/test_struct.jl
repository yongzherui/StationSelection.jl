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
        stations = DataFrame(id = [10, 20], lon = [0.0, 0.0], lat = [0.0, 0.0])
        requests = DataFrame(
            id = [1],
            start_station_id = [10],
            end_station_id = [20],
            request_time = [DateTime(2024, 1, 1)]
        )
        walking_costs = Dict{Tuple{Int,Int}, Float64}((10, 20) => 100.0, (20, 10) => 100.0, (10, 10) => 0.0, (20, 20) => 0.0)
        routing_costs = Dict{Tuple{Int,Int}, Float64}((10, 20) => 10.0, (20, 10) => 10.0, (10, 10) => 0.0, (20, 20) => 0.0)

        data = StationSelection.create_station_selection_data(
            stations, requests, walking_costs;
            routing_costs=routing_costs
        )

        @test StationSelection.get_walking_cost(data, 1, 2) == 100.0
        @test StationSelection.get_routing_cost(data, 1, 2) == 10.0
        @test StationSelection.get_walking_cost_by_id(data, 10, 20) == 100.0
        @test StationSelection.get_routing_cost_by_id(data, 10, 20) == 10.0
        @test data.scenarios[1].requests.origin_idx == [1]
        @test data.scenarios[1].requests.dest_idx == [2]

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
            ],
            origin_idx = [1, 1, 2, 2, 1, 1],
            dest_idx = [2, 2, 3, 1, 2, 3]
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
            request_time = ["2024-01-01 08:00:00", "2024-01-01 08:05:00"],
            origin_idx = [1, 2],
            dest_idx = [2, 3]
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

end
