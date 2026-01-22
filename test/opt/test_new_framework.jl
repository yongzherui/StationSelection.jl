using StationSelection
using Test
using DataFrames
using Dates

@testset "New Optimization Framework" begin

    @testset "Data Structures" begin
        @testset "StationSelectionData creation" begin
            # Create simple test data
            stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
            requests = DataFrame(
                id=[1, 2],
                start_station_id=[1, 2],
                end_station_id=[2, 3],
                request_time=DateTime.(["2025-01-15 10:00:00", "2025-01-15 11:00:00"], "yyyy-mm-dd HH:MM:SS")
            )
            costs = Dict{Tuple{Int,Int}, Float64}()
            for i in 1:3, j in 1:3
                costs[(i, j)] = i == j ? 0.0 : 1.0
            end

            # Test creation without scenarios
            data = create_station_selection_data(stations, requests, costs)

            @test data.n_stations == 3
            @test length(data.scenarios) == 1
            @test data.id_to_idx[1] == 1
            @test data.id_to_idx[2] == 2
            @test data.id_to_idx[3] == 3
            @test data.idx_to_id[1] == 1
            @test data.idx_to_id[2] == 2
            @test data.idx_to_id[3] == 3

            # Test scenario data
            scenario = data.scenarios[1]
            @test scenario.label == "all_requests"
            @test scenario.pickup_counts[1] == 1  # One pickup at station 1
            @test scenario.pickup_counts[2] == 1  # One pickup at station 2
            @test scenario.pickup_counts[3] == 0  # No pickups at station 3
            @test scenario.dropoff_counts[2] == 1  # One dropoff at station 2
            @test scenario.dropoff_counts[3] == 1  # One dropoff at station 3
            @test scenario.total_counts[2] == 2   # 1 pickup + 1 dropoff
        end

        @testset "StationSelectionData with scenarios" begin
            stations = DataFrame(id=[1, 2], lat=[27.9, 27.91], lon=[113.1, 113.11])
            requests = DataFrame(
                id=[1, 2, 3],
                start_station_id=[1, 1, 2],
                end_station_id=[2, 2, 1],
                request_time=DateTime.([
                    "2025-01-15 10:00:00",
                    "2025-01-15 11:00:00",
                    "2025-01-16 10:00:00"
                ], "yyyy-mm-dd HH:MM:SS")
            )
            costs = Dict{Tuple{Int,Int}, Float64}(
                (1, 1) => 0.0, (1, 2) => 1.0,
                (2, 1) => 1.0, (2, 2) => 0.0
            )

            scenarios = [
                ("2025-01-15 00:00:00", "2025-01-15 23:59:59"),
                ("2025-01-16 00:00:00", "2025-01-16 23:59:59")
            ]

            data = create_station_selection_data(stations, requests, costs; scenarios=scenarios)

            @test length(data.scenarios) == 2
            @test n_scenarios(data) == 2

            # First scenario: 2 requests
            @test nrow(data.scenarios[1].requests) == 2
            @test data.scenarios[1].pickup_counts[1] == 2

            # Second scenario: 1 request
            @test nrow(data.scenarios[2].requests) == 1
            @test data.scenarios[2].pickup_counts[2] == 1
        end

        @testset "Accessor functions" begin
            stations = DataFrame(id=[10, 20, 30], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
            requests = DataFrame(
                id=[1],
                start_station_id=[10],
                end_station_id=[20],
                request_time=DateTime.(["2025-01-15 10:00:00"], "yyyy-mm-dd HH:MM:SS")
            )
            costs = Dict{Tuple{Int,Int}, Float64}()
            for i in [10, 20, 30], j in [10, 20, 30]
                costs[(i, j)] = i == j ? 0.0 : Float64(abs(i - j))
            end

            data = create_station_selection_data(stations, requests, costs)

            @test get_station_id(data, 1) == 10
            @test get_station_id(data, 2) == 20
            @test get_station_id(data, 3) == 30

            @test get_station_idx(data, 10) == 1
            @test get_station_idx(data, 20) == 2
            @test get_station_idx(data, 30) == 3

            @test get_walking_cost(data, 10, 20) == 10.0
            @test get_walking_cost(data, 10, 30) == 20.0

            @test has_routing_costs(data) == false
        end
    end

    @testset "Model Structs" begin
        @testset "BaseModel" begin
            model = BaseModel(5)
            @test model.k == 5
            @test model.strict_equality == true

            model2 = BaseModel(3; strict_equality=false)
            @test model2.k == 3
            @test model2.strict_equality == false

            @test_throws ArgumentError BaseModel(0)
            @test_throws ArgumentError BaseModel(-1)
        end

        @testset "TwoStageLambdaModel" begin
            model = TwoStageLambdaModel(5)
            @test model.k == 5
            @test model.lambda == 0.0

            model2 = TwoStageLambdaModel(3; lambda=0.5)
            @test model2.k == 3
            @test model2.lambda == 0.5

            @test_throws ArgumentError TwoStageLambdaModel(0)
            @test_throws ArgumentError TwoStageLambdaModel(5; lambda=-1.0)
        end

        @testset "TwoStageLModel" begin
            model = TwoStageLModel(3, 5)
            @test model.k == 3
            @test model.l == 5

            @test_throws ArgumentError TwoStageLModel(0, 5)
            @test_throws ArgumentError TwoStageLModel(5, 3)  # l < k
        end

        @testset "RoutingTransportModel" begin
            model = RoutingTransportModel(3, 5)
            @test model.k == 3
            @test model.l == 5
            @test model.lambda == 1.0

            model2 = RoutingTransportModel(2, 4; lambda=0.5)
            @test model2.lambda == 0.5

            @test_throws ArgumentError RoutingTransportModel(5, 3)
        end
    end

    @testset "Type Hierarchy" begin
        @test BaseModel <: AbstractSingleScenarioModel
        @test BaseModel <: AbstractStationSelectionModel

        @test TwoStageLambdaModel <: AbstractTwoStageModel
        @test TwoStageLambdaModel <: AbstractMultiScenarioModel
        @test TwoStageLambdaModel <: AbstractStationSelectionModel

        @test TwoStageLModel <: AbstractTwoStageModel

        @test RoutingTransportModel <: AbstractRoutingModel
        @test RoutingTransportModel <: AbstractTwoStageModel
    end

    @testset "optimize_model - BaseModel" begin
        # Simple test case matching legacy test
        stations = DataFrame(id=[1, 2], lat=[27.9, 27.91], lon=[113.1, 113.11])
        requests = DataFrame(
            id=[1],
            start_station_id=[1],
            end_station_id=[2],
            request_time=DateTime.(["2025-01-15 10:42:33"], "yyyy-mm-dd HH:MM:SS")
        )
        costs = Dict{Tuple{Int,Int}, Float64}(
            (1, 2) => 5.0, (2, 1) => 10.0, (1, 1) => 0.0, (2, 2) => 0.0
        )

        data = create_station_selection_data(stations, requests, costs)
        model = BaseModel(1)

        result = optimize_model(model, data)

        @test result.status == true
        @test sum(values(result.stations)) == 1
        # Station 2 should be selected (lower total walking cost)
        @test result.stations[1] == false
        @test result.stations[2] == true
    end

    @testset "optimize_model - TwoStageLambdaModel" begin
        stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 2],
            end_station_id=[2, 3],
            request_time=DateTime.(["2025-01-15 10:00:00", "2025-01-16 10:00:00"], "yyyy-mm-dd HH:MM:SS")
        )
        costs = Dict{Tuple{Int,Int}, Float64}()
        for i in 1:3, j in 1:3
            costs[(i, j)] = i == j ? 0.0 : 1.0
        end

        scenarios = [
            ("2025-01-15 00:00:00", "2025-01-15 23:59:59"),
            ("2025-01-16 00:00:00", "2025-01-16 23:59:59")
        ]

        data = create_station_selection_data(stations, requests, costs; scenarios=scenarios)
        model = TwoStageLambdaModel(2; lambda=0.0)

        result = optimize_model(model, data)

        @test result.status == true
        @test sum(values(result.stations)) == 2
        # Result should have scenario columns
        @test ncol(result.station_df) > 4
    end

    @testset "optimize_model - TwoStageLModel" begin
        stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 2],
            end_station_id=[2, 3],
            request_time=DateTime.(["2025-01-15 10:00:00", "2025-01-16 10:00:00"], "yyyy-mm-dd HH:MM:SS")
        )
        costs = Dict{Tuple{Int,Int}, Float64}()
        for i in 1:3, j in 1:3
            costs[(i, j)] = i == j ? 0.0 : 1.0
        end

        scenarios = [
            ("2025-01-15 00:00:00", "2025-01-15 23:59:59"),
            ("2025-01-16 00:00:00", "2025-01-16 23:59:59")
        ]

        data = create_station_selection_data(stations, requests, costs; scenarios=scenarios)

        # l=3 permanent stations, k=2 active per scenario
        model = TwoStageLModel(2, 3)

        result = optimize_model(model, data)

        @test result.status == true
        @test sum(values(result.stations)) == 3  # All 3 stations built
    end

    @testset "optimize_model - RoutingTransportModel" begin
        stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 2],
            end_station_id=[2, 3],
            request_time=DateTime.(["2025-01-15 10:00:00", "2025-01-15 11:00:00"], "yyyy-mm-dd HH:MM:SS")
        )

        walking_costs = Dict{Tuple{Int,Int}, Float64}()
        routing_costs = Dict{Tuple{Int,Int}, Float64}()
        for i in 1:3, j in 1:3
            walking_costs[(i, j)] = i == j ? 0.0 : 1.0
            routing_costs[(i, j)] = i == j ? 0.0 : 2.0  # Routing costs different from walking
        end

        data = create_station_selection_data(
            stations, requests, walking_costs;
            routing_costs=routing_costs
        )

        model = RoutingTransportModel(2, 3; lambda=0.5)

        result = optimize_model(model, data)

        @test result.status == true
        @test has_routing_costs(data) == true
    end

end
