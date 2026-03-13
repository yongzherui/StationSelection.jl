# Check if Gurobi is available
gurobi_available = try
    using Gurobi
    true
catch
    false
end

@testset "Solution Analysis" begin
    using JuMP

    if !gurobi_available
        @warn "Gurobi not available, skipping solution analysis tests"
        @test true
        return
    end

    """
    Test Setup:

    We create a simple network with 4 stations arranged in a line:

        Station 1 --- Station 2 --- Station 3 --- Station 4
           (0)          (1)          (2)          (3)

    Walking costs: distance * 100 (simulates meters)
    Routing costs: distance * 50 (simulates travel time in seconds)

    Test Requests (all in same time window, 2-minute window):
    - Order 1: 1→3 at 08:00:00 (origin station 1, dest station 3)
    - Order 2: 1→4 at 08:00:30 (origin station 1, dest station 4)
    - Order 3: 2→4 at 08:01:00 (origin station 2, dest station 4)

    With k=3, l=4 stations (select 3, activate 4):
    - The model should select stations that minimize walking + routing costs

    Expected behavior:
    - Orders should be assigned to nearby stations
    - Walking distance = distance from origin to pickup + dropoff to destination
    - In-vehicle time = routing cost from pickup to dropoff

    Note that there are two kinds of solutions here. we could have 1, 3, 4 or also 1, 2, 4 (solver chooses 1, 2, 4)

    1, 3, 4 is also valid since only order 3 will have to walk. Likewise for 1, 2, 4 only order 1 will have to walk.
    """

    # Create test data - 4 stations in a line
    stations = DataFrame(
        id = [1, 2, 3, 4],
        lon = [113.0, 113.1, 113.2, 113.3],
        lat = [28.0, 28.0, 28.0, 28.0]
    )

    # 3 requests in the same time window
    requests = DataFrame(
        order_id = [1, 2, 3],
        start_station_id = [1, 1, 2],  # Origins
        end_station_id = [3, 4, 4],    # Destinations
        request_time = [
            DateTime(2024, 1, 1, 8, 0, 0),   # 0 seconds from start
            DateTime(2024, 1, 1, 8, 0, 30),  # 30 seconds from start
            DateTime(2024, 1, 1, 8, 1, 0)    # 60 seconds from start
        ]
    )

    # Walking cost: |i - j| * 200 (high cost to penalize walking)
    # Routing cost: |i - j| * 10 (low cost - vehicle travel is efficient)
    # This ratio ensures the optimizer prefers vehicle use over walking
    walking_costs = Dict{Tuple{Int,Int}, Float64}()
    routing_costs = Dict{Tuple{Int,Int}, Float64}()
    for i in 1:4, j in 1:4
        walking_costs[(i, j)] = abs(i - j) * 200.0
        routing_costs[(i, j)] = abs(i - j) * 10.0
    end

    # Single scenario covering all requests
    scenarios = [("2024-01-01 08:00:00", "2024-01-01 09:00:00")]

    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )

    # Create Gurobi environment
    env = Gurobi.Env()

    @testset "ClusteringTwoStageODModel annotation" begin
        model = ClusteringTwoStageODModel(3, 4)

        result = run_opt(model, data; optimizer_env=env, silent=true)

        @test result.termination_status == MOI.OPTIMAL

        # Annotate orders
        annotated = annotate_orders_with_solution(result, data)

        # Check that all orders are annotated
        @test nrow(annotated) == 3

        # Check columns exist (no pooling columns for this model)
        @test :scenario_idx in propertynames(annotated)
        @test :assigned_pickup_id in propertynames(annotated)
        @test :assigned_dropoff_id in propertynames(annotated)
        @test :walking_distance_total in propertynames(annotated)
        @test :in_vehicle_time_direct in propertynames(annotated)
        @test :in_vehicle_time_actual in propertynames(annotated)

        # No pooling columns
        @test !(:is_pooled in propertynames(annotated))

        # All orders should have assignments
        @test all(!ismissing, annotated.assigned_pickup_id)
        @test all(!ismissing, annotated.assigned_dropoff_id)

        println("\n--- ClusteringTwoStageODModel Annotation Results ---")
        for row in eachrow(annotated)
            println("Order $(row.order_id): $(row.start_station_id)→$(row.end_station_id)")
            println("  Assigned: $(row.assigned_pickup_id)→$(row.assigned_dropoff_id)")
            println("  Walking: total=$(row.walking_distance_total)")
            println("  In-vehicle: $(row.in_vehicle_time_actual)")
        end

        # Verify consistency
        for row in eachrow(annotated)
            @test row.in_vehicle_time_actual ≈ row.in_vehicle_time_direct
        end

        # Test vehicle routing distance (no pooling difference for this model)
        vrd = calculate_model_vehicle_routing_distance(result, data)
        println("\nVehicle routing distance: $vrd")
        @test vrd >= 0
    end

end
