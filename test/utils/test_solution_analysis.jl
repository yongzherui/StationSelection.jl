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

    @testset "TwoStageSingleDetourModel annotation" begin
        # Use k=3, l=4 (select 3 stations, activate up to 4)
        # time_window=120 (2 minutes) - all requests in same time window
        # routing_delay=60 (1 minute) - for potential pooling across time windows
        model = TwoStageSingleDetourModel(
            3, 4,           # k, l
            1.0,            # vehicle_routing_weight
            120.0,          # time_window (2 min)
            60.0            # routing_delay (1 min)
        )

        # Run optimization
        result = run_opt(model, data; optimizer_env=env, silent=true)

        @test result.termination_status == MOI.OPTIMAL

        # Annotate orders
        annotated = annotate_orders_with_solution(result, data)

        # Check that all orders are annotated
        @test nrow(annotated) == 3

        # Check columns exist
        @test :scenario_idx in propertynames(annotated)
        @test :time_id in propertynames(annotated)
        @test :assigned_pickup_id in propertynames(annotated)
        @test :assigned_dropoff_id in propertynames(annotated)
        @test :walking_distance_pickup in propertynames(annotated)
        @test :walking_distance_dropoff in propertynames(annotated)
        @test :walking_distance_total in propertynames(annotated)
        @test :in_vehicle_time_direct in propertynames(annotated)
        @test :is_pooled in propertynames(annotated)
        @test :in_vehicle_time_actual in propertynames(annotated)

        # All orders should have assignments (not missing)
        @test all(!ismissing, annotated.assigned_pickup_id)
        @test all(!ismissing, annotated.assigned_dropoff_id)

        # Print results for inspection
        println("\n--- TwoStageSingleDetourModel Annotation Results ---")
        for row in eachrow(annotated)
            println("Order $(row.order_id): $(row.start_station_id)→$(row.end_station_id)")
            println("  Assigned: $(row.assigned_pickup_id)→$(row.assigned_dropoff_id)")
            println("  Walking: pickup=$(row.walking_distance_pickup), dropoff=$(row.walking_distance_dropoff), total=$(row.walking_distance_total)")
            println("  In-vehicle: direct=$(row.in_vehicle_time_direct), actual=$(row.in_vehicle_time_actual)")
            println("  Pooled: $(row.is_pooled)" * (row.is_pooled ? " ($(row.pooling_type), $(row.pooling_role))" : ""))
        end

        # Verify walking distance calculation
        for row in eachrow(annotated)
            expected_walk_pickup = abs(row.start_station_id - row.assigned_pickup_id) * 200.0
            expected_walk_dropoff = abs(row.assigned_dropoff_id - row.end_station_id) * 200.0
            @test row.walking_distance_pickup ≈ expected_walk_pickup
            @test row.walking_distance_dropoff ≈ expected_walk_dropoff
            @test row.walking_distance_total ≈ expected_walk_pickup + expected_walk_dropoff
        end

        # Verify in-vehicle time calculation (direct)
        for row in eachrow(annotated)
            expected_ivt = abs(row.assigned_pickup_id - row.assigned_dropoff_id) * 10.0
            @test row.in_vehicle_time_direct ≈ expected_ivt
        end

        # Test aggregation functions
        total_walking = calculate_model_walking_distance(annotated)
        total_ivt = calculate_model_in_vehicle_time(annotated)

        @test total_walking == sum(annotated.walking_distance_total)
        @test total_ivt == sum(annotated.in_vehicle_time_actual)

        println("\nTotal walking distance: $total_walking")
        println("Total in-vehicle time: $total_ivt")

        # Test vehicle routing distance
        vrd_with_pooling = calculate_model_vehicle_routing_distance(result, data; with_pooling=true)
        vrd_without_pooling = calculate_model_vehicle_routing_distance(result, data; with_pooling=false)

        println("Vehicle routing distance (with pooling): $vrd_with_pooling")
        println("Vehicle routing distance (without pooling): $vrd_without_pooling")

        # With pooling should be <= without pooling (pooling saves vehicle distance)
        @test vrd_with_pooling <= vrd_without_pooling + 1e-6
    end

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

    @testset "Pooling scenario test" begin
        # Create scenario more likely to trigger pooling
        # Two orders with same origin going to different destinations
        pooling_requests = DataFrame(
            order_id = [1, 2],
            start_station_id = [1, 1],  # Same origin
            end_station_id = [3, 4],    # Different destinations (3 and 4)
            request_time = [
                DateTime(2024, 1, 1, 8, 0, 0),
                DateTime(2024, 1, 1, 8, 0, 10)  # Same time window (within 120s)
            ]
        )

        pooling_data = StationSelection.create_station_selection_data(
            stations, pooling_requests, walking_costs;
            routing_costs=routing_costs,
            scenarios=scenarios
        )

        model = TwoStageSingleDetourModel(
            3, 4,           # k, l
            1.0,            # vehicle_routing_weight
            120.0,          # time_window
            60.0            # routing_delay
        )

        result = run_opt(model, pooling_data; optimizer_env=env, silent=true)

        if result.termination_status == MOI.OPTIMAL
            annotated = annotate_orders_with_solution(result, pooling_data)

            println("\n--- Pooling Scenario Results ---")
            n_pooled = sum(annotated.is_pooled)
            println("Pooled orders: $n_pooled / $(nrow(annotated))")

            for row in eachrow(annotated)
                println("Order $(row.order_id): pooled=$(row.is_pooled)")
                if row.is_pooled
                    println("  Type: $(row.pooling_type), Role: $(row.pooling_role)")
                    println("  Triplet: ($(row.pooling_j_id), $(row.pooling_k_id), $(row.pooling_l_id))")
                    println("  Detour time: $(row.detour_time)")
                end
            end
        end
    end

    @testset "Edge cases" begin
        # Test with single order
        # With k=2, l=2 and cost structure favoring vehicle use,
        # the order 1→4 should be assigned to stations 1→4 with minimal walking
        #
        # NOTE: Cost structure matters for avoiding degenerate solutions!
        # With walking=100*dist and routing=50*dist, the objective includes:
        #   - Assignment cost: walking + in_vehicle_time_weight * routing
        #   - Flow cost: vehicle_routing_weight * routing
        # When weights=1.0, total vehicle cost = 2 * routing = 100*dist = walking cost
        #
        # This creates multiple equivalent optima for order 1→4:
        # | Assignment | Walking      | IVT   | Flow  | Total |
        # |------------|--------------|-------|-------|-------|
        # | 1→4        | 0+0=0        | 150   | 150   | 300   |
        # | 3→4        | 200+0=200    | 50    | 50    | 300   |
        # | 3→3        | 200+100=300  | 0     | 0     | 300   |
        # | 4→4        | 300+0=300    | 0     | 0     | 300   |
        #
        # The optimizer may choose any of these, including 4→4 where the
        # customer walks the entire distance with no vehicle ride!
        #
        # Our test uses walking=200*dist, routing=10*dist to ensure walking
        # is expensive relative to vehicle use, producing intuitive results.
        single_request = DataFrame(
            order_id = [1],
            start_station_id = [1],
            end_station_id = [4],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)]
        )

        single_data = StationSelection.create_station_selection_data(
            stations, single_request, walking_costs;
            routing_costs=routing_costs,
            scenarios=scenarios
        )

        model = TwoStageSingleDetourModel(2, 2, 1.0, 120.0, 60.0)
        result = run_opt(model, single_data; optimizer_env=env, silent=true)

        if result.termination_status == MOI.OPTIMAL
            annotated = annotate_orders_with_solution(result, single_data)

            @test nrow(annotated) == 1
            @test annotated.is_pooled[1] == false  # Single order cannot be pooled

            # With the cost structure favoring vehicle use,
            # the optimal solution should minimize walking
            @test annotated.assigned_pickup_id[1] == 1  # Pickup at origin station
            @test annotated.assigned_dropoff_id[1] == 4  # Dropoff at destination station
            @test annotated.walking_distance_total[1] ≈ 0.0  # No walking needed

            println("\n--- Single Order Test ---")
            println("Order assigned: $(annotated.assigned_pickup_id[1])→$(annotated.assigned_dropoff_id[1])")
            println("Walking: $(annotated.walking_distance_total[1])")
            println("In-vehicle: $(annotated.in_vehicle_time_actual[1])")

        end
    end
end
