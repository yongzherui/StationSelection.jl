# Check if Gurobi is available
gurobi_available = try
    using Gurobi
    true
catch
    false
end

@testset "Model Integration" begin
    using JuMP

    if !gurobi_available
        @warn "Gurobi not available, skipping integration tests"
        @test true  # Placeholder to avoid empty testset
        return
    end

    # Create test data that works for all models
    stations = DataFrame(
        id = [1, 2, 3, 4, 5],
        lon = [113.0, 113.1, 113.2, 113.3, 113.4],
        lat = [28.0, 28.1, 28.2, 28.3, 28.4]
    )

    requests = DataFrame(
        id = [1, 2, 3, 4, 5, 6],
        start_station_id = [1, 1, 2, 3, 4, 2],
        end_station_id = [2, 3, 3, 4, 5, 5],
        request_time = [
            DateTime(2024, 1, 1, 8, 0, 0),
            DateTime(2024, 1, 1, 8, 1, 0),
            DateTime(2024, 1, 1, 8, 2, 0),
            DateTime(2024, 1, 1, 8, 3, 0),
            DateTime(2024, 1, 1, 8, 4, 0),
            DateTime(2024, 1, 1, 8, 5, 0)
        ]
    )

    # Create costs
    walking_costs = Dict{Tuple{Int,Int}, Float64}()
    routing_costs = Dict{Tuple{Int,Int}, Float64}()
    for i in 1:5, j in 1:5
        walking_costs[(i, j)] = abs(i - j) * 100.0
        routing_costs[(i, j)] = abs(i - j) * 50.0
    end

    # Create scenario
    scenarios = [("2024-01-01 08:00:00", "2024-01-01 09:00:00")]

    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )

    # Create Gurobi environment once
    env = Gurobi.Env()

    @testset "TwoStageSingleDetourModel build" begin
        model = TwoStageSingleDetourModel(2, 3, 1.0, 120.0, 60.0; max_walking_distance=500.0)

        build_result = StationSelection.build_model(
            model, data; optimizer_env=env
        )
        m = build_result.model
        var_counts = build_result.counts.variables
        con_counts = build_result.counts.constraints
        extra_counts = build_result.counts.extras

        # Check that model was created
        @test m isa JuMP.Model

        # Check variables exist
        @test haskey(var_counts, "station_selection")
        @test haskey(var_counts, "scenario_activation")
        @test haskey(var_counts, "assignment")
        @test haskey(var_counts, "flow")
        @test haskey(var_counts, "detour")

        @test var_counts["station_selection"] == 5  # n stations
        @test var_counts["scenario_activation"] == 5  # n * S = 5 * 1

        # Check constraints exist
        @test haskey(con_counts, "station_limit")
        @test haskey(con_counts, "scenario_activation_limit")
        @test haskey(con_counts, "activation_linking")
        @test haskey(con_counts, "assignment")
        @test haskey(con_counts, "assignment_to_active")
        @test haskey(con_counts, "assignment_to_flow")

        @test con_counts["station_limit"] == 1
        @test con_counts["scenario_activation_limit"] == 1  # S scenarios
        @test con_counts["activation_linking"] == 5  # n * S

        # Check extra counts
        @test haskey(extra_counts, "same_source")
        @test haskey(extra_counts, "same_dest")

        # Check model variables are accessible
        @test haskey(object_dictionary(m), :y)
        @test haskey(object_dictionary(m), :z)
        @test haskey(object_dictionary(m), :x)
        @test haskey(object_dictionary(m), :f)
    end

    @testset "ClusteringTwoStageODModel build" begin
        model = ClusteringTwoStageODModel(2, 3, 1.0)

        build_result = StationSelection.build_model(
            model, data; optimizer_env=env
        )
        m = build_result.model
        var_counts = build_result.counts.variables
        con_counts = build_result.counts.constraints
        extra_counts = build_result.counts.extras

        # Check that model was created
        @test m isa JuMP.Model

        # Check variables exist
        @test haskey(var_counts, "station_selection")
        @test haskey(var_counts, "scenario_activation")
        @test haskey(var_counts, "assignment")

        @test var_counts["station_selection"] == 5  # n stations
        @test var_counts["scenario_activation"] == 5  # n * S = 5 * 1

        # Check constraints exist
        @test haskey(con_counts, "station_limit")
        @test haskey(con_counts, "scenario_activation_limit")
        @test haskey(con_counts, "activation_linking")
        @test haskey(con_counts, "assignment")
        @test haskey(con_counts, "assignment_to_active")

        @test con_counts["station_limit"] == 1
        @test con_counts["scenario_activation_limit"] == 1
        @test con_counts["activation_linking"] == 5

        # Check extra counts
        @test haskey(extra_counts, "total_od_pairs")
        @test extra_counts["total_od_pairs"] > 0

        # Check model variables are accessible
        @test haskey(object_dictionary(m), :y)
        @test haskey(object_dictionary(m), :z)
        @test haskey(object_dictionary(m), :x)
    end

    @testset "ClusteringBaseModel build" begin
        model = ClusteringBaseModel(3)

        build_result = StationSelection.build_model(
            model, data; optimizer_env=env
        )
        m = build_result.model
        var_counts = build_result.counts.variables
        con_counts = build_result.counts.constraints
        extra_counts = build_result.counts.extras

        # Check that model was created
        @test m isa JuMP.Model

        # Check variables exist
        @test haskey(var_counts, "station_selection")
        @test haskey(var_counts, "assignment")

        @test var_counts["station_selection"] == 5  # n stations
        @test var_counts["assignment"] == 25  # n * n = 5 * 5

        # Check constraints exist
        @test haskey(con_counts, "station_limit")
        @test haskey(con_counts, "assignment")
        @test haskey(con_counts, "assignment_to_selected")

        @test con_counts["station_limit"] == 1
        @test con_counts["assignment"] == 5  # n stations
        @test con_counts["assignment_to_selected"] == 25  # n * n

        # Check extra counts
        @test haskey(extra_counts, "total_requests")
        @test extra_counts["total_requests"] > 0

        # Check model variables are accessible
        @test haskey(object_dictionary(m), :y)
        @test haskey(object_dictionary(m), :x)
    end

    @testset "run_opt without optimization" begin
        # Test run_opt with do_optimize=false for all three models

        @testset "TwoStageSingleDetourModel" begin
            model = TwoStageSingleDetourModel(2, 3, 1.0, 120.0, 60.0; max_walking_distance=500.0)
            result = run_opt(
                model, data;
                optimizer_env=env,
                silent=true,
                do_optimize=false
            )

            @test result.termination_status == MOI.OPTIMIZE_NOT_CALLED
            @test isnothing(result.objective_value)
            @test isnothing(result.solution)
            @test result.model isa JuMP.Model
            @test !isempty(result.counts.variables)
            @test !isempty(result.counts.constraints)
        end

        @testset "ClusteringTwoStageODModel" begin
            model = ClusteringTwoStageODModel(2, 3, 1.0)
            result = run_opt(
                model, data;
                optimizer_env=env,
                silent=true,
                do_optimize=false
            )

            @test result.termination_status == MOI.OPTIMIZE_NOT_CALLED
            @test isnothing(result.objective_value)
            @test isnothing(result.solution)
            @test result.model isa JuMP.Model
            @test !isempty(result.counts.variables)
            @test !isempty(result.counts.constraints)
        end

        @testset "ClusteringBaseModel" begin
            model = ClusteringBaseModel(3)
            result = run_opt(
                model, data;
                optimizer_env=env,
                silent=true,
                do_optimize=false
            )

            @test result.termination_status == MOI.OPTIMIZE_NOT_CALLED
            @test isnothing(result.objective_value)
            @test isnothing(result.solution)
            @test result.model isa JuMP.Model
            @test !isempty(result.counts.variables)
            @test !isempty(result.counts.constraints)
        end
    end

    @testset "Warm start" begin
        model = TwoStageSingleDetourModel(2, 3, 1.0, 120.0, 60.0; max_walking_distance=500.0)

        warm_start_solution = StationSelection.get_warm_start_solution(
            model,
            data;
            optimizer_env=env,
            silent=true,
            show_counts=false
        )

        @test haskey(warm_start_solution, :x)
        @test haskey(warm_start_solution, :y)
        @test haskey(warm_start_solution, :z)
        @test haskey(warm_start_solution, :f)
        @test haskey(warm_start_solution, :u)
        @test haskey(warm_start_solution, :v)
        @test haskey(warm_start_solution, :mapping)

        build_result = StationSelection.build_model(model, data; optimizer_env=env)
        StationSelection.apply_warm_start!(build_result.model, warm_start_solution)

        # Check a couple of start values to ensure they were applied.
        @test JuMP.start_value(build_result.model[:y][1]) == warm_start_solution[:y][1]
        @test JuMP.start_value(build_result.model[:z][1, 1]) == warm_start_solution[:z][1, 1]

        first_time = first(keys(build_result.model[:x][1]))
        first_od = first(keys(build_result.model[:x][1][first_time]))
        x_vars = build_result.model[:x][1][first_time][first_od]
        x_vals = warm_start_solution[:x][1][first_time][first_od]
        @test JuMP.start_value(x_vars[1]) == x_vals[1]

        if !isempty(build_result.model[:u][1][first_time])
            @test JuMP.start_value(build_result.model[:u][1][first_time][1]) ==
                warm_start_solution[:u][1][first_time][1]
        end
        if !isempty(build_result.model[:v][1][first_time])
            @test JuMP.start_value(build_result.model[:v][1][first_time][1]) ==
                warm_start_solution[:v][1][first_time][1]
        end
    end

    @testset "Mapping creation" begin
        @testset "TwoStageSingleDetourMap" begin
            model = TwoStageSingleDetourModel(2, 3, 1.0, 120.0, 60.0; max_walking_distance=500.0)
            mapping = StationSelection.create_map(model, data)

            @test length(mapping.station_id_to_array_idx) == 5
            @test length(mapping.array_idx_to_station_id) == 5
            @test mapping.time_window == 120
            @test length(mapping.scenarios) == 1
            @test haskey(mapping.Omega_s_t, 1)
            @test haskey(mapping.Q_s_t, 1)
        end

        @testset "ClusteringTwoStageODMap" begin
            model = ClusteringTwoStageODModel(2, 3, 1.0)
            mapping = StationSelection.create_map(model, data)

            @test length(mapping.station_id_to_array_idx) == 5
            @test length(mapping.array_idx_to_station_id) == 5
            @test length(mapping.scenarios) == 1
            @test haskey(mapping.Omega_s, 1)
            @test haskey(mapping.Q_s, 1)
            @test length(mapping.Omega_s[1]) > 0  # Has OD pairs
        end

        @testset "ClusteringBaseModelMap" begin
            model = ClusteringBaseModel(3)
            mapping = StationSelection.create_map(model, data)

            @test length(mapping.station_id_to_array_idx) == 5
            @test length(mapping.array_idx_to_station_id) == 5
            @test mapping.n_stations == 5
            @test !isempty(mapping.request_counts)

            # Check request counts include both pickups and dropoffs
            total_requests = sum(values(mapping.request_counts))
            @test total_requests == 12  # 6 requests × 2 (pickup + dropoff)
        end
    end

    @testset "Model construction validation" begin
        @testset "TwoStageSingleDetourModel" begin
            @test_throws ArgumentError TwoStageSingleDetourModel(0, 5, 1.0, 120.0, 60.0; max_walking_distance=500.0)  # k must be positive
            @test_throws ArgumentError TwoStageSingleDetourModel(5, 3, 1.0, 120.0, 60.0; max_walking_distance=500.0)  # l must be >= k
            @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, -1.0, 120.0, 60.0; max_walking_distance=500.0) # vehicle_routing_weight must be non-negative
            @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, 1.0, 120.0, 60.0; in_vehicle_time_weight=-1.0, max_walking_distance=500.0) # in_vehicle_time_weight must be non-negative
            @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, 1.0, 0.0, 60.0; max_walking_distance=500.0)   # time_window must be positive
            @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, 1.0, 120.0, -1.0; max_walking_distance=500.0) # routing_delay must be non-negative
            @test_throws ArgumentError TwoStageSingleDetourModel(3, 5, 1.0, 120.0, 60.0; max_walking_distance=-1.0) # max_walking_distance must be non-negative
        end

        @testset "ClusteringTwoStageODModel" begin
            @test_throws ArgumentError ClusteringTwoStageODModel(0, 5, 1.0)   # k must be positive
            @test_throws ArgumentError ClusteringTwoStageODModel(5, 3, 1.0)   # l must be >= k
            @test_throws ArgumentError ClusteringTwoStageODModel(3, 5, -1.0)   # vehicle_routing_weight must be non-negative
            @test_throws ArgumentError ClusteringTwoStageODModel(3, 5, 1.0; in_vehicle_time_weight=-1.0)   # in_vehicle_time_weight must be non-negative
        end

        @testset "ClusteringBaseModel" begin
            @test_throws ArgumentError ClusteringBaseModel(0)   # k must be positive
            @test_throws ArgumentError ClusteringBaseModel(-1)  # k must be positive
        end
    end
end
