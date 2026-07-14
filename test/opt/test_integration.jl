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

    @testset "TwoStageODPolicy build" begin
        model = ClusteringModel(TwoStageODPolicy(2, 3))

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

    @testset "SingleStagePolicy build" begin
        model = ClusteringModel(SingleStagePolicy(3))

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

    @testset "TwoStagePolicy build" begin
        model = ClusteringModel(TwoStagePolicy(2, 3))

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
        @test haskey(extra_counts, "total_endpoint_groups")
        @test extra_counts["total_endpoint_groups"] > 0

        # Check model variables are accessible
        @test haskey(object_dictionary(m), :y)
        @test haskey(object_dictionary(m), :z)
        @test haskey(object_dictionary(m), :x)
    end

    @testset "run_opt without optimization" begin
        # Test run_opt with do_optimize=false for all policies

        @testset "TwoStageODPolicy" begin
            model = ClusteringModel(TwoStageODPolicy(2, 3))
            result = run_opt(
                data,
                model,
                DirectSolver(
                    optimizer_env=env,
                    silent=true,
                    do_optimize=false,
                )
            )

            @test result.termination_status == MOI.OPTIMIZE_NOT_CALLED
            @test isnothing(result.objective_value)
            @test isnothing(result.solution)
            @test result.model isa JuMP.Model
            @test !isempty(result.counts.variables)
            @test !isempty(result.counts.constraints)
        end

        @testset "SingleStagePolicy" begin
            model = ClusteringModel(SingleStagePolicy(3))
            result = run_opt(
                data,
                model,
                DirectSolver(
                    optimizer_env=env,
                    silent=true,
                    do_optimize=false,
                )
            )

            @test result.termination_status == MOI.OPTIMIZE_NOT_CALLED
            @test isnothing(result.objective_value)
            @test isnothing(result.solution)
            @test result.model isa JuMP.Model
            @test !isempty(result.counts.variables)
            @test !isempty(result.counts.constraints)
        end

        @testset "TwoStagePolicy" begin
            model = ClusteringModel(TwoStagePolicy(2, 3))
            result = run_opt(
                data,
                model,
                DirectSolver(
                    optimizer_env=env,
                    silent=true,
                    do_optimize=false,
                )
            )

            @test result.termination_status == MOI.OPTIMIZE_NOT_CALLED
            @test isnothing(result.objective_value)
            @test isnothing(result.solution)
            @test result.model isa JuMP.Model
            @test !isempty(result.counts.variables)
            @test !isempty(result.counts.constraints)
        end
    end

    @testset "Mapping creation" begin
        @testset "ClusteringTwoStageODMap" begin
            model = ClusteringModel(TwoStageODPolicy(2, 3))
            mapping = StationSelection.create_map(model, data)

            @test length(mapping.station_id_to_array_idx) == 5
            @test length(mapping.array_idx_to_station_id) == 5
            @test length(mapping.scenarios) == 1
            @test haskey(mapping.Omega_s, 1)
            @test haskey(mapping.Q_s, 1)
            @test length(mapping.Omega_s[1]) > 0  # Has OD pairs
        end

        @testset "ClusteringBaseModelMap" begin
            model = ClusteringModel(SingleStagePolicy(3))
            mapping = StationSelection.create_map(model, data)

            @test length(mapping.station_id_to_array_idx) == 5
            @test length(mapping.array_idx_to_station_id) == 5
            @test mapping.n_stations == 5
            @test !isempty(mapping.request_counts)

            # Check request counts include both pickups and dropoffs
            total_requests = sum(values(mapping.request_counts))
            @test total_requests == 12  # 6 requests × 2 (pickup + dropoff)
        end

        @testset "ClusteringTwoStageStationMap" begin
            model = ClusteringModel(TwoStagePolicy(2, 3))
            mapping = StationSelection.create_map(model, data)

            @test length(mapping.station_id_to_array_idx) == 5
            @test length(mapping.array_idx_to_station_id) == 5
            @test length(mapping.scenarios) == 1
            @test haskey(mapping.I_s, 1)
            @test haskey(mapping.q_s, 1)
            @test length(mapping.I_s[1]) > 0
        end
    end

    @testset "Model construction validation" begin
        @testset "TwoStageODPolicy" begin
            @test_throws ArgumentError TwoStageODPolicy(0, 5)   # k must be positive
            @test_throws ArgumentError TwoStageODPolicy(5, 3)   # l must be >= k
            @test_throws ArgumentError TwoStageODPolicy(3, 5; in_vehicle_time_weight=-1.0)   # in_vehicle_time_weight must be non-negative
        end

        @testset "SingleStagePolicy" begin
            @test_throws ArgumentError SingleStagePolicy(0)   # k must be positive
            @test_throws ArgumentError SingleStagePolicy(-1)  # k must be positive
        end

        @testset "TwoStagePolicy" begin
            @test_throws ArgumentError TwoStagePolicy(0, 5)   # k must be positive
            @test_throws ArgumentError TwoStagePolicy(5, 3)   # l must be >= k
        end
    end
end
