using Test
using DataFrames
using Dates
using CSV
using JuMP
const MOI = JuMP.MOI

@testset "Exact DARP Route Runner" begin
    function make_alpha_test_data()
        stations = DataFrame(
            id = [1, 2, 3],
            lon = [0.0, 1.0, 2.0],
            lat = [0.0, 0.0, 0.0],
        )
        requests = DataFrame(
            order_id = [1, 2],
            pax_num = [1, 1],
            start_station_id = [1, 1],
            end_station_id = [3, 3],
            request_time = [
                DateTime(2024, 1, 1, 8, 0, 0),
                DateTime(2024, 1, 1, 8, 10, 0),
            ],
        )

        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:3, j in 1:3
            walking_costs[(i, j)] = abs(i - j) * 60.0
            routing_costs[(i, j)] = abs(i - j) * 30.0
        end

        return StationSelection.create_station_selection_data(
            stations,
            requests,
            walking_costs;
            routing_costs=routing_costs,
            scenarios=[("2024-01-01 08:00:00", "2024-01-01 09:00:00")],
        )
    end

    @testset "combined initialization keeps same sequence variants with distinct alpha" begin
        data = make_alpha_test_data()
        model = StationSelection.ExactDARPRouteModel(
            2, 2;
            generate_routes=true,
            max_route_length=2,
            max_walking_distance=1000.0,
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            time_window_sec=3600,
            vehicle_capacity=18,
        )
        base = StationSelection._build_exact_darp_route_base(model, data)

        mktempdir() do tmpdir
            routes_df = DataFrame(
                route_id = [10],
                station_ids = ["1|3"],
                travel_time = [60.0],
            )
            alpha_df = DataFrame(
                route_id = [10],
                pickup_id = [1],
                dropoff_id = [3],
                value = [9.0],
            )
            routes_file = joinpath(tmpdir, "routes_input.csv")
            alpha_file = joinpath(tmpdir, "alpha_profile.csv")
            CSV.write(routes_file, routes_df)
            CSV.write(alpha_file, alpha_df)

            init_spec = StationSelection.RoutePoolInitSpec(
                :combined;
                routes_file=routes_file,
                alpha_profile_file=alpha_file
            )
            state = StationSelection.initialize_route_pool(
                init_spec,
                data,
                base.Q_s_t,
                base.valid_jk_pairs;
                vehicle_capacity=18,
                max_detour_time=3600.0,
                max_detour_ratio=10.0,
                stop_dwell_time=10.0,
                initial_generated_max_route_length=2
            )

            bucket_state = state.bucket_states[(1, 0)]
            direct_variants = [r for r in values(bucket_state.routes_by_id) if r.station_indices == [1, 3]]
            @test length(direct_variants) >= 2
        end
    end

    @testset "injected route pool builds alpha map" begin
        data = make_alpha_test_data()
        model = StationSelection.ExactDARPRouteModel(
            2, 2;
            generate_routes=true,
            max_route_length=2,
            max_walking_distance=1000.0,
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            time_window_sec=3600,
            vehicle_capacity=18,
        )
        base = StationSelection._build_exact_darp_route_base(model, data)
        init_spec = StationSelection.RoutePoolInitSpec(:generated)
        state = StationSelection.initialize_route_pool(
            init_spec,
            data,
            base.Q_s_t,
            base.valid_jk_pairs;
            vehicle_capacity=18,
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            stop_dwell_time=10.0,
            initial_generated_max_route_length=2
        )
        mapping = StationSelection.create_exact_darp_route_od_map(model, data, state)
        @test !isempty(mapping.routes_s[1][0])
        @test !isempty(mapping.alpha_profile)
        bucket_state = state.bucket_states[(1, 0)]
        @test !isempty(bucket_state.direct_seed_route_ids)
        @test issubset(bucket_state.direct_seed_route_ids, bucket_state.protected_route_ids)
    end

    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end

    @testset "iterative runner smoke test" begin
        if !gurobi_available
            @test true
        else
            data = make_alpha_test_data()
            model = StationSelection.ExactDARPRouteModel(
                2, 2;
                generate_routes=true,
                max_route_length=2,
                max_walking_distance=1000.0,
                max_detour_time=3600.0,
                max_detour_ratio=10.0,
                time_window_sec=3600,
                vehicle_capacity=18,
            )
            config = StationSelection.ExactDARPRouteRunnerConfig(
                StationSelection.RoutePoolInitSpec(:generated);
                iterative=true,
                max_iterations=2,
                route_length_schedule=[2, 3],
                prune_enabled=true,
                expand_enabled=true,
            )

            runner_result = StationSelection.run_exact_darp_route_iterative(
                model,
                data,
                config;
                silent=true,
                warm_start=false
            )
            @test runner_result.final_result.termination_status == MOI.OPTIMAL
            @test !isempty(runner_result.iterations)
            @test hasfield(StationSelection.ExactDARPRouteIterationSummary, :objective_delta)
            @test hasfield(StationSelection.ExactDARPRouteIterationSummary, :relative_objective_improvement)
            @test hasfield(StationSelection.ExactDARPRouteIterationSummary, :build_time_sec)
            @test hasfield(StationSelection.ExactDARPRouteIterationSummary, :warm_start_time_sec)
            @test hasfield(StationSelection.ExactDARPRouteIterationSummary, :solve_time_sec)
            @test hasfield(StationSelection.ExactDARPRouteIterationSummary, :runtime_sec)
            @test hasproperty(runner_result.iterations[1], :runtime_sec)

            direct_result = StationSelection.run_opt(
                data,
                model,
                StationSelection.HeuristicSolver(
                    config=StationSelection.SolverConfig(silent=true, warm_start=false),
                    init_spec=config.init_spec,
                    max_iterations=config.max_iterations,
                    route_length_schedule=config.route_length_schedule,
                    prune_enabled=config.prune_enabled,
                    expand_enabled=config.expand_enabled,
                    min_active_value_to_keep=config.min_theta_to_keep,
                    pool_target_size=config.route_pool_target_size,
                    bucket_multiplier=config.route_pool_bucket_x_multiplier,
                    random_retention_seed=config.random_retention_seed,
                    objective_improvement_tol=config.objective_improvement_tol,
                    pool_change_tol=config.route_pool_change_tol,
                    export_iteration_artifacts=config.export_iteration_artifacts,
                    enrichment=config.enrichment,
                )
            )
            @test direct_result.termination_status == MOI.OPTIMAL
            @test haskey(direct_result.metadata, "exact_darp_route_runner")
            @test haskey(direct_result.metadata["exact_darp_route_runner"]["iterations"][1], "objective_delta")
            @test haskey(direct_result.metadata["exact_darp_route_runner"]["iterations"][1], "relative_objective_improvement")
            @test haskey(direct_result.metadata["exact_darp_route_runner"]["iterations"][1], "build_time_sec")
            @test haskey(direct_result.metadata["exact_darp_route_runner"]["iterations"][1], "warm_start_time_sec")
            @test haskey(direct_result.metadata["exact_darp_route_runner"]["iterations"][1], "solve_time_sec")
            @test haskey(direct_result.metadata["exact_darp_route_runner"]["iterations"][1], "runtime_sec")
        end
    end

    @testset "route pool target size validates direct-route feasibility floor" begin
        data = make_alpha_test_data()
        model = StationSelection.ExactDARPRouteModel(
            2, 2;
            generate_routes=true,
            max_route_length=2,
            max_walking_distance=1000.0,
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            time_window_sec=3600,
            vehicle_capacity=18,
        )
        config = StationSelection.ExactDARPRouteRunnerConfig(
            StationSelection.RoutePoolInitSpec(:generated);
            iterative=true,
            max_iterations=1,
            route_length_schedule=[2],
            prune_enabled=true,
            expand_enabled=false,
            route_pool_target_size=1,
        )

        @test_throws ArgumentError StationSelection.run_exact_darp_route_iterative(
            model,
            data,
            config;
            do_optimize=false
        )
    end
end
