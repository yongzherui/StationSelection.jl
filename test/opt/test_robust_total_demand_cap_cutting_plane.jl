@testset "RobustTotalDemandCapModel cutting-plane" begin
    using Gurobi

    stations = DataFrame(
        id = [1, 2, 3],
        lon = [113.0, 113.1, 113.2],
        lat = [28.0, 28.1, 28.2],
    )

    requests = DataFrame(
        id = [1, 2, 3, 4],
        start_station_id = [1, 1, 2, 3],
        end_station_id = [2, 3, 3, 1],
        request_time = [
            DateTime(2024, 1, 1, 8, 0, 0),
            DateTime(2024, 1, 1, 8, 5, 0),
            DateTime(2024, 1, 1, 8, 10, 0),
            DateTime(2024, 1, 1, 8, 15, 0),
        ],
    )

    walking_costs = Dict{Tuple{Int, Int}, Float64}()
    routing_costs = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:3, j in 1:3
        walking_costs[(i, j)] = abs(i - j) * 10.0
        routing_costs[(i, j)] = abs(i - j) * 5.0
    end

    scenarios = [("2024-01-01 08:00:00", "2024-01-01 09:00:00")]
    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios,
    )
    env_local = Gurobi.Env()

    q_hat = Dict(1 => Dict((1, 2) => 2.0, (1, 3) => 5.0, (2, 3) => 1.0, (3, 1) => 4.0))
    B = [6.0]

    @testset "builder and solve mode" begin
        model = RobustTotalDemandCapModel(
            2, 3;
            in_vehicle_time_weight=1.0,
            max_walking_distance=100.0,
            q_hat=q_hat,
            B=B,
            solve_mode=:cutting_plane,
        )

        build_result = StationSelection.build_robust_total_demand_cap_cutting_plane_master(
            model, data; optimizer_env=env_local
        )
        @test haskey(build_result.counts.variables, "eta")
        @test haskey(object_dictionary(build_result.model), :eta)
    end

    @testset "greedy separation" begin
        model = RobustTotalDemandCapModel(
            2, 3;
            in_vehicle_time_weight=1.0,
            max_walking_distance=100.0,
            q_hat=q_hat,
            B=B,
            solve_mode=:cutting_plane,
        )
        mapping = StationSelection.create_map(model, data)
        od_to_idx = Dict(od => idx for (idx, od) in enumerate(mapping.Omega_s[1]))
        scores = Dict(
            od_to_idx[(1, 2)] => 3.0,
            od_to_idx[(1, 3)] => 10.0,
            od_to_idx[(2, 3)] => 2.0,
            od_to_idx[(3, 1)] => 5.0,
        )
        q_wc, worst_case_value = StationSelection._separate_budgeted_uncertainty(mapping, 1, scores)
        @test q_wc[od_to_idx[(1, 3)]] == 5.0
        @test q_wc[od_to_idx[(3, 1)]] == 1.0
        @test worst_case_value == 55.0
    end

    @testset "run_opt" begin
        model = RobustTotalDemandCapModel(
            2, 3;
            in_vehicle_time_weight=1.0,
            max_walking_distance=100.0,
            q_hat=q_hat,
            B=B,
            solve_mode=:cutting_plane,
        )

        result = run_opt(
            model, data;
            optimizer_env=env_local,
            silent=true,
            cutting_plane_max_iters=10,
            cutting_plane_tol=1e-6,
        )

        @test result.termination_status == MOI.OPTIMAL
        @test !isnothing(result.objective_value)
        @test result.metadata["solve_mode"] == "cutting_plane"
        @test result.metadata["cutting_plane_iterations"] >= 1
        @test result.metadata["initial_cuts_added"] >= 1
    end
end
