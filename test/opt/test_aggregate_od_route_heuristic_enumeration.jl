@testset "AggregateODRouteModel heuristic enumeration warm start" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping AggregateODRouteModel heuristic enumeration tests"
        @test true
        return
    end

    function heuristic_enumeration_fixture()
        stations = DataFrame(id=[1, 2, 3], lon=[0.0, 1.0, 2.0], lat=[0.0, 0.0, 0.0])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 1],
            end_station_id=[3, 3],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 5)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:3, j in 1:3
            walking_costs[(i, j)] = i == j ? 0.0 : 100.0
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            2;
            max_walking_distance=1.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
        )
        return data, model
    end

    @testset "single feasible candidate matches DirectSolver optimum" begin
        data, model = heuristic_enumeration_fixture()

        direct_result = run_opt(data, model, DirectSolver(optimizer_env=Gurobi.Env(), silent=true))
        @test direct_result.termination_status == MOI.OPTIMAL

        heuristic_result = run_opt(
            data,
            model,
            HeuristicEnumerationSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                candidate_open_stations=[[1, 3]],
            ),
        )

        @test heuristic_result.termination_status == MOI.OPTIMAL
        @test isapprox(heuristic_result.objective_value, direct_result.objective_value; atol=1e-6)
        @test heuristic_result.metadata["solve_method"] == "heuristic_enumeration"
        @test heuristic_result.metadata["n_candidates"] == 1
        @test heuristic_result.metadata["n_feasible_candidates"] == 1
        @test heuristic_result.metadata["winning_candidate_index"] == 1
    end

    @testset "infeasible candidate is skipped in favor of the feasible one" begin
        data, model = heuristic_enumeration_fixture()

        result = run_opt(
            data,
            model,
            HeuristicEnumerationSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                candidate_open_stations=[[1, 2], [1, 3]],
            ),
        )

        @test result.termination_status == MOI.OPTIMAL
        @test result.metadata["n_candidates"] == 2
        @test result.metadata["n_feasible_candidates"] == 1
        @test result.metadata["winning_candidate_index"] == 2
        rows = result.metadata["candidate_rows"]
        @test rows[1].feasible == false
        @test rows[2].feasible == true
    end

    @testset "all-infeasible candidates raise an ArgumentError" begin
        data, model = heuristic_enumeration_fixture()

        @test_throws ArgumentError run_opt(
            data,
            model,
            HeuristicEnumerationSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                candidate_open_stations=[[1, 2], [2, 3]],
            ),
        )
    end
end
