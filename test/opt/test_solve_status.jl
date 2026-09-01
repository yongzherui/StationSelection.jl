@testset "SolveStatus" begin
    @testset "labels print without the SOLVE_ prefix" begin
        # Result CSVs across benchmarks/ write `string(result.termination_status)`, so the
        # printed labels are a load-bearing interface, not cosmetics.
        @test string(SOLVE_OPTIMAL) == "OPTIMAL"
        @test string(SOLVE_FEASIBLE) == "FEASIBLE"
        @test string(SOLVE_INFEASIBLE) == "INFEASIBLE"
        @test string(SOLVE_NOT_SOLVED) == "NOT_SOLVED"
    end

    @testset "a refuted feasibility gate returns INFEASIBLE instead of throwing" begin
        # Four stations pairwise 1000 m apart -- further than `max_walking_distance`, so no
        # station can stand in for another and neither demand group gets a direct-walk
        # fallback. Two disjoint requests (1 -> 2 and 3 -> 4) therefore require all four
        # stations, which k = 2 cannot supply: infeasible on `y` alone, decided by
        # `check_feasibility` before a single route is ever priced or enumerated.
        n = 4
        stations = DataFrame(
            id = collect(1:n),
            lon = [113.0, 113.5, 114.0, 114.5],
            lat = [28.0, 28.5, 29.0, 29.5],
        )
        requests = DataFrame(
            id = [1, 2],
            start_station_id = [1, 3],
            end_station_id = [2, 4],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0), DateTime(2024, 1, 1, 8, 30, 0)],
        )
        # Symmetric, as add_aggregate_od_route_endpoint_feasibility_constraints! requires.
        walking_costs = Dict{Tuple{Int, Int}, Float64}(
            (i, j) => (i == j ? 0.0 : 1000.0) for i in 1:n, j in 1:n
        )
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (i, j) => (i == j ? 0.0 : 50.0) for i in 1:n, j in 1:n
        )
        data = StationSelection.create_station_selection_data(
            stations, requests, walking_costs;
            routing_costs = routing_costs,
            scenarios = [("2024-01-01 07:00:00", "2024-01-01 09:00:00")],
        )
        problem = StationSelectionProblem(data, 2; max_walking_distance = 300.0)

        # Both live aggregate-OD-route formulations share the gate, on either solver.
        cases = (
            (AggregateODRouteJointRoutingAssignmentFormulation(), CGSolver()),
            (AggregateODRouteBaseFormulation(max_stops = 4), DirectMIPSolver()),
        )
        for (formulation, solver) in cases
            result = run_opt(problem, formulation, solver)
            @test result.termination_status == SOLVE_INFEASIBLE
            # No incumbent is claimed for a problem with no feasible solution.
            @test result.objective_value === nothing
            @test result.solution === nothing
            @test occursin("endpoint feasibility", result.metadata["infeasibility_reason"])
        end
    end

    @testset "a certified solve still reports OPTIMAL, and keeps the raw MOI code" begin
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        result = run_opt(
            problem, AggregateODRouteJointRoutingAssignmentFormulation(), CGSolver(),
        )
        @test result.termination_status == SOLVE_OPTIMAL
        @test result.metadata["cg_converged"]           # what makes OPTIMAL earned here
        @test result.metadata["moi_termination_status"] == "OPTIMAL"
        @test result.objective_value !== nothing
    end
end
