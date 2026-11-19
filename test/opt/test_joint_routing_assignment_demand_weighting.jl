@testset "Joint: demand_weighting (:count default, :unit = Q 1)" begin
    using DataFrames
    using Dates

    # Stations on a line at 1..n, walking and driving both 1 per unit.
    function line_instance(ods; n, R, k)
        stations = DataFrame(id = collect(1:n), lon = zeros(n), lat = zeros(n))
        requests = DataFrame(
            id = collect(1:length(ods)),
            start_station_id = first.(ods),
            end_station_id = last.(ods),
            request_time = fill(DateTime(2026, 1, 1, 8, 0), length(ods)),
        )
        cost = Dict((i, j) => Float64(abs(i - j)) for i in 1:n for j in 1:n)
        data = create_station_selection_data(
            stations, requests, cost;
            routing_costs = cost,
            scenarios = [("2026-01-01 08:00:00", "2026-01-01 09:00:00")],
        )
        return data, StationSelectionProblem(data, k; walking_limit_s = R)
    end

    joint(dw) = AggregateODRouteJointRoutingAssignmentFormulation(
        pickup_deadline_s = Inf, route_regularization_weight = 1.0, walk_cost_weight = 1.0, repositioning_time = 0.0,
        ride_time_limit_ratio = 1.5, max_stops = 4, demand_weighting = dw)

    # Three riders 1 -> 7, one rider 2 -> 6, k = 2, stations reachable within 1 unit.
    # Candidate sets and their objective (route + walking):
    #   {2,6}: route 4, walk unit 1+1 = 2 -> 6,  count 3+3 = 6 -> 10
    #   {1,7}: route 6, walk unit 1+1 = 2 -> 8,  count 1+1 = 2 -> 8
    #   {1,6}/{2,7}: route 5, walk unit 2 -> 7,  count 4 -> 9
    # so :unit picks {2,6} (6) and :count picks {1,7} (8).
    ods = [(1, 7), (1, 7), (1, 7), (2, 6)]
    data, problem = line_instance(ods; n = 7, R = 1.5, k = 2)

    @testset "map: Q_s counts requests per OD group" begin
        @test AggregateODRouteJointRoutingAssignmentFormulation(pickup_deadline_s = Inf).demand_weighting === :count
        m_count = StationSelection.create_aggregate_od_route_map(problem, joint(:count), data)
        m_unit = StationSelection.create_aggregate_od_route_map(problem, joint(:unit), data)
        @test m_count.Omega_s[1] == m_unit.Omega_s[1]
        q = Dict(zip(m_count.Omega_s[1], m_count.Q_s[1]))
        o17 = (data.station_id_to_array_idx[1], data.station_id_to_array_idx[7])
        o26 = (data.station_id_to_array_idx[2], data.station_id_to_array_idx[6])
        @test q[o17] == 3 && q[o26] == 1
        @test sum(m_count.Q_s[1]) == length(ods)
        @test all(==(1), m_unit.Q_s[1])
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(pickup_deadline_s = Inf, demand_weighting = :riders)
    end

    @testset "derived Benders formulations carry it" begin
        for dw in (:count, :unit)
            f = joint(dw)
            @test AggregateODRouteJointRoutingAssignmentMasterFormulation(f).demand_weighting === dw
            @test AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation(f).demand_weighting === dw
        end
    end

    @testset "CG, DirectMIP and Benders agree; :count moves the stations" begin
        expected = Dict(:unit => 6.0, :count => 8.0)
        for dw in (:unit, :count)
            f = joint(dw)
            direct = run_opt(problem, f, DirectMIPSolver(config = SolverOptions(silent = true)))
            @test direct.termination_status == SOLVE_OPTIMAL
            @test isapprox(direct.objective_value, expected[dw]; atol = 1e-6)
            # The CG master asserts each priced column's reduced cost against its own; a
            # pricer without the demand factor fails that check under :count.
            for pricing in (CGPricingConfig(mode = :exact), CGPricingConfig(mode = :darp))
                cg = run_opt(problem, f, CGSolver(
                    config = SolverOptions(silent = true), max_iterations = 100,
                    reduced_cost_tol = 1e-7, recover_integer_solution = true, pricing = pricing))
                @test cg.termination_status == SOLVE_OPTIMAL
                @test isapprox(cg.objective_value, expected[dw]; atol = 1e-6)
            end
            # The activated oracles exercise the closed-form and Pareto completions, which
            # credit `w * Q * walk`.
            for oracle in (:direct_enumeration, :column_generation,
                           :column_generation_activated, :column_generation_activated_lpo)
                benders = run_opt(problem, f, BendersSolver(
                    config = SolverOptions(silent = true), max_iterations = 50,
                    optimality_tol = 1e-6, subproblem = BendersSubproblemConfig(oracle = oracle)))
                @test benders.termination_status == SOLVE_OPTIMAL
                @test isapprox(benders.objective_value, expected[dw]; atol = 1e-6)
            end
        end
    end
end
