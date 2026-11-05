@testset "Joint: unserved_penalty (unserved groups allowed at a penalty)" begin
    using DataFrames
    using Dates

    # Stations on a line at 1..n, walking and driving both 1 per unit. R = 0.5 puts only a
    # rider's own stop within walking range, and no OD is walkable, so serving a rider needs
    # both of its endpoints built.
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
        return data, StationSelectionProblem(data, k; max_walking_distance = R)
    end

    joint(dw, P) = AggregateODRouteJointRoutingAssignmentFormulation(
        route_regularization_weight = 1.0, walk_cost_weight = 1.0, repositioning_time = 0.0,
        detour_factor = 1.5, max_stops = 4, demand_weighting = dw, unserved_penalty = P)
    cg(; recover = true) = CGSolver(config = SolverOptions(silent = true), max_iterations = 100,
        reduced_cost_tol = 1e-7, recover_integer_solution = recover,
        pricing = CGPricingConfig(mode = :exact))

    # One rider 1 -> 3 (route 2), two riders 5 -> 8 (route 3), k = 3: four endpoints, so one
    # OD group must go unserved. P = 1000.
    #   :unit  -> serve 1->3: 2 + 1000 = 1002 (serve 5->8: 3 + 1000 = 1003)
    #   :count -> serve 5->8: 3 + 1 x 1000 = 1003 (serve 1->3: 2 + 2 x 1000 = 2002)
    ods = [(1, 3), (5, 8), (5, 8)]
    data, problem = line_instance(ods; n = 8, R = 0.5, k = 3)
    P = 1000.0

    @testset "constructor" begin
        @test AggregateODRouteJointRoutingAssignmentFormulation().unserved_penalty == Inf
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(unserved_penalty = 0.0)
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(unserved_penalty = -1.0)
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(unserved_penalty = NaN)
        # Benders has no slack: deriving its formulations from a finite penalty is an error.
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentMasterFormulation(joint(:unit, P))
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation(joint(:unit, P))
        @test AggregateODRouteJointRoutingAssignmentMasterFormulation(joint(:unit, Inf)).demand_weighting === :unit
    end

    @testset "Inf: k too small is infeasible" begin
        result = run_opt(problem, joint(:unit, Inf), cg())
        @test result.termination_status == SOLVE_INFEASIBLE
    end

    @testset "finite penalty: CG and DirectMIP agree, Q weights the penalty" begin
        expected = Dict(:unit => (1002.0, [1, 3]), :count => (1003.0, [5, 8]))
        for dw in (:unit, :count)
            obj, must_build = expected[dw]
            f = joint(dw, P)
            for solver in (cg(), DirectMIPSolver(config = SolverOptions(silent = true)))
                result = run_opt(problem, f, solver)
                @test result.termination_status == SOLVE_OPTIMAL
                @test isapprox(result.objective_value, obj; atol = 1e-6)
                m = result.model
                y = [JuMP.value(v) for v in m[:y]]
                built = [data.array_idx_to_station_id[j] for j in eachindex(y) if y[j] > 0.5]
                @test all(in(built), must_build)
                u = m[:joint_routing_assignment_unserved]
                @test sum(JuMP.value(v) for v in values(u)) ≈ 1.0 atol = 1e-6
            end
        end
    end

    @testset "a feasible budget leaves no one unserved" begin
        data4, problem4 = line_instance(ods; n = 8, R = 0.5, k = 4)
        strict = run_opt(problem4, joint(:unit, Inf), cg())
        soft = run_opt(problem4, joint(:unit, P), cg())
        @test strict.termination_status == SOLVE_OPTIMAL
        @test soft.termination_status == SOLVE_OPTIMAL
        @test isapprox(soft.objective_value, strict.objective_value; atol = 1e-6)
        @test all(JuMP.value(v) < 1e-6 for v in values(soft.model[:joint_routing_assignment_unserved]))
    end
end
