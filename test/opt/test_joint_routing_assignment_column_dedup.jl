@testset "add_joint_routing_assignment_column! dedups per scenario, not globally" begin
    # Two scenarios, each with a single request between the same station pair (1 -> 2), so
    # both scenarios' Omega_s end up as [(1,2)] -- identical demand-group position p=1 in
    # both. A column signature is derived from `(p, j, k)` assignments alone (no scenario),
    # so two columns for the same (j,k) route at the same tau -- one per scenario -- used to
    # collide on that bare signature and get silently `:skipped` for whichever scenario lost
    # the race (see notes/2026-08-28_study5_dominance_fix_pilot_infeasible_repro.md): a
    # coverage row could end up with zero terms even though its demand group had a perfectly
    # good route available, just discovered under a different scenario's column first.
    stations = DataFrame(id = [1, 2], lon = [113.0, 113.1], lat = [28.0, 28.1])
    requests = DataFrame(
        id = [1, 2],
        start_station_id = [1, 1],
        end_station_id = [2, 2],
        request_time = [
            DateTime(2024, 1, 1, 8, 0, 0),
            DateTime(2024, 1, 1, 12, 0, 0),
        ],
    )
    walking_costs = Dict{Tuple{Int, Int}, Float64}(
        (1, 1) => 0.0, (2, 2) => 0.0, (1, 2) => 100.0, (2, 1) => 100.0,
    )
    routing_costs = Dict{Tuple{Int, Int}, Float64}((1, 2) => 50.0, (2, 1) => 50.0)
    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs; routing_costs = routing_costs,
        scenarios = [
            ("2024-01-01 07:00:00", "2024-01-01 09:00:00"),
            ("2024-01-01 11:00:00", "2024-01-01 13:00:00"),
        ],
    )
    @test StationSelection.n_scenarios(data) == 2

    problem = StationSelectionProblem(data, 1; max_walking_distance = 500.0)
    formulation = AggregateODRouteJointRoutingAssignmentFormulation()
    mapping = StationSelection.create_aggregate_od_route_map(problem, formulation, data)
    @test mapping.Omega_s[1] == [(1, 2)]
    @test mapping.Omega_s[2] == [(1, 2)]

    solver = CGSolver(initial_columns = StationSelection.JointRoutingAssignmentRouteColumn[])
    build_result = StationSelection.build_model(problem, formulation, solver)
    m = build_result.model

    col_s1 = StationSelection.JointRoutingAssignmentRouteColumn(
        1, [1, 2], [(1, 1, 2)], 50.0; metadata = Dict{String, Any}("scenario" => 1),
    )
    col_s2 = StationSelection.JointRoutingAssignmentRouteColumn(
        2, [1, 2], [(1, 1, 2)], 50.0; metadata = Dict{String, Any}("scenario" => 2),
    )

    theta1, action1 = StationSelection.add_joint_routing_assignment_column!(m, data, mapping, col_s1)
    theta2, action2 = StationSelection.add_joint_routing_assignment_column!(m, data, mapping, col_s2)

    @test action1 == :added
    @test action2 == :added
    @test theta1 !== theta2
    @test length(m[:joint_routing_assignment_columns]) == 2

    coverage = m[:joint_routing_assignment_coverage]
    @test normalized_coefficient(coverage[(1, 1)], theta1) == 1.0
    @test normalized_coefficient(coverage[(2, 1)], theta2) == 1.0
    @test normalized_coefficient(coverage[(1, 1)], theta2) == 0.0
    @test normalized_coefficient(coverage[(2, 1)], theta1) == 0.0
end
