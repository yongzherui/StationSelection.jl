@testset "Joint CG seeds: singleton columns keep the RMP feasible" begin
    using DataFrames
    using Dates
    using JuMP

    # Stations on a line 1..4; driving 1 per unit, walking 2 per unit, one-unit walks.
    n = 4
    stations = DataFrame(id = collect(1:n), lon = zeros(n), lat = zeros(n))
    walking = Dict((i, j) => 2.0 * abs(i - j) for i in 1:n for j in 1:n)
    routing = Dict((i, j) => Float64(abs(i - j)) for i in 1:n for j in 1:n)
    # Door-to-door ratio 2.0 leaves:
    #   A = (1,2): drive 1, budget 2 -> only (1,2)
    #   B = (2,3): drive 1, budget 2 -> only (2,3)
    #   C = (1,3): drive 2, budget 4 -> (1,2), (1,3), (2,3)
    # so the bundled seed for (1,2) carries {A, C} and the one for (2,3) carries {B, C}. A and
    # B force both bundles, which then cover C twice: with bundles alone the set-partitioning
    # master is infeasible before any pricing (the k = 20 sub35 failure of 2026-09-27).
    ods = [(1, 2), (2, 3), (1, 3)]
    requests = DataFrame(id = collect(1:3), start_station_id = first.(ods), end_station_id = last.(ods),
                         request_time = fill(DateTime(2026, 1, 1, 8, 0), 3))
    data = create_station_selection_data(stations, requests, walking; routing_costs = routing,
        scenarios = [("2026-01-01 08:00:00", "2026-01-01 09:00:00")])
    problem = StationSelectionProblem(data, 3; max_walking_distance = 2.5, door_to_door_ratio = 2.0)
    formulation = AggregateODRouteJointRoutingAssignmentFormulation(
        route_regularization_weight = 1.0, walk_cost_weight = 1.0, repositioning_time = 0.0,
        max_wait_time = 10.0, detour_factor = 1.5, max_stops = 4)

    mapping = StationSelection.create_aggregate_od_route_map(problem, formulation, data)
    seeds = joint_routing_assignment_two_stop_seed_columns(data, mapping)
    bundles = [c for c in seeds if c.metadata["seed"] == "two_stop"]
    singles = [c for c in seeds if c.metadata["seed"] == "two_stop_singleton"]

    # The bundles are still there, unchanged: one per (scenario, j, k).
    @test length(bundles) == length(unique((c.metadata["scenario"], c.route[1], c.route[2]) for c in bundles))
    @test any(length(c.assignments) > 1 for c in bundles)
    # Every assignment of every multi-group bundle also exists as its own singleton,
    # and one-group bundles are not duplicated.
    single_keys = Set((c.metadata["scenario"], only(c.assignments)) for c in singles)
    for c in bundles, a in c.assignments
        @test (c.metadata["scenario"], a) in single_keys || length(c.assignments) == 1
    end
    @test all(length(c.assignments) == 1 for c in singles)
    @test length(single_keys) == length(singles)

    # The restricted master built from these seeds is feasible (it was not with bundles alone).
    build = build_model(problem, formulation, CGSolver())
    m = build.model
    set_silent(m)
    optimize!(m)
    @test termination_status(m) == MOI.OPTIMAL
end
