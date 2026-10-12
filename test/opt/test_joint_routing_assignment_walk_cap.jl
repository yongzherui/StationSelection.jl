@testset "Joint CG: total walking cap" begin
    using DataFrames
    using Dates
    using JuMP

    # Stations on a line 1..5; driving 1 per unit, walking 2 per unit (so a rider may walk one
    # unit). A tight set drives little and walks a lot; a spread set the reverse.
    # Small enough for DirectMIPSolver's exhaustive enumeration.
    n = 5
    stations = DataFrame(id = collect(1:n), lon = zeros(n), lat = zeros(n))
    walking = Dict((i, j) => 2.0 * abs(i - j) for i in 1:n for j in 1:n)
    routing = Dict((i, j) => Float64(abs(i - j)) for i in 1:n for j in 1:n)
    ods = [(1, 5), (2, 4), (5, 1), (1, 3), (4, 2)]
    requests = DataFrame(id = collect(1:length(ods)), start_station_id = first.(ods),
                         end_station_id = last.(ods),
                         request_time = fill(DateTime(2026, 1, 1, 8, 0), length(ods)))
    data = create_station_selection_data(stations, requests, walking; routing_costs = routing,
        scenarios = [("2026-01-01 08:00:00", "2026-01-01 09:00:00")])
    problem = StationSelectionProblem(data, 3; max_walking_distance = 2.5)
    form(cap) = AggregateODRouteJointRoutingAssignmentFormulation(
        route_regularization_weight = 1.0, walk_cost_weight = 0.01, repositioning_time = 1.0,
        max_wait_time = 10.0, detour_factor = 3.0, max_stops = 4, walk_cap = cap)
    cg() = CGSolver(config = SolverOptions(silent = true),
                    pricing = CGPricingConfig(mode = :exact), recover_integer_solution = true)

    # Total walking of a solved model's plan, in the cap row's units.
    function total_walk(res)
        m = res.model; mapping = res.mapping
        w = 0.0
        for (cid, v) in m[:joint_routing_assignment_theta]
            value(v) > 1e-6 || continue
            w += value(v) * joint_routing_assignment_column_walk(data, mapping, m[:joint_routing_assignment_columns][cid])
        end
        for ((s, p), v) in m[:x_walk]
            o, d = mapping.Omega_s[s][p]
            w += value(v) * mapping.Q_s[s][p] * StationSelection.od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)
        end
        return w
    end
    stations_of(res) = sort([j for j in eachindex(res.model[:y]) if value(res.model[:y][j]) > 0.5])

    free = run_opt(problem, form(Inf), cg())
    @test free.termination_status == SOLVE_OPTIMAL
    @test !haskey(free.model, :joint_routing_assignment_walk_cap_row)
    w_free = total_walk(free)
    @test w_free > 1.0

    # A cap the uncapped optimum already meets changes nothing.
    loose = run_opt(problem, form(w_free + 1.0), cg())
    @test loose.objective_value ≈ free.objective_value atol = 1e-6
    @test value(loose.model[:joint_routing_assignment_walk_cap_slack]) ≈ 0.0 atol = 1e-6

    # A binding cap: the CG optimum must equal the enumerated-pool optimum of the same capped
    # model (DirectMIPSolver), meet the cap with zero slack, and cost more route than before.
    cap = w_free - 1.0
    tight = run_opt(problem, form(cap), cg())
    direct = run_opt(problem, form(cap), DirectMIPSolver(config = SolverOptions(silent = true)))
    @test tight.termination_status == SOLVE_OPTIMAL
    @test direct.termination_status == SOLVE_OPTIMAL
    @test tight.objective_value ≈ direct.objective_value atol = 1e-6
    @test total_walk(tight) <= cap + 1e-6
    @test value(tight.model[:joint_routing_assignment_walk_cap_slack]) ≈ 0.0 atol = 1e-6
    @test tight.objective_value > free.objective_value + 1e-6
    # The LP master's cap dual was positive at some iteration (the cap priced walking), and
    # the integer recovery rebuilt the cap row.
    @test haskey(tight.model, :joint_routing_assignment_walk_cap_row)
    @test maximum(tight.model[:joint_routing_assignment_walk_cap_dual_history]) > 0.0
    @test all(==(0.0), loose.model[:joint_routing_assignment_walk_cap_dual_history][end:end])
    @test tight.model[:joint_routing_assignment_walk_cap] == cap

    # The cap is the epsilon-constraint: sweeping it reaches every set the DirectMIP sweep
    # reaches, so CG and enumeration agree on the whole trade-off curve.
    for c in (w_free - 2.0, w_free - 3.0, w_free - 4.0)
        a = run_opt(problem, form(c), cg())
        b = run_opt(problem, form(c), DirectMIPSolver(config = SolverOptions(silent = true)))
        @test a.objective_value ≈ b.objective_value atol = 1e-6
    end

    # An unreachable cap is not infeasible: the slack absorbs it and says by how much.
    hopeless = run_opt(problem, form(0.0), cg())
    @test hopeless.termination_status in (SOLVE_OPTIMAL, SOLVE_FEASIBLE)
    @test value(hopeless.model[:joint_routing_assignment_walk_cap_slack]) > 0.0

    # Benders cannot carry the coupling row.
    @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentMasterFormulation(form(cap))
end
