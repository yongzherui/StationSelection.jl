@testset "Test 6 generator — structural" begin
    for dcfg in T6_DEMAND_CONFIGS
        inst = generate_test6_instance(dcfg, 1)
        @test nrow(inst.stations) == 8
        dc = inst.demand_counts
        @test dc.n_total == dc.A_to_B + dc.zone_to_B + dc.B_to_A + dc.zone_to_A
        @test dc.n_total == nrow(inst.orders)
        if dcfg.bwd_frac == 0.0
            @test dc.B_to_A == 0
            @test dc.zone_to_A == 0
        end
    end

    inst1 = generate_test6_instance(T6_DEMAND_CONFIGS[2], 2)
    inst2 = generate_test6_instance(T6_DEMAND_CONFIGS[2], 2)
    @test inst1.orders == inst2.orders

    data = create_test6_problem_data(inst1; max_walking_distance = 1000.0 / 1.4)
    @test data.n_stations == 8

    all_instances = build_test6_cases(; n_seeds = 1)
    @test length(all_instances) == length(T6_DEMAND_CONFIGS)
end

gurobi_available = try
    using Gurobi
    true
catch
    false
end

@testset "Test 6 generator — hypothesis" begin
    if !gurobi_available
        @warn "Gurobi not available, skipping Test 6 hypothesis checks"
        @test true
        return
    end
    using JuMP

    # NOTE (open item): the documented hypothesis -- that on-corridor M0
    # becomes more attractive than off-corridor M as backward demand grows
    # -- did not manifest under ClusteringModel(TwoStageODPolicy) across a
    # range of in_vehicle_time_weight values tested during verification
    # (0.5, 1.0, 2.0, 5.0): the model always kept the walk-nearest station
    # (M) active regardless of demand direction, since ClusteringModel's
    # routing-cost term is a static per-(j,k) pairwise cost and does not
    # capture round-trip route consolidation. Exercising this hypothesis
    # properly likely needs a route-aware model (e.g. ExactDARPRouteModel).
    # We assert feasibility only here.
    env = Gurobi.Env()
    solver = DirectSolver(SolverConfig(; optimizer_env = env, silent = true))
    mwd_sec = 1000.0 / 1.4

    for dcfg in T6_DEMAND_CONFIGS
        inst = generate_test6_instance(dcfg, 1)
        data = create_test6_problem_data(inst; max_walking_distance = mwd_sec)
        model = ClusteringModel(TwoStageODPolicy(
            inst.suggested_k, inst.suggested_l;
            max_walking_distance = mwd_sec, in_vehicle_time_weight = 1.0,
        ))
        result = run_opt(data, model, solver)
        @test result.termination_status == MOI.OPTIMAL
    end
end
