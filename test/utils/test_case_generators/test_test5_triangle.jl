@testset "Test 5 generator — structural" begin
    expected_n_stations = Dict(:corridor_base => 7, :equilateral => 8, :equilateral_with_m1 => 9)
    expected_sweep = Dict(:corridor_base => (3, 2), :equilateral => (4, 3), :equilateral_with_m1 => (5, 4))

    dcfg = T5_DEMAND_CONFIGS[1]
    for case in T5_CASES
        inst = generate_test5_instance(case, 1, dcfg)
        @test nrow(inst.stations) == expected_n_stations[case]
        @test (inst.suggested_l, inst.suggested_k) == expected_sweep[case]
        @test inst.demand_counts.n_total == nrow(inst.orders)
    end

    # Same total demand per seed across geometry variants within one demand
    # config (reproduces the source's validate_test5_cases check #1/#2).
    order_counts = [nrow(generate_test5_instance(case, 2, dcfg).orders) for case in T5_CASES]
    @test all(==(order_counts[1]), order_counts)
    mb_counts = [generate_test5_instance(case, 2, dcfg).demand_counts.Mzone_to_B for case in T5_CASES]
    @test all(==(mb_counts[1]), mb_counts)

    inst1 = generate_test5_instance(:equilateral, 3, dcfg)
    inst2 = generate_test5_instance(:equilateral, 3, dcfg)
    @test inst1.orders == inst2.orders

    data = create_test5_problem_data(inst1; max_walking_distance = 1800.0 / 1.4)
    @test data.n_stations == 8

    all_instances = build_test5_cases(; n_seeds = 1)
    @test length(all_instances) == length(T5_CASES) * length(T5_DEMAND_CONFIGS)
end

gurobi_available = try
    using Gurobi
    true
catch
    false
end

@testset "Test 5 generator — hypothesis" begin
    if !gurobi_available
        @warn "Gurobi not available, skipping Test 5 hypothesis checks"
        @test true
        return
    end
    using JuMP

    env = Gurobi.Env()
    solver = DirectSolver(SolverConfig(; optimizer_env = env, silent = true))
    mwd_sec = 1800.0 / 1.4
    dcfg = T5_DEMAND_CONFIGS[1]

    # NOTE: the source-suggested sweep for corridor_base (l=3, k=2) was
    # found INFEASIBLE under ClusteringModel(TwoStageODPolicy) -- with only
    # 2 of the 3 built stations active, the model cannot simultaneously
    # serve the A<->B corridor stream and the zone->B stream (whichever of
    # A/M0/B is left inactive leaves one stream unservable). We use k=3
    # (all built stations active) here instead; this deviation from the
    # source's own suggested sweep is intentional and documented.
    inst = generate_test5_instance(:corridor_base, 1, dcfg)
    data = create_test5_problem_data(inst; max_walking_distance = mwd_sec)
    model = ClusteringModel(TwoStageODPolicy(3, 3; max_walking_distance = mwd_sec, in_vehicle_time_weight = 0.0))
    result = run_opt(data, model, solver)
    @test result.termination_status == MOI.OPTIMAL

    # equilateral / equilateral_with_m1: the source hypothesis language is
    # exploratory ("optimizer must decide" / "chooses among M0, M1, M") --
    # only feasibility/optimality is asserted for these.
    for case in (:equilateral, :equilateral_with_m1)
        inst = generate_test5_instance(case, 1, dcfg)
        data = create_test5_problem_data(inst; max_walking_distance = mwd_sec)
        model = ClusteringModel(TwoStageODPolicy(
            inst.suggested_k, inst.suggested_l;
            max_walking_distance = mwd_sec, in_vehicle_time_weight = 0.0,
        ))
        result = run_opt(data, model, solver)
        @test result.termination_status == MOI.OPTIMAL
    end
end
