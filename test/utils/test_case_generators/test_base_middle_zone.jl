@testset "Middle-zone benchmark generator — structural" begin
    inst = generate_middle_zone_benchmark_instance("ab30_m10", 1, 30, 10)

    @test nrow(inst.stations) == 8
    @test count(==("terminal"), inst.stations.role) == 2
    @test count(==("on_corridor"), inst.stations.role) == 1
    @test count(==("off_corridor"), inst.stations.role) == 1
    @test count(==("zone_origin"), inst.stations.role) == 4

    @test inst.demand_counts.n_total == nrow(inst.orders)
    @test inst.demand_counts.A_to_B + inst.demand_counts.Mzone_to_B == inst.demand_counts.n_total

    # Reproducibility: same seed -> identical orders
    inst2 = generate_middle_zone_benchmark_instance("ab30_m10", 1, 30, 10)
    @test inst.orders == inst2.orders

    data = create_middle_zone_problem_data(inst; max_walking_distance = 1000.0 / 1.4)
    @test data.n_stations == nrow(inst.stations)
    @test nrow(data.scenarios[1].requests) == nrow(inst.orders)

    all_instances = build_middle_zone_benchmark_cases(; n_seeds = 1)
    @test length(all_instances) == length(MZB_PROFILES)
end
