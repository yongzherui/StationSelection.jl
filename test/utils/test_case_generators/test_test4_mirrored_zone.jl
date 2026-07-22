@testset "Test 4 generator — structural" begin
    inst_single = generate_test4_instance("single_zone", false, 1)
    inst_mirror = generate_test4_instance("mirrored_zones", true, 1)

    @test nrow(inst_single.stations) == 8
    @test nrow(inst_mirror.stations) == 13
    @test count(==("off_corridor"), inst_single.stations.role) == 1
    @test count(==("off_corridor"), inst_mirror.stations.role) == 2

    # Demand consistency guarantee: both variants draw the same total n_MB
    # from the same seed/Poisson call.
    @test inst_single.demand_counts.Mzone_to_B == inst_mirror.demand_counts.Mzone_to_B
    @test inst_single.demand_counts.n_total == inst_mirror.demand_counts.n_total
    @test inst_mirror.demand_counts.upper_to_B + inst_mirror.demand_counts.lower_to_B ==
          inst_mirror.demand_counts.Mzone_to_B
    @test inst_single.demand_counts.upper_to_B == inst_single.demand_counts.Mzone_to_B
    @test inst_single.demand_counts.lower_to_B == 0

    inst1 = generate_test4_instance("mirrored_zones", true, 4)
    inst2 = generate_test4_instance("mirrored_zones", true, 4)
    @test inst1.orders == inst2.orders

    data = create_test4_problem_data(inst_mirror; max_walking_distance = 1000.0 / 1.4)
    @test data.n_stations == 13

    all_instances = build_test4_cases(; n_seeds = 1)
    @test length(all_instances) == length(T4_VARIANTS)
end

gurobi_available = try
    using Gurobi
    true
catch
    false
end

@testset "Test 4 generator — hypothesis" begin
    if !gurobi_available
        @warn "Gurobi not available, skipping Test 4 hypothesis checks"
        @test true
        return
    end
    using JuMP

    env = Gurobi.Env()
    solver = DirectSolver(SolverConfig(; optimizer_env = env, silent = true))
    mwd_sec = 1000.0 / 1.4

    function active_roles(inst)
        data = create_test4_problem_data(inst; max_walking_distance = mwd_sec)
        model = ClusteringModel(TwoStageODPolicy(
            inst.suggested_k, inst.suggested_l;
            max_walking_distance = mwd_sec, in_vehicle_time_weight = 0.0,
        ))
        result = run_opt(data, model, solver)
        @test result.termination_status == MOI.OPTIMAL
        z = value.(result.model[:z])
        mapping_ids = data.array_idx_to_station_id
        roles = String[]
        for idx in 1:data.n_stations
            sid = mapping_ids[idx]
            z[idx, 1] > 0.5 || continue
            push!(roles, only(inst.stations.role[inst.stations.station_id .== sid]))
        end
        return roles
    end

    # single_zone: demand is directionally biased north -> the off-corridor
    # stop should be activated. mirrored_zones: symmetric demand -> the
    # on-corridor stop M0 should be activated instead (this is the
    # documented Test 4 hypothesis and holds cleanly under pure walk-cost
    # minimisation).
    roles_single = active_roles(generate_test4_instance("single_zone", false, 1))
    roles_mirror = active_roles(generate_test4_instance("mirrored_zones", true, 1))

    @test "off_corridor" in roles_single
    @test "on_corridor" in roles_mirror
    @test !("off_corridor" in roles_mirror)
end
