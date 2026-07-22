@testset "Test 1 generator — structural" begin
    instances = build_test1_cases(; n_seeds = 1)
    @test length(instances) == length(T1_FLEET_CONFIGS)

    for inst in instances
        @test nrow(inst.stations) == 8  # geometry copied unchanged from the base benchmark
        @test inst.fleet_size == 2      # capacity sub-sweep: fleet fixed at 2
    end
    @test sort([inst.capacity for inst in instances]) == sort(StationSelection.T1_CAPACITY_SWEEP_VALUES)

    # Geometry/demand are byte-identical across configs at the same seed (only
    # the vehicle fleet config differs).
    @test all(inst.orders == instances[1].orders for inst in instances)

    inst = generate_test1_instance(T1_FLEET_CONFIGS[1], 1)
    data = create_test1_problem_data(inst; max_walking_distance = 1000.0 / 1.4)
    @test data.n_stations == nrow(inst.stations)
end

gurobi_available = try
    using Gurobi
    true
catch
    false
end

@testset "Test 1 generator — hypothesis (capacity sub-sweep)" begin
    if !gurobi_available
        @warn "Gurobi not available, skipping Test 1 hypothesis checks"
        @test true
        return
    end
    using JuMP

    # NOTE: the fleet-size sub-sweep (F=2..5, capacity fixed) from the source
    # script is intentionally NOT ported (see test1_vehicle.jl docstring) — no
    # model in src/ currently enforces a fleet-size cap. Only the capacity
    # sub-sweep (fleet fixed at 2, capacity 20->5) is checkable, via
    # ExactDARPRouteModel's `vehicle_capacity` parameter.
    env = Gurobi.Env()
    solver = DirectSolver(SolverConfig(; optimizer_env = env, silent = true))
    mwd_sec = 1000.0 / 1.4
    window = [("2026-01-01 08:00:00", "2026-01-01 11:00:00")]

    objectives = Float64[]
    for cfg in T1_FLEET_CONFIGS
        inst = generate_test1_instance(cfg, 1)
        data = create_test1_problem_data(inst; max_walking_distance = mwd_sec, scenarios = window)
        model = ExactDARPRouteModel(
            inst.suggested_k, inst.suggested_l;
            vehicle_capacity = inst.capacity, generate_routes = true,
            max_walking_distance = mwd_sec,
        )
        result = run_opt(data, model, solver)
        @test result.termination_status == MOI.OPTIMAL
        push!(objectives, result.objective_value)
    end

    # T1_FLEET_CONFIGS is ordered by decreasing capacity (20, 15, 10, 5).
    # Hypothesis: shrinking capacity forces costlier (more consolidated)
    # routing, so the objective should be monotonically non-decreasing.
    @test issorted(objectives)
end
