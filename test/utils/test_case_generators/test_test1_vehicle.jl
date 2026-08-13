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

# NOTE: the "hypothesis (capacity sub-sweep)" testset that used to live here
# exercised ExactDARPRouteModel's `vehicle_capacity` parameter to check that
# shrinking capacity forces costlier, more-consolidated routing. It was
# removed along with ExactDARPRouteModel (ineffective benchmark, difficult to
# solve) -- see test1_vehicle.jl's docstring. No model in src/ currently
# checks this hypothesis; T1_FLEET_CONFIGS/build_test1_cases still generate
# the capacity sub-sweep instances for whichever model picks this back up.
