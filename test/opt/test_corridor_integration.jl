@testset "Corridor Model Integration" begin
    using JuMP

    if !gurobi_available
        @warn "Gurobi not available, skipping corridor integration tests"
        @test true
        return
    end

    # Create test data
    stations = DataFrame(
        id = [1, 2, 3, 4, 5],
        lon = [113.0, 113.1, 113.2, 113.3, 113.4],
        lat = [28.0, 28.1, 28.2, 28.3, 28.4]
    )

    requests = DataFrame(
        id = [1, 2, 3, 4, 5, 6],
        start_station_id = [1, 1, 2, 3, 4, 2],
        end_station_id = [2, 3, 3, 4, 5, 5],
        request_time = [
            DateTime(2024, 1, 1, 8, 0, 0),
            DateTime(2024, 1, 1, 8, 1, 0),
            DateTime(2024, 1, 1, 8, 2, 0),
            DateTime(2024, 1, 1, 8, 3, 0),
            DateTime(2024, 1, 1, 8, 4, 0),
            DateTime(2024, 1, 1, 8, 5, 0)
        ]
    )

    walking_costs = Dict{Tuple{Int,Int}, Float64}()
    routing_costs = Dict{Tuple{Int,Int}, Float64}()
    for i in 1:5, j in 1:5
        walking_costs[(i, j)] = abs(i - j) * 100.0
        routing_costs[(i, j)] = abs(i - j) * 50.0
    end

    scenarios = [("2024-01-01 08:00:00", "2024-01-01 09:00:00")]
    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )

    env = Gurobi.Env()

    # =========================================================================
    # ZCorridorODModel tests
    # =========================================================================

    @testset "ZCorridorODModel construction validation" begin
        @test_throws ArgumentError ZCorridorODModel(0, 5)   # k must be positive
        @test_throws ArgumentError ZCorridorODModel(5, 3)   # l must be >= k
        @test_throws ArgumentError ZCorridorODModel(3, 5; in_vehicle_time_weight=-1.0)
        @test_throws ArgumentError ZCorridorODModel(3, 5; corridor_weight=-1.0)
        @test_throws ArgumentError ZCorridorODModel(3, 5; max_cluster_diameter=0.0)
        @test_throws ArgumentError ZCorridorODModel(3, 5; max_cluster_diameter=-1.0)

        model = ZCorridorODModel(2, 3; corridor_weight=2.0, max_cluster_diameter=500.0)
        @test model.k == 2
        @test model.l == 3
        @test model.corridor_weight == 2.0
        @test model.max_cluster_diameter == 500.0
        @test model.in_vehicle_time_weight == 1.0
        @test model isa AbstractCorridorODModel
    end

    @testset "ZCorridorODModel mapping creation" begin
        model = ZCorridorODModel(2, 3; max_cluster_diameter=500.0)
        mapping = StationSelection.create_map(model, data)

        @test mapping isa CorridorTwoStageODMap
        @test length(mapping.station_id_to_array_idx) == 5
        @test length(mapping.array_idx_to_station_id) == 5
        @test length(mapping.scenarios) == 1
        @test haskey(mapping.Omega_s, 1)
        @test haskey(mapping.Q_s, 1)
        @test length(mapping.Omega_s[1]) > 0

        @test mapping.n_clusters >= 1
        @test length(mapping.cluster_labels) == 5
        @test length(mapping.cluster_medoids) == mapping.n_clusters
        @test length(mapping.corridor_indices) == mapping.n_clusters^2
        @test length(mapping.corridor_costs) == mapping.n_clusters^2
        @test length(mapping.cluster_station_sets) == mapping.n_clusters
    end

    @testset "ZCorridorODModel build" begin
        model = ZCorridorODModel(2, 3; corridor_weight=1.0, max_cluster_diameter=500.0)

        build_result = StationSelection.build_model(
            model, data; optimizer_env=env
        )
        m = build_result.model
        var_counts = build_result.counts.variables
        con_counts = build_result.counts.constraints
        extra_counts = build_result.counts.extras

        @test m isa JuMP.Model

        @test haskey(var_counts, "station_selection")
        @test haskey(var_counts, "scenario_activation")
        @test haskey(var_counts, "assignment")
        @test haskey(var_counts, "cluster_activation")
        @test haskey(var_counts, "corridor")

        @test var_counts["station_selection"] == 5
        @test var_counts["scenario_activation"] == 5

        mapping = build_result.mapping
        @test var_counts["cluster_activation"] == mapping.n_clusters * 1
        @test var_counts["corridor"] == mapping.n_clusters^2 * 1

        @test haskey(con_counts, "station_limit")
        @test haskey(con_counts, "scenario_activation_limit")
        @test haskey(con_counts, "activation_linking")
        @test haskey(con_counts, "assignment")
        @test haskey(con_counts, "assignment_to_active")
        @test haskey(con_counts, "cluster_activation")
        @test haskey(con_counts, "corridor_activation")

        @test con_counts["station_limit"] == 1
        @test con_counts["scenario_activation_limit"] == 1
        @test con_counts["activation_linking"] == 5
        @test con_counts["cluster_activation"] == mapping.n_clusters * 1
        @test con_counts["corridor_activation"] == mapping.n_clusters^2 * 1

        @test haskey(extra_counts, "total_od_pairs")
        @test extra_counts["total_od_pairs"] > 0
        @test haskey(extra_counts, "n_clusters")
        @test haskey(extra_counts, "n_corridors")

        @test haskey(object_dictionary(m), :y)
        @test haskey(object_dictionary(m), :z)
        @test haskey(object_dictionary(m), :x)
        @test haskey(object_dictionary(m), :α)
        @test haskey(object_dictionary(m), :f_corridor)
    end

    @testset "ZCorridorODModel run_opt without optimization" begin
        model = ZCorridorODModel(2, 3; corridor_weight=1.0, max_cluster_diameter=500.0)
        result = run_opt(
            model, data;
            optimizer_env=env,
            silent=true,
            do_optimize=false
        )

        @test result.termination_status == MOI.OPTIMIZE_NOT_CALLED
        @test isnothing(result.objective_value)
        @test isnothing(result.solution)
        @test result.model isa JuMP.Model
        @test !isempty(result.counts.variables)
        @test !isempty(result.counts.constraints)
    end

    # =========================================================================
    # XCorridorODModel tests
    # =========================================================================

    @testset "XCorridorODModel construction validation" begin
        @test_throws ArgumentError XCorridorODModel(0, 5)
        @test_throws ArgumentError XCorridorODModel(5, 3)
        @test_throws ArgumentError XCorridorODModel(3, 5; in_vehicle_time_weight=-1.0)
        @test_throws ArgumentError XCorridorODModel(3, 5; corridor_weight=-1.0)
        @test_throws ArgumentError XCorridorODModel(3, 5; max_cluster_diameter=0.0)

        model = XCorridorODModel(2, 3; corridor_weight=2.0, max_cluster_diameter=500.0)
        @test model.k == 2
        @test model.l == 3
        @test model.corridor_weight == 2.0
        @test model isa AbstractCorridorODModel
    end

    @testset "XCorridorODModel build" begin
        model = XCorridorODModel(2, 3; corridor_weight=1.0, max_cluster_diameter=500.0)

        build_result = StationSelection.build_model(
            model, data; optimizer_env=env
        )
        m = build_result.model
        var_counts = build_result.counts.variables
        con_counts = build_result.counts.constraints

        @test m isa JuMP.Model

        # XCorridorODModel has no cluster_activation variables (no α)
        @test haskey(var_counts, "station_selection")
        @test haskey(var_counts, "scenario_activation")
        @test haskey(var_counts, "assignment")
        @test haskey(var_counts, "corridor")
        @test !haskey(var_counts, "cluster_activation")

        # x-based corridor constraint instead of z-based
        @test haskey(con_counts, "corridor_x_activation")
        @test !haskey(con_counts, "cluster_activation")
        @test !haskey(con_counts, "corridor_activation")

        @test haskey(object_dictionary(m), :y)
        @test haskey(object_dictionary(m), :z)
        @test haskey(object_dictionary(m), :x)
        @test haskey(object_dictionary(m), :f_corridor)
        @test !haskey(object_dictionary(m), :α)
    end

    @testset "XCorridorODModel run_opt without optimization" begin
        model = XCorridorODModel(2, 3; corridor_weight=1.0, max_cluster_diameter=500.0)
        result = run_opt(
            model, data;
            optimizer_env=env,
            silent=true,
            do_optimize=false
        )

        @test result.termination_status == MOI.OPTIMIZE_NOT_CALLED
        @test isnothing(result.objective_value)
        @test result.model isa JuMP.Model
    end

    # =========================================================================
    # Shared activation logic test data
    # =========================================================================

    # 4 stations, each in its own cluster
    corr_stations = DataFrame(
        id = [1, 2, 3, 4],
        lon = [113.0, 113.1, 113.2, 113.3],
        lat = [28.0, 28.1, 28.2, 28.3]
    )
    corr_requests = DataFrame(
        id = [1],
        start_station_id = [1],
        end_station_id = [3],
        request_time = [DateTime(2024, 1, 1, 8, 0, 0)]
    )
    corr_walking = Dict{Tuple{Int,Int}, Float64}()
    corr_routing = Dict{Tuple{Int,Int}, Float64}()
    for i in 1:4, j in 1:4
        corr_walking[(i, j)] = (i == j) ? 0.0 : 100000.0
        corr_routing[(i, j)] = (i == j) ? 1.0 : abs(i - j) * 1000.0
    end
    corr_data = StationSelection.create_station_selection_data(
        corr_stations, corr_requests, corr_walking;
        routing_costs=corr_routing,
        scenarios=[("2024-01-01 08:00:00", "2024-01-01 09:00:00")]
    )

    @testset "ZCorridorODModel zone and corridor activation" begin
        # z-based: corridors activate when both zones have active stations,
        # regardless of actual trip direction.
        # With stations 1 and 3 selected: corridors (1,3) AND (3,1) both activate.

        corr_model = ZCorridorODModel(
            2, 2;
            in_vehicle_time_weight=1.0,
            corridor_weight=0.001,
            max_cluster_diameter=100.0
        )

        corr_mapping = StationSelection.create_map(corr_model, corr_data)
        @test corr_mapping.n_clusters == 4
        for c in 1:4
            @test length(corr_mapping.cluster_station_sets[c]) == 1
        end
        @test length(corr_mapping.corridor_indices) == 16

        corr_result = run_opt(
            corr_model, corr_data;
            optimizer_env=env, silent=true, do_optimize=true
        )
        @test corr_result.termination_status == MOI.OPTIMAL

        m = corr_result.model

        y_val = JuMP.value.(m[:y])
        @test y_val[corr_mapping.station_id_to_array_idx[1]] ≈ 1.0 atol=1e-5
        @test y_val[corr_mapping.station_id_to_array_idx[3]] ≈ 1.0 atol=1e-5
        @test sum(y_val) ≈ 2.0 atol=1e-5

        α_val = JuMP.value.(m[:α])
        idx1 = corr_mapping.station_id_to_array_idx[1]
        idx3 = corr_mapping.station_id_to_array_idx[3]
        zone1 = corr_mapping.cluster_labels[idx1]
        zone3 = corr_mapping.cluster_labels[idx3]

        @test α_val[zone1, 1] ≈ 1.0 atol=1e-5
        @test α_val[zone3, 1] ≈ 1.0 atol=1e-5
        for z in setdiff(1:4, [zone1, zone3])
            @test α_val[z, 1] ≈ 0.0 atol=1e-5
        end
        @test sum(α_val[:, 1]) ≈ 2.0 atol=1e-5

        f_val = JuMP.value.(m[:f_corridor])
        n_active_corridors = 0
        for (g, (a, b)) in enumerate(corr_mapping.corridor_indices)
            a_active = (a == zone1 || a == zone3)
            b_active = (b == zone1 || b == zone3)

            if a == b && a_active
                @test f_val[g, 1] ≈ 1.0 atol=1e-5
                n_active_corridors += 1
            elseif a == b && !a_active
                @test f_val[g, 1] ≈ 0.0 atol=1e-5
            elseif a != b && a_active && b_active
                @test f_val[g, 1] ≈ 1.0 atol=1e-5
                n_active_corridors += 1
            else
                @test f_val[g, 1] ≈ 0.0 atol=1e-5
            end
        end

        # z-based: 2 self + 2 cross (both directions) = 4
        @test n_active_corridors == 4
        @test sum(f_val[:, 1]) ≈ 4.0 atol=1e-5

        corridor_cost = 0.0
        for (g, _) in enumerate(corr_mapping.corridor_indices)
            corridor_cost += corr_mapping.corridor_costs[g] * JuMP.value(m[:f_corridor][g, 1])
        end
        corridor_penalty = 0.001 * corridor_cost
        @test corridor_penalty ≈ 4.002 atol=1e-3
    end

    @testset "XCorridorODModel zone and corridor activation" begin
        # x-based: corridors activate only when an actual assignment crosses them.
        # Request: origin=1, dest=3 → assigned to (pickup=1, dropoff=3)
        # Only corridor (zone1→zone3) should activate, NOT (zone3→zone1).

        corr_model = XCorridorODModel(
            2, 2;
            in_vehicle_time_weight=1.0,
            corridor_weight=0.001,
            max_cluster_diameter=100.0
        )

        corr_mapping = StationSelection.create_map(corr_model, corr_data)
        @test corr_mapping.n_clusters == 4

        corr_result = run_opt(
            corr_model, corr_data;
            optimizer_env=env, silent=true, do_optimize=true
        )
        @test corr_result.termination_status == MOI.OPTIMAL

        m = corr_result.model

        y_val = JuMP.value.(m[:y])
        @test y_val[corr_mapping.station_id_to_array_idx[1]] ≈ 1.0 atol=1e-5
        @test y_val[corr_mapping.station_id_to_array_idx[3]] ≈ 1.0 atol=1e-5
        @test sum(y_val) ≈ 2.0 atol=1e-5

        # No α variables in XCorridorODModel
        @test !haskey(object_dictionary(m), :α)

        f_val = JuMP.value.(m[:f_corridor])
        idx1 = corr_mapping.station_id_to_array_idx[1]
        idx3 = corr_mapping.station_id_to_array_idx[3]
        zone1 = corr_mapping.cluster_labels[idx1]
        zone3 = corr_mapping.cluster_labels[idx3]

        n_active_corridors = 0
        for (g, (a, b)) in enumerate(corr_mapping.corridor_indices)
            if a == zone1 && b == zone3
                # The actual trip direction: zone1 → zone3
                @test f_val[g, 1] ≈ 1.0 atol=1e-5
                n_active_corridors += 1
            elseif a == zone3 && b == zone1
                # Reverse direction: NOT activated (no trip goes zone3→zone1)
                @test f_val[g, 1] ≈ 0.0 atol=1e-5
            elseif a == b
                # Self-corridors: with x-based, these only activate if a trip
                # has pickup and dropoff in the same zone. Not the case here.
                @test f_val[g, 1] ≈ 0.0 atol=1e-5
            else
                @test f_val[g, 1] ≈ 0.0 atol=1e-5
            end
        end

        # x-based: only 1 corridor active (zone1→zone3)
        @test n_active_corridors == 1
        @test sum(f_val[:, 1]) ≈ 1.0 atol=1e-5

        # Corridor cost: only (zone1→zone3) with cost 2000
        corridor_cost = 0.0
        for (g, _) in enumerate(corr_mapping.corridor_indices)
            corridor_cost += corr_mapping.corridor_costs[g] * JuMP.value(m[:f_corridor][g, 1])
        end
        corridor_penalty = 0.001 * corridor_cost
        @test corridor_penalty ≈ 2.0 atol=1e-3
    end
end
