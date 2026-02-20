@testset "Transportation Model Integration" begin
    using JuMP

    if !gurobi_available
        @warn "Gurobi not available, skipping transportation integration tests"
        @test true
        return
    end

    # Create test data: 5 stations with clear zone separation
    stations = DataFrame(
        id = [1, 2, 3, 4, 5],
        lon = [113.0, 113.1, 113.2, 113.3, 113.4],
        lat = [28.0, 28.1, 28.2, 28.3, 28.4]
    )

    # Requests with directional patterns:
    # Zone 1 -> Zone 2 traffic (trips 1-3)
    # Zone 2 -> Zone 1 traffic (trips 4-5)
    requests = DataFrame(
        id = [1, 2, 3, 4, 5, 6],
        start_station_id = [1, 1, 2, 3, 4, 2],
        end_station_id = [3, 4, 5, 1, 2, 5],
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
    # TransportationModel construction validation
    # =========================================================================

    @testset "TransportationModel construction validation" begin
        @test_throws ArgumentError TransportationModel(0, 5)   # k must be positive
        @test_throws ArgumentError TransportationModel(5, 3)   # l must be >= k
        @test_throws ArgumentError TransportationModel(3, 5; in_vehicle_time_weight=-1.0)
        @test_throws ArgumentError TransportationModel(3, 5; activation_cost=-1.0)
        @test_throws ArgumentError TransportationModel(3, 5; max_cluster_diameter=0.0)
        @test_throws ArgumentError TransportationModel(3, 5; max_cluster_diameter=-1.0)

        # Walking distance limit validation
        @test_throws ArgumentError TransportationModel(3, 5;
            use_walking_distance_limit=true)  # missing max_walking_distance
        @test_throws ArgumentError TransportationModel(3, 5;
            use_walking_distance_limit=true, max_walking_distance=-1.0)

        model = TransportationModel(2, 3;
            in_vehicle_time_weight=2.0, activation_cost=10.0, max_cluster_diameter=500.0)
        @test model.k == 2
        @test model.l == 3
        @test model.in_vehicle_time_weight == 2.0
        @test model.activation_cost == 10.0
        @test model.max_cluster_diameter == 500.0
        @test model isa AbstractTransportationModel
        @test model isa AbstractODModel
        @test !model.use_walking_distance_limit
        @test isnothing(model.max_walking_distance)
    end

    @testset "TransportationModel n_clusters constructor" begin
        # Both specified: error
        @test_throws ArgumentError TransportationModel(2, 3;
            max_cluster_diameter=500.0, n_clusters=2)

        # n_clusters only
        model = TransportationModel(2, 3; n_clusters=2)
        @test model.n_clusters == 2
        @test isnothing(model.max_cluster_diameter)

        # Neither: defaults to max_cluster_diameter=1000.0
        model = TransportationModel(2, 3)
        @test model.max_cluster_diameter == 1000.0
        @test isnothing(model.n_clusters)

        # n_clusters must be positive
        @test_throws ArgumentError TransportationModel(2, 3; n_clusters=0)
    end

    @testset "TransportationModel with n_clusters mapping and build" begin
        model = TransportationModel(2, 3; n_clusters=2)
        mapping = StationSelection.create_map(model, data; optimizer_env=env)

        @test mapping isa TransportationMap
        @test mapping.n_clusters == 2
        @test length(mapping.cluster_labels) == 5
        @test length(mapping.cluster_medoids) == 2
        @test length(mapping.cluster_station_sets) == 2
    end

    @testset "TransportationModel with n_clusters run_opt" begin
        model = TransportationModel(3, 4; n_clusters=2, in_vehicle_time_weight=1.0, activation_cost=0.0)
        result = run_opt(model, data; optimizer_env=env, silent=true, do_optimize=true)
        @test result.termination_status == MOI.OPTIMAL
        @test !isnothing(result.objective_value)
        @test result.objective_value >= 0
    end

    # =========================================================================
    # TransportationModel mapping creation
    # =========================================================================

    @testset "TransportationModel mapping creation" begin
        model = TransportationModel(2, 3; max_cluster_diameter=500.0)
        mapping = StationSelection.create_map(model, data; optimizer_env=env)

        @test mapping isa TransportationMap
        @test length(mapping.station_id_to_array_idx) == 5
        @test length(mapping.array_idx_to_station_id) == 5
        @test length(mapping.scenarios) == 1

        # Clustering data
        @test mapping.n_clusters >= 1
        @test length(mapping.cluster_labels) == 5
        @test length(mapping.cluster_medoids) == mapping.n_clusters
        @test length(mapping.cluster_station_sets) == mapping.n_clusters

        # Anchor data
        @test length(mapping.active_anchors) > 0
        @test !isempty(mapping.anchor_scenarios)
        @test !isempty(mapping.I_g_pick)
        @test !isempty(mapping.I_g_drop)
        @test !isempty(mapping.m_pick)
        @test !isempty(mapping.m_drop)
        @test !isempty(mapping.P_g)
        @test !isempty(mapping.M_gs)

        # Each anchor should be an ordered pair of zones
        for (zone_a, zone_b) in mapping.active_anchors
            @test zone_a >= 1 && zone_a <= mapping.n_clusters
            @test zone_b >= 1 && zone_b <= mapping.n_clusters
        end

        # P(g) should contain station pairs from correct zones
        for (g_idx, (zone_a, zone_b)) in enumerate(mapping.active_anchors)
            for (j, k) in mapping.P_g[g_idx]
                @test mapping.cluster_labels[j] == zone_a
                @test mapping.cluster_labels[k] == zone_b
            end
        end
    end

    # =========================================================================
    # TransportationModel build
    # =========================================================================

    @testset "TransportationModel build" begin
        model = TransportationModel(2, 3;
            in_vehicle_time_weight=1.0, activation_cost=1.0, max_cluster_diameter=500.0)

        build_result = StationSelection.build_model(
            model, data; optimizer_env=env
        )
        m = build_result.model
        var_counts = build_result.counts.variables
        con_counts = build_result.counts.constraints
        extra_counts = build_result.counts.extras

        @test m isa JuMP.Model

        # Variable counts
        @test haskey(var_counts, "station_selection")
        @test haskey(var_counts, "scenario_activation")
        @test haskey(var_counts, "transportation_assignment")
        @test haskey(var_counts, "transportation_aggregation")
        @test haskey(var_counts, "transportation_flow")
        @test haskey(var_counts, "transportation_activation")

        @test var_counts["station_selection"] == 5
        @test var_counts["scenario_activation"] == 5
        @test var_counts["transportation_assignment"] > 0
        @test var_counts["transportation_aggregation"] > 0
        @test var_counts["transportation_flow"] > 0
        @test var_counts["transportation_activation"] > 0

        # Constraint counts
        @test haskey(con_counts, "station_limit")
        @test haskey(con_counts, "scenario_activation_limit")
        @test haskey(con_counts, "activation_linking")
        @test haskey(con_counts, "transportation_assignment")
        @test haskey(con_counts, "transportation_aggregation")
        @test haskey(con_counts, "transportation_flow_conservation")
        @test haskey(con_counts, "transportation_flow_activation")
        @test haskey(con_counts, "transportation_viability")

        @test con_counts["station_limit"] == 1
        @test con_counts["scenario_activation_limit"] == 1
        @test con_counts["activation_linking"] == 5
        @test con_counts["transportation_assignment"] > 0
        @test con_counts["transportation_aggregation"] > 0
        @test con_counts["transportation_flow_conservation"] > 0
        @test con_counts["transportation_flow_activation"] > 0
        @test con_counts["transportation_viability"] > 0

        # Extra counts
        @test haskey(extra_counts, "n_clusters")
        @test haskey(extra_counts, "n_active_anchors")
        @test haskey(extra_counts, "total_trips")
        @test extra_counts["total_trips"] > 0

        # Variables present in model
        @test haskey(object_dictionary(m), :y)
        @test haskey(object_dictionary(m), :z)
        @test haskey(object_dictionary(m), :x_pick)
        @test haskey(object_dictionary(m), :x_drop)
        @test haskey(object_dictionary(m), :p_agg)
        @test haskey(object_dictionary(m), :d_agg)
        @test haskey(object_dictionary(m), :f_transport)
        @test haskey(object_dictionary(m), :u_anchor)
    end

    # =========================================================================
    # TransportationModel run_opt without optimization
    # =========================================================================

    @testset "TransportationModel run_opt without optimization" begin
        model = TransportationModel(2, 3;
            in_vehicle_time_weight=1.0, activation_cost=1.0, max_cluster_diameter=500.0)
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
    # TransportationModel run_opt with optimization
    # =========================================================================

    @testset "TransportationModel run_opt with optimization" begin
        model = TransportationModel(3, 4;
            in_vehicle_time_weight=1.0, activation_cost=0.0, max_cluster_diameter=500.0)
        result = run_opt(
            model, data;
            optimizer_env=env,
            silent=true,
            do_optimize=true
        )

        @test result.termination_status == MOI.OPTIMAL
        @test !isnothing(result.objective_value)
        @test result.objective_value >= 0

        m = result.model
        mapping = result.mapping

        # Check y: exactly l stations selected
        y_val = JuMP.value.(m[:y])
        @test sum(y_val) ≈ 4.0 atol=1e-5

        # Check z: exactly k stations active per scenario
        z_val = JuMP.value.(m[:z])
        @test sum(z_val[:, 1]) ≈ 3.0 atol=1e-5

        # Check flow conservation: for each anchor/scenario,
        # total pickup = total dropoff = total trips
        x_pick = m[:x_pick]
        x_drop = m[:x_drop]
        p_agg = m[:p_agg]
        d_agg = m[:d_agg]
        f_transport = m[:f_transport]

        for (g_idx, anchor) in enumerate(mapping.active_anchors)
            zone_a, zone_b = anchor
            stations_a = mapping.cluster_station_sets[zone_a]
            stations_b = mapping.cluster_station_sets[zone_b]

            for s in mapping.anchor_scenarios[g_idx]
                total_trips_gs = mapping.M_gs[(g_idx, s)]

                # Total pickups should equal total trips
                total_p = sum(JuMP.value(p_agg[g_idx][s][j]) for j in stations_a)
                @test total_p ≈ total_trips_gs atol=1e-5

                # Total dropoffs should equal total trips
                total_d = sum(JuMP.value(d_agg[g_idx][s][k]) for k in stations_b)
                @test total_d ≈ total_trips_gs atol=1e-5

                # Total flow should equal total trips
                total_flow = sum(JuMP.value(f_transport[g_idx][s][(j, k)])
                    for (j, k) in mapping.P_g[g_idx])
                @test total_flow ≈ total_trips_gs atol=1e-5
            end
        end
    end

    # =========================================================================
    # Directionality test: anchors distinguish (a->b) from (b->a)
    # =========================================================================

    @testset "TransportationModel directionality" begin
        # Simple test: 4 stations, each in own zone, 1 request from 1->3
        dir_stations = DataFrame(
            id = [1, 2, 3, 4],
            lon = [113.0, 113.1, 113.2, 113.3],
            lat = [28.0, 28.1, 28.2, 28.3]
        )
        dir_requests = DataFrame(
            id = [1],
            start_station_id = [1],
            end_station_id = [3],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)]
        )
        dir_walking = Dict{Tuple{Int,Int}, Float64}()
        dir_routing = Dict{Tuple{Int,Int}, Float64}()
        for i in 1:4, j in 1:4
            dir_walking[(i, j)] = (i == j) ? 0.0 : 100000.0
            dir_routing[(i, j)] = (i == j) ? 1.0 : abs(i - j) * 1000.0
        end
        dir_data = StationSelection.create_station_selection_data(
            dir_stations, dir_requests, dir_walking;
            routing_costs=dir_routing,
            scenarios=[("2024-01-01 08:00:00", "2024-01-01 09:00:00")]
        )

        model = TransportationModel(
            2, 2;
            in_vehicle_time_weight=1.0,
            activation_cost=0.001,
            max_cluster_diameter=100.0
        )

        mapping = StationSelection.create_map(model, dir_data; optimizer_env=env)

        # Each station should be in its own cluster
        @test mapping.n_clusters == 4

        # Only one anchor should exist: (zone_of_station_1, zone_of_station_3)
        @test length(mapping.active_anchors) == 1
        zone1 = mapping.cluster_labels[mapping.station_id_to_array_idx[1]]
        zone3 = mapping.cluster_labels[mapping.station_id_to_array_idx[3]]
        @test mapping.active_anchors[1] == (zone1, zone3)

        # The reverse anchor (zone3, zone1) should NOT exist
        @test (zone3, zone1) ∉ mapping.active_anchors

        result = run_opt(
            model, dir_data;
            optimizer_env=env, silent=true, do_optimize=true
        )
        @test result.termination_status == MOI.OPTIMAL

        m = result.model

        # Only 1 anchor activation
        u_anchor = m[:u_anchor]
        @test JuMP.value(u_anchor[1][1]) ≈ 1.0 atol=1e-5
    end
end
