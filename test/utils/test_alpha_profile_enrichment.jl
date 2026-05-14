using Test
using DataFrames
using Dates

@testset "Alpha Profile Enrichment" begin

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    function make_route(station_indices::Vector{Int})
        legs = Tuple{Int,Int}[]
        m = length(station_indices)
        for i in 1:m, j in (i+1):m
            push!(legs, (station_indices[i], station_indices[j]))
        end
        StationSelection.RouteData(1, station_indices, 100.0, legs)
    end

    function make_bucket_state(routes_by_id, alpha_profile)
        StationSelection.RoutePoolState(
            1, 0,
            Set{Tuple{Int,Int}}(),
            0,
            routes_by_id,
            alpha_profile,
            Dict{String,Int}(),
            Dict{Int,Set{Symbol}}(),
            Set{Int}(),
            Set{Int}(),
            Set{Int}(),
            2,
        )
    end

    # -------------------------------------------------------------------------
    # AlphaEnrichmentConfig construction
    # -------------------------------------------------------------------------

    @testset "AlphaEnrichmentConfig defaults" begin
        cfg = StationSelection.AlphaEnrichmentConfig()
        @test cfg.enabled == true
        @test cfg.pressure_threshold == 0.70
        @test cfg.binding_threshold == 0.95
        @test cfg.alpha_scale_factor == 1.5
        @test cfg.min_profile_difference == 2
        @test cfg.max_profiles_per_route_sequence == 3
        @test cfg.max_new_profiles_per_iteration == 30
        @test cfg.max_candidate_routes_for_enrichment == 20
    end

    @testset "AlphaEnrichmentConfig disabled default in AlphaRouteRunnerConfig" begin
        init_spec = StationSelection.RoutePoolInitSpec(:generated)
        cfg = StationSelection.AlphaRouteRunnerConfig(
            init_spec;
            route_length_schedule=[2],
        )
        @test cfg.enrichment.enabled == false
    end

    @testset "AlphaEnrichmentConfig validation" begin
        @test_throws ArgumentError StationSelection.AlphaEnrichmentConfig(pressure_threshold=0.95, binding_threshold=0.90)
        @test_throws ArgumentError StationSelection.AlphaEnrichmentConfig(alpha_scale_factor=-0.1)
        @test_throws ArgumentError StationSelection.AlphaEnrichmentConfig(max_profiles_per_route_sequence=0)
    end

    # -------------------------------------------------------------------------
    # _build_leg_segments
    # -------------------------------------------------------------------------

    @testset "_build_leg_segments linear route" begin
        route = make_route([1, 2, 3])
        segs = StationSelection._build_leg_segments(route)
        @test segs[(1, 2)] == [1]
        @test segs[(1, 3)] == [1, 2]
        @test segs[(2, 3)] == [2]
    end

    @testset "_build_leg_segments repeated-station route A→B→A" begin
        # Route [1, 2, 1]: leg (1,2) uses seg 1, leg (2,1) uses seg 2
        legs = [(1, 2), (2, 1)]
        route = StationSelection.RouteData(1, [1, 2, 1], 100.0, legs)
        segs = StationSelection._build_leg_segments(route)
        @test segs[(1, 2)] == [1]
        @test segs[(2, 1)] == [2]
    end

    # -------------------------------------------------------------------------
    # _profile_is_valid
    # -------------------------------------------------------------------------

    @testset "_profile_is_valid: segment capacity" begin
        route    = make_route([1, 2, 3])
        capacity = 2
        no_existing = Dict{Tuple{Int,Int},Float64}[]

        # alpha[A,C]=2 alone — each segment carries 2, fine
        alpha_ac2 = Dict((1,2) => 0.0, (1,3) => 2.0, (2,3) => 0.0)
        @test StationSelection._profile_is_valid(alpha_ac2, no_existing, capacity, route, 0)

        # alpha[A,B]=2, alpha[A,C]=1 — seg 1 carries 2+1=3 > 2, invalid
        alpha_overflow = Dict((1,2) => 2.0, (1,3) => 1.0, (2,3) => 0.0)
        @test !StationSelection._profile_is_valid(alpha_overflow, no_existing, capacity, route, 0)

        # alpha[A,B]=2, alpha[B,C]=2 — segs disjoint, each carries 2, valid
        alpha_disjoint = Dict((1,2) => 2.0, (1,3) => 0.0, (2,3) => 2.0)
        @test StationSelection._profile_is_valid(alpha_disjoint, no_existing, capacity, route, 0)
    end

    @testset "_profile_is_valid: min profile difference" begin
        route    = make_route([1, 2, 3])
        capacity = 2

        existing = [Dict((1,2) => 1.0, (1,3) => 1.0, (2,3) => 1.0)]

        # L1 = 0 (exact duplicate) — invalid
        duplicate = Dict((1,2) => 1.0, (1,3) => 1.0, (2,3) => 1.0)
        @test !StationSelection._profile_is_valid(duplicate, existing, capacity, route, 0)

        # L1 = 3 (1,2)=>0, (1,3)=>2, (2,3)=>0: seg1=0+2=2, seg2=2+0=2 — valid for min_diff=2
        far_profile = Dict((1,2) => 0.0, (1,3) => 2.0, (2,3) => 0.0)
        @test StationSelection._profile_is_valid(far_profile, existing, capacity, route, 2)

        # L1 = 1: only (1,2) changes 1→0, others unchanged — invalid for min_diff=2
        too_close = Dict((1,2) => 0.0, (1,3) => 1.0, (2,3) => 1.0)
        @test !StationSelection._profile_is_valid(too_close, existing, capacity, route, 2)
    end

    @testset "_profile_is_valid: all-zero rejected" begin
        route    = make_route([1, 2, 3])
        capacity = 2
        alpha_zeros = Dict((1,2) => 0.0, (1,3) => 0.0, (2,3) => 0.0)
        @test !StationSelection._profile_is_valid(alpha_zeros, Dict{Tuple{Int,Int},Float64}[], capacity, route, 0)
    end

    # -------------------------------------------------------------------------
    # _build_enriched_alpha
    # -------------------------------------------------------------------------

    @testset "_build_enriched_alpha: binding leg gets more capacity" begin
        route     = make_route([1, 2, 3])
        capacity  = 2
        cfg       = StationSelection.AlphaEnrichmentConfig(
            pressure_threshold=0.70, binding_threshold=0.95, alpha_scale_factor=1.5,
        )
        existing  = Dict((1,2) => 1.0, (1,3) => 1.0, (2,3) => 1.0)
        pressure  = Dict{Tuple{Int,Int},Float64}((1,3) => 1.0)  # A→C is fully binding

        result = StationSelection._build_enriched_alpha(route, existing, pressure, capacity, cfg)
        @test !isnothing(result)
        @test result[(1,3)] > existing[(1,3)]   # pressured leg gets more
        @test result[(1,2)] < existing[(1,2)] || result[(2,3)] < existing[(2,3)]  # competing legs squeezed
    end

    @testset "_build_enriched_alpha: no pressured leg returns nothing" begin
        route    = make_route([1, 2, 3])
        capacity = 2
        cfg      = StationSelection.AlphaEnrichmentConfig()
        existing = Dict((1,2) => 1.0, (1,3) => 1.0, (2,3) => 1.0)
        pressure = Dict{Tuple{Int,Int},Float64}()  # nothing is pressured

        result = StationSelection._build_enriched_alpha(route, existing, pressure, capacity, cfg)
        @test isnothing(result)
    end

    @testset "_build_enriched_alpha: repeated station route A→B→A" begin
        legs  = [(1, 2), (2, 1)]
        route = StationSelection.RouteData(1, [1, 2, 1], 100.0, legs)
        cfg   = StationSelection.AlphaEnrichmentConfig(
            pressure_threshold=0.70, binding_threshold=0.95, alpha_scale_factor=1.5,
        )
        existing = Dict((1,2) => 1.0, (2,1) => 1.0)
        pressure = Dict{Tuple{Int,Int},Float64}((2,1) => 1.0)  # return leg is binding

        result = StationSelection._build_enriched_alpha(route, existing, pressure, 2, cfg)
        @test !isnothing(result)
        @test result[(2,1)] > existing[(2,1)]
    end

    # -------------------------------------------------------------------------
    # _route_sequence_profile_count and _all_profiles_for_sequence
    # -------------------------------------------------------------------------

    @testset "_route_sequence_profile_count" begin
        route_a = StationSelection.RouteData(1, [1, 3], 60.0, [(1,3)])
        route_b = StationSelection.RouteData(2, [1, 3], 60.0, [(1,3)])
        route_c = StationSelection.RouteData(3, [1, 2, 3], 80.0, [(1,2),(1,3),(2,3)])

        routes_by_id = Dict(1 => route_a, 2 => route_b, 3 => route_c)
        alpha = Dict{NTuple{3,Int},Float64}(
            (1,1,3) => 1.0, (2,1,3) => 2.0,
            (3,1,2) => 1.0, (3,1,3) => 1.0, (3,2,3) => 1.0
        )
        bs = make_bucket_state(routes_by_id, alpha)

        @test StationSelection._route_sequence_profile_count(bs, [1, 3]) == 2
        @test StationSelection._route_sequence_profile_count(bs, [1, 2, 3]) == 1
        @test StationSelection._route_sequence_profile_count(bs, [2, 3]) == 0
    end

    @testset "_all_profiles_for_sequence excludes removed routes" begin
        route_a = StationSelection.RouteData(1, [1, 3], 60.0, [(1,3)])
        route_b = StationSelection.RouteData(2, [1, 3], 60.0, [(1,3)])

        routes_by_id = Dict(1 => route_a, 2 => route_b)
        alpha = Dict{NTuple{3,Int},Float64}((1,1,3) => 1.0, (2,1,3) => 2.0)

        bs = make_bucket_state(routes_by_id, alpha)
        push!(bs.removed_route_ids, 2)

        profiles = StationSelection._all_profiles_for_sequence(bs, [1, 3])
        @test length(profiles) == 1
        @test profiles[1][(1, 3)] == 1.0
    end

    # -------------------------------------------------------------------------
    # Gurobi-backed integration test
    # -------------------------------------------------------------------------

    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end

    if gurobi_available
        @testset "enrich_alpha_profiles! adds profile when binding (Gurobi)" begin
            stations = DataFrame(id=[1,2,3], lon=[0.0,1.0,2.0], lat=[0.0,0.0,0.0])
            # Two requests for the same OD (1→3) to stress capacity
            requests = DataFrame(
                order_id = [1, 2, 3, 4],
                pax_num  = [1, 1, 1, 1],
                start_station_id = [1, 1, 1, 1],
                end_station_id   = [3, 3, 3, 3],
                request_time = fill(DateTime(2024,1,1,8,0,0), 4),
            )
            walking_costs = Dict{Tuple{Int,Int},Float64}()
            routing_costs = Dict{Tuple{Int,Int},Float64}()
            for i in 1:3, j in 1:3
                walking_costs[(i,j)] = abs(i-j)*60.0
                routing_costs[(i,j)] = abs(i-j)*30.0
            end
            data = StationSelection.create_station_selection_data(
                stations, requests, walking_costs;
                routing_costs=routing_costs,
                scenarios=[("2024-01-01 08:00:00", "2024-01-01 09:00:00")],
            )

            # vehicle_capacity=1 forces binding on every route
            model = StationSelection.AlphaRouteModel(
                2, 2;
                generate_routes=true,
                max_route_length=3,
                max_walking_distance=1000.0,
                max_detour_time=3600.0,
                max_detour_ratio=10.0,
                time_window_sec=3600,
                vehicle_capacity=1,
            )

            enrichment_cfg = StationSelection.AlphaEnrichmentConfig(
                enabled=true,
                pressure_threshold=0.50,
                binding_threshold=0.90,
                alpha_scale_factor=2.0,
                min_profile_difference=1,
                max_profiles_per_route_sequence=5,
                max_new_profiles_per_iteration=50,
            )
            runner_cfg = StationSelection.AlphaRouteRunnerConfig(
                StationSelection.RoutePoolInitSpec(:generated);
                route_length_schedule=[2, 3],
                max_iterations=2,
                enrichment=enrichment_cfg,
            )

            env = Gurobi.Env()
            base = StationSelection._build_alpha_route_base(model, data)
            state = StationSelection.initialize_route_pool(
                runner_cfg.init_spec,
                data,
                base.Q_s_t,
                base.valid_jk_pairs;
                vehicle_capacity=model.vehicle_capacity,
                max_detour_time=model.max_detour_time,
                max_detour_ratio=model.max_detour_ratio,
                stop_dwell_time=model.stop_dwell_time,
                initial_generated_max_route_length=first(runner_cfg.route_length_schedule),
            )

            pool_before = sum(length(b.routes_by_id) for b in values(state.bucket_states))

            strategy = StationSelection.AlphaRouteIterativeStrategy(runner_cfg)
            result = StationSelection.run_iteration_subproblem(
                strategy, model, data, state;
                optimizer_env=env, silent=true,
            )

            @test result.termination_status == StationSelection.MOI.OPTIMAL

            info = StationSelection.enrich_alpha_profiles!(
                state, result, enrichment_cfg, model.vehicle_capacity,
            )
            # With capacity=1 and 4 requests, some routes should be binding
            @test !info.skipped || info.n_binding_legs == 0
        end
    end

end
