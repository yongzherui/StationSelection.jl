using Test
using DataFrames
using Dates
using JuMP
const MOI = JuMP.MOI

@testset "Alpha Route Column Generation" begin
    function make_data(
        stations::Vector{Int},
        requests::DataFrame;
        scenario_window::Tuple{String, String}=(
            "2024-01-01 08:00:00",
            "2024-01-01 09:00:00",
        ),
        walking_scale::Float64=60.0,
        routing_scale::Float64=30.0,
    )
        station_df = DataFrame(
            id = stations,
            lon = Float64.(stations .- first(stations)),
            lat = zeros(Float64, length(stations)),
        )

        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in stations, j in stations
            walking_costs[(i, j)] = abs(i - j) * walking_scale
            routing_costs[(i, j)] = abs(i - j) * routing_scale
        end

        return StationSelection.create_station_selection_data(
            station_df,
            requests,
            walking_costs;
            routing_costs=routing_costs,
            scenarios=[scenario_window],
        )
    end

    function make_model(; vehicle_capacity::Int=2, max_route_length::Int=3)
        return StationSelection.AlphaRouteModel(
            2, 2;
            generate_routes=true,
            max_route_length=max_route_length,
            max_walking_distance=10_000.0,
            max_detour_time=3600.0,
            max_detour_ratio=10.0,
            time_window_sec=3600,
            vehicle_capacity=vehicle_capacity,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
        )
    end

    function make_state(model, data)
        base = StationSelection._build_alpha_route_base(model, data)
        return StationSelection.AlphaRouteColumnGenerationState(
            StationSelection.initialize_route_pool(
                StationSelection.RoutePoolInitSpec(:direct_only),
                data,
                base.Q_s_t,
                base.valid_jk_pairs;
                vehicle_capacity=model.vehicle_capacity,
                route_generation_method=model.route_generation_method,
                iterative_config=model.iterative_route_generation_config,
                max_detour_time=model.max_detour_time,
                max_detour_ratio=model.max_detour_ratio,
                stop_dwell_time=model.stop_dwell_time,
                initial_generated_max_route_length=nothing,
            ),
            nothing,
            nothing,
        )
    end

    function make_duals(entries::Vector{Tuple{Int, Int, Int, Int, Float64}})
        return StationSelection.AlphaRouteCGDuals(
            Dict((s, t_id, j_idx, k_idx) => value for (s, t_id, j_idx, k_idx, value) in entries)
        )
    end

    function alpha_value(column, j_idx, k_idx)
        return get(column.alpha_profile, (column.route.id, j_idx, k_idx), 0.0)
    end

    function bucket_pricing(
        model,
        data,
        dual_entries;
        rc_tolerance::Float64=-1e-6,
        max_columns::Int=10,
        time_limit_sec::Float64=5.0,
    )
        base = StationSelection._build_alpha_route_base(model, data)
        qbar = StationSelection.compute_qbar(data.scenarios[1], base.valid_jk_pairs, model.time_window_sec)
        bucket_duals = Dict(0 => Dict((j_idx, k_idx) => value for (_, _, j_idx, k_idx, value) in dual_entries))
        return StationSelection.price_scenario(
            1,
            model,
            data,
            bucket_duals,
            qbar;
            rc_tolerance=rc_tolerance,
            max_columns=max_columns,
            time_limit_sec=time_limit_sec,
            return_all_negative=true,
        )
    end

    @testset "direct_only init seeds only direct routes" begin
        requests = DataFrame(
            order_id = [1],
            pax_num = [1],
            start_station_id = [1],
            end_station_id = [3],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)],
        )
        data = make_data([1, 2, 3], requests)
        model = make_model(vehicle_capacity=3, max_route_length=2)
        base = StationSelection._build_alpha_route_base(model, data)

        state = StationSelection.initialize_route_pool(
            StationSelection.RoutePoolInitSpec(:direct_only),
            data,
            base.Q_s_t,
            base.valid_jk_pairs;
            vehicle_capacity=model.vehicle_capacity,
            route_generation_method=model.route_generation_method,
            iterative_config=model.iterative_route_generation_config,
            max_detour_time=model.max_detour_time,
            max_detour_ratio=model.max_detour_ratio,
            stop_dwell_time=model.stop_dwell_time,
            initial_generated_max_route_length=nothing,
        )

        bucket_state = state.bucket_states[(1, 0)]
        @test !isempty(bucket_state.routes_by_id)
        @test all(length(route.station_indices) == 2 for route in values(bucket_state.routes_by_id))
        @test length(bucket_state.routes_by_id) == length(bucket_state.direct_seed_route_ids)
        @test bucket_state.current_generated_max_route_length == 2
    end

    @testset "compute_qbar respects bucketed valid pairs" begin
        requests = DataFrame(
            order_id = [1],
            pax_num = [2],
            start_station_id = [1],
            end_station_id = [2],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)],
        )
        data = make_data([1, 2], requests)
        model = make_model(vehicle_capacity=3, max_route_length=2)
        base = StationSelection._build_alpha_route_base(model, data)
        qbar = StationSelection.compute_qbar(data.scenarios[1], base.valid_jk_pairs, model.time_window_sec)
        @test qbar[0].caps[(1, 2)] == 2
    end

    @testset "two-station exact pricing returns profitable direct column" begin
        requests = DataFrame(
            order_id = [1],
            pax_num = [2],
            start_station_id = [1],
            end_station_id = [2],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)],
        )
        data = make_data([1, 2], requests; routing_scale=10.0)
        model = make_model(vehicle_capacity=3, max_route_length=2)
        pricing = bucket_pricing(model, data, [(1, 0, 1, 2, 50.0)])

        @test pricing.status == :optimal
        @test !isempty(pricing.columns)
        best = pricing.columns[1]
        @test best.scenario_idx == 1
        @test best.time_id == 0
        @test best.route.station_indices == [1, 2]
        @test alpha_value(best, 1, 2) == 2.0
        @test best.reduced_cost < 0.0
    end

    @testset "three-station simultaneous capacity is enforced through onboard load" begin
        requests = DataFrame(
            order_id = [1, 2, 3],
            pax_num = [1, 1, 1],
            start_station_id = [1, 1, 2],
            end_station_id = [2, 3, 3],
            request_time = fill(DateTime(2024, 1, 1, 8, 0, 0), 3),
        )
        data = make_data([1, 2, 3], requests; routing_scale=1.0)
        model = make_model(vehicle_capacity=2, max_route_length=3)
        pricing = bucket_pricing(model, data, [
            (1, 0, 1, 2, 10.0),
            (1, 0, 1, 3, 10.0),
            (1, 0, 2, 3, 10.0),
        ])

        triple = findfirst(col -> col.route.station_indices == [1, 2, 3], pricing.columns)
        @test triple !== nothing
        col = pricing.columns[triple]
        seg_12 = alpha_value(col, 1, 2) + alpha_value(col, 1, 3)
        seg_23 = alpha_value(col, 1, 3) + alpha_value(col, 2, 3)
        @test seg_12 <= model.vehicle_capacity + 1e-9
        @test seg_23 <= model.vehicle_capacity + 1e-9
    end

    @testset "zero demand cap prevents pickup even with positive dual" begin
        requests = DataFrame(
            order_id = [1],
            pax_num = [1],
            start_station_id = [1],
            end_station_id = [2],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)],
        )
        data = make_data([1, 2, 3], requests; routing_scale=10.0)
        model = make_model(vehicle_capacity=3, max_route_length=2)
        pricing = bucket_pricing(model, data, [
            (1, 0, 1, 2, 10.0),
            (1, 0, 1, 3, 20.0),
        ])

        @test all(alpha_value(col, 1, 3) == 0.0 for col in pricing.columns)
    end

    @testset "station with no outgoing demand is still visited for dropoff" begin
        requests = DataFrame(
            order_id = [1],
            pax_num = [1],
            start_station_id = [1],
            end_station_id = [2],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)],
        )
        data = make_data([1, 2, 3], requests; routing_scale=5.0)
        model = make_model(vehicle_capacity=2, max_route_length=3)
        base = StationSelection._build_alpha_route_base(model, data)
        qbar = StationSelection.compute_qbar(data.scenarios[1], base.valid_jk_pairs, model.time_window_sec)
        pricing_data = StationSelection.AlphaRouteBucketPricingData(
            1,
            0,
            [1, 2, 3],
            StationSelection.AlphaRouteBucketDemandCaps(1, 0, qbar[0].caps),
            Dict((1, 2) => 10.0),
            model.vehicle_capacity,
            model.max_route_length,
            model.stop_dwell_time,
            model.route_regularization_weight,
            model.repositioning_time,
        )
        label = StationSelection.AlphaRoutePricingLabel(
            1,
            BitSet([1]),
            [1],
            0.0,
            Dict((1, 2) => 1),
            Dict((1, 2) => 1),
            -10.0,
        )
        candidates = StationSelection.candidate_next_stations(label, pricing_data, data)
        @test 2 in candidates
    end

    @testset "demand caps prevent over-generation" begin
        requests = DataFrame(
            order_id = [1],
            pax_num = [2],
            start_station_id = [1],
            end_station_id = [2],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)],
        )
        data = make_data([1, 2], requests; routing_scale=10.0)
        model = make_model(vehicle_capacity=10, max_route_length=2)
        pricing = bucket_pricing(model, data, [(1, 0, 1, 2, 100.0)])

        @test !isempty(pricing.columns)
        @test alpha_value(pricing.columns[1], 1, 2) == 2.0
    end

    @testset "pricing accepts normalized negative master duals" begin
        requests = DataFrame(
            order_id = [1],
            pax_num = [2],
            start_station_id = [1],
            end_station_id = [2],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)],
        )
        data = make_data([1, 2], requests; routing_scale=10.0)
        model = make_model(vehicle_capacity=3, max_route_length=2)
        state = make_state(model, data)
        pricing = StationSelection.solve_alpha_route_pricing(
            model,
            data,
            state,
            StationSelection.AlphaRouteCGDuals(
                Dict((1, 0, 1, 2) => 50.0),
                Dict((1, 0, 1, 2) => -50.0),
            );
            max_columns=10,
            time_limit_sec=5.0,
        )

        @test !isempty(pricing.columns)
        @test pricing.columns[1].route.station_indices == [1, 2]
    end

    gurobi_available = try
        using Gurobi
        Gurobi.Env()
        true
    catch
        false
    end

    @testset "restricted master build and runner smoke" begin
        if !gurobi_available
            @test true
        else
            requests = DataFrame(
                order_id = [1, 2],
                pax_num = [1, 1],
                start_station_id = [1, 1],
                end_station_id = [3, 3],
                request_time = [
                    DateTime(2024, 1, 1, 8, 0, 0),
                    DateTime(2024, 1, 1, 8, 10, 0),
                ],
            )
            data = make_data([1, 2, 3], requests)
            model = make_model(vehicle_capacity=2, max_route_length=3)

            base = StationSelection._build_alpha_route_base(model, data)
            route_pool = StationSelection.initialize_route_pool(
                StationSelection.RoutePoolInitSpec(:direct_only),
                data,
                base.Q_s_t,
                base.valid_jk_pairs;
                vehicle_capacity=model.vehicle_capacity,
                route_generation_method=model.route_generation_method,
                iterative_config=model.iterative_route_generation_config,
                max_detour_time=model.max_detour_time,
                max_detour_ratio=model.max_detour_ratio,
                stop_dwell_time=model.stop_dwell_time,
                initial_generated_max_route_length=nothing,
            )

            build_result = StationSelection.build_alpha_route_restricted_master(
                model,
                data,
                route_pool,
            )
            m = build_result.model

            @test all(!JuMP.is_binary(v) for v in m[:y])
            @test all(!JuMP.is_binary(v) for v in m[:z])
            @test all(!JuMP.is_integer(v) for v in values(m[:theta_r_ts]))
            @test haskey(m.obj_dict, :arm_capacity_constraints)
            @test !isempty(m[:arm_capacity_constraints])

            config = StationSelection.AlphaRouteColumnGenerationConfig(
                max_iterations=2,
                pricing_time_limit_sec=5.0,
            )
            runner_result = StationSelection.run_alpha_route_column_generation(
                model,
                data,
                config;
                silent=true,
            )
            @test runner_result.final_result.termination_status == MOI.OPTIMAL
            @test haskey(runner_result.final_result.metadata, "alpha_route_column_generation")
            @test runner_result.convergence_reason in ("no_negative_reduced_cost_column", "max_iterations", "pricing_time_limit")
        end
    end
end
