@testset "Objective Decomposition" begin
    using JSON
    using CSV
    using DataFrames
    using JuMP

    # =========================================================================
    # Shared test data (4 stations in a line, used by unit and integration tests)
    #
    # Costs are kept deliberately simple:
    #   walking_costs[(i,j)] = |i-j| * 200
    #   routing_costs[(i,j)] = |i-j| * 10
    # =========================================================================

    stations = DataFrame(
        id  = [1, 2, 3, 4],
        lon = [113.0, 113.1, 113.2, 113.3],
        lat = [28.0,  28.0,  28.0,  28.0 ]
    )

    requests = DataFrame(
        order_id         = [1, 2, 3],
        start_station_id = [1, 1, 2],
        end_station_id   = [3, 4, 4],
        request_time     = [
            DateTime(2024, 1, 1, 8, 0, 0),
            DateTime(2024, 1, 1, 8, 0, 30),
            DateTime(2024, 1, 1, 8, 1, 0)
        ]
    )

    walking_costs = Dict{Tuple{Int,Int}, Float64}()
    routing_costs = Dict{Tuple{Int,Int}, Float64}()
    for i in 1:4, j in 1:4
        walking_costs[(i, j)] = abs(i - j) * 200.0
        routing_costs[(i, j)] = abs(i - j) * 10.0
    end

    scenarios = [("2024-01-01 08:00:00", "2024-01-01 09:00:00")]
    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs = routing_costs,
        scenarios     = scenarios
    )

    # =========================================================================
    # Unit tests: no solver required — build synthetic export dirs by hand
    # =========================================================================

    @testset "Unit: struct construction" begin
        d = ObjectiveDecomposition(
            "TestModel",
            100.0, 200.0, 0.5, 100.0,   # walking, routing raw/weight/weighted
            50.0, 1.0, 50.0,             # corridor raw/weight/penalty
            10, 1.0, 10.0,               # n_routes, fr_weight, fr_penalty
            300.0, 5.0, 3.0, 1.0, 300.0, 8.0,  # vehicle flow, ss, sd, weight, cost, savings
            552.0,                       # computed_total
            552.0                        # reported_objective
        )

        @test d.model_type == "TestModel"
        @test d.walking_cost ≈ 100.0
        @test d.routing_cost_raw ≈ 200.0
        @test d.in_vehicle_time_weight ≈ 0.5
        @test d.weighted_routing_cost ≈ 100.0
        @test d.corridor_cost_raw ≈ 50.0
        @test d.corridor_weight ≈ 1.0
        @test d.corridor_penalty ≈ 50.0
        @test d.n_activated_routes == 10
        @test d.flow_regularization_weight ≈ 1.0
        @test d.flow_regularization_penalty ≈ 10.0
        @test d.vehicle_flow_cost_raw ≈ 300.0
        @test d.same_source_savings_raw ≈ 5.0
        @test d.same_dest_savings_raw ≈ 3.0
        @test d.vehicle_routing_weight ≈ 1.0
        @test d.vehicle_routing_cost ≈ 300.0
        @test d.pooling_savings ≈ 8.0
        @test d.computed_total ≈ 552.0
        @test d.reported_objective == 552.0
    end

    @testset "Unit: show output contains key fields" begin
        d = ObjectiveDecomposition(
            "XCorridorWithFlowRegularizerModel",
            1000.0, 2000.0, 0.5, 1000.0,
            500.0, 1.0, 500.0,
            15, 1.0, 15.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            2515.0, 2515.0
        )
        output = sprint(show, d)

        @test occursin("XCorridorWithFlowRegularizerModel", output)
        @test occursin("Walking cost", output)
        @test occursin("Routing cost", output)
        @test occursin("Corridor", output)
        @test occursin("Flow reg.", output)
        @test occursin("Computed total", output)
        @test occursin("match", output)  # should say ✓ match
    end

    @testset "Unit: show reports MISMATCH when totals differ" begin
        d = ObjectiveDecomposition(
            "SomeModel",
            1000.0, 2000.0, 1.0, 2000.0,
            0.0, 0.0, 0.0,
            0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            3000.0, 9999.0   # deliberately wrong reported objective
        )
        output = sprint(show, d)
        @test occursin("MISMATCH", output)
    end

    @testset "Unit: reported_objective = nothing" begin
        d = ObjectiveDecomposition(
            "SomeModel",
            500.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0,
            0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            500.0, nothing
        )
        output = sprint(show, d)
        @test occursin("not available", output)
    end

    # =========================================================================
    # Unit: synthetic run_dir — hand-crafted CSVs, exact arithmetic check
    #
    # Network:  station 1, 2, 3, 4  (routing_cost[i,j] = |i-j|*10)
    # Scenario 1 has two OD pairs:
    #   (1,3) with q=2 → assigned to (1,2):  walk=0+200=200, route=10
    #   (2,4) with q=1 → assigned to (2,4):  walk=0+0=0,   route=20
    #
    # od_counts from data (all requests in scenario 1):
    #   (1,3) → 1 request, (1,4) → 1 request, (2,4) → 1 request
    #
    # We'll fabricate assignment_variables.csv with these assignments and use
    # the real od_counts from `data`, then verify the computed sums.
    # =========================================================================

    @testset "Unit: decompose from synthetic CSVs (ClusteringTwoStageODModel)" begin
        # od_counts from `data` (1 scenario, 3 requests):
        #   (1,3) → 1, (1,4) → 1, (2,4) → 1
        #
        # Assign:
        #   OD (1,3), s=1 → pickup=1, dropoff=2  (val=1)
        #     walk = |1-1|*200 + |2-3|*200 = 0+200 = 200, route = |1-2|*10 = 10
        #     q=1 → contrib: walk=200, route=10
        #   OD (2,4), s=1 → pickup=2, dropoff=4  (val=1)
        #     walk = |2-2|*200 + |4-4|*200 = 0+0 = 0, route = |2-4|*10 = 20
        #     q=1 → contrib: walk=0, route=20
        #
        # Expected (w_ivt=0.5):
        #   walking_cost     = 200 + 0 = 200
        #   routing_cost_raw = 10 + 20 = 30
        #   weighted_routing = 0.5 * 30 = 15
        #   n_activated_routes = 2 distinct (pickup,dropoff,scenario) tuples
        #   computed_total   = 200 + 15 = 215

        mktempdir() do run_dir
            export_dir = joinpath(run_dir, "variable_exports")
            mkpath(export_dir)

            # Write metrics.json
            metrics = Dict(
                "method" => "ClusteringTwoStageODModel",
                "model"  => Dict("in_vehicle_time_weight" => 0.5),
                "solve"  => Dict("objective_value" => 215.0)
            )
            open(joinpath(run_dir, "metrics.json"), "w") do f
                JSON.print(f, metrics)
            end

            # Write assignment_variables.csv
            assign_df = DataFrame(
                scenario   = [1, 1],
                od_idx     = [1, 2],
                origin_id  = [1, 2],
                dest_id    = [3, 4],
                pickup_idx = [1, 2],
                dropoff_idx= [2, 4],
                pickup_id  = [1, 2],
                dropoff_id = [2, 4],
                value      = [1.0, 1.0]
            )
            CSV.write(joinpath(export_dir, "assignment_variables.csv"), assign_df)

            d = decompose_objective(run_dir, data)

            @test d.model_type == "ClusteringTwoStageODModel"
            @test d.walking_cost ≈ 200.0
            @test d.routing_cost_raw ≈ 30.0
            @test d.in_vehicle_time_weight ≈ 0.5
            @test d.weighted_routing_cost ≈ 15.0
            @test d.corridor_cost_raw ≈ 0.0
            @test d.corridor_penalty ≈ 0.0
            @test d.n_activated_routes == 2
            @test d.flow_regularization_penalty ≈ 0.0
            @test d.vehicle_flow_cost_raw ≈ 0.0
            @test d.computed_total ≈ 215.0
            @test d.reported_objective ≈ 215.0
        end
    end

    @testset "Unit: decompose corridor penalty from synthetic CSVs" begin
        # Two corridors used, costs 50.0 and 30.0
        # corridor_weight = 2.0 → penalty = 2*(50+30) = 160
        # One OD assignment: (1,4)→(1,4), q=1, w_ivt=1.0
        #   walk = 0+0=0, route = 30, weighted = 30
        # computed_total = 0 + 30 + 160 = 190

        mktempdir() do run_dir
            export_dir = joinpath(run_dir, "variable_exports")
            mkpath(export_dir)

            metrics = Dict(
                "method" => "XCorridorODModel",
                "model"  => Dict(
                    "in_vehicle_time_weight" => 1.0,
                    "corridor_weight"        => 2.0
                ),
                "solve"  => Dict("objective_value" => 190.0)
            )
            open(joinpath(run_dir, "metrics.json"), "w") do f
                JSON.print(f, metrics)
            end

            assign_df = DataFrame(
                scenario   = [1],
                od_idx     = [1],
                origin_id  = [1],
                dest_id    = [4],
                pickup_idx = [1],
                dropoff_idx= [4],
                pickup_id  = [1],
                dropoff_id = [4],
                value      = [1.0]
            )
            CSV.write(joinpath(export_dir, "assignment_variables.csv"), assign_df)

            # corridor_costs.csv: two corridors
            costs_df = DataFrame(
                corridor_idx  = [1, 2],
                cluster_a     = [1, 1],
                cluster_b     = [2, 3],
                medoid_a_id   = [1, 1],
                medoid_b_id   = [2, 3],
                corridor_cost = [50.0, 30.0]
            )
            CSV.write(joinpath(export_dir, "corridor_costs.csv"), costs_df)

            # corridor_usage.csv: both corridors active in scenario 1
            usage_df = DataFrame(
                corridor_idx = [1, 2],
                cluster_a    = [1, 1],
                cluster_b    = [2, 3],
                scenario     = [1, 1],
                value        = [1.0, 1.0]
            )
            CSV.write(joinpath(export_dir, "corridor_usage.csv"), usage_df)

            d = decompose_objective(run_dir, data)

            @test d.corridor_cost_raw ≈ 80.0   # 50+30
            @test d.corridor_weight ≈ 2.0
            @test d.corridor_penalty ≈ 160.0
            @test d.routing_cost_raw ≈ 30.0    # |1-4|*10
            @test d.weighted_routing_cost ≈ 30.0
            @test d.computed_total ≈ 190.0
            @test d.reported_objective ≈ 190.0
        end
    end

    @testset "Unit: flow regularization penalty from synthetic CSVs" begin
        # Two distinct (pickup, dropoff, scenario) routes activated
        # flow_regularization_weight = 3.0 → penalty = 3*2 = 6
        # One OD assignment: (1,3)→(1,2), q=1, w_ivt=1.0
        #   walk = 0+200=200, route=10
        # computed_total = 200+10+6 = 216

        mktempdir() do run_dir
            export_dir = joinpath(run_dir, "variable_exports")
            mkpath(export_dir)

            metrics = Dict(
                "method" => "XCorridorWithFlowRegularizerModel",
                "model"  => Dict(
                    "in_vehicle_time_weight"     => 1.0,
                    "corridor_weight"            => 0.0,
                    "flow_regularization_weight" => 3.0
                ),
                "solve"  => Dict("objective_value" => 216.0)
            )
            open(joinpath(run_dir, "metrics.json"), "w") do f
                JSON.print(f, metrics)
            end

            # Two rows → two distinct (pickup_id, dropoff_id, scenario) routes
            assign_df = DataFrame(
                scenario   = [1, 1],
                od_idx     = [1, 2],
                origin_id  = [1, 1],
                dest_id    = [3, 4],
                pickup_idx = [1, 1],
                dropoff_idx= [2, 2],
                pickup_id  = [1, 1],
                dropoff_id = [2, 2],
                value      = [1.0, 1.0]
            )
            CSV.write(joinpath(export_dir, "assignment_variables.csv"), assign_df)

            d = decompose_objective(run_dir, data)

            # Both rows share (pickup=1, dropoff=2, s=1) → only 1 distinct route
            @test d.n_activated_routes == 1
            @test d.flow_regularization_weight ≈ 3.0
            @test d.flow_regularization_penalty ≈ 3.0
        end
    end

    @testset "Unit: TSD vehicle routing from synthetic CSVs" begin
        # flow_variables: arc (1,4) with cost |1-4|*10=30, val=1.0
        # same_source_pooling: j=1, k=2, l=4 → saving = c(1,4)-c(2,4) = 30-20=10
        # same_dest_pooling: empty
        # vehicle_routing_weight = 1.0
        # vehicle_routing_cost = 1.0 * 30 = 30
        # pooling_savings = 1.0 * 10 = 10
        # walking/routing from assignment: one row (1,4)→(1,4), q=1, w_ivt=0.0
        #   walk=0, route=30, weighted=0
        # computed_total = 0 + 30 - 10 = 20

        mktempdir() do run_dir
            export_dir = joinpath(run_dir, "variable_exports")
            mkpath(export_dir)

            metrics = Dict(
                "method" => "TwoStageSingleDetourModel",
                "model"  => Dict(
                    "in_vehicle_time_weight" => 0.0,
                    "vehicle_routing_weight" => 1.0
                ),
                "solve"  => Dict("objective_value" => 20.0)
            )
            open(joinpath(run_dir, "metrics.json"), "w") do f
                JSON.print(f, metrics)
            end

            # Assignment (walking/routing contribution = 0 since w_ivt=0)
            assign_df = DataFrame(
                scenario   = [1],
                time_id    = [0],
                origin_id  = [1],
                dest_id    = [4],
                pickup_idx = [1],
                dropoff_idx= [4],
                pickup_id  = [1],
                dropoff_id = [4],
                value      = [1.0]
            )
            CSV.write(joinpath(export_dir, "assignment_variables.csv"), assign_df)

            flow_df = DataFrame(
                scenario = [1],
                time_id  = [0],
                j_array  = [1],
                k_array  = [4],
                j_id     = [1],
                k_id     = [4],
                value    = [1.0]
            )
            CSV.write(joinpath(export_dir, "flow_variables.csv"), flow_df)

            ss_df = DataFrame(
                scenario = [1],
                time_id  = [0],
                xi_idx   = [1],
                j_id     = [1],
                k_id     = [2],
                l_id     = [4],
                value    = [1.0]
            )
            CSV.write(joinpath(export_dir, "same_source_pooling.csv"), ss_df)

            d = decompose_objective(run_dir, data)

            @test d.vehicle_flow_cost_raw ≈ 30.0    # c(1,4) = 30
            @test d.same_source_savings_raw ≈ 10.0  # c(1,4)-c(2,4) = 30-20
            @test d.same_dest_savings_raw ≈ 0.0
            @test d.vehicle_routing_weight ≈ 1.0
            @test d.vehicle_routing_cost ≈ 30.0
            @test d.pooling_savings ≈ 10.0
            @test d.computed_total ≈ 20.0
            @test d.reported_objective ≈ 20.0
        end
    end

    @testset "Unit: missing metrics.json raises error" begin
        mktempdir() do run_dir
            @test_throws ErrorException decompose_objective(run_dir, data)
        end
    end

    @testset "Unit: empty/missing assignment CSV returns zero costs" begin
        mktempdir() do run_dir
            export_dir = joinpath(run_dir, "variable_exports")
            mkpath(export_dir)

            metrics = Dict(
                "method" => "ClusteringTwoStageODModel",
                "model"  => Dict("in_vehicle_time_weight" => 1.0),
                "solve"  => Dict("objective_value" => 0.0)
            )
            open(joinpath(run_dir, "metrics.json"), "w") do f
                JSON.print(f, metrics)
            end
            # No assignment_variables.csv written

            d = decompose_objective(run_dir, data)

            @test d.walking_cost ≈ 0.0
            @test d.routing_cost_raw ≈ 0.0
            @test d.n_activated_routes == 0
            @test d.computed_total ≈ 0.0
        end
    end

    # =========================================================================
    # Integration tests: solve a real model, export, then decompose
    # =========================================================================

    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end

    if !gurobi_available
        @warn "Gurobi not available — skipping objective decomposition integration tests"
        @test true
    else

    env = Gurobi.Env()

    @testset "Integration: ClusteringTwoStageODModel round-trip" begin
        model = ClusteringTwoStageODModel(3, 4)
        result = run_opt(model, data; optimizer_env=env, silent=true)
        @test result.termination_status == JuMP.MOI.OPTIMAL

        reported = result.objective_value

        mktempdir() do run_dir
            export_variables(result, run_dir)

            # Write a minimal metrics.json (export_variables doesn't write one)
            metrics = Dict(
                "method" => "ClusteringTwoStageODModel",
                "model"  => Dict(
                    "in_vehicle_time_weight" => model.in_vehicle_time_weight
                ),
                "solve"  => Dict("objective_value" => reported)
            )
            open(joinpath(run_dir, "metrics.json"), "w") do f
                JSON.print(f, metrics)
            end

            d = decompose_objective(run_dir, data)

            @test d.model_type == "ClusteringTwoStageODModel"
            @test d.walking_cost > 0.0
            @test d.corridor_penalty ≈ 0.0
            @test d.flow_regularization_penalty ≈ 0.0
            @test d.vehicle_flow_cost_raw ≈ 0.0
            @test d.reported_objective ≈ reported

            # computed_total should match solver's reported value within 1.0
            @test abs(d.computed_total - reported) < 1.0

            println("\n--- ClusteringTwoStageODModel decomposition ---")
            println(d)
        end
    end

    @testset "Integration: XCorridorWithFlowRegularizerModel round-trip" begin
        # Use n_clusters=2 (simple clustering) so corridor costs are deterministic
        model = XCorridorWithFlowRegularizerModel(
            3, 4;
            n_clusters              = 2,
            corridor_weight         = 1.0,
            flow_regularization_weight = 1.0,
            in_vehicle_time_weight  = 0.5
        )
        result = run_opt(model, data; optimizer_env=env, silent=true)
        @test result.termination_status == JuMP.MOI.OPTIMAL

        reported = result.objective_value

        mktempdir() do run_dir
            export_variables(result, run_dir)

            # Verify corridor_costs.csv is written (Part 0 of the plan)
            export_dir = joinpath(run_dir, "variable_exports")
            @test isfile(joinpath(export_dir, "corridor_costs.csv"))

            costs_df = CSV.read(joinpath(export_dir, "corridor_costs.csv"), DataFrame)
            @test :corridor_idx  in propertynames(costs_df)
            @test :cluster_a     in propertynames(costs_df)
            @test :cluster_b     in propertynames(costs_df)
            @test :medoid_a_id   in propertynames(costs_df)
            @test :medoid_b_id   in propertynames(costs_df)
            @test :corridor_cost in propertynames(costs_df)
            @test nrow(costs_df) > 0
            @test all(costs_df.corridor_cost .>= 0.0)

            metrics = Dict(
                "method" => "XCorridorWithFlowRegularizerModel",
                "model"  => Dict(
                    "in_vehicle_time_weight"     => model.in_vehicle_time_weight,
                    "corridor_weight"            => model.corridor_weight,
                    "flow_regularization_weight" => model.flow_regularization_weight
                ),
                "solve"  => Dict("objective_value" => reported)
            )
            open(joinpath(run_dir, "metrics.json"), "w") do f
                JSON.print(f, metrics)
            end

            d = decompose_objective(run_dir, data)

            @test d.model_type == "XCorridorWithFlowRegularizerModel"
            @test d.walking_cost > 0.0
            @test d.in_vehicle_time_weight ≈ 0.5
            @test d.corridor_weight ≈ 1.0
            @test d.flow_regularization_weight ≈ 1.0
            @test d.n_activated_routes > 0
            @test d.flow_regularization_penalty > 0.0
            @test d.reported_objective ≈ reported

            # computed_total should match solver's reported value within 1.0
            @test abs(d.computed_total - reported) < 1.0

            println("\n--- XCorridorWithFlowRegularizerModel decomposition ---")
            println(d)
        end
    end

    @testset "Integration: TwoStageSingleDetourModel round-trip" begin
        model = TwoStageSingleDetourModel(3, 4, 1.0, 120.0, 60.0)
        result = run_opt(model, data; optimizer_env=env, silent=true)
        @test result.termination_status == JuMP.MOI.OPTIMAL

        reported = result.objective_value

        mktempdir() do run_dir
            export_variables(result, run_dir)

            metrics = Dict(
                "method" => "TwoStageSingleDetourModel",
                "model"  => Dict(
                    "in_vehicle_time_weight" => model.in_vehicle_time_weight,
                    "vehicle_routing_weight" => model.vehicle_routing_weight
                ),
                "solve"  => Dict("objective_value" => reported)
            )
            open(joinpath(run_dir, "metrics.json"), "w") do f
                JSON.print(f, metrics)
            end

            d = decompose_objective(run_dir, data)

            @test d.model_type == "TwoStageSingleDetourModel"
            @test d.vehicle_routing_weight ≈ model.vehicle_routing_weight
            @test d.vehicle_flow_cost_raw > 0.0  # vehicle must move
            @test d.corridor_penalty ≈ 0.0
            @test d.flow_regularization_penalty ≈ 0.0
            @test d.reported_objective ≈ reported

            println("\n--- TwoStageSingleDetourModel decomposition ---")
            println(d)
        end
    end

    end  # gurobi_available
end
