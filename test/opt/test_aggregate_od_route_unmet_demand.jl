@testset "AggregateODRouteModel unmet_demand_penalty (\"always feasible\" mode)" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping unmet_demand_penalty tests"
        @test true
        return
    end

    # One request, o=1, d=5. Pickup candidates {1,2} disjoint from dropoff
    # candidates {4,5}. With l=1, no single station can serve both sides --
    # a genuine station-budget infeasibility (not a collision): tests the
    # endpoint-chain relaxation (sum(z)<=1, dropped sum(y)>=1 row) directly.
    function budget_infeasible_fixture()
        stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
        requests = DataFrame(
            id=[1],
            start_station_id=[1],
            end_station_id=[5],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            walking_costs[(i, j)] = 100.0
        end
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 2)] = 3.0
        walking_costs[(4, 5)] = 3.0
        walking_costs[(5, 5)] = 0.0
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    end

    # Same-station collision: station 1 is the cheapest candidate on BOTH
    # sides of the request (o=4,d=5); stations 2/3 are each the sole
    # alternate on their own side. With l=1 (only station 1 can be open),
    # both sides resolve to station 1 -- tests the same-station pair +
    # coverage-constraint fix independently from the budget-relaxation fix.
    function collision_fixture()
        stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
        requests = DataFrame(
            id=[1],
            start_station_id=[4],
            end_station_id=[5],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            walking_costs[(i, j)] = 100.0
        end
        walking_costs[(4, 1)] = 0.0
        walking_costs[(4, 2)] = 5.0
        walking_costs[(1, 5)] = 0.0
        walking_costs[(3, 5)] = 5.0
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    end

    function fixture_model(style::Symbol, l::Int)
        return AggregateODRouteModel(
            l;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
            unmet_demand_penalty=100_000.0,
        )
    end

    @testset "$name / style=$style" for (name, data, expected_obj) in [
            ("budget", budget_infeasible_fixture(), 100_000.0),
            ("collision", collision_fixture(), 0.0),
        ],
        style in (:big_m_nearest, :endpoint_chain)

        model = fixture_model(style, 1)

        ground_truth = run_opt(
            data, model,
            DirectSolver(
                optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
            ),
        )
        @test ground_truth.termination_status == MOI.OPTIMAL
        @test isapprox(ground_truth.objective_value, expected_obj; atol=1e-6)
        assert_service_near_binary(ground_truth.model)

        inner_cg = ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        )

        benders_y = run_opt(
            data, model,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=BendersY(), inner_solver=inner_cg, max_iterations=50,
                reprice_subproblem=true,
            ),
        )
        @test benders_y.termination_status == MOI.OPTIMAL
        @test isapprox(benders_y.objective_value, ground_truth.objective_value; atol=1e-6)
        @test get(benders_y.metadata, "feasibility_cuts_added", nothing) == 0

        benders_xy = run_opt(
            data, model,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=BendersXY(), inner_solver=inner_cg, max_iterations=50,
            ),
        )
        @test benders_xy.termination_status == MOI.OPTIMAL
        @test isapprox(benders_xy.objective_value, ground_truth.objective_value; atol=1e-6)

        benders_yz = run_opt(
            data, model,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=BendersYZ(), inner_solver=inner_cg, max_iterations=50,
                reprice_subproblem=true,
            ),
        )
        @test benders_yz.termination_status == MOI.OPTIMAL
        @test isapprox(benders_yz.objective_value, ground_truth.objective_value; atol=1e-6)
        @test get(benders_yz.metadata, "feasibility_cuts_added", nothing) == 0

        benders_yzh = run_opt(
            data, model,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=BendersYZH(), inner_solver=inner_cg, max_iterations=50,
            ),
        )
        @test benders_yzh.termination_status == MOI.OPTIMAL
        @test isapprox(benders_yzh.objective_value, ground_truth.objective_value; atol=1e-6)
    end

    @testset "byte-identical when unmet_demand_penalty is unset (control)" begin
        data = budget_infeasible_fixture()
        model = AggregateODRouteModel(
            1;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
        result = run_opt(
            data, model,
            DirectSolver(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        )
        @test result.termination_status != MOI.OPTIMAL
    end

    @testset "cut_derivation=:restricted_mw_fixed_pi rejects unmet_demand_penalty" begin
        data = budget_infeasible_fixture()
        model = fixture_model(:big_m_nearest, 1)
        inner_cg = ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        )
        @test_throws ArgumentError run_opt(
            data, model,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=BendersY(), inner_solver=inner_cg, max_iterations=50,
                reprice_subproblem=true, cut_derivation=:restricted_mw_fixed_pi,
            ),
        )
    end

    @testset "public subproblem API: adversarial y_hat (nothing open) never throws" begin
        data = budget_infeasible_fixture()
        model = fixture_model(:big_m_nearest, 1)
        y_hat_empty = zeros(5)

        res_y = solve_benders_y_subproblem(data, model, y_hat_empty; optimizer_env=Gurobi.Env(), silent=true)
        @test res_y.termination_status == MOI.OPTIMAL
        @test isapprox(res_y.objective_value, 100_000.0; atol=1e-6)
        @test all(v -> isapprox(v, 0.0; atol=1e-6), values(res_y.service))

        res_yz = solve_benders_yz_subproblem(data, model, y_hat_empty; optimizer_env=Gurobi.Env(), silent=true)
        @test res_yz.termination_status == MOI.OPTIMAL
        @test isapprox(res_yz.objective_value, 100_000.0; atol=1e-6)
        @test all(v -> isapprox(v, 0.0; atol=1e-6), values(res_yz.service))

        res_yzh = solve_benders_yzh_master(data, model; optimizer_env=Gurobi.Env(), silent=true)
        @test res_yzh.termination_status == MOI.OPTIMAL
        @test isapprox(res_yzh.objective_value, 100_000.0; atol=1e-6)
        @test all(v -> isapprox(v, 0.0; atol=1e-6), values(res_yzh.service))
    end
end
