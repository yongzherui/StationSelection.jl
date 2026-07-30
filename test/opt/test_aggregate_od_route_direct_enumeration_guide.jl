@testset "AggregateODRouteModel direct_enumeration_guide" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping direct_enumeration_guide tests"
        @test true
        return
    end

    # Same hand-designed synthetic 5-station fixture as
    # test_aggregate_od_route_route_weight_schedule.jl: l=4 of 5 stations, request A (o=1,d=5)
    # has two genuine candidates on each side, request B (o=2,d=4) pins stations 2 and 4 open
    # unconditionally, station 3 is a pure decoy.
    function guide_fixture()
        stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 2],
            end_station_id=[5, 4],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            walking_costs[(i, j)] = 100.0
        end
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 2)] = 3.0
        walking_costs[(4, 5)] = 3.0
        walking_costs[(5, 5)] = 0.0
        walking_costs[(2, 2)] = 0.0
        walking_costs[(4, 4)] = 0.0
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    end

    function guide_model()
        return AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0,
            route_regularization_weight=3.0,
            walk_cost_weight=0.37,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
    end

    data = guide_fixture()
    model = guide_model()

    function make_solver(decomposition, cut_derivation; direct_enumeration_guide=false)
        BendersSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            decomposition=decomposition,
            inner_solver=ColumnGenerationSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
                final_ip_time_limit_sec=30.0,
            ),
            max_iterations=200,
            reprice_subproblem=true,
            cut_derivation=cut_derivation,
            lifted_walking_objective=true,
            direct_enumeration_guide=direct_enumeration_guide,
            direct_enumeration_max_routes=1000,
            direct_enumeration_time_limit_sec=5.0,
        )
    end

    @testset "guided two-phase solve matches a plain lifted solve" begin
        for decomposition in (BendersY(), BendersYZ())
            for cut_derivation in (:standard, :zero_completion)
                @testset "$(typeof(decomposition)), cut_derivation=$cut_derivation" begin
                    baseline = run_opt(data, model, make_solver(decomposition, cut_derivation))
                    guided = run_opt(data, model, make_solver(decomposition, cut_derivation; direct_enumeration_guide=true))

                    @test baseline.termination_status == MOI.OPTIMAL
                    @test guided.termination_status == MOI.OPTIMAL
                    @test isapprox(guided.objective_value, baseline.objective_value; atol=1e-6)

                    # phase 1's own (heuristic-guide, double-counted) objective should still
                    # certify the same final answer once cuts are seeded into phase 2.
                    @test isapprox(guided.metadata["phase1_objective"], guided.objective_value; atol=1e-6)
                    @test guided.metadata["phase1_cuts_harvested"] > 0
                    @test guided.metadata["enumerated_routes"] > 0
                    # Seeded with phase 1's cuts, phase 2 should need very few extra iterations.
                    @test guided.metadata["phase2_iterations"] <= 3
                end
            end
        end
    end

    @testset "unsupported configurations throw" begin
        @test_throws ArgumentError BendersSolver(
            decomposition=BendersXY(), lifted_walking_objective=true,
            direct_enumeration_guide=true,
        )
        @test_throws ArgumentError BendersSolver(
            decomposition=BendersY(), lifted_walking_objective=false,
            direct_enumeration_guide=true,
        )
        @test_throws ArgumentError BendersSolver(
            decomposition=BendersY(), lifted_walking_objective=true,
            direct_enumeration_guide=true, direct_enumeration_max_routes=0,
        )
        @test_throws ArgumentError BendersSolver(
            decomposition=BendersY(), lifted_walking_objective=true,
            direct_enumeration_guide=true, direct_enumeration_time_limit_sec=0.0,
        )
    end
end
