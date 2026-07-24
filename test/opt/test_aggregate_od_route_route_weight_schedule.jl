@testset "AggregateODRouteModel route_regularization_weight_schedule" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping route_regularization_weight_schedule tests"
        @test true
        return
    end

    # Same hand-designed synthetic 5-station fixture as
    # test_aggregate_od_route_lifted_walking_objective.jl: l=4 of 5 stations, request A (o=1,d=5)
    # has two genuine candidates on each side, request B (o=2,d=4) pins stations 2 and 4 open
    # unconditionally, station 3 is a pure decoy.
    function schedule_fixture()
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

    beta_final = 3.0
    function schedule_model()
        return AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0,
            route_regularization_weight=beta_final,
            walk_cost_weight=0.37,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
    end

    data = schedule_fixture()
    model = schedule_model()
    schedule = [beta_final / 10, beta_final / 2, beta_final]

    function make_solver(decomposition, cut_derivation; schedule_arg=nothing)
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
            route_regularization_weight_schedule=schedule_arg,
        )
    end

    @testset "Test: schedule reaches the same optimum as a direct (unscheduled) lifted solve" begin
        for decomposition in (BendersY(), BendersYZ())
            for cut_derivation in (:standard, :zero_completion)
                @testset "$(typeof(decomposition)), cut_derivation=$cut_derivation" begin
                    direct = run_opt(data, model, make_solver(decomposition, cut_derivation))
                    ramped = run_opt(data, model, make_solver(decomposition, cut_derivation; schedule_arg=schedule))

                    @test direct.termination_status == MOI.OPTIMAL
                    @test ramped.termination_status == MOI.OPTIMAL
                    @test isapprox(ramped.objective_value, direct.objective_value; atol=1e-6)

                    @test ramped.metadata["route_regularization_weight_schedule"] == schedule
                    stage_log = ramped.metadata["route_regularization_weight_stage_log"]
                    @test length(stage_log) == length(schedule) - 1
                    @test [row.route_regularization_weight for row in stage_log] == schedule[2:end]
                    @test issorted([row.iterations_to_reach for row in stage_log])

                    # The direct (unscheduled) solve is a single implicit stage: no transitions.
                    @test isempty(direct.metadata["route_regularization_weight_stage_log"])
                    @test direct.metadata["route_regularization_weight_schedule"] == [beta_final]
                end
            end
        end
    end

    @testset "unsupported configurations throw" begin
        @test_throws ArgumentError BendersSolver(
            decomposition=BendersY(), lifted_walking_objective=false,
            route_regularization_weight_schedule=schedule,
        )
        @test_throws ArgumentError BendersSolver(
            decomposition=BendersY(), lifted_walking_objective=true,
            route_regularization_weight_schedule=Float64[],
        )
        @test_throws ArgumentError BendersSolver(
            decomposition=BendersY(), lifted_walking_objective=true,
            route_regularization_weight_schedule=[2.0, 1.0],
        )
        @test_throws ArgumentError BendersSolver(
            decomposition=BendersY(), lifted_walking_objective=true,
            route_regularization_weight_schedule=[0.0, 1.0],
        )
        # Schedule not ending at model.route_regularization_weight -- only checkable at run_opt
        # time (the solver doesn't know the model yet at construction).
        @test_throws ArgumentError run_opt(
            data, model,
            make_solver(BendersY(), :standard; schedule_arg=[0.1, 1.0]),
        )
    end
end
