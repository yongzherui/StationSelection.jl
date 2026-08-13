@testset "AggregateODRouteBendersYXFormulation" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping AggregateODRouteBendersYXFormulation tests"
        @test true
        return
    end

    # 4 stations on a line (lon 0..3), l=2: which 2 stations to build is a genuine
    # tradeoff (no single station dominates both requests), and two requests land in two
    # different scenarios (well-separated request_time) so MultiCut(:scenario) actually
    # exercises more than one cut group.
    function benders_yx_fixture()
        stations = DataFrame(id=collect(1:4), lon=Float64.(0:3), lat=zeros(4))
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 2],
            end_station_id=[4, 3],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 14)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:4, j in 1:4
            walking_costs[(i, j)] = abs(i - j) * 10.0
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        return create_station_selection_data(
            stations, requests, walking_costs; routing_costs=routing_costs,
            scenarios=[
                ("2024-01-01 00:00:00", "2024-01-01 10:00:00"),
                ("2024-01-01 10:00:00", "2024-01-01 23:59:59"),
            ],
        )
    end

    data = benders_yx_fixture()
    @test length(data.scenarios) == 2

    problem = StationSelectionProblem(data, 2; max_walking_distance=1000.0)
    common_kwargs = (
        route_regularization_weight=0.5, walk_cost_weight=1.0, repositioning_time=0.0,
        max_wait_time=Inf, detour_factor=1.5, max_stops=3, allow_walk_only=false,
    )

    direct_result = run_opt(
        problem, AggregateODRouteBaseFormulation(; common_kwargs...),
        DirectMIPSolver(config=SolverOptions(silent=true, mip_gap=0.0)),
    )
    @test direct_result.termination_status == MOI.OPTIMAL
    y_direct = round.(Int, JuMP.value.(direct_result.model[:y]))

    @testset "SingleCut matches DirectMIPSolver" begin
        benders_result = run_opt(
            problem,
            AggregateODRouteBendersYXFormulation(; common_kwargs..., cut_mode=SingleCut()),
            BendersSolver(config=SolverOptions(silent=true), optimality_tol=1e-7),
        )
        @test benders_result.termination_status == MOI.OPTIMAL
        @test isapprox(benders_result.objective_value, direct_result.objective_value; atol=1e-5)
        @test benders_result.metadata["benders_iterations"] > 1
        y_benders = round.(Int, JuMP.value.(benders_result.model[:y]))
        @test y_benders == y_direct
    end

    @testset "MultiCut matches DirectMIPSolver" begin
        benders_result = run_opt(
            problem,
            AggregateODRouteBendersYXFormulation(; common_kwargs..., cut_mode=MultiCut()),
            BendersSolver(config=SolverOptions(silent=true), optimality_tol=1e-7),
        )
        @test benders_result.termination_status == MOI.OPTIMAL
        @test isapprox(benders_result.objective_value, direct_result.objective_value; atol=1e-5)
        y_benders = round.(Int, JuMP.value.(benders_result.model[:y]))
        @test y_benders == y_direct
        @test length(benders_result.model[:aggregate_od_route_benders_yx_theta]) == 2
    end
end
