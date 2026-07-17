using CSV

gurobi_available = try
    using Gurobi
    true
catch
    false
end

@testset "export_variables" begin
    if !gurobi_available
        @warn "Gurobi not available, skipping export_variables tests"
        @test true
        return
    end

    @testset "AggregateODRouteMap" begin
        stations = DataFrame(id=[1, 2, 3], lon=[0.0, 1.0, 2.0], lat=[0.0, 0.0, 0.0])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 1],
            end_station_id=[3, 3],
            request_time=[DateTime(2024, 1, 1, 8, 0), DateTime(2024, 1, 1, 8, 5)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:3, j in 1:3
            walking_costs[(i, j)] = i == j ? 0.0 : 100.0
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        data = create_station_selection_data(
            stations, requests, walking_costs;
            routing_costs=routing_costs,
            scenarios=[
                ("2024-01-01 07:59:00", "2024-01-01 08:02:00"),
                ("2024-01-01 08:03:00", "2024-01-01 08:10:00"),
            ],
        )
        model = AggregateODRouteModel(
            2;
            max_walking_distance=1.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
        )

        result = run_opt(data, model, DirectSolver(optimizer_env=Gurobi.Env(), silent=true))
        @test result.termination_status == MOI.OPTIMAL

        mktempdir() do tmpdir
            StationSelection.export_variables(result, tmpdir)
            export_dir = joinpath(tmpdir, "variable_exports")

            assign_df = CSV.read(joinpath(export_dir, "assignment_variables.csv"), DataFrame)
            @test :demand in propertynames(assign_df)
            @test !isempty(assign_df)
            @test all(assign_df.value .== 1)

            columns_df = CSV.read(joinpath(export_dir, "route_columns.csv"), DataFrame)
            @test nrow(columns_df) == length(result.mapping.columns)

            activations_df = CSV.read(joinpath(export_dir, "route_activations.csv"), DataFrame)
            @test issubset(Set(activations_df.column_id), Set(columns_df.column_id))
        end
    end

    @testset "ClusteringTwoStageODMap regression" begin
        stations = DataFrame(id=[1, 2, 3], lon=[0.0, 1.0, 2.0], lat=[0.0, 0.0, 0.0])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 1],
            end_station_id=[3, 3],
            request_time=[DateTime(2024, 1, 1, 8, 0), DateTime(2024, 1, 1, 8, 5)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:3, j in 1:3
            walking_costs[(i, j)] = i == j ? 0.0 : 100.0
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        data = create_station_selection_data(
            stations, requests, walking_costs; routing_costs=routing_costs,
        )
        model = ClusteringModel(TwoStageODPolicy(2, 3))

        result = run_opt(data, model, DirectSolver(optimizer_env=Gurobi.Env(), silent=true))
        @test result.termination_status == MOI.OPTIMAL

        mktempdir() do tmpdir
            StationSelection.export_variables(result, tmpdir)
            export_dir = joinpath(tmpdir, "variable_exports")
            @test isfile(joinpath(export_dir, "assignment_variables.csv"))
            assign_df = CSV.read(joinpath(export_dir, "assignment_variables.csv"), DataFrame)
            @test :origin_id in propertynames(assign_df)
            @test :pickup_id in propertynames(assign_df)
        end
    end
end
