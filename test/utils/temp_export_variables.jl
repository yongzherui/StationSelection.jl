@testset "Export Variables (Edge Case)" begin
    using JuMP
    using JSON
    using CSV

    # Check if Gurobi is available
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end

    if !gurobi_available
        @warn "Gurobi not available, skipping export variables test"
        @test true
        return
    end

    # Create test data - 4 stations in a line
    stations = DataFrame(
        id = [1, 2, 3, 4],
        lon = [113.0, 113.1, 113.2, 113.3],
        lat = [28.0, 28.0, 28.0, 28.0]
    )

    # 3 requests in the same time window (matches top test case)
    requests = DataFrame(
        order_id = [1, 2, 3],
        start_station_id = [1, 1, 2],
        end_station_id = [3, 4, 4],
        request_time = [
            DateTime(2024, 1, 1, 8, 0, 0),
            DateTime(2024, 1, 1, 8, 0, 30),
            DateTime(2024, 1, 1, 8, 1, 0)
        ]
    )

    # Walking cost: |i - j| * 200 (high cost to penalize walking)
    # Routing cost: |i - j| * 10 (low cost - vehicle travel is efficient)
    walking_costs = Dict{Tuple{Int,Int}, Float64}()
    routing_costs = Dict{Tuple{Int,Int}, Float64}()
    for i in 1:4, j in 1:4
        walking_costs[(i, j)] = abs(i - j) * 200.0
        routing_costs[(i, j)] = abs(i - j) * 10.0
    end

    scenarios = [("2024-01-01 08:00:00", "2024-01-01 09:00:00")]

    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )

    env = Gurobi.Env()
    model = TwoStageSingleDetourModel(3, 4, 1.0, 120.0, 60.0)
    result = run_opt(model, data; optimizer_env=env, silent=true)

    if result.termination_status == MOI.OPTIMAL
        mktempdir() do export_root
            export_variables(result, export_root)
            export_dir = joinpath(export_root, "variable_exports")

            @test isdir(export_dir)
            @test isfile(joinpath(export_dir, "station_id_mapping.csv"))
            @test isfile(joinpath(export_dir, "scenario_info.csv"))
            @test isfile(joinpath(export_dir, "station_selection.csv"))
            @test isfile(joinpath(export_dir, "scenario_activation.csv"))
            @test isfile(joinpath(export_dir, "assignment_variables.csv"))
            @test isfile(joinpath(export_dir, "flow_variables.csv"))
            @test isfile(joinpath(export_dir, "same_source_pooling.csv"))
            @test isfile(joinpath(export_dir, "same_dest_pooling.csv"))
            @test isfile(joinpath(export_dir, "variable_export_metadata.json"))

            metadata = JSON.parsefile(joinpath(export_dir, "variable_export_metadata.json"))
            @test metadata["model_type"] == "TwoStageSingleDetourModel"

            # station_id_mapping.csv
            station_id_mapping = CSV.read(joinpath(export_dir, "station_id_mapping.csv"), DataFrame)
            expected_station_id_mapping = DataFrame(
                array_idx = [1, 2, 3, 4],
                station_id = [1, 2, 3, 4]
            )
            @test station_id_mapping == expected_station_id_mapping

            # scenario_info.csv
            scenario_info = CSV.read(joinpath(export_dir, "scenario_info.csv"), DataFrame)
            scenario_info.start_time = string.(scenario_info.start_time)
            scenario_info.end_time = string.(scenario_info.end_time)
            expected_scenario_info = DataFrame(
                scenario_idx = [1],
                label = ["2024-01-01 08:00:00_2024-01-01 09:00:00"],
                start_time = ["2024-01-01T08:00:00"],
                end_time = ["2024-01-01T09:00:00"]
            )
            @test scenario_info == expected_scenario_info

            # station_selection.csv
            station_selection = CSV.read(joinpath(export_dir, "station_selection.csv"), DataFrame)
            expected_station_selection = DataFrame(
                array_idx = [1, 2, 3, 4],
                station_id = [1, 2, 3, 4],
                selected = [1, 1, 1, 1],
                value = [1.0, 1.0, 1.0, 1.0]
            )
            @test station_selection == expected_station_selection

            # scenario_activation.csv
            scenario_activation = CSV.read(joinpath(export_dir, "scenario_activation.csv"), DataFrame)
            expected_scenario_activation = DataFrame(
                station_idx = [1, 2, 4],
                station_id = [1, 2, 4],
                scenario_idx = [1, 1, 1],
                scenario_label = [
                    "2024-01-01 08:00:00_2024-01-01 09:00:00",
                    "2024-01-01 08:00:00_2024-01-01 09:00:00",
                    "2024-01-01 08:00:00_2024-01-01 09:00:00"
                ],
                value = [1.0, 1.0, 1.0]
            )
            @test scenario_activation == expected_scenario_activation

            # assignment_variables.csv
            assignment_variables = CSV.read(joinpath(export_dir, "assignment_variables.csv"), DataFrame)
            expected_assignment_variables = DataFrame(
                scenario = [1, 1, 1],
                time_id = [0, 0, 0],
                origin_id = [2, 1, 1],
                dest_id = [4, 3, 4],
                pickup_idx = [2, 1, 1],
                dropoff_idx = [4, 2, 4],
                pickup_id = [2, 1, 1],
                dropoff_id = [4, 2, 4],
                value = [1.0, 1.0, 1.0]
            )
            @test assignment_variables == expected_assignment_variables

            # flow_variables.csv
            flow_variables = CSV.read(joinpath(export_dir, "flow_variables.csv"), DataFrame)
            expected_flow_variables = DataFrame(
                scenario = [1, 1, 1],
                time_id = [0, 0, 0],
                j_array = [1, 1, 2],
                k_array = [2, 4, 4],
                j_id = [1, 1, 2],
                k_id = [2, 4, 4],
                value = [1.0, 1.0, 1.0]
            )
            @test flow_variables == expected_flow_variables

            # same_source_pooling.csv
            same_source_pooling = CSV.read(joinpath(export_dir, "same_source_pooling.csv"), DataFrame)
            expected_same_source_pooling = DataFrame(
                scenario = [1],
                time_id = [0],
                xi_idx = [2],
                j_id = [1],
                k_id = [2],
                l_id = [4],
                value = [1.0]
            )
            @test same_source_pooling == expected_same_source_pooling

            # same_dest_pooling.csv
            same_dest_pooling = CSV.read(joinpath(export_dir, "same_dest_pooling.csv"), DataFrame)
            expected_same_dest_pooling = DataFrame(
                scenario = [1],
                time_id = [0],
                xi_idx = [2],
                j_id = [1],
                k_id = [2],
                l_id = [4],
                time_delta = [0],
                value = [1.0]
            )
            @test same_dest_pooling == expected_same_dest_pooling

            @test metadata["n_assignments"] == 3
            @test metadata["n_activated_flows"] == 3
            @test metadata["n_stations_selected"] == 4
            @test metadata["n_scenario_activations"] == 3
            @test metadata["n_activated_same_dest"] == 1
            @test metadata["time_window_sec"] == 120
            @test metadata["n_activated_same_source"] == 1
        end
    end
end
