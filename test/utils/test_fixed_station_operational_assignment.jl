using Test
using DataFrames
using Dates
using CSV
using JuMP
const MOI = JuMP.MOI

@testset "Fixed Station Operational Assignment" begin
    @testset "load_fixed_station_decisions uses y fallback for missing z" begin
        mktempdir() do tmpdir
            export_dir = joinpath(tmpdir, "selection_run", "variable_exports")
            mkpath(export_dir)

            CSV.write(joinpath(export_dir, "station_selection.csv"), DataFrame(
                station_id = [1, 2, 3],
                selected = [1, 1, 0],
                value = [1.0, 1.0, 0.0],
            ))
            CSV.write(joinpath(export_dir, "scenario_info.csv"), DataFrame(
                scenario_idx = [1, 2],
                label = ["morning", "evening"],
                start_time = ["2024-01-01T08:00:00", "2024-01-01T17:00:00"],
                end_time = ["2024-01-01T09:00:00", "2024-01-01T18:00:00"],
            ))
            CSV.write(joinpath(export_dir, "scenario_activation.csv"), DataFrame(
                station_id = [2],
                scenario_idx = [1],
                scenario_label = ["morning"],
                value = [1.0],
            ))

            fixed = StationSelection.load_fixed_station_decisions(
                dirname(export_dir);
                scenario_indices=[1, 2]
            )

            @test fixed.built_station_ids == Set([1, 2])
            @test fixed.active_station_ids_by_scenario[1] == Set([2])
            @test fixed.active_station_ids_by_scenario[2] == Set([1, 2])
            @test fixed.y_fallback_scenarios == [2]
            @test fixed.z_available
        end
    end

    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end

    @testset "solve_fixed_station_operational_assignment exports route-model assignments" begin
        if !gurobi_available
            @warn "Gurobi not available, skipping fixed-station operational solve test"
            @test true
        else
            stations = DataFrame(
                id = [1, 2, 3],
                lon = [0.0, 1.0, 2.0],
                lat = [0.0, 0.0, 0.0],
            )
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

            walking_costs = Dict{Tuple{Int, Int}, Float64}()
            routing_costs = Dict{Tuple{Int, Int}, Float64}()
            for i in 1:3, j in 1:3
                walking_costs[(i, j)] = abs(i - j) * 60.0
                routing_costs[(i, j)] = abs(i - j) * 30.0
            end

            data = StationSelection.create_station_selection_data(
                stations,
                requests,
                walking_costs;
                routing_costs=routing_costs,
                scenarios=[("2024-01-01 08:00:00", "2024-01-01 09:00:00")],
            )

            fixed = StationSelection.FixedStationDecisions(
                Set([1, 3]),
                Dict(1 => Set([1, 3])),
                Dict(1 => "morning"),
                "synthetic",
                false,
                [1],
            )

            model = StationSelection.RouteVehicleCapacityModel(
                2,
                2;
                route_regularization_weight=0.0,
                vehicle_capacity=4,
                max_walking_distance=1000.0,
                max_detour_time=3600.0,
                max_detour_ratio=10.0,
                time_window_sec=3600,
                max_stations_visited=3,
            )

            mktempdir() do tmpdir
                result = StationSelection.solve_fixed_station_operational_assignment(
                    model,
                    data,
                    fixed,
                    tmpdir;
                    silent=true,
                )

                @test result.result.termination_status == MOI.OPTIMAL
                @test isfile(joinpath(tmpdir, "variable_exports", "assignment_variables.csv"))
                @test isfile(joinpath(tmpdir, "variable_exports", "station_selection.csv"))
                @test isfile(joinpath(tmpdir, "variable_exports", "scenario_activation.csv"))

                assign_df = CSV.read(joinpath(tmpdir, "variable_exports", "assignment_variables.csv"), DataFrame)
                @test nrow(assign_df) > 0
                @test all(assign_df.pickup_id .∈ Ref([1, 3]))
                @test all(assign_df.dropoff_id .∈ Ref([1, 3]))
            end
        end
    end
end
