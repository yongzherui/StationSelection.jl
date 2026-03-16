using StationSelection
using Test
using DataFrames
using Dates
using CSV

@testset "transform_orders_from_assignments" begin
    function write_common_inputs(tmpdir::String)
        order_df = DataFrame(
            order_id = [1, 2],
            pax_num = [1, 1],
            order_time = ["2024-01-01 08:01:59", "2024-01-01 08:02:00"],
            available_pickup_station_list = ["[1]", "[1]"],
            available_pickup_walkingtime_list = ["[0]", "[0]"],
            available_dropoff_station_list = ["[3]", "[3]"],
            available_dropoff_walkingtime_list = ["[0]", "[0]"]
        )
        cluster_df = DataFrame(
            id = [1, 2, 3],
            lon = [0.0, 1.0, 2.0],
            lat = [0.0, 0.0, 0.0],
            selected = [1, 1, 1]
        )
        scenario_df = DataFrame(
            scenario_idx = [1],
            label = ["morning"],
            start_time = ["2024-01-01T08:00:00"],
            end_time = ["2024-01-01T09:00:00"]
        )

        order_file = joinpath(tmpdir, "orders.csv")
        cluster_file = joinpath(tmpdir, "cluster.csv")
        export_dir = joinpath(tmpdir, "selection_run", "variable_exports")
        mkpath(export_dir)

        CSV.write(order_file, order_df)
        CSV.write(cluster_file, cluster_df)
        CSV.write(joinpath(export_dir, "scenario_info.csv"), scenario_df)

        return order_file, cluster_file, dirname(export_dir)
    end

    @testset "ClusteringTwoStageODModel remains scenario-based" begin
        mktempdir() do tmpdir
            order_file, cluster_file, selection_run_dir = write_common_inputs(tmpdir)

            assignments_df = DataFrame(
                scenario = [1],
                od_idx = [1],
                origin_id = [1],
                dest_id = [3],
                pickup_id = [2],
                dropoff_id = [3],
                value = [1.0]
            )
            CSV.write(
                joinpath(selection_run_dir, "variable_exports", "assignment_variables.csv"),
                assignments_df
            )

            transformed_df, stats = transform_orders_from_assignments(
                order_file,
                selection_run_dir,
                cluster_file,
                "ClusteringTwoStageODModel"
            )

            @test all(transformed_df.assigned_pickup_id .== 2)
            @test all(transformed_df.assigned_dropoff_id .== 3)
            @test stats["n_x_assigned"] == 2
            @test stats["n_fallback"] == 0
        end
    end

    @testset "TwoStageRouteModel matches exact time buckets" begin
        mktempdir() do tmpdir
            order_file, cluster_file, selection_run_dir = write_common_inputs(tmpdir)

            assignments_df = DataFrame(
                scenario = [1, 1],
                time_id = [0, 1],
                od_idx = [1, 1],
                origin_id = [1, 1],
                dest_id = [3, 3],
                pickup_id = [2, 3],
                dropoff_id = [3, 2],
                value = [1.0, 1.0]
            )
            CSV.write(
                joinpath(selection_run_dir, "variable_exports", "assignment_variables.csv"),
                assignments_df
            )

            transformed_df, stats = transform_orders_from_assignments(
                order_file,
                selection_run_dir,
                cluster_file,
                "TwoStageRouteModel";
                time_window_sec = 120
            )

            @test transformed_df.assigned_pickup_id == [2, 3]
            @test transformed_df.assigned_dropoff_id == [3, 2]
            @test stats["n_x_assigned"] == 2
            @test stats["n_fallback"] == 0
        end
    end

    @testset "TwoStageRouteModel requires time_id export" begin
        mktempdir() do tmpdir
            order_file, cluster_file, selection_run_dir = write_common_inputs(tmpdir)

            assignments_df = DataFrame(
                scenario = [1],
                od_idx = [1],
                origin_id = [1],
                dest_id = [3],
                pickup_id = [2],
                dropoff_id = [3],
                value = [1.0]
            )
            CSV.write(
                joinpath(selection_run_dir, "variable_exports", "assignment_variables.csv"),
                assignments_df
            )

            err = try
                transform_orders_from_assignments(
                    order_file,
                    selection_run_dir,
                    cluster_file,
                    "TwoStageRouteModel";
                    time_window_sec = 120
                )
                nothing
            catch e
                e
            end

            @test err isa ErrorException
            @test occursin("missing required column 'time_id'", sprint(showerror, err))
        end
    end

    @testset "TwoStageRouteModel errors on missing exact bucket" begin
        mktempdir() do tmpdir
            order_file, cluster_file, selection_run_dir = write_common_inputs(tmpdir)

            assignments_df = DataFrame(
                scenario = [1],
                time_id = [0],
                od_idx = [1],
                origin_id = [1],
                dest_id = [3],
                pickup_id = [2],
                dropoff_id = [3],
                value = [1.0]
            )
            CSV.write(
                joinpath(selection_run_dir, "variable_exports", "assignment_variables.csv"),
                assignments_df
            )

            err = try
                transform_orders_from_assignments(
                    order_file,
                    selection_run_dir,
                    cluster_file,
                    "TwoStageRouteModel";
                    time_window_sec = 120
                )
                nothing
            catch e
                e
            end

            @test err isa ErrorException
            @test occursin("No exact route assignment found", sprint(showerror, err))
        end
    end
end
