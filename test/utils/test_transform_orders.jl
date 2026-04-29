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
            origin_station_id = [1, 1],
            destination_station_id = [3, 3]
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

    @testset "TwoStageRouteWithTimeModel matches exact time buckets" begin
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
                "TwoStageRouteWithTimeModel";
                time_window_sec = 120
            )

            @test transformed_df.assigned_pickup_id == [2, 3]
            @test transformed_df.assigned_dropoff_id == [3, 2]
            @test stats["n_x_assigned"] == 2
            @test stats["n_fallback"] == 0
        end
    end

    @testset "TwoStageRouteWithTimeModel requires time_id export" begin
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
                    "TwoStageRouteWithTimeModel";
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

    @testset "TwoStageRouteWithTimeModel errors on missing exact bucket" begin
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
                    "TwoStageRouteWithTimeModel";
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

    @testset "month backtest transform writes daily files and period stats" begin
        mktempdir() do tmpdir
            order_df = DataFrame(
                order_id = [1, 2, 3, 4],
                pax_num = [1, 1, 1, 1],
                order_time = [
                    "2024-01-01 08:05:00",
                    "2024-01-01 10:05:00",
                    "2024-01-02 08:05:00",
                    "2024-01-02 10:05:00",
                ],
                origin_station_id = [1, 1, 1, 1],
                destination_station_id = [3, 3, 3, 3],
            )
            cluster_df = DataFrame(
                id = [1, 2, 3],
                lon = [0.0, 1.0, 2.0],
                lat = [0.0, 0.0, 0.0],
                selected = [1, 1, 1],
            )
            scenario_df = DataFrame(
                scenario_idx = [1, 2, 3, 4],
                label = ["d1_morning", "d1_afternoon", "d2_morning", "d2_afternoon"],
                start_time = [
                    "2024-01-01T08:00:00",
                    "2024-01-01T10:00:00",
                    "2024-01-02T08:00:00",
                    "2024-01-02T10:00:00",
                ],
                end_time = [
                    "2024-01-01T08:59:59",
                    "2024-01-01T10:59:59",
                    "2024-01-02T08:59:59",
                    "2024-01-02T10:59:59",
                ],
            )
            assignments_df = DataFrame(
                scenario = [1, 2, 3],
                od_idx = [1, 1, 1],
                origin_id = [1, 1, 1],
                dest_id = [3, 3, 3],
                pickup_id = [2, 3, 1],
                dropoff_id = [3, 2, 2],
                value = [1.0, 1.0, 1.0],
            )

            order_file = joinpath(tmpdir, "orders.csv")
            cluster_file = joinpath(tmpdir, "cluster.csv")
            run_dir = joinpath(tmpdir, "selection_run")
            export_dir = joinpath(run_dir, "variable_exports")
            output_dir = joinpath(tmpdir, "backtest")
            mkpath(export_dir)

            CSV.write(order_file, order_df)
            CSV.write(cluster_file, cluster_df)
            CSV.write(joinpath(export_dir, "scenario_info.csv"), scenario_df)
            CSV.write(joinpath(export_dir, "assignment_variables.csv"), assignments_df)

            transformed_df, stats, manifest_df = transform_orders_for_month_backtest(
                order_file,
                run_dir,
                cluster_file,
                "ClusteringTwoStageODModel";
                output_dir=output_dir,
                start_date=DateTime("2024-01-01 00:00:00", "yyyy-mm-dd HH:MM:SS"),
                end_date=DateTime("2024-01-02 23:59:59", "yyyy-mm-dd HH:MM:SS"),
                scenario_profile=:four_period,
            )

            @test transformed_df.assigned_pickup_id == [2, 3, 1, 1]
            @test transformed_df.assigned_dropoff_id == [3, 2, 2, 3]
            @test stats["n_x_assigned"] == 3
            @test stats["n_fallback"] == 1
            @test stats["n_daily_files"] == 2
            @test stats["x_assigned_by_period"]["1"] == 2
            @test stats["x_assigned_by_period"]["2"] == 1
            @test stats["fallback_by_period"]["2"] == 1
            @test nrow(manifest_df) == 2
            @test all(isfile.(manifest_df.orders_file))
            @test isfile(joinpath(output_dir, "assignment_stats.json"))
            @test isfile(joinpath(output_dir, "orders_transformed_month.csv"))
            @test isfile(joinpath(output_dir, "daily_manifest.csv"))
        end
    end
end
