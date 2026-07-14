@testset "Grid and Zhuzhou generators" begin
    @test isdefined(StationSelection, :GridInstance)
    @test isdefined(StationSelection, :ZhuzhouInstance)

    grid = generate_grid_instance(3, 2, 4; endpoint_overlap = 1.0, seed = 7)
    @test length(grid.stations) == 6
    @test length(grid.active_pairs) == 4
    @test all(origin != destination for (origin, destination) in grid.active_pairs)

    grid_data = create_grid_problem_data(grid; max_walking_distance = 1.0)
    @test grid_data.n_stations == 6
    @test nrow(grid_data.scenarios[1].requests) == 4
    @test !isnothing(grid_data.routing_costs)

    mktempdir() do dir
        write(
            joinpath(dir, "station_request_counts.csv"),
            join(
                [
                    "id,name,lon,lat,pickup_count,dropoff_count",
                    "1,A,113.0,27.0,10,8",
                    "2,B,113.001,27.0,9,7",
                    "3,C,113.0,27.001,8,6",
                ],
                "\n",
            ),
        )
        write(
            joinpath(dir, "segment.csv"),
            join(
                [
                    "id,from,to,name,time",
                    "1,1,2,s12,5.0",
                    "2,2,3,s23,6.0",
                    "3,3,1,s31,7.0",
                    "4,2,1,s21,5.0",
                    "5,3,2,s32,6.0",
                    "6,1,3,s13,7.0",
                ],
                "\n",
            ),
        )
        write(
            joinpath(dir, "order.csv"),
            join(
                [
                    "id,timestamp,a,b,origin,destination",
                    "1,2026-01-01 08:00:00,x,x,1,2",
                    "2,2026-01-01 08:05:00,x,x,2,3",
                    "3,2026-01-01 08:10:00,x,x,3,1",
                ],
                "\n",
            ),
        )

        zhuzhou = generate_zhuzhou_instance(
            dir,
            3,
            "2026-01-01 08:00:00",
            "2026-01-01 08:10:00",
        )
        @test length(zhuzhou.stations) == 3
        @test zhuzhou.active_pairs == [(1, 2), (2, 3), (3, 1)]

        zhuzhou_data = create_zhuzhou_problem_data(zhuzhou; max_walking_distance = 0.0)
        @test zhuzhou_data.n_stations == 3
        @test nrow(zhuzhou_data.scenarios[1].requests) == 3
        @test !isnothing(zhuzhou_data.routing_costs)
    end
end
