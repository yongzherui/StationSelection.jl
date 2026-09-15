@testset "Fielbaum (2021) toy grid generator" begin
    @test isdefined(StationSelection, :FielbaumGridInstance)

    instance = generate_fielbaum_grid_instance()

    @testset "Fig. 4 layout" begin
        @test instance.ny == 10 && instance.nx == 10
        @test length(instance.stations) == 100

        # Fast streets are rows 2/6/9 and columns 3/6/9; slow are rows 3/5/7 and
        # columns 1/8/10; everything else is a mid-speed one-way.
        @test [r for r in 1:10 if instance.row_streets[r].class === :fast] == [2, 6, 9]
        @test [r for r in 1:10 if instance.row_streets[r].class === :slow] == [3, 5, 7]
        @test [c for c in 1:10 if instance.col_streets[c].class === :fast] == [3, 6, 9]
        @test [c for c in 1:10 if instance.col_streets[c].class === :slow] == [1, 8, 10]

        # "Mid-speed streets are unidirectional, and the others are bidirectional."
        for street in vcat(instance.row_streets, instance.col_streets)
            @test (street.class === :mid) == (street.direction != 0)
        end
        @test [r for r in 1:10 if instance.row_streets[r].direction == +1] == [1, 8]
        @test [r for r in 1:10 if instance.row_streets[r].direction == -1] == [4, 10]
        @test [c for c in 1:10 if instance.col_streets[c].direction == +1] == [4, 7]
        @test [c for c in 1:10 if instance.col_streets[c].direction == -1] == [2, 5]

        # A layout that breaks the paper's class/direction pairing is rejected.
        @test_throws ArgumentError fielbaum_validate_streets(
            [FielbaumStreet(:mid, 0)], "row",
        )
        @test_throws ArgumentError fielbaum_validate_streets(
            [FielbaumStreet(:fast, 1)], "row",
        )
    end

    @testset "arc lengths and speeds" begin
        # "All the arcs are assumed to have the same length (0.15 km)".
        @test all(≈(FIELBAUM_LINK_LENGTH_M), values(instance.arc_length))

        # 6 bidirectional streets per axis x 9 segments x 2 directions, plus
        # 4 one-way streets per axis x 9 segments.
        @test length(instance.arc_time) == 2 * (6 * 9 * 2 + 4 * 9)

        for ((from, to), minutes) in instance.arc_time
            origin, target = instance.stations[from], instance.stations[to]
            class = origin.row == target.row ?
                instance.row_streets[origin.row].class :
                instance.col_streets[origin.col].class
            @test minutes ≈ 60 * (FIELBAUM_LINK_LENGTH_M / 1000) / fielbaum_speed_kmh(class)
        end
        @test sort(unique(round.(collect(values(instance.arc_time)); digits = 6))) ==
            round.([0.15 * 60 / 40, 0.15 * 60 / 30, 0.15 * 60 / 20]; digits = 6)
    end

    @testset "one-way streets make drive times asymmetric" begin
        @test all(isfinite, instance.drive_time)

        # Row 1 is one-way eastbound, so the westbound leg has to detour.
        east = instance.drive_time[grid_station_id(1, 4, 10), grid_station_id(1, 5, 10)]
        west = instance.drive_time[grid_station_id(1, 5, 10), grid_station_id(1, 4, 10)]
        @test east ≈ 0.15 * 60 / 30
        @test west > east

        # Walking ignores the one-way rules, so it stays symmetric.
        @test instance.walk_time ≈ permutedims(instance.walk_time)
        @test instance.walk_time[1, 2] ≈ 0.15 * 60 / FIELBAUM_WALKING_SPEED_KMH
        @test maximum(instance.walk_time) ≈ 18 * 0.15 * 60 / FIELBAUM_WALKING_SPEED_KMH
    end

    @testset "fast streets are worth a detour" begin
        # Row 3 is slow (20 km/h) and row 2 is fast (40 km/h), reachable over the
        # slow column 1 / column 10. Crossing the grid along row 3 must route via
        # row 2 instead of staying put -- this is the effect the paper's grid is
        # built to create, and the uniform-cost grid in generators/grid.jl cannot
        # express it.
        along_row_3 = 9 * 0.15 * 60 / 20
        via_row_2 = 2 * (0.15 * 60 / 20) + 9 * 0.15 * 60 / 40
        crossing = instance.drive_time[grid_station_id(3, 1, 10), grid_station_id(3, 10, 10)]
        @test crossing ≈ via_row_2
        @test crossing < along_row_3
    end

    @testset "network and demand variants" begin
        spacings = fielbaum_node_spacings(10, :non_uniform)
        @test length(spacings) == 9
        @test minimum(spacings) ≈ FIELBAUM_NON_UNIFORM_CENTER_SPACING_M
        @test maximum(spacings) ≈ FIELBAUM_NON_UNIFORM_EDGE_SPACING_M
        @test spacings ≈ reverse(spacings)
        @test fielbaum_node_spacings(10, :uniform) == fill(FIELBAUM_LINK_LENGTH_M, 9)
        @test_throws ArgumentError fielbaum_node_spacings(10, :quadratic)

        non_uniform = generate_fielbaum_grid_instance(layout = :non_uniform)
        @test sort(unique(round.(collect(values(non_uniform.arc_length)); digits = 4))) ==
            sort(unique(round.(spacings; digits = 4)))

        # Concentrated demand pulls endpoints towards the centre.
        centre = (1 + 10) / 2
        mean_offset(inst) = sum(
            abs(inst.stations[o].row - centre) + abs(inst.stations[o].col - centre) +
            abs(inst.stations[d].row - centre) + abs(inst.stations[d].col - centre)
            for (o, d) in inst.active_pairs
        ) / length(inst.active_pairs)
        uniform_demand = generate_fielbaum_grid_instance(demand = :uniform, seed = 11)
        concentrated = generate_fielbaum_grid_instance(demand = :concentrated, seed = 11)
        @test mean_offset(concentrated) < mean_offset(uniform_demand)
        @test all(o != d for (o, d) in concentrated.active_pairs)
    end

    @testset "demand stream" begin
        # "two requests arriving every 15 s" over one hour.
        @test length(instance.active_pairs) == 480
        @test length(instance.request_times) == 480
        @test instance.request_times[1] == instance.request_times[2]
        @test instance.request_times[3] - instance.request_times[1] == Second(15)
        @test last(instance.request_times) - first(instance.request_times) == Second(3585)
        @test all(in((1, 2, 3)), instance.passengers)
        @test count(==(1), instance.passengers) / 480 > 0.7  # 0.8 in the paper

        @test generate_fielbaum_grid_instance(seed = 3).active_pairs ==
            generate_fielbaum_grid_instance(seed = 3).active_pairs
        @test generate_fielbaum_grid_instance(seed = 3).active_pairs !=
            generate_fielbaum_grid_instance(seed = 4).active_pairs
    end

    @testset "StationSelectionData conversion" begin
        data = create_fielbaum_grid_problem_data(instance)
        @test data.n_stations == 100
        @test nrow(data.scenarios[1].requests) == 480
        @test !isnothing(data.routing_costs)

        # Costs are minutes, and the routing matrix keeps its direction.
        @test get_routing_cost(data, grid_station_id(1, 4, 10), grid_station_id(1, 5, 10)) <
            get_routing_cost(data, grid_station_id(1, 5, 10), grid_station_id(1, 4, 10))

        # Pairs beyond the walking-time limit read back as Inf.
        @test get_walking_cost(data, 1, 1) == 0.0
        @test isinf(get_walking_cost(data, grid_station_id(1, 1, 10), grid_station_id(10, 10, 10)))
        tight = create_fielbaum_grid_problem_data(instance; max_walking_time = 2.0)
        @test count(isfinite, tight.walking_costs) < count(isfinite, data.walking_costs)
    end

    @test isnothing(print_fielbaum_grid_summary(instance))
end
