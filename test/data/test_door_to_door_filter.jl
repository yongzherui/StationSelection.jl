@testset "Door-to-door ratio pair filter" begin
    using DataFrames
    using Dates

    # Four stations on a line, 100 s of walking apart; the vehicle is twice as fast.
    ids = [1, 2, 3, 4]
    pos = Dict(1 => 0.0, 2 => 100.0, 3 => 200.0, 4 => 300.0)
    walking_costs = Dict((i, j) => abs(pos[i] - pos[j]) for i in ids, j in ids)
    routing_costs = Dict((i, j) => 0.5 * abs(pos[i] - pos[j]) for i in ids, j in ids)
    stations = DataFrame(id = ids, lon = zeros(4), lat = zeros(4))
    requests = DataFrame(
        id = [1, 2],
        start_station_id = [1, 2],
        end_station_id = [4, 2],          # the second is o == d
        request_time = [DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 5)],
    )
    data = StationSelection.create_station_selection_data(
        stations, requests, walking_costs; routing_costs = routing_costs)
    R = 150.0
    od = (1, 4)                           # compact indices equal the ids here

    pairs(ratio) = Set(StationSelection.compute_valid_jk_pairs(
        Set([od]), data, R; door_to_door_ratio = ratio)[od])

    # drive(1, 4) = 150. walk + ride + walk: (1,3) 200, (1,4) 150, (2,3) 250, (2,4) 200.
    @test pairs(Inf) == Set([(1, 3), (1, 4), (2, 3), (2, 4)])
    @test pairs(2.0) == pairs(Inf)        # budget 300
    @test pairs(1.5) == Set([(1, 3), (1, 4), (2, 4)])   # budget 225 drops (2, 3)
    @test pairs(1.0) == Set([(1, 4)])     # only the direct pair has zero walking
    @test pairs(Inf) == Set(StationSelection.compute_valid_jk_pairs(Set([od]), data, R)[od])

    @testset "problem validation" begin
        @test StationSelectionProblem(data, 2; max_walking_distance = R).door_to_door_ratio == Inf
        @test_throws ArgumentError StationSelectionProblem(data, 2; max_walking_distance = R,
                                                           door_to_door_ratio = 0.9)
        data_no_routing = StationSelection.create_station_selection_data(
            stations, requests, walking_costs)
        @test_throws ArgumentError StationSelectionProblem(data_no_routing, 2;
                                                           max_walking_distance = R,
                                                           door_to_door_ratio = 2.0)
        @test StationSelectionProblem(data_no_routing, 2; max_walking_distance = R) isa
              StationSelectionProblem
    end

    @testset "map drops ODs the filter leaves unservable" begin
        formulation = ClusteringTwoStageODFormulation(2)
        off = StationSelection.create_map(
            StationSelectionProblem(data, 2; max_walking_distance = R), formulation, data)
        @test Set(off.Omega_s[1]) == Set([(1, 4), (2, 2)])

        # o == d has budget 2.0 * drive(2, 2) = 0: no pair survives, so the OD is dropped
        # rather than leaving `sum(x) == demand` over an empty set.
        on = @test_logs (:warn, r"door_to_door_ratio") match_mode = :any StationSelection.create_map(
            StationSelectionProblem(data, 2; max_walking_distance = R, door_to_door_ratio = 2.0),
            formulation, data)
        @test on.Omega_s[1] == [(1, 4)]
        @test on.Q_s[1] == [1]
        @test Set(StationSelection.get_valid_jk_pairs(on, 1, 4)) == pairs(2.0)
    end
end
