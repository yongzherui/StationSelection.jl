using StationSelection
using Test
using DataFrames
using Dates

@testset "generate_routes_from_orders — Y-network" begin
    # Y-network: L=1 at (-1,0), R=2 at (1,0), M=3 at (0,1.1), T=4 at (0,2.2)
    # Euclidean routing costs:
    #   c(L,R) = c(R,L) = 2.0
    #   c(L,M) = c(M,L) ≈ 1.487
    #   c(R,M) = c(M,R) ≈ 1.487
    #   c(L,T) = c(T,L) ≈ 2.417
    #   c(R,T) = c(T,R) ≈ 2.417
    #   c(M,T) = c(T,M) = 1.1

    stations = DataFrame(
        id  = [1, 2, 3, 4],
        lon = [-1.0, 1.0, 0.0, 0.0],
        lat = [0.0,  0.0, 1.1, 2.2]
    )

    sids   = [1, 2, 3, 4]
    coords = Dict(1 => (-1.0, 0.0), 2 => (1.0, 0.0), 3 => (0.0, 1.1), 4 => (0.0, 2.2))

    routing_costs = Dict{Tuple{Int,Int}, Float64}()
    for i in sids, j in sids
        dx = coords[i][1] - coords[j][1]
        dy = coords[i][2] - coords[j][2]
        routing_costs[(i, j)] = sqrt(dx^2 + dy^2)
    end

    # Minimal walking costs (same Euclidean distances)
    walking_costs = copy(routing_costs)

    # Single empty scenario (routes are generated purely from od_pairs)
    requests = DataFrame(
        start_station_id = Int[],
        end_station_id   = Int[],
        request_time     = DateTime[]
    )

    data = create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs = routing_costs
    )

    # OD pairs covering all walking choices for 2 geographic passengers
    od_pairs = [(1, 4), (2, 4), (3, 4)]

    # ── Test 1: generous detour budget → 9 unique routes ──────────────────────
    # Worst-case detours:
    #   [L,R,T] for L→T: c(L,R)+c(R,T)-c(L,T) = 2.0+2.417-2.417 = 2.0
    #   [M,L,T] for M→T: c(M,L)+c(L,T)-c(M,T) ≈ 1.487+2.417-1.1 ≈ 2.804
    #   [M,R,T] for M→T: same ≈ 2.804
    # → need max_detour_time ≥ 2.804 to include [3,1,4] and [3,2,4]
    routes = generate_routes_from_orders(od_pairs, data;
        vehicle_capacity = 2,
        max_detour_time  = 3.0)

    @test length(routes) == 9

    sid_sets = [r.station_ids for r in routes]
    for expected in [
        [1, 4], [2, 4], [3, 4],
        [1, 2, 4], [2, 1, 4],
        [1, 3, 4], [3, 1, 4],
        [2, 3, 4], [3, 2, 4]
    ]
        @test expected in sid_sets
    end

    # Verify that with max_detour_time=2.0, routes with M-passenger detour are filtered
    # ([3,1,4] and [3,2,4] have detour ≈ 2.804 for the M→T passenger)
    routes_tight = generate_routes_from_orders(od_pairs, data;
        vehicle_capacity = 2,
        max_detour_time  = 2.0)
    @test length(routes_tight) == 7
    sid_sets_tight = [r.station_ids for r in routes_tight]
    @test !([3, 1, 4] in sid_sets_tight)
    @test !([3, 2, 4] in sid_sets_tight)

    # ── Test 2: no detour budget → only 3 direct routes ───────────────────────
    routes_direct = generate_routes_from_orders(od_pairs, data;
        vehicle_capacity = 2,
        max_detour_time  = 0.0)

    @test length(routes_direct) == 3
    sid_sets_direct = [r.station_ids for r in routes_direct]
    for expected in [[1, 4], [2, 4], [3, 4]]
        @test expected in sid_sets_direct
    end

    # ── Test 3: od_capacity keys are correct ──────────────────────────────────
    # Each size-1 route covers exactly its own OD pair
    for r in routes_direct
        @test length(r.od_capacity) == 1
        @test haskey(r.od_capacity, (r.station_ids[1], r.station_ids[end]))
    end

    # The pooled route [1,2,4] covers both (1,4) and (2,4)
    r_lrt = findfirst(r -> r.station_ids == [1, 2, 4], routes)
    @test !isnothing(r_lrt)
    rc = routes[r_lrt].od_capacity
    @test haskey(rc, (1, 4))
    @test haskey(rc, (2, 4))

    # ── Test 4: empty od_pairs → empty result ─────────────────────────────────
    @test isempty(generate_routes_from_orders(Tuple{Int,Int}[], data; vehicle_capacity = 2))
end
