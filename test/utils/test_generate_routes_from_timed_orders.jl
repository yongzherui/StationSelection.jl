using StationSelection
using Test
using DataFrames
using Dates

@testset "generate_routes_from_timed_orders — cross-window chaining" begin
    # 4-station linear network:  A=1  B=2  C=3  D=4
    #   A ── B ── C ── D   (uniformly spaced, distance 1.0 between adjacent)
    # Routing costs (Euclidean):
    #   c(A,B)=1, c(A,C)=2, c(A,D)=3
    #   c(B,C)=1, c(B,D)=2
    #   c(C,D)=1
    stations = DataFrame(
        id  = [1, 2, 3, 4],
        lon = [0.0, 1.0, 2.0, 3.0],
        lat = [0.0, 0.0, 0.0, 0.0]
    )
    routing_costs = Dict{Tuple{Int,Int}, Float64}()
    walking_costs = Dict{Tuple{Int,Int}, Float64}()
    for i in 1:4, j in 1:4
        d = abs(i - j) * 1.0
        routing_costs[(i, j)] = d
        walking_costs[(i, j)] = d
    end

    requests = DataFrame(
        start_station_id = Int[],
        end_station_id   = Int[],
        request_time     = DateTime[]
    )
    data = create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs = routing_costs
    )

    station_id_to_idx = Dict(1 => 1, 2 => 2, 3 => 3, 4 => 4)
    time_window_sec   = 10   # 10-second windows for easy arithmetic

    # ── Helper: construct _TimedOrder (internal type) ─────────────────────────
    # Exposed via generate_routes_from_timed_orders through _TimedOrder
    function make_order(o_id, d_id, t_id, demand, vbs_pairs)
        StationSelection._TimedOrder(o_id, d_id, t_id, demand, vbs_pairs)
    end

    # ── Test 1: single order → single direct route ────────────────────────────
    @testset "single order — direct route" begin
        orders = [make_order(1, 4, 0, 1, [(1, 4)])]
        routes = generate_routes_from_timed_orders(
            orders, data, station_id_to_idx;
            vehicle_capacity = 4,
            max_wait_time    = 10.0,   # 1 window
            time_window_sec  = time_window_sec
        )
        @test length(routes) == 1
        @test routes[1].route.station_ids == [1, 4]
        # alpha: t_id=0, (j_idx=1, k_idx=4) → 1 passenger
        @test routes[1].alpha == Dict((0, (1, 4)) => 1)
    end

    # ── Test 2: cross-window chaining — vehicle arrives in time ───────────────
    @testset "cross-window chaining — feasible" begin
        # Order 1: t_id=0, picks up at A=1, drops at D=4  (t_start = 0s)
        # Order 2: t_id=5, picks up at B=2, drops at D=4  (t_start = 50s)
        # Vehicle picks up order 1 at A at t=0.
        # Travels A→B: arrives at t = 0 + c(A,B) = 1.
        # t2_start = 50s, max_wait = 20s → must arrive by t=70.
        # arr=1 < 50 → prune (vehicle arrives before order 2 is ready)
        # So NO cross-window pooling in this case.
        orders = [
            make_order(1, 4, 0, 1, [(1, 4)]),
            make_order(1, 4, 5, 1, [(2, 4)])
        ]
        routes = generate_routes_from_timed_orders(
            orders, data, station_id_to_idx;
            vehicle_capacity = 4,
            max_wait_time    = Float64(2 * time_window_sec),
            time_window_sec  = time_window_sec
        )
        # Each order has its own direct route; cross-window not feasible (early arrival)
        sids = [r.route.station_ids for r in routes]
        @test [1, 4] in sids
        @test [2, 4] in sids
        # No pooled route (arrival at B=1s is too early for t=50s order)
        @test !([1, 2, 4] in sids)
    end

    @testset "cross-window chaining — adjacent windows" begin
        # Order 1: t_id=0, pickup at A=1, dropoff at D=4  (t_start = 0s)
        # Order 2: t_id=0, pickup at B=2, dropoff at D=4  (t_start = 0s, same window)
        # max_wait = 10s (1 window)
        # Vehicle starts at A at t=0. Travels to B: arrives at t=0+1=1s.
        # 1s >= 0s (t2_start) ✓ and 1s <= 0+10=10s ✓ → pooling feasible
        orders = [
            make_order(1, 4, 0, 1, [(1, 4)]),
            make_order(2, 4, 0, 2, [(2, 4)])
        ]
        routes = generate_routes_from_timed_orders(
            orders, data, station_id_to_idx;
            vehicle_capacity = 4,
            max_wait_time    = Float64(time_window_sec),
            time_window_sec  = time_window_sec
        )
        sids = [r.route.station_ids for r in routes]
        @test [1, 4] in sids
        @test [2, 4] in sids
        # Pooled route [1, 2, 4] should exist
        @test [1, 2, 4] in sids
        pooled_idx = findfirst(r -> r.route.station_ids == [1, 2, 4], routes)
        @test !isnothing(pooled_idx)
        α = routes[pooled_idx].alpha
        # alpha: both orders are t_id=0; (1,4) → 1 pax, (2,4) → 2 pax
        @test get(α, (0, (1, 4)), 0) == 1
        @test get(α, (0, (2, 4)), 0) == 2
    end

    # ── Test 3: delay limit prunes infeasible routes ──────────────────────────
    @testset "delay limit — prunes detour" begin
        # Order 1: A→D (direct = c(A,D) = 3)
        # Order 2: B→D (direct = c(B,D) = 2)
        # Route [B, A, D]: order 2 boards at B at t=0, vehicle goes to A, then D.
        #   in_vehicle for order 2 = c(B,A) + c(A,D) = 1 + 3 = 4
        #   direct for order 2 = c(B,D) = 2
        #   detour = 4 - 2 = 2
        # With max_delay_time=1.0, this route is pruned.
        orders = [
            make_order(1, 4, 0, 1, [(1, 4)]),
            make_order(2, 4, 0, 1, [(2, 4)])
        ]
        routes_tight = generate_routes_from_timed_orders(
            orders, data, station_id_to_idx;
            vehicle_capacity = 2,
            max_wait_time    = Float64(time_window_sec),
            max_delay_time   = 1.0,
            time_window_sec  = time_window_sec
        )
        sids_tight = [r.route.station_ids for r in routes_tight]
        # [B, A, D] = [2, 1, 4] should be pruned (detour = 2 > limit = 1)
        @test !([2, 1, 4] in sids_tight)
        # [A, B, D] = [1, 2, 4]: order 1 boards A at t=0, vehicle goes B then D.
        #   in_vehicle for order 1 = c(A,B) + c(B,D) = 1 + 2 = 3
        #   direct = c(A,D) = 3  → detour = 0 ≤ 1 ✓
        @test [1, 2, 4] in sids_tight

        # With generous limit, both directions appear
        routes_loose = generate_routes_from_timed_orders(
            orders, data, station_id_to_idx;
            vehicle_capacity = 2,
            max_wait_time    = Float64(time_window_sec),
            max_delay_time   = 2.0,
            time_window_sec  = time_window_sec
        )
        sids_loose = [r.route.station_ids for r in routes_loose]
        @test [1, 2, 4] in sids_loose
        @test [2, 1, 4] in sids_loose
    end

    # ── Test 4: capacity limit ─────────────────────────────────────────────────
    @testset "capacity limit" begin
        # Three orders, each with demand 2; vehicle capacity = 4
        # Can pool at most 2 orders (2+2=4 ≤ 4), not all three (2+2+2=6 > 4)
        orders = [
            make_order(1, 4, 0, 2, [(1, 4)]),
            make_order(2, 4, 0, 2, [(2, 4)]),
            make_order(3, 4, 0, 2, [(3, 4)])
        ]
        routes = generate_routes_from_timed_orders(
            orders, data, station_id_to_idx;
            vehicle_capacity = 4,
            max_wait_time    = Float64(time_window_sec),
            time_window_sec  = time_window_sec
        )
        sids = [r.route.station_ids for r in routes]
        # No 3-order route (would exceed capacity)
        for sid in sids
            @test length(sid) <= 3   # at most 2 stops + 1 dropoff = 3 stations? Actually max 3 pickups, no...
        end
        # Actually: 3 pickups + 1 shared dropoff = station sequence of length ≤ 4
        # But capacity of 4 = 2 orders max (each has demand 2)
        # So no route should pick up all 3 orders
        for r in routes
            total_picked = count(j -> (r.route.od_capacity |> keys |> x -> any(p -> p[1] == stations.id[j], x)), 1:3)
            # Simpler: check alpha sum per route ≤ vehicle_capacity
            total_pax = sum(values(r.alpha))
            @test total_pax <= 4
        end
    end

    # ── Test 5: empty orders → empty result ───────────────────────────────────
    @testset "empty orders" begin
        @test isempty(generate_routes_from_timed_orders(
            StationSelection._TimedOrder[], data, station_id_to_idx;
            max_wait_time = 10.0
        ))
    end

    # ── Test 6: shared VBS stations — deduplication ───────────────────────────
    @testset "shared VBS stations — deduplication reduces route count" begin
        # Order 1: pickup always at A=1, dropoff at C=3 OR D=4
        # Order 2: pickup always at B=2, dropoff at C=3 OR D=4
        # All in same time window (t_id=0), generous max_wait_time, no delay limit
        # Naive routes (no dedup): 20  →  After dedup by (station_seq, picked): 12
        orders = [
            make_order(1, 3, 0, 1, [(1, 3), (1, 4)]),
            make_order(2, 3, 0, 1, [(2, 3), (2, 4)])
        ]
        routes = generate_routes_from_timed_orders(
            orders, data, station_id_to_idx;
            vehicle_capacity = 4,
            max_wait_time    = Float64(time_window_sec),
            time_window_sec  = time_window_sec
        )
        @test length(routes) == 12
        sids = [r.route.station_ids for r in routes]
        # Single-order routes
        @test [1, 3] in sids
        @test [1, 4] in sids
        @test [2, 3] in sids
        @test [2, 4] in sids
        # Two-order: order 1 pickup first
        @test [1, 2, 3] in sids
        @test [1, 2, 4] in sids
        @test [1, 2, 3, 4] in sids
        @test [1, 2, 4, 3] in sids
        # Two-order: order 2 pickup first
        @test [2, 1, 3] in sids
        @test [2, 1, 4] in sids
        @test [2, 1, 3, 4] in sids
        @test [2, 1, 4, 3] in sids
        # Interleaved routes must NOT appear (vehicle was empty mid-route)
        @test !([1, 3, 2, 4] in sids)
        @test !([1, 4, 2, 3] in sids)
        @test !([2, 3, 1, 4] in sids)
        @test !([2, 4, 1, 3] in sids)
    end

    # ── Test 7: max_wait_time = 0 → only same-window same-station routes ──────
    @testset "max_wait_time = 0 — no chaining" begin
        # With max_wait_time=0, vehicle must arrive at pickup EXACTLY at t_start.
        # For two orders at same t_id but different stations, chaining requires travel,
        # so arr > t_start → prune. Only single-order routes survive.
        orders = [
            make_order(1, 4, 0, 1, [(1, 4)]),
            make_order(2, 4, 0, 1, [(2, 4)])
        ]
        routes = generate_routes_from_timed_orders(
            orders, data, station_id_to_idx;
            vehicle_capacity = 2,
            max_wait_time    = 0.0,
            time_window_sec  = time_window_sec
        )
        sids = [r.route.station_ids for r in routes]
        @test [1, 4] in sids
        @test [2, 4] in sids
        # Pooled routes require vehicle to travel (arr > 0 = t_start), so pruned
        @test !([1, 2, 4] in sids)
        @test !([2, 1, 4] in sids)
    end
end
