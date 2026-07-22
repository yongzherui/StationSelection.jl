"""
Station-generic nearest-open endpoint mapping + direct walking, for
`AggregateODRouteModel` with
`NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)` and
`allow_walk_only=true`. Covers the 8 scenarios from the direct-walking spec:
distinct nearest stations, coincident nearest stations (direct walking),
nearest station closed, shared physical endpoints, walking-infeasible
collisions, invalid same-station data, LP integrality under fixed `y`, and a
compact-model-vs-procedural-enumeration comparison.

Direct walking reuses the existing `WALK_ONLY_PAIR = (0,0)` sentinel slot in
`x` as `w_p` (see `src/opt/constraints/aggregate_od_route.jl`,
`_add_nearest_open_endpoint_linked_x!`) rather than introducing a new
variable type.
"""
# Small local powerset helper (avoids adding Combinatorics as a test dependency).
function _powerset_binary(n::Int)
    return [[(mask >> (i - 1)) & 1 for i in 1:n] for mask in 0:(2^n - 1)]
end

@testset "AggregateODRouteModel nearest-open direct walking" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping direct-walking tests"
        @test true
        return
    end

    # Solves `_build_nearest_open_y_subproblem_lp` directly with `y` fixed to
    # `y_hat` -- the same LP BendersY uses to derive duals, exercised here
    # with continuous [0,1] z/x/w to check integrality falls out of the
    # constraints (spec Test 7) without needing station-limit selection (`l`)
    # to land on a specific y by construction.
    function solve_fixed_y(data::StationSelectionData, model::AggregateODRouteModel, y_hat::Vector{Float64})
        mapping = create_map(model, data)
        requests, demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)
        m, fix_cons, x, cover_cons = StationSelection._build_nearest_open_y_subproblem_lp(
            data, model, mapping, requests, demand, feasible_pairs, mapping.columns,
            y_hat, Gurobi.Env(), true,
        )
        optimize!(m)
        return m, x, requests, feasible_pairs, mapping
    end

    x_value(x, request, pair) = value(x[(request, pair)])

    @testset "endpoint_chain style remains available" begin
        stations = DataFrame(id=collect(1:3), lon=Float64.(1:3), lat=zeros(3))
        requests = DataFrame(
            id=[1], start_station_id=[1], end_station_id=[3],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:3, j in 1:3)
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 2)] = 4.0
        walking_costs[(3, 3)] = 0.0
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:3, j in 1:3)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            2; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:endpoint_chain),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        m, x, requests_flat, _feasible_pairs, _mapping = solve_fixed_y(data, model, [0.0, 1.0, 1.0])
        @test primal_status(m) == MOI.FEASIBLE_POINT
        @test isapprox(x_value(x, only(requests_flat), (2, 3)), 1.0; atol=1e-6)
        @test m[:nearest_endpoint_selector_style] == :endpoint_chain
    end

    # --- Test 1: distinct nearest stations ------------------------------
    @testset "distinct nearest stations" begin
        stations = DataFrame(id=collect(1:4), lon=Float64.(1:4), lat=zeros(4))
        requests = DataFrame(
            id=[1], start_station_id=[1], end_station_id=[3],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:4, j in 1:4)
        walking_costs[(1, 1)] = 0.0   # pickup A (nearest)
        walking_costs[(1, 2)] = 4.0   # pickup, farther
        walking_costs[(3, 3)] = 0.0   # dropoff B (nearest)
        walking_costs[(3, 4)] = 4.0   # dropoff, farther
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:4, j in 1:4)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            4; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        m, x, requests_flat, feasible_pairs, _mapping = solve_fixed_y(data, model, ones(4))
        @test primal_status(m) == MOI.FEASIBLE_POINT
        request = only(requests_flat)
        @test isapprox(x_value(x, request, (1, 3)), 1.0; atol=1e-6)
        # dist(1,3) exceeds 2*max_walking_distance here, so WALK_ONLY_PAIR
        # is never even a candidate for this OD -- w_p=0 is structural, not
        # merely a solved-to-zero outcome.
        @test !(StationSelection.WALK_ONLY_PAIR in feasible_pairs[request])
        for pair in feasible_pairs[request]
            pair == (1, 3) && continue
            @test isapprox(x_value(x, request, pair), 0.0; atol=1e-6)
        end
    end

    # --- Test 2: same nearest station -> direct walking -----------------
    @testset "same nearest station" begin
        # Physical request endpoints must themselves be rows of the station
        # DataFrame (origin/destination are resolved via
        # station_id_to_array_idx) -- stations 4 and 5 here are the physical
        # request origin/destination, each within range only of common
        # candidate station 1 (stations 2, 3 are unreachable decoys).
        stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
        requests = DataFrame(
            id=[1], start_station_id=[4], end_station_id=[5],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:5, j in 1:5)
        walking_costs[(4, 1)] = 0.0   # o's nearest (and only) pickup candidate
        walking_costs[(1, 5)] = 0.0   # d's nearest (and only) dropoff candidate
        # Direct walk cost must exceed max_walking_distance (5.0, else station
        # 5 would itself become a second pickup candidate for o=4 via this
        # same (4,5) entry) but stay within 2*max_walking_distance=10.0.
        walking_costs[(4, 5)] = 7.0
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:5, j in 1:5)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            5; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        m, x, requests_flat, feasible_pairs, _mapping = solve_fixed_y(data, model, ones(5))
        @test primal_status(m) == MOI.FEASIBLE_POINT
        request = only(requests_flat)
        @test feasible_pairs[request] == [StationSelection.WALK_ONLY_PAIR]
        @test isapprox(x_value(x, request, StationSelection.WALK_ONLY_PAIR), 1.0; atol=1e-6)
    end

    # --- Test 3: nearest station closed -> falls through to second ------
    @testset "nearest station closed" begin
        stations = DataFrame(id=collect(1:3), lon=Float64.(1:3), lat=zeros(3))
        requests = DataFrame(
            id=[1], start_station_id=[1], end_station_id=[3],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:3, j in 1:3)
        walking_costs[(1, 1)] = 0.0   # nearest pickup, will be closed
        walking_costs[(1, 2)] = 4.0   # second-ranked pickup
        walking_costs[(3, 3)] = 0.0   # forced singleton dropoff
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:3, j in 1:3)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            2; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        y_hat = [0.0, 1.0, 1.0]  # station 1 (nearest) closed; 2, 3 open
        m, x, requests_flat, feasible_pairs, _mapping = solve_fixed_y(data, model, y_hat)
        @test primal_status(m) == MOI.FEASIBLE_POINT
        request = only(requests_flat)
        @test isapprox(x_value(x, request, (2, 3)), 1.0; atol=1e-6)
        @test !(StationSelection.WALK_ONLY_PAIR in feasible_pairs[request])
    end

    # --- Test 4: shared physical endpoint reuses the same z chain -------
    @testset "shared endpoint" begin
        stations = DataFrame(id=collect(1:4), lon=Float64.(1:4), lat=zeros(4))
        # Two requests share physical origin endpoint 1, with distinct destinations.
        requests = DataFrame(
            id=[1, 2], start_station_id=[1, 1], end_station_id=[3, 4],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:4, j in 1:4)
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 2)] = 4.0
        walking_costs[(3, 3)] = 0.0
        walking_costs[(4, 4)] = 0.0
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:4, j in 1:4)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            4; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        build_result = build_model(model, data; optimizer_env=Gurobi.Env())
        m = build_result.model
        cache = m[:nearest_endpoint_chain_cache]
        pickup_chains = [key for key in keys(cache) if key[1] == :pickup]
        # Both requests' origin endpoint (station 1, candidates {1,2}) must
        # resolve to exactly one shared pickup chain -- not two.
        @test count(key -> Set(key[2]) == Set([1, 2]), pickup_chains) == 1
    end

    # --- Test 5: walking infeasible + distinct stations ------------------
    @testset "walking infeasible, distinct stations" begin
        stations = DataFrame(id=collect(1:4), lon=Float64.(1:4), lat=zeros(4))
        requests = DataFrame(
            id=[1], start_station_id=[1], end_station_id=[3],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:4, j in 1:4)
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 2)] = 4.0
        walking_costs[(3, 3)] = 0.0
        walking_costs[(3, 4)] = 4.0
        walking_costs[(1, 3)] = 50.0  # direct walk far exceeds 2*max_walking_distance=10
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:4, j in 1:4)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            4; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        m, x, requests_flat, feasible_pairs, _mapping = solve_fixed_y(data, model, ones(4))
        @test primal_status(m) == MOI.FEASIBLE_POINT
        request = only(requests_flat)
        @test !(StationSelection.WALK_ONLY_PAIR in feasible_pairs[request])
        @test isapprox(x_value(x, request, (1, 3)), 1.0; atol=1e-6)
    end

    # --- Test 6: invalid same-station data (diagnostic, not silent infeasibility) ---
    @testset "invalid same-station data raises diagnostic" begin
        # Same layout as Test 2, but the direct-walk distance is far too long
        # -- both endpoints CAN map to the same station (common candidate 1),
        # yet direct walking is not offered as a fallback. This should be
        # caught as a clear diagnostic, not left as a silent infeasibility.
        stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
        requests = DataFrame(
            id=[1], start_station_id=[4], end_station_id=[5],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:5, j in 1:5)
        walking_costs[(4, 1)] = 0.0
        walking_costs[(1, 5)] = 0.0
        walking_costs[(4, 5)] = 50.0  # exceeds 2*max_walking_distance=10
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:5, j in 1:5)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            5; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        @test_throws ArgumentError build_model(model, data; optimizer_env=Gurobi.Env())
    end

    # --- Test 7: LP integrality under several fixed binary y patterns ----
    @testset "LP integrality for fixed binary y" begin
        stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
        requests = DataFrame(
            id=[1, 2], start_station_id=[1, 2], end_station_id=[5, 2],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:5, j in 1:5)
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 2)] = 3.0
        walking_costs[(4, 5)] = 3.0
        walking_costs[(5, 5)] = 0.0
        walking_costs[(2, 2)] = 0.0  # request B: origin == destination endpoint (o=d=2)
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:5, j in 1:5)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            4; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        for y_hat in (
            [1.0, 1.0, 1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0, 0.0, 1.0],
            [1.0, 1.0, 0.0, 1.0, 1.0],
        )
            m, x, requests_flat, feasible_pairs, mapping = solve_fixed_y(data, model, y_hat)
            @test primal_status(m) == MOI.FEASIBLE_POINT
            for request in requests_flat
                for pair in feasible_pairs[request]
                    val = x_value(x, request, pair)
                    @test val < 1e-6 || val > 1.0 - 1e-6
                end
            end
            assert_endpoint_chain_near_binary(m)
        end
    end

    # --- Test 8: compact model matches procedural enumeration for every feasible y ---
    @testset "compact model matches procedural nearest-open enumeration" begin
        stations = DataFrame(id=collect(1:4), lon=Float64.(1:4), lat=zeros(4))
        requests = DataFrame(
            id=[1, 2], start_station_id=[1, 2], end_station_id=[2, 4],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
        )
        walking_costs = Dict((i, j) => 100.0 for i in 1:4, j in 1:4)
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 3)] = 4.0
        walking_costs[(2, 2)] = 0.0   # request A: o=1,d=2 can coincide at station 2 if o's 2nd-ranked (3) is closed...
        walking_costs[(2, 3)] = 4.0
        walking_costs[(2, 4)] = 100.0
        walking_costs[(4, 4)] = 0.0
        walking_costs[(4, 3)] = 4.0
        walking_costs[(1, 2)] = 3.0
        routing_costs = Dict((i, j) => abs(i - j) + 1.0 for i in 1:4, j in 1:4)
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            2; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0, allow_walk_only=true,
        )
        mapping = create_map(model, data)
        requests_flat, _demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)

        for open_set in _powerset_binary(4)
            sum(open_set) == 0 && continue  # skip the trivially-infeasible all-closed pattern
            y_hat = Float64.(open_set)
            open_ids = Set(j for j in 1:4 if open_set[j] == 1)
            expected, expected_infeasible = StationSelection._fixed_assignments_from_y(
                data, requests_flat, feasible_pairs, y_hat;
                style=:big_m_nearest, max_walking_distance=model.max_walking_distance,
                allow_walk_only=model.allow_walk_only,
            )
            m, fix_cons, x, cover_cons = StationSelection._build_nearest_open_y_subproblem_lp(
                data, model, mapping, requests_flat, _demand, feasible_pairs, mapping.columns,
                y_hat, Gurobi.Env(), true,
            )
            optimize!(m)
            if !isempty(expected_infeasible)
                @test primal_status(m) != MOI.FEASIBLE_POINT
                continue
            end
            @test primal_status(m) == MOI.FEASIBLE_POINT
            for request in requests_flat
                selected = [pair for pair in feasible_pairs[request] if x_value(x, request, pair) > 0.5]
                @test length(selected) == 1
                @test only(selected) == expected[request]
            end
        end
    end
end
