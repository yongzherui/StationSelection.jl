@testset "Joint pricer: door-to-door ratio caps the ride limit" begin
    using DataFrames
    using Dates
    using JuMP

    # Stations on a line at positions 1..n. Walking takes `walk_per_unit` per unit of
    # distance, driving 1 per unit.
    function line_instance(ods; n, walk_per_unit, R, ratio, k = 2)
        stations = DataFrame(id = collect(1:n), lon = zeros(n), lat = zeros(n))
        requests = DataFrame(
            id = collect(1:length(ods)),
            start_station_id = first.(ods),
            end_station_id = last.(ods),
            request_time = fill(DateTime(2026, 1, 1, 8, 0), length(ods)),
        )
        walking = Dict((i, j) => walk_per_unit * abs(i - j) for i in 1:n for j in 1:n)
        routing = Dict((i, j) => Float64(abs(i - j)) for i in 1:n for j in 1:n)
        data = create_station_selection_data(
            stations, requests, walking;
            routing_costs = routing,
            scenarios = [("2026-01-01 08:00:00", "2026-01-01 09:00:00")],
        )
        problem = StationSelectionProblem(data, k; max_walking_distance = R,
                                          door_to_door_ratio = ratio)
        return data, problem
    end

    joint(; detour_factor = 1.5, max_stops = 4) = AggregateODRouteJointRoutingAssignmentFormulation(
        route_regularization_weight = 1.0, walk_cost_weight = 1.0, repositioning_time = 0.0,
        max_wait_time = 10.0, detour_factor = detour_factor, max_stops = max_stops)

    # Shortest ride from a visit of `j` to a LATER visit of `k` along `route`.
    function best_ride(route, j, k, data)
        best = Inf
        for a in eachindex(route), b in (a + 1):lastindex(route)
            (route[a] == j && route[b] == k) || continue
            t = sum(StationSelection.get_routing_cost(data, route[i], route[i + 1]) for i in a:(b - 1))
            best = min(best, t)
        end
        return best
    end

    # Every assignment a column carries must fit the capped ride limit.
    function columns_respect_limits(columns, mapping, data, detour_factor; s = 1)
        for column in columns, (p, j, k) in column.assignments
            o, d = mapping.Omega_s[s][p]
            limit = StationSelection.joint_routing_assignment_ride_limit(
                data, mapping, detour_factor, o, d, j, k)
            best_ride(column.route, j, k, data) <= limit + 1e-9 || return false
        end
        return true
    end

    @testset "ride limit = min(detour limit, door-to-door budget)" begin
        # Five stations, walking twice as slow as driving, one-station walking radius.
        data, problem = line_instance([(1, 5)]; n = 5, walk_per_unit = 2.0, R = 2.5, ratio = 2.0)
        mapping = StationSelection.create_aggregate_od_route_map(problem, joint(), data)
        @test mapping.door_to_door_ratio == 2.0
        limit(df, j, k) = StationSelection.joint_routing_assignment_ride_limit(
            data, mapping, df, 1, 5, j, k)

        # drive(1,5) = 4, budget 8. (1,5): no walking, 8 vs 1.5 * 4 = 6 -> 6 (ride limit binds).
        @test limit(1.5, 1, 5) ≈ 6.0
        # (2,4): walks 2 + 2, budget 4 vs 1.5 * 2 = 3 -> 3; with detour 3.0: 6 vs 4 -> 4 (G2 binds).
        @test limit(1.5, 2, 4) ≈ 3.0
        @test limit(3.0, 2, 4) ≈ 4.0
        # Never below the direct ride: the pair filter guarantees it.
        for (j, k) in StationSelection.get_valid_jk_pairs(mapping, 1, 5)
            StationSelection.is_walk_only_pair((j, k)) && continue
            @test limit(1.0, j, k) >= StationSelection.get_routing_cost(data, j, k) - 1e-9
        end

        # Ratio off: exactly the old limit.
        _, problem_off = line_instance([(1, 5)]; n = 5, walk_per_unit = 2.0, R = 2.5, ratio = Inf)
        mapping_off = StationSelection.create_aggregate_od_route_map(problem_off, joint(), data)
        @test StationSelection.joint_routing_assignment_ride_limit(
            data, mapping_off, 3.0, 1, 5, 2, 4) ≈ 6.0

        # A ratio that disagrees with the pair filter is an error, not a silent bad seed.
        mapping_off.door_to_door_ratio = 1.0
        @test_throws ErrorException StationSelection.joint_routing_assignment_ride_limit(
            data, mapping_off, 3.0, 1, 5, 2, 4)
    end

    @testset "pricing candidates carry the capped limit" begin
        data, problem = line_instance([(1, 5)]; n = 5, walk_per_unit = 2.0, R = 2.5, ratio = 2.0)
        mapping = StationSelection.create_aggregate_od_route_map(problem, joint(), data)
        alpha = Dict((1, 1) => 100.0)           # large coverage dual: every pair has rho > 0
        gamma = Dict{Tuple{Tuple{Int, Int}, Int}, Float64}()
        candidates = StationSelection.joint_routing_assignment_pricing_candidates(
            data, mapping, alpha, gamma, gamma, 1.0, 3.0, 1)
        @test !isempty(candidates)
        for c in candidates
            @test c.ride_limit ≈ StationSelection.joint_routing_assignment_ride_limit(
                data, mapping, 3.0, 1, 5, c.origin, c.destination)
        end
        c24 = only(filter(c -> (c.origin, c.destination) == (2, 4), candidates))
        @test c24.ride_limit ≈ 4.0              # G2 binds below 3.0 * 2
    end

    @testset "replay: a detour inside the ride limit but outside G2 is not credited" begin
        # OD (1,2), drive 1. Detour factor 3 allows route 1 -> 3 -> 2 (ride 3); ratio 2 caps
        # the ride at 2. Walking is slow, so (1,2) is the only station pair left under G2.
        # (Enumeration never builds [1, 3, 2] -- it reuses Base's route DFS -- so this checks
        # the certification rule on the route directly.)
        function credited(ratio, route)
            data, problem = line_instance([(1, 2)]; n = 3, walk_per_unit = 100.0, R = 250.0, ratio = ratio)
            mapping = StationSelection.create_aggregate_od_route_map(problem, joint(), data)
            gamma = Dict{Tuple{Tuple{Int, Int}, Int}, Float64}()
            candidates = StationSelection.joint_routing_assignment_pricing_candidates(
                data, mapping, Dict((1, 1) => 1000.0), gamma, gamma, 1.0, 3.0, 1)
            travel = Dict((i, j) => StationSelection.get_routing_cost(data, i, j)
                          for i in 1:3 for j in 1:3 if i != j)
            pd = create_joint_routing_assignment_pricing_data(
                1, collect(1:3), travel, candidates;
                route_regularization_weight = 1.0, max_wait_time = 10.0)
            certified = StationSelection._replay_joint_routing_assignment_route_all_certifications(route, pd)
            return (1, 2) in get(certified, 1, Tuple{Int, Int}[])
        end
        @test credited(Inf, [1, 3, 2])
        @test !credited(2.0, [1, 3, 2])
        @test credited(2.0, [1, 2])             # the direct hop always stays feasible
    end

    @testset "CG pricers and direct MIP agree under a finite ratio" begin
        ods = [(1, 5), (2, 4), (5, 1), (1, 3), (4, 2)]
        formulation = joint(detour_factor = 3.0, max_stops = 4)
        objective = Dict{Tuple{Float64, Symbol}, Float64}()
        for ratio in (Inf, 2.0)
            data, problem = line_instance(ods; n = 5, walk_per_unit = 2.0, R = 2.5, ratio = ratio, k = 3)
            direct = run_opt(problem, formulation, DirectMIPSolver())
            @test direct.termination_status == SOLVE_OPTIMAL
            objective[(ratio, :direct)] = direct.objective_value
            mapping = StationSelection.create_aggregate_od_route_map(problem, formulation, data)
            @test columns_respect_limits(
                StationSelection.enumerate_joint_routing_assignment_columns(problem, formulation, data),
                mapping, data, 3.0)
            if isfinite(ratio)
                # Discriminating: with G2 off, enumeration credits some assignment whose pair
                # survives the ratio filter but whose ride exceeds the capped limit -- the
                # case the check above rules out.
                data_off, problem_off = line_instance(ods; n = 5, walk_per_unit = 2.0, R = 2.5,
                                                      ratio = Inf, k = 3)
                mapping_off = StationSelection.create_aggregate_od_route_map(problem_off, formulation, data_off)
                cols_off = StationSelection.enumerate_joint_routing_assignment_columns(
                    problem_off, formulation, data_off)
                @test any(cols_off) do column
                    any(column.assignments) do (p, j, k)
                        o, d = mapping_off.Omega_s[1][p]
                        (j, k) in StationSelection.get_valid_jk_pairs(mapping, o, d) || return false
                        limit = StationSelection.joint_routing_assignment_ride_limit(
                            data, mapping, 3.0, o, d, j, k)
                        best_ride(column.route, j, k, data) > limit + 1e-9
                    end
                end
            end
            # :relaxed_cluster takes each cluster's LARGEST member limit: still a relaxation.
            for pricing in (CGPricingConfig(mode = :exact), CGPricingConfig(mode = :darp),
                            CGPricingConfig(mode = :darp_modified),
                            CGPricingConfig(mode = :relaxed_cluster, relaxed_cluster_count = 2))
                result = run_opt(problem, formulation, CGSolver(
                    max_iterations = 100, reduced_cost_tol = 1e-7,
                    recover_integer_solution = true, pricing = pricing))
                @test result.termination_status == SOLVE_OPTIMAL
                @test result.metadata["cg_converged"]
                @test isapprox(result.objective_value, direct.objective_value; atol = 1e-6)
                @test columns_respect_limits(
                    values(result.model[:joint_routing_assignment_columns]), mapping, data, 3.0)
            end
        end
        # G2 only removes options, so it can never make the optimum cheaper.
        @test objective[(2.0, :direct)] >= objective[(Inf, :direct)] - 1e-6
    end

    @testset "supplied initial columns are checked against this problem" begin
        ods = [(1, 5), (2, 4), (5, 1), (1, 3), (4, 2)]
        formulation = joint(detour_factor = 3.0, max_stops = 4)
        data, off = line_instance(ods; n = 5, walk_per_unit = 2.0, R = 2.5, ratio = Inf, k = 3)
        _, on = line_instance(ods; n = 5, walk_per_unit = 2.0, R = 2.5, ratio = 2.0, k = 3)
        cols_off = StationSelection.enumerate_joint_routing_assignment_columns(off, formulation, data)
        cols_on = StationSelection.enumerate_joint_routing_assignment_columns(on, formulation, data)
        for c in cols_off; c.metadata["scenario"] = 1; end
        for c in cols_on; c.metadata["scenario"] = 1; end

        # A pool priced on this problem is accepted...
        @test StationSelection.build_model(on, formulation, CGSolver(initial_columns = cols_on)) isa
              StationSelection.BuildResult
        # ...one priced with G2 off carries assignments G2 forbids, and is refused.
        @test_throws ErrorException StationSelection.build_model(
            on, formulation, CGSolver(initial_columns = cols_off))
    end

    @testset "an o == d request is an error under a finite ratio" begin
        # Budget 2.0 * drive(3, 3) = 0: unservable under G2.
        ods = [(1, 5), (3, 3), (4, 2)]
        formulation = joint(detour_factor = 3.0, max_stops = 4)
        data, problem = line_instance(ods; n = 5, walk_per_unit = 2.0, R = 2.5, ratio = 2.0, k = 3)
        @test_throws ArgumentError StationSelection.create_aggregate_od_route_map(problem, formulation, data)
        @test_throws ArgumentError run_opt(problem, formulation, CGSolver())
        # With G2 off the same instance builds.
        _, off = line_instance(ods; n = 5, walk_per_unit = 2.0, R = 2.5, ratio = Inf, k = 3)
        @test (3, 3) in StationSelection.create_aggregate_od_route_map(off, formulation, data).Omega_s[1]
    end
end
