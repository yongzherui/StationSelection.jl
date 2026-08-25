@testset "DARP-style passenger pricing" begin
    using Random
    using DataFrames
    using Dates
    using JuMP

    complete_line_cost(n) = Dict(
        (i, j) => Float64(abs(i - j)) for i in 1:n for j in 1:n if i != j
    )

    function darp_data(candidates; n=3, max_wait_time=10.0, max_stops=5)
        return create_joint_routing_assignment_darp_pricing_data(
            1, collect(1:n), complete_line_cost(n), candidates;
            route_regularization_weight=1.0,
            max_wait_time=max_wait_time,
            max_stops=max_stops,
        )
    end

    @testset "initial station enumerates pickup choices and credits pickup reward" begin
        candidates = [PassengerAssignmentCandidate(1, 1, 2, 2.0, 10.0)]
        data = darp_data(candidates; n=2, max_stops=2)
        starts = filter(label -> label.current == 1,
            initial_joint_routing_assignment_darp_pricing_labels(data))

        @test length(starts) == 2 # skip or board
        boarded = only(filter(label -> haskey(label.onboard, 1), starts))
        @test boarded.reduced_cost ≈ -10.0

        delivered = only(extend_joint_routing_assignment_darp_pricing_label(
            boarded, (2, Tuple{Int, Int, Int, Float64}[]), data,
        ))
        @test delivered.served == Set([(1, 1, 2)])
        @test isempty(delivered.onboard)
        @test delivered.reduced_cost ≈ -9.0 # delivery does not credit reward twice
    end

    @testset "pickup window is tested at arrival" begin
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 5.0, 4.0),
            PassengerAssignmentCandidate(2, 2, 3, 5.0, 7.0),
        ]
        data = darp_data(candidates; n=3, max_wait_time=0.5)
        label = JointRoutingAssignmentDarpPricingLabel(
            1, [1], 0.0, Dict(1 => (1, 2, 0.0)),
            Set{Tuple{Int, Int, Int}}(), 0.0, -4.0, 1,
        )

        # Reaching node 2 takes one unit, so passenger 2 may not board there
        # even though the vehicle departed before the 0.5 pickup cutoff.
        actions = StationSelection._joint_routing_assignment_darp_candidate_next_nodes(label, data)
        at_two = filter(action -> action[1] == 2, actions)
        @test length(at_two) == 1
        @test isempty(only(at_two)[2])
    end

    @testset "onboard reward accounting makes subset dominance sound" begin
        empty_label = JointRoutingAssignmentDarpPricingLabel(
            2, [2], 1.0, Dict{Int, Tuple{Int, Int, Float64}}(),
            Set{Tuple{Int, Int, Int}}(), 1.0, 0.0, 1,
        )
        onboard_label = JointRoutingAssignmentDarpPricingLabel(
            2, [1, 2], 1.0, Dict(1 => (1, 3, 1.0)),
            Set{Tuple{Int, Int, Int}}(), 1.0, -10.0, 2,
        )
        @test !StationSelection._dominates_joint_routing_assignment_darp_label(
            empty_label, onboard_label, false,
        )

        sufficiently_cheaper_empty = JointRoutingAssignmentDarpPricingLabel(
            2, [2], 1.0, Dict{Int, Tuple{Int, Int, Float64}}(),
            Set{Tuple{Int, Int, Int}}(), 1.0, -11.0, 1,
        )
        @test StationSelection._dominates_joint_routing_assignment_darp_label(
            sufficiently_cheaper_empty, onboard_label, false,
        )
    end

    @testset "set and bitset dominance implementations agree" begin
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 4.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 3, 3.0, 7.0),
        ]
        data = darp_data(candidates; n=3)
        rng = MersenneTwister(17)
        for _ in 1:100
            function random_label()
                onboard = Dict{Int, Tuple{Int, Int, Float64}}()
                served = Set{Tuple{Int, Int, Int}}()
                for candidate in candidates
                    state = rand(rng, 0:2)
                    state == 1 && (onboard[candidate.p] = (
                        candidate.origin, candidate.destination, 4 * rand(rng),
                    ))
                    state == 2 && push!(served, (
                        candidate.p, candidate.origin, candidate.destination,
                    ))
                end
                return JointRoutingAssignmentDarpPricingLabel(
                    3, [3], 5 * rand(rng), onboard, served, rand(rng),
                    10 * rand(rng) - 10, rand(rng, 1:5),
                )
            end
            a, b = random_label(), random_label()
            abs = StationSelection._make_joint_routing_assignment_darp_label_bitsets(a, data)
            bbs = StationSelection._make_joint_routing_assignment_darp_label_bitsets(b, data)
            bounded = rand(rng, Bool)
            @test StationSelection._dominates_joint_routing_assignment_darp_label(
                a, b, bounded,
            ) == StationSelection._dominates_joint_routing_assignment_darp_label(
                a, b, abs, bbs, bounded,
            )
        end
    end

    @testset "duplicate assignment triples are rejected" begin
        duplicate = PassengerAssignmentCandidate(1, 1, 2, 2.0, 10.0)
        @test_throws ArgumentError darp_data([duplicate, duplicate]; n=2)
    end

    @testset "incomplete labels project to valid improving columns" begin
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 4.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 3, 4.0, 7.0),
        ]
        data = darp_data(candidates; n=3)
        ctx = StationSelection.JointRoutingAssignmentDarpSearchContext(data)
        incomplete = JointRoutingAssignmentDarpPricingLabel(
            2, [1, 2], 1.0, Dict(2 => (2, 3, 0.0)),
            Set([(1, 1, 2)]), 1.0, -16.0, 2,
        )

        @test StationSelection._pricing_best_signature(ctx, incomplete) == ((1, 1, 2),)
        projected = StationSelection._pricing_candidate_from_label(ctx, incomplete)
        @test projected.payload.assignments == [(1, 1, 2)]
        @test projected.payload.route == [1, 2]
        @test projected.reduced_cost ≈ -9.0 # -16 + refund passenger 2's reward 7

        scored = Dict{Any, Any}()
        accept = StationSelection._pricing_accept_closure(
            ctx, 1, Dict{Any, Float64}(), scored, CGSolver(reduced_cost_tol=1e-6), 10,
        )
        @test !accept(incomplete) # accepted, but candidate quota not yet reached
        @test only(values(scored)).reduced_cost ≈ -9.0

        nonimproving = JointRoutingAssignmentDarpPricingLabel(
            2, [1, 2], 10.0, Dict(2 => (2, 3, 0.0)),
            Set([(1, 1, 2)]), 10.0, -5.0, 2,
        )
        rejected = Dict{Any, Any}()
        reject = StationSelection._pricing_accept_closure(
            ctx, 1, Dict{Any, Float64}(), rejected, CGSolver(reduced_cost_tol=1e-6), 10,
        )
        @test !reject(nonimproving)
        @test isempty(rejected) # projected rc = -5 + 7 = 2

        complete = JointRoutingAssignmentDarpPricingLabel(
            2, [1, 2], 1.0, Dict{Int, Tuple{Int, Int, Float64}}(),
            Set([(1, 1, 2)]), 1.0, -9.0, 2,
        )
        @test StationSelection._pricing_candidate_from_label(ctx, complete).reduced_cost ≈ -9.0
    end

    @testset "exhausted DARP and exact searches agree on small random instances" begin
        rng = MersenneTwister(20260825)
        nodes = collect(1:4)
        travel = complete_line_cost(4)
        for _ in 1:10
            candidates = PassengerAssignmentCandidate[]
            for p in 1:2
                passenger_pairs = Set{Tuple{Int, Int}}()
                while length(passenger_pairs) < 2
                    j, k = rand(rng, nodes, 2)
                    j == k && continue
                    (j, k) in passenger_pairs && continue
                    push!(passenger_pairs, (j, k))
                    direct = travel[(j, k)]
                    push!(candidates, PassengerAssignmentCandidate(
                        p, j, k, direct + rand(rng, 0:2), Float64(rand(rng, 2:10)),
                    ))
                end
            end

            exact_data = create_joint_routing_assignment_pricing_data(
                1, nodes, travel, candidates;
                route_regularization_weight=1.0, max_wait_time=3.0,
                max_stops=4, compensated_dominance=false,
            )
            darp = create_joint_routing_assignment_darp_pricing_data(
                1, nodes, travel, candidates;
                route_regularization_weight=1.0, max_wait_time=3.0, max_stops=4,
            )
            exact_ctx = StationSelection.JointRoutingAssignmentSearchContext(exact_data)
            darp_ctx = StationSelection.JointRoutingAssignmentDarpSearchContext(darp)
            exact_labels, exact_exhausted, _ = StationSelection._run_label_setting(
                exact_ctx; time_limit=30.0, reduced_cost_tol=0.0,
                use_reduced_cost_pruning=false,
            )
            darp_labels, darp_exhausted, _ = StationSelection._run_label_setting(
                darp_ctx; time_limit=30.0, reduced_cost_tol=0.0,
                use_reduced_cost_pruning=false,
            )
            @test exact_exhausted && darp_exhausted
            exact_best = minimum(
                StationSelection._pricing_candidate_from_label(exact_ctx, label).reduced_cost
                for label in exact_labels
            )
            darp_best = minimum(
                StationSelection._pricing_candidate_from_label(darp_ctx, label).reduced_cost
                for label in darp_labels
            )
            @test darp_best ≈ exact_best atol=1e-9
        end
    end

    @testset "CGSolver end-to-end matches exact pricing" begin
        stations = DataFrame(
            id=[1, 2, 3],
            lon=[-71.10, -71.09, -71.08],
            lat=[42.35, 42.35, 42.35],
        )
        requests = DataFrame(
            id=[1, 2, 3],
            start_station_id=[1, 1, 2],
            end_station_id=[2, 3, 3],
            request_time=fill(DateTime(2026, 1, 1, 8, 0), 3),
        )
        walking = Dict(
            (i, j) => (i == j ? 0.0 : 100.0 * abs(i - j))
            for i in 1:3 for j in 1:3
        )
        routing = Dict(
            (i, j) => Float64(abs(i - j))
            for i in 1:3 for j in 1:3
        )
        data = create_station_selection_data(
            stations, requests, walking;
            routing_costs=routing,
            scenarios=[("2026-01-01 08:00:00", "2026-01-01 09:00:00")],
        )
        problem = StationSelectionProblem(data, 2; max_walking_distance=250.0)
        solver = CGSolver(max_iterations=100, reduced_cost_tol=1e-7)
        exact_formulation = AggregateODRouteJointRoutingAssignmentFormulation(
                pricing_mode=:exact,
                route_regularization_weight=1.0,
                walk_cost_weight=1.0,
                repositioning_time=0.0,
                max_wait_time=10.0,
                max_stops=5,
        )
        darp_formulation = AggregateODRouteJointRoutingAssignmentFormulation(
                pricing_mode=:darp,
                route_regularization_weight=1.0,
                walk_cost_weight=1.0,
                repositioning_time=0.0,
                max_wait_time=10.0,
                max_stops=5,
        )

        exact_build = StationSelection.build_model(problem, exact_formulation, solver)
        darp_build = StationSelection.build_model(problem, darp_formulation, solver)
        exact_seed_routes = sort([
            Tuple(column.route) for column in values(exact_build.model[:joint_routing_assignment_columns])
        ])
        darp_seed_routes = sort([
            Tuple(column.route) for column in values(darp_build.model[:joint_routing_assignment_columns])
        ])
        @test !isempty(exact_seed_routes)
        @test exact_seed_routes == darp_seed_routes
        @test all(length(route) == 2 for route in exact_seed_routes)

        exact_result = run_opt(problem, exact_formulation, solver)
        darp_result = run_opt(problem, darp_formulation, solver)

        @test exact_result.termination_status == JuMP.MOI.OPTIMAL
        @test darp_result.termination_status == JuMP.MOI.OPTIMAL
        @test exact_result.metadata["cg_converged"]
        @test darp_result.metadata["cg_converged"]
        @test exact_result.model[:joint_routing_assignment_pricing_mode] == :exact
        @test darp_result.model[:joint_routing_assignment_pricing_mode] == :darp
        @test !isempty(exact_result.model[:joint_routing_assignment_columns])
        @test !isempty(darp_result.model[:joint_routing_assignment_columns])
        @test isapprox(darp_result.objective_value, exact_result.objective_value; atol=1e-6)
    end
end
