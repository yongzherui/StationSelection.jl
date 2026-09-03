@testset "Relaxed-cluster certification pricer" begin
    using Random
    SS = StationSelection

    # ── shared fixtures ──────────────────────────────────────────────────────
    # Stations on a line, so cluster membership and every distance is checkable by hand.
    function line_travel_cost(n::Int)
        costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:n, j in 1:n
            i == j && continue
            costs[(i, j)] = Float64(abs(i - j))
        end
        return costs
    end

    # Two well-separated groups of three: {1,2,3} and {11,12,13} in line coordinates, so
    # k-medoids with K=2 has exactly one sensible answer.
    two_group_nodes() = [1, 2, 3, 11, 12, 13]
    function two_group_travel_cost()
        costs = Dict{Tuple{Int, Int}, Float64}()
        for i in two_group_nodes(), j in two_group_nodes()
            i == j && continue
            costs[(i, j)] = Float64(abs(i - j))
        end
        return costs
    end

    relaxed_data(clustering, travel_cost, candidates; kwargs...) =
        SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
            1, clustering, travel_cost, candidates;
            route_regularization_weight = 1.0, max_wait_time = 100.0,
            repositioning_time = 0.0, max_stops = typemax(Int), kwargs...,
        )

    exact_data(nodes, travel_cost, candidates) =
        create_joint_routing_assignment_pricing_data(
            1, nodes, travel_cost, candidates;
            route_regularization_weight = 1.0, max_wait_time = 100.0,
            repositioning_time = 0.0, max_stops = typemax(Int),
        )

    # ── clustering ───────────────────────────────────────────────────────────
    @testset "k-medoids partition is total, non-empty and deterministic" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)

        @test clustering.n_clusters == 2
        @test sort(reduce(vcat, clustering.members)) == sort(nodes)
        @test all(!isempty, clustering.members)
        @test Set(keys(clustering.cluster_of)) == Set(nodes)
        # The two natural groups, not some interleaving of them.
        @test Set(Set.(clustering.members)) == Set([Set([1, 2, 3]), Set([11, 12, 13])])
        # No RNG anywhere: the same input must give the same partition, or `n_clusters`
        # is not a reproducible swept parameter.
        @test cluster_stations_by_travel_cost(nodes, costs, 2).members == clustering.members
        @test station_cluster_sizes(clustering) == [3, 3]
    end

    @testset "K >= n is the identity partition, not an error" begin
        nodes = two_group_nodes()
        for k in (length(nodes), length(nodes) + 5)
            clustering = cluster_stations_by_travel_cost(nodes, two_group_travel_cost(), k)
            @test clustering.n_clusters == length(nodes)
            @test all(cell -> length(cell) == 1, clustering.members)
        end
        @test_throws ArgumentError cluster_stations_by_travel_cost(nodes, two_group_travel_cost(), 0)
    end

    # ── the relaxation's two defining inequalities ───────────────────────────
    @testset "cluster travel under-estimates every real arc it covers" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        cluster_travel = SS._relaxed_cluster_travel_cost(clustering, costs)

        for ((u, v), cost) in costs
            cu, cv = clustering.cluster_of[u], clustering.cluster_of[v]
            @test cluster_travel[(cu, cv)] <= cost + 1e-9
        end
        # Same-cluster travel is free, and the closest members of the two groups are
        # 3 and 11 (distance 8).
        @test cluster_travel[(1, 1)] == 0.0
        @test cluster_travel[(2, 2)] == 0.0
        @test minimum(cluster_travel[(c, d)] for c in 1:2, d in 1:2 if c != d) == 8.0
    end

    @testset "cluster travel satisfies the triangle inequality" begin
        # Not decoration: station-age pruning assumes `travel(current, dest)` is a genuine
        # lower bound on the time still needed, and a raw min-over-member-pairs matrix
        # violates that. This is the metric closure doing its job.
        nodes = [1, 2, 3, 4, 5, 6]
        # A deliberately non-metric-inducing layout: cluster A = {1,4}, B = {2,5},
        # C = {3,6}, where each cluster has one member near each other cluster.
        costs = Dict{Tuple{Int, Int}, Float64}()
        coords = Dict(1 => 0.0, 4 => 100.0, 2 => 1.0, 5 => 101.0, 3 => 200.0, 6 => 102.0)
        for i in nodes, j in nodes
            i == j && continue
            costs[(i, j)] = abs(coords[i] - coords[j])
        end
        clustering = StationClustering(
            3, nodes, Dict(1 => 1, 4 => 1, 2 => 2, 5 => 2, 3 => 3, 6 => 3),
            [[1, 4], [2, 5], [3, 6]], [1, 2, 3],
        )
        cluster_travel = SS._relaxed_cluster_travel_cost(clustering, costs)
        for c in 1:3, d in 1:3, e in 1:3
            @test cluster_travel[(c, e)] <= cluster_travel[(c, d)] + cluster_travel[(d, e)] + 1e-9
        end
    end

    @testset "relaxed reward/ride-limit are the per-passenger cluster-pair maxima" begin
        nodes = two_group_nodes()
        clustering = cluster_stations_by_travel_cost(nodes, two_group_travel_cost(), 2)
        # One passenger, three ways to serve them, all from cluster 1 to cluster 2.
        candidates = [
            PassengerAssignmentCandidate(1, 1, 11, 20.0, 3.0),
            PassengerAssignmentCandidate(1, 2, 12, 30.0, 5.0),
            PassengerAssignmentCandidate(1, 3, 13, 25.0, 4.0),
        ]
        relaxed = SS._aggregate_relaxed_cluster_candidates(clustering, candidates)
        @test length(relaxed) == 1
        @test relaxed[1].p == 1
        @test (relaxed[1].origin, relaxed[1].destination) ==
            (clustering.cluster_of[1], clustering.cluster_of[11])
        @test relaxed[1].reward == 5.0          # max over alternatives, never the sum
        @test relaxed[1].ride_limit == 30.0     # maxima taken independently
    end

    @testset "distinct passengers add, one passenger's alternatives do not" begin
        nodes = two_group_nodes()
        clustering = cluster_stations_by_travel_cost(nodes, two_group_travel_cost(), 2)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 11, 50.0, 3.0),
            PassengerAssignmentCandidate(1, 2, 12, 50.0, 5.0),
            PassengerAssignmentCandidate(2, 1, 13, 50.0, 7.0),
        ]
        data = relaxed_data(clustering, two_group_travel_cost(), candidates)
        # Passenger 1 collapses to one relaxed candidate, passenger 2 keeps its own.
        @test data.n_relaxed_candidates == 2
        # A route visiting cluster 1 then cluster 2 collects max(3,5) + 7 = 12.
        _rc, _tau, reward = SS._relaxed_cluster_route_reduced_cost([1, 2], data)
        @test reward ≈ 12.0
    end

    # ── intra-cluster rewards must not disappear ─────────────────────────────
    @testset "an entirely intra-cluster passenger is still credited" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        # Served 1 -> 3, both inside cluster 1: after clustering there is no arc left to
        # certify this on, so it has to be credited on arrival or the bound breaks.
        candidates = [PassengerAssignmentCandidate(1, 1, 3, 50.0, 9.0)]
        data = relaxed_data(clustering, costs, candidates)

        cluster_1 = clustering.cluster_of[1]
        @test haskey(data.intra_layer_mask, cluster_1)
        # Credited by a one-cluster route -- the image of the real route 1 -> 3.
        rc, tau, reward = SS._relaxed_cluster_route_reduced_cost([cluster_1], data)
        @test reward ≈ 9.0
        @test tau == 0.0
        @test rc ≈ -9.0
        # And the seed label already carries it, so the search can see it at depth 1.
        seeds = SS._initial_joint_routing_assignment_relaxed_cluster_labels(data)
        seed = only(filter(l -> l.current == cluster_1, seeds))
        @test seed.reduced_cost ≈ -9.0
        @test !isempty(seed.activated_reward_layers)
        # Intra opportunities are kept out of the branching tables (see data.jl) while
        # remaining in the ones the reward bound reads.
        @test !haskey(data.inner.assignments_by_destination, cluster_1)
        @test !haskey(data.inner.assignments_by_origin, cluster_1)
        @test haskey(data.inner.origin_layer_mask, cluster_1)
        @test any(o -> o.origin == o.destination, data.inner.opportunities)
    end

    # ── the bound itself ─────────────────────────────────────────────────────
    @testset "every real route's cluster image is at least as good" begin
        # The relaxation's whole claim, checked route by route: map each real route to its
        # cluster sequence (consecutive repeats collapsed, exactly as types.jl's argument
        # does) and compare reduced costs.
        Random.seed!(20260903)
        nodes = collect(1:8)
        costs = line_travel_cost(8)
        candidates = PassengerAssignmentCandidate[]
        for p in 1:6
            j = rand(1:8)
            k = rand(filter(!=(j), 1:8))
            push!(candidates, PassengerAssignmentCandidate(p, j, k, 4.0 + rand() * 6.0, 1.0 + rand() * 5.0))
            # A second alternative for the same passenger, so the per-passenger max
            # matters rather than being vacuous.
            j2 = rand(1:8)
            k2 = rand(filter(!=(j2), 1:8))
            push!(candidates, PassengerAssignmentCandidate(p, j2, k2, 4.0 + rand() * 6.0, 1.0 + rand() * 5.0))
        end
        exact = exact_data(nodes, costs, candidates)

        function cluster_image(route, clustering)
            image = Int[]
            for station in route
                cell = clustering.cluster_of[station]
                (isempty(image) || image[end] != cell) && push!(image, cell)
            end
            return image
        end

        for n_clusters in (2, 3, 4, 8)
            clustering = cluster_stations_by_travel_cost(nodes, costs, n_clusters)
            relaxed = relaxed_data(clustering, costs, candidates)
            for _ in 1:300
                # No consecutive repeats: a real route never revisits the station it is
                # standing on, and replaying one would ask for a nonexistent (j, j) arc.
                route = Int[rand(1:8)]
                while length(route) < rand(2:5)
                    push!(route, rand(filter(!=(route[end]), 1:8)))
                end
                _assignments, _tau, exact_rc, _positions =
                    SS._joint_routing_assignment_column_from_route(route, exact)
                relaxed_rc, _, _ =
                    SS._relaxed_cluster_route_reduced_cost(cluster_image(route, clustering), relaxed)
                @test relaxed_rc <= exact_rc + 1e-9
            end
        end
    end

    @testset "relaxed labels agree with a from-scratch replay of their own route" begin
        # The relaxed twin of the invariant `../exact/accept.jl` asserts on every accepted
        # column: incremental reward-layer accounting must equal a direct
        # passenger-by-passenger recomputation, or the search is scoring routes wrong.
        Random.seed!(4242)
        nodes = collect(1:8)
        costs = line_travel_cost(8)
        candidates = [
            PassengerAssignmentCandidate(p, rand(1:8), rand(1:8), 3.0 + rand() * 8.0, 1.0 + rand() * 4.0)
            for p in 1:8
        ]
        # Origins are allowed to equal destinations here on purpose: those become the
        # intra-cluster credits, which is exactly the path this invariant must cover.
        clustering = cluster_stations_by_travel_cost(nodes, costs, 3)
        data = relaxed_data(clustering, costs, candidates)
        ctx = SS.JointRoutingAssignmentRelaxedClusterSearchContext(data)
        labels, _exhausted, _stats = SS._run_label_setting(
            ctx; time_limit = 20.0, reduced_cost_tol = 1e-6,
        )
        @test !isempty(labels)
        for label in labels
            replayed, _tau, _reward = SS._relaxed_cluster_route_reduced_cost(label.route, data)
            @test replayed ≈ label.reduced_cost atol = 1e-6
        end
    end

    @testset "the relaxed optimum lower-bounds the exact optimum" begin
        # The bound at the level the certificate actually uses it: min over the relaxed
        # search vs. min over the exact search, on the same candidates.
        Random.seed!(99)
        nodes = collect(1:7)
        costs = line_travel_cost(7)
        candidates = PassengerAssignmentCandidate[]
        for p in 1:5, _ in 1:2
            j = rand(1:7)
            k = rand(filter(!=(j), 1:7))
            push!(candidates, PassengerAssignmentCandidate(p, j, k, 5.0 + rand() * 5.0, 1.0 + rand() * 4.0))
        end
        exact = exact_data(nodes, costs, candidates)
        exact_labels, exact_exhausted, _ = SS._run_label_setting(
            SS.JointRoutingAssignmentSearchContext(exact);
            time_limit = 30.0, reduced_cost_tol = 1e-6,
        )
        @test exact_exhausted
        exact_min = minimum(l.reduced_cost for l in exact_labels)

        for n_clusters in (2, 3, 4, 7)
            clustering = cluster_stations_by_travel_cost(nodes, costs, n_clusters)
            relaxed_labels, relaxed_exhausted, _ = SS._run_label_setting(
                SS.JointRoutingAssignmentRelaxedClusterSearchContext(
                    relaxed_data(clustering, costs, candidates),
                );
                time_limit = 30.0, reduced_cost_tol = 1e-6,
            )
            @test relaxed_exhausted
            @test minimum(l.reduced_cost for l in relaxed_labels) <= exact_min + 1e-9
        end
    end

    # ── formulation / solver wiring ──────────────────────────────────────────
    @testset "wiring rejects the configurations that cannot work" begin
        # :relaxed_cluster is not a pricing_mode -- it produces no columns.
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(
            pricing_mode = :relaxed_cluster,
        )
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(
            relaxed_cluster_count = 0,
        )
        @test_throws ArgumentError CGSolver(certification_pricing_mode = :not_a_relaxation)
        @test AggregateODRouteJointRoutingAssignmentFormulation().relaxed_cluster_count === nothing
        @test AggregateODRouteJointRoutingAssignmentFormulation(
            relaxed_cluster_count = 4,
        ).relaxed_cluster_count == 4
    end

    @testset "end to end: certify-first reaches the same certified optimum" begin
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        base = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4),
            CGSolver(),
        )
        @test base.termination_status == SOLVE_OPTIMAL
        @test base.metadata["cg_certification_pricing_mode"] === nothing
        @test base.metadata["cg_certified_by_relaxation"] === false
        @test base.metadata["cg_certification_rounds"] == 0
        @test base.metadata["cg_certification_refuted_rounds"] == 0
        @test base.metadata["cg_certification_inconclusive_rounds"] == 0
        @test all(r -> r.certification_outcome == "none", base.metadata["cg_iteration_log"])

        for n_clusters in (2, 4, data.n_stations)
            formulation = AggregateODRouteJointRoutingAssignmentFormulation(
                max_stops = 4, relaxed_cluster_count = n_clusters,
            )
            result = run_opt(problem, formulation,
                             CGSolver(certification_pricing_mode = :relaxed_cluster))
            # Whether the relaxation certifies or not, the answer must not move: a failed
            # certification only wastes a round, it never changes which columns are priced.
            @test result.termination_status == SOLVE_OPTIMAL
            @test result.objective_value ≈ base.objective_value atol = 1e-6
            @test result.metadata["cg_certification_pricing_mode"] === :relaxed_cluster
            @test result.metadata["cg_certification_rounds"] >= 1
            if result.metadata["cg_certified_by_relaxation"] === true
                @test result.metadata["cg_stop_reason"] == "converged_by_certification"
                @test result.metadata["cg_converged"] === true
                @test result.metadata["cg_optimality_scope"] == "full_route_universe"
                @test last(result.metadata["cg_iteration_log"]).certification_certified === true
                @test last(result.metadata["cg_iteration_log"]).certification_outcome == "certified"
            end
            # Every attempt is accounted for by exactly one outcome, so a sweep that never
            # certifies can still be read: `refuted` says the relaxation was too loose,
            # `inconclusive` says it ran out of budget. A miscount here would make those
            # two indistinguishable, which is the whole point of recording them.
            log = result.metadata["cg_iteration_log"]
            attempted = count(r -> r.certification_outcome != "none", log)
            @test attempted == result.metadata["cg_certification_rounds"]
            @test count(r -> r.certification_outcome == "refuted", log) ==
                result.metadata["cg_certification_refuted_rounds"]
            @test count(r -> r.certification_outcome == "inconclusive", log) ==
                result.metadata["cg_certification_inconclusive_rounds"]
            @test count(r -> r.certification_outcome == "certified", log) ==
                (result.metadata["cg_certified_by_relaxation"] === true ? 1 : 0)
            @test sum(row.certification_sec for row in result.metadata["cg_iteration_log"]) <=
                result.metadata["cg_certification_sec"] + 1e-6
        end
    end

    @testset "identity clustering certifies, and does so at the exact optimum" begin
        # K = n makes the relaxation coincide with the exact pricing graph arc for arc, so
        # it must certify exactly when exhaustive pricing would -- the sanity end of a
        # cluster-count sweep.
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        result = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(
                max_stops = 4, relaxed_cluster_count = data.n_stations,
            ),
            CGSolver(certification_pricing_mode = :relaxed_cluster),
        )
        @test result.termination_status == SOLVE_OPTIMAL
        @test result.metadata["cg_certified_by_relaxation"] === true
        @test result.metadata["cg_stop_reason"] == "converged_by_certification"
    end

    @testset "certification is refused without a clustering, never silently skipped" begin
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        @test_throws ArgumentError run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4),
            CGSolver(certification_pricing_mode = :relaxed_cluster),
        )
    end

    @testset "a relaxed context refuses to be used as a column source" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        data = relaxed_data(clustering, costs,
                            [PassengerAssignmentCandidate(1, 1, 11, 50.0, 9.0)])
        ctx = SS.JointRoutingAssignmentRelaxedClusterSearchContext(data)
        label = only(filter(l -> l.current == clustering.cluster_of[1],
                            SS._initial_joint_routing_assignment_relaxed_cluster_labels(data)))
        @test_throws ErrorException SS._pricing_candidate_from_label(ctx, label)
    end
end
