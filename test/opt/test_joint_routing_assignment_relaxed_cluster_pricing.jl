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

    # The relaxed travel matrix now lives on the built pricing data, over the AUGMENTED
    # node set (cluster nodes, then one service node per intra-capable cluster). These
    # fixtures pass only inter-cluster candidates, so no service node forms and the node
    # indices are plain cluster indices.
    relaxed_travel(clustering, travel_cost, candidates) =
        relaxed_data(clustering, travel_cost, candidates).inner.travel_cost

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
        # Each cell's medoid is the member minimizing the summed distance to its own
        # cell -- the midpoint on a line.
        @test clustering.medoids == [2, 12]
    end

    @testset "K >= n is the identity partition, not an error" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        for k in (length(nodes), length(nodes) + 5)
            clustering = cluster_stations_by_travel_cost(nodes, costs, k)
            @test clustering.n_clusters == length(nodes)
            @test all(cell -> length(cell) == 1, clustering.members)
        end
        @test_throws ArgumentError cluster_stations_by_travel_cost(nodes, costs, 0)
    end

    # ── the relaxation's two defining inequalities ───────────────────────────
    @testset "cluster travel under-estimates every real arc it covers" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        cluster_travel = relaxed_travel(clustering, costs,
            [PassengerAssignmentCandidate(1, 1, 11, 50.0, 5.0)])

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
        # The metric closure is load-bearing for VALIDITY, not tidiness, and this is the
        # shape that proves it is needed: stations on a line, so the underlying travel
        # matrix is already a perfect metric, yet `min` over member pairs destroys that as
        # soon as one cell has spread-out members.
        #
        #   A = {0}   B = {10, 50}   C = {60}
        #   min(A,B) = 10, min(B,C) = 10, min(A,C) = 60  ->  60 > 10 + 10
        #
        # Left uncorrected, station-age pruning would drop a clock at A targeting C
        # (`0 + 60 > ride_limit`) even though the relaxed route A->B->C reaches C at t=20,
        # so the search would UNDER-collect reward and could then falsely certify.
        nodes = [1, 2, 3, 4]
        coords = Dict(1 => 0.0, 2 => 10.0, 3 => 50.0, 4 => 60.0)
        costs = Dict{Tuple{Int, Int}, Float64}()
        for i in nodes, j in nodes
            i == j && continue
            costs[(i, j)] = abs(coords[i] - coords[j])
        end
        clustering = StationClustering(
            3, nodes, Dict(1 => 1, 2 => 2, 3 => 2, 4 => 3),
            [[1], [2, 3], [4]], [1, 2, 4],
        )
        cluster_travel = relaxed_travel(clustering, costs,
            [PassengerAssignmentCandidate(1, 1, 4, 500.0, 5.0)])

        # The assertion that fails without the closure: the raw min-over-member-pairs arc
        # A->C is 60, the closure routes it through B for 20.
        @test cluster_travel[(1, 3)] ≈ 20.0
        for c in 1:3, d in 1:3, e in 1:3
            @test cluster_travel[(c, e)] <= cluster_travel[(c, d)] + cluster_travel[(d, e)] + 1e-9
        end
        # And the closure must not have broken the underestimate it exists to preserve.
        for ((u, v), cost) in costs
            @test cluster_travel[(clustering.cluster_of[u], clustering.cluster_of[v])] <= cost + 1e-9
        end
    end

    @testset "relaxed reward/ride-limit are the per-passenger cluster-pair maxima" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
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
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 11, 50.0, 3.0),
            PassengerAssignmentCandidate(1, 2, 12, 50.0, 5.0),
            PassengerAssignmentCandidate(2, 1, 13, 50.0, 7.0),
        ]
        data = relaxed_data(clustering, costs, candidates)
        # Passenger 1 collapses to one relaxed candidate, passenger 2 keeps its own.
        @test data.n_relaxed_candidates == 2
        # A route visiting cluster 1 then cluster 2 collects max(3,5) + 7 = 12.
        _assignments, tau, rc, _pos =
            SS._joint_routing_assignment_column_from_route([1, 2], data.inner)
        # beta = 1, repositioning = 0 in this fixture, so reward = tau - rc.
        @test tau - rc ≈ 12.0
    end

    # ── intra-cluster rewards must not disappear ─────────────────────────────
    @testset "an intra-cluster passenger is served by a charged, optional arc" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        # Served 1 -> 3, both inside cluster 1 (= {1,2,3} on the line, so the cheapest
        # within-cell hop is 1). After clustering there is no inter-cluster arc left to
        # certify this on, so it gets its own service node and its own charge.
        candidates = [PassengerAssignmentCandidate(1, 1, 3, 50.0, 9.0)]
        data = relaxed_data(clustering, costs, candidates)
        cluster_1 = clustering.cluster_of[1]

        service = data.service_node[cluster_1]
        @test service == clustering.n_clusters + 1        # appended after the cluster nodes
        @test data.intra_travel[cluster_1] == 1.0         # min hop inside {1,2,3}
        @test SS.relaxed_cluster_of_node(data, service) == cluster_1
        @test relaxed_cluster_n_nodes(data) == clustering.n_clusters + 1

        # The candidate is now an ORDINARY (p, C, C') opportunity -- no special casing.
        opp = only(data.inner.opportunities)
        @test (opp.origin, opp.destination) == (cluster_1, service)

        # Merely visiting the cluster earns nothing: the arc is optional and unpaid.
        assignments, _tau, rc, _pos =
            SS._joint_routing_assignment_column_from_route([cluster_1], data.inner)
        @test isempty(assignments)
        @test rc == 0.0

        # Taking the arc costs tau_intra and pays the reward. This is the tightening: the
        # earlier credit-on-arrival encoding gave the same 9.0 for free, i.e. rc = -9.
        assignments, tau, rc, _pos =
            SS._joint_routing_assignment_column_from_route([cluster_1, service], data.inner)
        @test length(assignments) == 1
        @test tau ≈ 1.0
        @test rc ≈ -8.0
        @test rc > -9.0 + 1e-9        # strictly tighter than the free-intra encoding
    end

    @testset "a singleton cluster gets no service arc" begin
        # tau_intra needs two distinct members, and a one-station cell can never hold an
        # intra-cluster passenger anyway -- so it must not acquire a node.
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, length(nodes))  # identity
        candidates = [PassengerAssignmentCandidate(1, 1, 11, 50.0, 9.0)]
        data = relaxed_data(clustering, costs, candidates)
        @test isempty(data.service_node)
        @test isempty(data.intra_travel)
        @test relaxed_cluster_n_nodes(data) == clustering.n_clusters
    end

    # ── resource parity with the exact pricer ───────────────────────────────
    @testset "the relaxed graph carries the exact pricer's resources verbatim" begin
        # The relaxed pricer IS the exact pricer (see types.jl), so the resources cannot
        # diverge by implementation -- only by a parameter being dropped or defaulted on
        # the way in. This pins that: every scalar the exact pricing data carries must
        # come back identical on the relaxed one.
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 11, 50.0, 9.0),
            PassengerAssignmentCandidate(2, 12, 2, 40.0, 4.0),
        ]
        exact = exact_data(nodes, costs, candidates)
        relaxed = relaxed_data(clustering, costs, candidates).inner
        for field in (:route_regularization_weight, :repositioning_time, :max_wait_time,
                      :max_stops, :bounded_max_stops, :compensated_dominance, :scenario)
            @test getfield(relaxed, field) == getfield(exact, field)
        end
    end

    @testset "ride limits are live on the relaxed graph" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        # Cluster 1 = {1,2,3}, cluster 2 = {11,12,13}, cheapest crossing is 3->11 = 8.
        # p=1 has a generous limit, p=2 a limit of 1.0 that no relaxed crossing can meet.
        candidates = [
            PassengerAssignmentCandidate(1, 1, 11, 50.0, 9.0),
            PassengerAssignmentCandidate(2, 1, 11, 1.0, 100.0),
        ]
        data = relaxed_data(clustering, costs, candidates)
        assignments, tau, rc, _pos =
            SS._joint_routing_assignment_column_from_route([1, 2], data.inner)
        @test tau ≈ 8.0
        # Only p=1 is certified; p=2's 100.0 reward is unreachable inside its ride limit,
        # so it must NOT be collected -- a relaxation that ignored ride limits would take
        # it and report rc = 8 - 109.
        @test [p for (p, _j, _k) in assignments] == [1]
        @test rc ≈ 8.0 - 9.0
    end

    @testset "the wait-time window is live on the relaxed graph" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        candidates = [PassengerAssignmentCandidate(1, 11, 1, 50.0, 9.0)]
        # A window of 1.0 closes before the vehicle can reach cluster 2 (crossing = 8), so
        # the pickup clock there never opens and the reward is unreachable.
        tight = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
            1, clustering, costs, candidates;
            route_regularization_weight = 1.0, max_wait_time = 1.0,
            repositioning_time = 0.0, max_stops = typemax(Int),
        )
        assignments, _tau, _rc, _pos =
            SS._joint_routing_assignment_column_from_route([1, 2, 1], tight.inner)
        @test isempty(assignments)
        # With a window that admits the crossing, the same route does collect it.
        loose = relaxed_data(clustering, costs, candidates)
        assignments, _tau, _rc, _pos =
            SS._joint_routing_assignment_column_from_route([1, 2, 1], loose.inner)
        @test length(assignments) == 1
    end

    @testset "one passenger's intra and inter alternatives are a max, not a sum" begin
        # The case the service-node encoding newly creates: passenger 1 can be served
        # INSIDE cluster 1 (via the service arc) or ACROSS to cluster 2. Those are
        # alternatives for the same passenger, so a route doing both must bank the better
        # one only -- which is precisely why the intra candidate has to live in the
        # passenger's own reward ladder rather than in a side table.
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 50.0, 9.0),    # intra, worth more
            PassengerAssignmentCandidate(1, 1, 11, 50.0, 5.0),   # inter, worth less
        ]
        data = relaxed_data(clustering, costs, candidates)
        cluster_1, cluster_2 = clustering.cluster_of[1], clustering.cluster_of[11]
        service = data.service_node[cluster_1]

        # Take the service arc and then cross: 1 + 8 of travel, and max(9, 5) of reward.
        _assignments, tau, rc, _pos = SS._joint_routing_assignment_column_from_route(
            [cluster_1, service, cluster_2], data.inner,
        )
        @test tau ≈ 9.0
        @test tau - rc ≈ 9.0        # NOT 14.0
        # Crossing without the service arc banks only the inter alternative.
        _assignments, tau, rc, _pos =
            SS._joint_routing_assignment_column_from_route([cluster_1, cluster_2], data.inner)
        @test tau ≈ 8.0
        @test tau - rc ≈ 5.0
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

        # The image of a real route: its cluster sequence with consecutive repeats
        # collapsed into blocks, PLUS the cluster's intra service node after any block that
        # touched two or more stations. That block paid at least the cheapest within-cell
        # hop, so its image may take the arc that costs exactly that -- which is how an
        # intra-cluster passenger's reward survives the collapse now that it is no longer
        # granted for free.
        function cluster_image(route, data)
            image = Int[]
            current_cluster = 0
            block = Set{Int}()
            function flush_block!()
                if current_cluster != 0 && length(block) >= 2
                    service = get(data.service_node, current_cluster, nothing)
                    isnothing(service) || push!(image, service)
                end
                empty!(block)
            end
            for station in route
                cell = data.clustering.cluster_of[station]
                if cell != current_cluster
                    flush_block!()
                    push!(image, cell)
                    current_cluster = cell
                end
                push!(block, station)
            end
            flush_block!()
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
                _a, _t, relaxed_rc, _p = SS._joint_routing_assignment_column_from_route(
                    cluster_image(route, relaxed), relaxed.inner,
                )
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
        ctx = SS.JointRoutingAssignmentSearchContext(data.inner)
        labels, _exhausted, _stats = SS._run_label_setting(
            ctx; time_limit = 20.0, reduced_cost_tol = 1e-6,
        )
        @test !isempty(labels)
        for label in labels
            _assignments, _tau, replayed, _pos =
                SS._joint_routing_assignment_column_from_route(label.route, data.inner)
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
                SS.JointRoutingAssignmentSearchContext(
                    relaxed_data(clustering, costs, candidates).inner,
                );
                time_limit = 30.0, reduced_cost_tol = 1e-6,
            )
            @test relaxed_exhausted
            @test minimum(l.reduced_cost for l in relaxed_labels) <= exact_min + 1e-9
        end
    end

    # ── the lower bound, randomized against ground truth ────────────────────
    @testset "randomized: the relaxed pricer lower-bounds the exact pricer" begin
        # The claim the whole certificate rests on, tested as a property over random
        # instances rather than on one hand-built case:
        #
        #     min over relaxed cluster routes  <=  min over real station routes
        #
        # Both sides are computed by **exhaustive brute-force enumeration** of every route
        # up to the stop cap, replayed from scratch. Nothing here trusts a label search, a
        # dominance rule, or a pruning bound: if the relaxation were unsound as
        # mathematics, this is what would catch it. The label search is then checked
        # separately against its own brute-force optimum, because a correct relaxation the
        # pricer fails to actually find is just as useless as an incorrect one.
        #
        # Three things the earlier deterministic tests do NOT exercise, all deliberately
        # forced here:
        #
        #   * **spread-member clusters.** The line fixtures above give interval cells,
        #     where min-over-member-pairs happens to already be metric, so the metric
        #     closure is inert and the tests would pass without it. The `interleaved`
        #     partition below scatters each cell across the whole map, which is exactly
        #     the shape that breaks the triangle inequality (`n_nonmetric` asserts this
        #     really happened).
        #   * **near-zero travel slack.** MEASURED, not assumed: an earlier version of
        #     this test used only well-separated random points, and mutating the reward
        #     aggregation from `max` to `min` -- which makes the relaxation formally
        #     unsound -- did NOT fail it. With generous inter-cluster travel slack the
        #     bound survives numerically even when the reward side is wrong, so the
        #     reward direction was never being tested. The `:tight` layout below packs
        #     each cell into a small blob far from the others, leaving almost no travel
        #     slack, so the per-passenger reward maximum becomes load-bearing and that
        #     mutation now fails. `n_reward_discriminating` asserts the shape occurred.
        #   * **a binding pickup window.** `max_wait_time` is set below the longest arc,
        #     so clocks genuinely expire -- the regime where the relaxation's "relaxed
        #     times are smaller, so clocks open more easily" argument has to hold.
        #   * **a binding stop cap.** `max_stops` of 3-4 makes `bounded_max_stops` true,
        #     which selects a different dominance specialization AND is what makes the
        #     brute force finite.
        Random.seed!(20260904)

        "Every route of 1..max_stops stops with no consecutive repeat, cheapest first."
        function brute_force_min_reduced_cost(route_nodes, max_stops, reduced_cost_of)
            best = Inf
            frontier = [[node] for node in route_nodes]
            while !isempty(frontier)
                next_frontier = Vector{Int}[]
                for route in frontier
                    best = min(best, reduced_cost_of(route))
                    length(route) < max_stops || continue
                    for node in route_nodes
                        node == route[end] && continue
                        push!(next_frontier, vcat(route, node))
                    end
                end
                frontier = next_frontier
            end
            return best
        end

        "Round-robin partition: every cell gets members from all over the map."
        function interleaved_clustering(nodes, n_clusters)
            cluster_of = Dict(node => (i - 1) % n_clusters + 1 for (i, node) in enumerate(nodes))
            members = [sort!([node for node in nodes if cluster_of[node] == c])
                       for c in 1:n_clusters]
            return StationClustering(n_clusters, collect(nodes), cluster_of, members,
                                     [first(cell) for cell in members])
        end

        "Does the raw min-over-member-pairs matrix (pre-closure) violate the triangle inequality?"
        function raw_cluster_matrix_is_nonmetric(clustering, costs)
            k = clustering.n_clusters
            raw = fill(Inf, k, k)
            for c in 1:k
                raw[c, c] = 0.0
            end
            for ((u, v), cost) in costs
                cu, cv = clustering.cluster_of[u], clustering.cluster_of[v]
                cu == cv && continue
                cost < raw[cu, cv] && (raw[cu, cv] = cost)
            end
            for a in 1:k, b in 1:k, c in 1:k
                raw[a, c] > raw[a, b] + raw[b, c] + 1e-9 && return true
            end
            return false
        end

        n_nonmetric = 0        # trials where the closure was load-bearing
        n_certified = 0        # trials where the relaxation found nothing improving
        n_improving = 0        # trials where an improving REAL route existed
        n_strictly_looser = 0  # trials where the relaxed optimum is strictly below the exact one
        n_reward_discriminating = 0  # cluster pairs carrying >1 distinct reward for one passenger

        for trial in 1:12
            # Two layouts. `:spread` scatters stations uniformly -- generous travel slack,
            # the regime that exercises the metric closure. `:tight` packs them into
            # far-apart blobs so min-over-member-pairs is almost the real arc, leaving the
            # reward maximum as the only thing holding the bound up.
            layout = trial % 2 == 0 ? :tight : :spread
            n_stations = rand(6:8)
            nodes = collect(1:n_stations)
            n_groups = rand(2:3)
            group_of = [(i - 1) % n_groups + 1 for i in 1:n_stations]

            # 2-D Euclidean positions, so the STATION travel matrix is a genuine metric --
            # any non-metricity downstream is created by the clustering, not inherited.
            if layout == :tight
                centres = [(120.0 * g, 45.0 * g) for g in 1:n_groups]
                xs = [centres[group_of[i]][1] + rand() - 0.5 for i in 1:n_stations]
                ys = [centres[group_of[i]][2] + rand() - 0.5 for i in 1:n_stations]
            else
                xs, ys = rand(n_stations) .* 100, rand(n_stations) .* 100
            end
            costs = Dict{Tuple{Int, Int}, Float64}()
            for i in nodes, j in nodes
                i == j && continue
                costs[(i, j)] = hypot(xs[i] - xs[j], ys[i] - ys[j])
            end
            longest_arc = maximum(values(costs))

            # Reward scale: the small one leaves nothing improving (so the relaxation gets
            # the chance to certify), the large one guarantees improving real routes exist
            # (so the bound is exercised where it is non-trivial). Cycled on a different
            # period from `layout` so every combination of the two occurs.
            reward_scale = trial % 3 == 0 ? 0.25 : 3.0
            candidates = PassengerAssignmentCandidate[]
            if layout == :tight
                # Deliberately give each passenger TWO alternatives inside the same group
                # pair, with different rewards -- the exact shape where `rho_bar` being a
                # maximum rather than any other aggregate is what keeps the bound valid.
                for p in 1:rand(3:5)
                    go, gd = 1, 1
                    while go == gd
                        go, gd = rand(1:n_groups), rand(1:n_groups)
                    end
                    origins = [i for i in nodes if group_of[i] == go]
                    dests = [i for i in nodes if group_of[i] == gd]
                    (length(origins) >= 2 && length(dests) >= 2) || continue
                    js, ks = origins[1:2], dests[1:2]
                    for (slot, factor) in enumerate((0.3, 1.0))
                        j, k = js[slot], ks[slot]
                        push!(candidates, PassengerAssignmentCandidate(
                            p, j, k, costs[(j, k)] * (1.0 + rand()),
                            reward_scale * factor * (5.0 + rand() * 25.0),
                        ))
                    end
                    n_reward_discriminating += 1
                end
            else
                for p in 1:rand(3:5), _ in 1:2
                    j = rand(nodes)
                    k = rand(filter(!=(j), nodes))
                    push!(candidates, PassengerAssignmentCandidate(
                        p, j, k, costs[(j, k)] * (1.0 + rand()),      # ride limit
                        reward_scale * (5.0 + rand() * 25.0),          # reward
                    ))
                end
            end
            isempty(candidates) && continue

            max_stops = rand(3:4)
            max_wait = longest_arc * (0.3 + 0.4 * rand())
            @test max_wait < longest_arc          # the pickup window really does bind
            compensated = isodd(trial)
            shared = (route_regularization_weight = 1.0, max_wait_time = max_wait,
                      repositioning_time = 5.0, max_stops = max_stops,
                      compensated_dominance = compensated)

            exact = create_joint_routing_assignment_pricing_data(1, nodes, costs, candidates; shared...)
            exact_bf = brute_force_min_reduced_cost(nodes, max_stops, route ->
                SS._joint_routing_assignment_column_from_route(route, exact)[3])
            exact_bf < -1e-6 && (n_improving += 1)

            # The group-aligned partition is what makes the `:tight` layout bite: its cells
            # are exactly the blobs, so inter-cluster travel is essentially the real arc.
            aligned_members = [sort!([i for i in nodes if group_of[i] == g]) for g in 1:n_groups]
            aligned = StationClustering(
                n_groups, nodes, Dict(i => group_of[i] for i in nodes),
                aligned_members, [first(cell) for cell in aligned_members],
            )
            n_clusters = rand(2:max(2, n_stations - 2))
            for clustering in (cluster_stations_by_travel_cost(nodes, costs, n_clusters),
                               interleaved_clustering(nodes, n_clusters),
                               aligned)
                raw_cluster_matrix_is_nonmetric(clustering, costs) && (n_nonmetric += 1)
                relaxed = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
                    1, clustering, costs, candidates; shared...,
                )
                # A service arc must always be traversable inside the ride limits of the
                # passengers it serves, or the intra reward is unreachable and the bound
                # silently loses it. Holds because tau_intra(C) <= tau(j_p,k_p) <= R_pjk
                # <= R_bar whenever detour_factor >= 1 -- checked, not assumed.
                for opp in relaxed.inner.opportunities
                    haskey(relaxed.service_node, opp.origin) || continue
                    opp.destination == relaxed.service_node[opp.origin] || continue
                    @test relaxed.intra_travel[opp.origin] <= opp.ride_limit + 1e-9
                end
                # The AUGMENTED node set: brute force must be free to take service arcs,
                # or it misses every intra-cluster reward and understates the relaxation.
                cluster_nodes = relaxed.inner.nodes

                # (1) THE BOUND, ground truth on both sides -- no label search involved.
                relaxed_bf = brute_force_min_reduced_cost(cluster_nodes, max_stops, route ->
                    SS._joint_routing_assignment_column_from_route(route, relaxed.inner)[3])
                @test relaxed_bf <= exact_bf + 1e-9
                relaxed_bf < exact_bf - 1e-6 && (n_strictly_looser += 1)

                # (2) The label search finds that optimum. Pruning is switched OFF so the
                # reported minimum is the true one even when it sits above the tolerance
                # (with pruning on, a label whose completions can never beat -tol is not
                # extended, which is sound for the certificate but not for comparing
                # minima). The stop cap is what keeps this finite.
                labels, exhausted, _ = SS._run_label_setting(
                    SS.JointRoutingAssignmentSearchContext(relaxed.inner);
                    time_limit = 60.0, reduced_cost_tol = 1e-6,
                    use_reduced_cost_pruning = false,
                )
                @test exhausted
                relaxed_search = isempty(labels) ? Inf : minimum(l.reduced_cost for l in labels)
                # A reward-free route has reduced cost >= 0, and the search only tracks
                # reward-carrying labels, so the two agree whenever anything is negative.
                @test min(relaxed_search, 0.0) <= relaxed_bf + 1e-6
                relaxed_bf < -1e-6 && @test relaxed_search ≈ relaxed_bf atol = 1e-6

                # (3) The operational corollary: a certificate is never wrong. If the
                # relaxation says no cluster route prices below the tolerance, then no
                # REAL route does either -- which is exactly what CG stops on.
                if relaxed_search >= -1e-6
                    @test exact_bf >= -1e-6
                    n_certified += 1
                end
            end
        end

        # Non-vacuity: each of these guards a way the test could pass while proving
        # nothing. Without them a bug that made every instance trivial would look green.
        @test n_nonmetric > 0     # the metric closure was actually load-bearing somewhere
        @test n_improving > 0     # improving real routes existed, so the bound was non-trivial
        @test n_certified > 0     # the certifying branch was reached, not just the failing one
        @test n_strictly_looser > 0  # the relaxation is a genuine relaxation, not an identity
        # One passenger, two rewards, one cluster pair: without this shape the per-passenger
        # maximum is never exercised and the test cannot see a wrong reward aggregate.
        @test n_reward_discriminating > 0
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

    # ── combinatorial-no-good certification ─────────────────────────────────
    @testset "no-good cuts: the mask compiles and the right routes survive" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        candidates = [PassengerAssignmentCandidate(1, 1, 11, 50.0, 9.0)]
        data = relaxed_data(clustering, costs, candidates)

        # Cut T = {1}: every node of cluster 1 leaves the bit clear, every other node sets
        # it -- so only a route that visits something outside cluster 1 can satisfy it.
        cuts = SS._relaxed_cluster_cuts(data, [Set([1])])
        @test cuts.all_satisfied == UInt64(1)
        @test cuts.node_mask[1] == UInt64(0)      # cluster 1 itself: does not satisfy
        @test cuts.node_mask[2] == UInt64(1)      # cluster 2: does
        # Two cuts occupy two bits.
        cuts2 = SS._relaxed_cluster_cuts(data, [Set([1]), Set([2])])
        @test cuts2.all_satisfied == UInt64(3)
        @test cuts2.node_mask[1] == UInt64(2)     # outside {2} only
        @test cuts2.node_mask[2] == UInt64(1)     # outside {1} only
        @test_throws ArgumentError SS._relaxed_cluster_cuts(
            data, [Set([1]) for _ in 1:(SS.RELAXED_CLUSTER_MAX_CUTS + 1)])
    end

    @testset "a cut removes exactly the routes confined to its cluster set" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)
        # One intra-cluster passenger inside cluster 1: it is served by a route that never
        # leaves cluster 1, which is precisely what a cut on {1} must remove.
        candidates = [PassengerAssignmentCandidate(1, 1, 3, 50.0, 9.0)]
        data = relaxed_data(clustering, costs, candidates)

        uncut = SS._run_label_setting(
            SS.RelaxedClusterCutSearchContext(data, Set{Int}[]);
            time_limit = 20.0, reduced_cost_tol = 1e-6,
        )[1]
        @test any(l -> l.reduced_cost < -1e-6, uncut)

        cut = SS._run_label_setting(
            SS.RelaxedClusterCutSearchContext(data, [Set([1])]);
            time_limit = 20.0, reduced_cost_tol = 1e-6,
        )[1]
        # Nothing survives: the only improving route lives entirely inside cluster 1.
        @test isempty(filter(l -> l.reduced_cost < -1e-6, cut))
        # And a cut on the OTHER cluster leaves it alone -- the cut must be specific.
        other = SS._run_label_setting(
            SS.RelaxedClusterCutSearchContext(data, [Set([2])]);
            time_limit = 20.0, reduced_cost_tol = 1e-6,
        )[1]
        @test any(l -> l.reduced_cost < -1e-6, other)
    end

    @testset "the no-good loop is wired and validated" begin
        @test_throws ArgumentError CGSolver(certification_max_rounds = 0)
        @test CGSolver(certification_pricing_mode = :relaxed_cluster_nogood,
                       certification_max_rounds = 4).certification_max_rounds == 4
        @test_throws ArgumentError CGSolver(certification_pricing_mode = :not_a_relaxation)
    end

    @testset "randomized: the cut search equals brute force over cut-satisfying routes" begin
        # The decisive test for the cut machinery, and the analogue of the randomized bound
        # test: compare the cut-aware label search against EXHAUSTIVE ENUMERATION filtered
        # by the cut predicate directly. Nothing here trusts the search's state key, its
        # signature, or its dominance.
        #
        # The failure this exists to catch is the worst one the loop can produce. The
        # satisfied-cuts mask must be part of BOTH the dominance state and the
        # best-so-far signature; if it is not, `best_by_signature` can discard a
        # cut-satisfying label in favour of a cut-violating one carrying the same reward
        # layers. The search then under-reports, the loop sees "nothing improving
        # survives", and issues a FALSE CERTIFICATE. Brute force cannot be fooled that way.
        #
        # The cut sequence mimics the real loop rather than using random subsets: cut the
        # support of the current best route, search again, repeat.
        Random.seed!(20260905)
        tol = 1e-6

        # Brute force starts only at opportunity ORIGINS, matching the search's seeding,
        # and that restriction is legitimate rather than a concession to the
        # implementation. What the loop needs is: if a real improving route R exists, the
        # search finds SOME improving cut-satisfying relaxed route. Truncate R to start at
        # its first opportunity origin -- that drops a reward-free prefix, so the
        # truncation is still improving. It is also still un-confined to every cut set T,
        # because a truncation confined to stations(T) would have been found improving by
        # the exhaustive exact search that made T a cut in the first place. So the
        # truncation's image is origin-seeded, cut-satisfying and improving. Routes that
        # can only be reached from a non-origin start are therefore never needed.
        function brute_force_cut_min(nodes, max_stops, node_clusters, cut_sets, rc_of, origins)
            satisfies(route) = all(
                any(!in(node_clusters[v], T) for v in route) for T in cut_sets)
            best = Inf
            frontier = [[v] for v in nodes if v in origins]
            while !isempty(frontier)
                next_frontier = Vector{Int}[]
                for route in frontier
                    satisfies(route) && (best = min(best, rc_of(route)))
                    length(route) < max_stops || continue
                    for v in nodes
                        v == route[end] && continue
                        push!(next_frontier, vcat(route, v))
                    end
                end
                frontier = next_frontier
            end
            return best
        end

        n_matched = 0        # rounds where an improving cut-satisfying route existed
        n_exhausted_by_cuts = 0   # rounds where the cuts killed everything improving
        for trial in 1:10
            n_stations = rand(6:8)
            nodes = collect(1:n_stations)
            xs, ys = rand(n_stations) .* 100, rand(n_stations) .* 100
            costs = Dict{Tuple{Int, Int}, Float64}()
            for i in nodes, j in nodes
                i == j && continue
                costs[(i, j)] = hypot(xs[i] - xs[j], ys[i] - ys[j])
            end
            candidates = PassengerAssignmentCandidate[]
            for p in 1:rand(3:5), _ in 1:2
                j = rand(nodes)
                k = rand(filter(!=(j), nodes))
                push!(candidates, PassengerAssignmentCandidate(
                    p, j, k, costs[(j, k)] * (1.0 + rand()), 3.0 * (5.0 + rand() * 25.0)))
            end
            max_stops = rand(3:4)
            clustering = cluster_stations_by_travel_cost(nodes, costs, rand(2:4))
            relaxed = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
                1, clustering, costs, candidates;
                route_regularization_weight = 1.0,
                max_wait_time = maximum(values(costs)) * 0.6,
                repositioning_time = 5.0, max_stops = max_stops,
                compensated_dominance = isodd(trial),
            )
            isempty(relaxed.inner.opportunities) && continue
            node_clusters = SS._relaxed_cluster_node_clusters(relaxed)
            rc_of(route) =
                SS._joint_routing_assignment_column_from_route(route, relaxed.inner)[3]

            cut_sets = Set{Int}[]
            for _round in 1:5
                # Pruning OFF so the reported minimum is the true one, not merely correct
                # below the tolerance.
                labels, exhausted, _ = SS._run_label_setting(
                    SS.RelaxedClusterCutSearchContext(relaxed, cut_sets);
                    time_limit = 60.0, reduced_cost_tol = tol,
                    use_reduced_cost_pruning = false,
                )
                @test exhausted
                search_min = isempty(labels) ? Inf :
                    minimum(l.reduced_cost for l in labels)
                origins = Set(o.origin for o in relaxed.inner.opportunities)
                bf_min = brute_force_cut_min(
                    relaxed.inner.nodes, max_stops, node_clusters, cut_sets, rc_of, origins)

                if bf_min < -tol
                    # Ground truth says an improving cut-satisfying route exists; the
                    # search must find it, and find exactly it.
                    @test search_min ≈ bf_min atol = 1e-6
                    n_matched += 1
                else
                    # Ground truth says the cuts have removed everything improving. The
                    # search must agree -- reporting one here would be a false REFUTATION,
                    # and failing to report one when bf_min < -tol (above) would be the
                    # false CERTIFICATE. Both directions are pinned.
                    @test search_min >= -tol - 1e-9
                    n_exhausted_by_cuts += 1
                    break
                end

                best = argmin(l -> l.reduced_cost,
                              filter(l -> l.reduced_cost < -tol, labels))
                push!(cut_sets, Set{Int}(node_clusters[v] for v in best.route))
            end
        end
        # Non-vacuity: both branches must actually have been taken, or the test proves
        # nothing about the case it was written for.
        @test n_matched > 0
        @test n_exhausted_by_cuts > 0
    end

    @testset "the bound moves monotonically as cuts are added" begin
        # Each cut only ever REMOVES relaxed routes, so the minimum over the survivors can
        # only rise. A dip in the trace means the search found a route an earlier cut
        # should already have excluded -- i.e. the cut machinery is broken -- so this is a
        # property test, not a smoke test.
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        result = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(
                max_stops = 4, relaxed_cluster_count = 3,
            ),
            CGSolver(certification_pricing_mode = :relaxed_cluster_nogood),
        )
        stats = result.metadata["cg_relaxed_cluster_guide_stats"]
        @test !isempty(stats)
        n_multi_round = 0
        for row in stats
            trace = row.nogood_rc_trace
            @test !isempty(trace)
            length(trace) > 1 && (n_multi_round += 1)
            for i in 2:length(trace)
                @test trace[i] >= trace[i - 1] - 1e-6
            end
            # A round that cut is a round whose subset search found nothing real; a round
            # that stopped found something. Either way the traces line up in length.
            @test length(row.nogood_subset_rc_trace) == length(trace)
            @test length(row.nogood_subset_size_trace) == length(trace)
            @test all(sz -> 0 <= sz <= row.n_stations, row.nogood_subset_size_trace)
        end
        # The invariant is vacuous if every attempt ended in one round, so require that at
        # least one attempt actually added a cut and searched again.
        @test n_multi_round > 0
    end

    @testset "no-good certification reaches the same optimum as plain CG" begin
        # The loop may only ever end in a TRUE certificate: every cut it adds removes a
        # cluster support an exhaustive exact search already found barren, so no real
        # improving route's image is ever cut off. If that reasoning were wrong, CG would
        # stop early here and the objective would move.
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        base = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4),
            CGSolver(recover_integer_solution = true),
        )
        @test base.termination_status == SOLVE_OPTIMAL

        for n_clusters in (2, 4, data.n_stations)
            result = run_opt(
                problem,
                AggregateODRouteJointRoutingAssignmentFormulation(
                    max_stops = 4, relaxed_cluster_count = n_clusters,
                ),
                CGSolver(recover_integer_solution = true,
                         certification_pricing_mode = :relaxed_cluster_nogood),
            )
            @test result.termination_status == SOLVE_OPTIMAL
            @test result.objective_value ≈ base.objective_value atol = 1e-6
            @test result.metadata["cg_certification_pricing_mode"] === :relaxed_cluster_nogood
            if result.metadata["cg_certified_by_relaxation"] === true
                @test result.metadata["cg_stop_reason"] == "converged_by_certification"
                @test result.metadata["cg_optimality_scope"] == "full_route_universe"
            end
        end
    end

    @testset "a refuted attempt harvests its columns instead of discarding them" begin
        # A refuted attempt has just run the REAL exact pricer over `stations(T)`, so the
        # improving labels it found are ordinary columns. They are handed to the master and
        # the regular pricing round is skipped for that iteration -- which is the whole
        # speedup, and it must not cost anything in correctness.
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        base = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4),
            CGSolver(recover_integer_solution = true),
        )
        # K = 2 is coarse enough that the relaxation is loose and gets refuted repeatedly,
        # which is exactly the path that harvests.
        result = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(
                max_stops = 4, relaxed_cluster_count = 2,
            ),
            CGSolver(recover_integer_solution = true,
                     certification_pricing_mode = :relaxed_cluster_nogood),
        )
        harvested = result.metadata["cg_certification_harvested_columns"]
        @test harvested isa Int
        @test harvested >= 0
        # The counter must be live, not vestigial: if this instance ever stops refuting,
        # the assertion below is the thing that flags that the path is no longer covered.
        @test result.metadata["cg_certification_refuted_rounds"] == 0 || harvested > 0
        # Harvesting changes only WHERE columns come from, never the answer.
        @test result.termination_status == SOLVE_OPTIMAL
        @test result.objective_value ≈ base.objective_value atol = 1e-6
        # Harvested columns come from a station SUBSET, so they must never be the reason
        # CG claims optimality -- that claim still has to come from a full-universe
        # certificate or an exhausted full-universe pricing round.
        @test result.metadata["cg_optimality_scope"] == "full_route_universe"
    end

    @testset "the plain relaxed-cluster round never harvests" begin
        # Its searches run on the cluster graph, whose routes are not real routes and can
        # never become columns. The shared result struct defaults the field to empty, and
        # this pins that the plain round cannot start emitting columns by accident.
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        result = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(
                max_stops = 4, relaxed_cluster_count = 2,
            ),
            CGSolver(recover_integer_solution = true,
                     certification_pricing_mode = :relaxed_cluster),
        )
        @test result.metadata["cg_certification_harvested_columns"] == 0
        @test result.termination_status == SOLVE_OPTIMAL
    end

    # ── the guided mode: relaxation as a station-subset guide ───────────────
    @testset "relaxation-guided pricing: subset extraction" begin
        nodes = two_group_nodes()
        costs = two_group_travel_cost()
        clustering = cluster_stations_by_travel_cost(nodes, costs, 2)

        # A cluster route is read back as the union of its cells' members.
        @test relaxed_cluster_station_subset(clustering, [[1]]) == [1, 2, 3]
        @test relaxed_cluster_station_subset(clustering, [[2]]) == [11, 12, 13]
        @test relaxed_cluster_station_subset(clustering, [[1, 2]]) == sort(nodes)
        # Several routes union together, and a repeated cell contributes once.
        @test relaxed_cluster_station_subset(clustering, [[1, 2], [2, 1, 2]]) == sort(nodes)
        @test relaxed_cluster_station_subset(clustering, [[1], [1]]) == [1, 2, 3]
        @test_throws ArgumentError relaxed_cluster_station_subset(clustering, [[3]])

        # Only candidates with BOTH endpoints in the subset survive -- a route confined to
        # the subset can never certify one that leaves it.
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 50.0, 9.0),    # inside cluster 1
            PassengerAssignmentCandidate(2, 1, 11, 50.0, 7.0),   # crosses out
            PassengerAssignmentCandidate(3, 12, 13, 50.0, 5.0),  # inside cluster 2
        ]
        kept = SS._restrict_candidates_to_subset(candidates, [1, 2, 3])
        @test [c.p for c in kept] == [1]
        @test only(kept).reward == 9.0   # rewards pass through untouched
    end

    @testset "relaxation-guided pricing is wired and validated" begin
        @test AggregateODRouteJointRoutingAssignmentFormulation(
            pricing_mode = :relaxed_cluster_guided, relaxed_cluster_count = 3,
        ).pricing_mode === :relaxed_cluster_guided
        # The guide needs a partition to guide it.
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(
            pricing_mode = :relaxed_cluster_guided,
        )
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(
            relaxed_cluster_count = 3, relaxed_cluster_guide_routes = 0,
        )
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(
            relaxed_cluster_count = 3, relaxed_cluster_guide_time_limit_sec = 0.0,
        )
        # Restricting the station set restricts the route universe, so it cannot certify --
        # the scope has to say so, exactly as :station_simple's does.
        @test SS._cg_pricing_universe_is_restricted(:relaxed_cluster_guided) === true
        @test SS._cg_optimality_scope(:relaxed_cluster_guided) ==
            "relaxed_cluster_station_subset_only"
    end

    @testset "guided pricing produces REAL columns and a valid solution" begin
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        exact = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4),
            CGSolver(recover_integer_solution = true),
        )
        @test exact.termination_status == SOLVE_OPTIMAL

        for n_clusters in (3, 5, data.n_stations)
            guided = run_opt(
                problem,
                AggregateODRouteJointRoutingAssignmentFormulation(
                    max_stops = 4, pricing_mode = :relaxed_cluster_guided,
                    relaxed_cluster_count = n_clusters,
                ),
                CGSolver(recover_integer_solution = true),
            )
            # Every column it found is a genuine route over genuine stations -- the master's
            # own dual cross-check (`_pricing_verify_column`) runs on each one and would
            # have errored otherwise, so reaching a solution at all is the assertion.
            @test guided.objective_value !== nothing
            # Restricting stations can only lose columns, never invent better ones, so the
            # guided optimum is never BETTER than the unrestricted one.
            @test guided.objective_value >= exact.objective_value - 1e-6
            @test guided.metadata["cg_optimality_scope"] ==
                "relaxed_cluster_station_subset_only"
            @test guided.metadata["cg_pricing_universe_restricted"] === true

            # The guide recorded what it actually handed the exact pricer. Read from the
            # METADATA, not the model: `recover_integer_solution` rebuilds the model, so
            # `result.model` is the recovery model and its stats vector is empty.
            stats = guided.metadata["cg_relaxed_cluster_guide_stats"]
            @test !isempty(stats)
            @test all(r -> 0 <= r.subset_size <= r.n_stations, stats)
            # At K = n every cell is a singleton, so the subset is exactly the stations of
            # the winning cluster routes -- never the whole instance unless the routes
            # really do span it.
            @test all(r -> r.n_stations == data.n_stations, stats)
        end
    end

    @testset "guided pricing warm-starts into a real certificate" begin
        # The intended production shape: harvest cheaply on station subsets, then hand off
        # to the unrestricted pricer, which is the phase that certifies. The optimum must
        # match a pure exact run -- if the handoff dropped the pool or left the restricted
        # scope in place, this is where it shows.
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        exact = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4),
            CGSolver(recover_integer_solution = true),
        )
        warm = run_opt(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(
                max_stops = 4, relaxed_cluster_count = 4,
            ),
            CGSolver(recover_integer_solution = true,
                     warm_start_pricing_mode = :relaxed_cluster_guided),
        )
        @test warm.termination_status == SOLVE_OPTIMAL
        @test warm.objective_value ≈ exact.objective_value atol = 1e-6
        @test warm.metadata["cg_warm_start_pricing_mode"] === :relaxed_cluster_guided
        @test warm.metadata["cg_final_pricing_mode"] === :exact
        # Phase 2 is the unrestricted pricer, so the certificate is full-universe again.
        @test warm.metadata["cg_optimality_scope"] == "full_route_universe"
        @test warm.metadata["cg_pricing_universe_restricted"] === false
    end

end
