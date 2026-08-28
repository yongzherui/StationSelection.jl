@testset "Passenger free-assignment label-setting pricing" begin
    using Random

    function line_travel_cost(n::Int)
        costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:n, j in 1:n
            i == j && continue
            costs[(i, j)] = Float64(abs(i - j))
        end
        return costs
    end

    function only_at(labels, current)
        return only(filter(l -> l.current == current, labels))
    end

    # Stand-in for the pre-`round.jl` per-pricer driver functions
    # (`joint_routing_assignment_pricing_by_label_setting` and friends), removed when
    # pricing was consolidated into `round.jl`'s generic two-phase `_run_pricing_round`
    # (see that file's docstring). `darp`/`darp_modified` kept a copy of this exact shape
    # (`driver.jl` in each of their directories) for standalone benchmarking; `exact`/
    # `station_simple` did not, so tests that want a bare, no-master-model comparison run
    # replicate it locally instead of resurrecting production API for it.
    function run_joint_routing_assignment_pricing_driver(
        ctx_ctor, pricing_data, existing_columns::AbstractVector{JointRoutingAssignmentRouteColumn};
        next_column_id::Int=1,
        max_new_columns::Int=typemax(Int) ÷ 2,
        n_candidates::Int=typemax(Int) ÷ 2,
        time_limit::Float64=30.0,
        reduced_cost_tol::Float64=1e-6,
        profile::Bool=false,
    )
        ctx = ctx_ctor(pricing_data)

        best_pool_tau = Dict{Any, Float64}()
        for column in existing_columns
            signature = StationSelection._pricing_pool_signature(ctx, column)
            best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
        end

        scored = Dict{Any, Any}()
        function accept!(label)
            candidate = StationSelection._pricing_candidate_from_label(ctx, label)
            isnothing(candidate) && return false
            candidate.reduced_cost < -reduced_cost_tol || return false
            candidate.tau < get(best_pool_tau, candidate.signature, Inf) - 1e-9 || return false
            current = get(scored, candidate.signature, nothing)
            if isnothing(current) ||
                    candidate.reduced_cost < current.reduced_cost - 1e-9 ||
                    (abs(candidate.reduced_cost - current.reduced_cost) <= 1e-9 && candidate.tau < current.tau - 1e-9)
                scored[candidate.signature] = candidate
            end
            return length(scored) >= n_candidates
        end

        _labels, exhausted, stats = StationSelection._run_label_setting(
            ctx; time_limit=time_limit, reduced_cost_tol=reduced_cost_tol, profile=profile, stop_if=accept!,
        )

        sorted = sort!(collect(values(scored)); by=c -> (c.reduced_cost, c.tau))
        truncated = sorted[1:min(length(sorted), max_new_columns)]
        columns = JointRoutingAssignmentRouteColumn[
            StationSelection._pricing_make_column(ctx, next_column_id + offset - 1, candidate)
            for (offset, candidate) in enumerate(truncated)
        ]
        return columns, exhausted, stats
    end

    joint_routing_assignment_pricing_by_label_setting(pricing_data, existing_columns; kwargs...) =
        run_joint_routing_assignment_pricing_driver(
            StationSelection.JointRoutingAssignmentSearchContext, pricing_data, existing_columns; kwargs...,
        )

    @testset "reward coarsening rounds upward and preserves exact default" begin
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 2.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 7.0),
            PassengerAssignmentCandidate(1, 2, 4, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 1, 2, 100.0, 3.0),
            PassengerAssignmentCandidate(2, 1, 3, 100.0, -1.0),
        ]
        unchanged = coarsen_passenger_assignment_rewards(candidates, 0)
        @test [c.reward for c in unchanged] == [c.reward for c in candidates]

        coarse = coarsen_passenger_assignment_rewards(candidates, 2)
        @test all(coarse[i].reward >= candidates[i].reward - 1e-9 for i in eachindex(candidates))
        @test [c.reward for c in coarse[1:4]] == [7.0, 7.0, 7.0, 10.0]
        @test coarse[5].reward == 3.0
        @test coarse[6].reward == -1.0
        @test_throws ArgumentError coarsen_passenger_assignment_rewards(candidates, -1)

        travel = line_travel_cost(4)
        exact = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        relaxed = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, coarse;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        for route in ([1, 2], [1, 3], [1, 4], [1, 2, 4])
            _ra, _rt, relaxed_rc = StationSelection._joint_routing_assignment_column_from_route(route, relaxed)
            _ea, _et, exact_rc = StationSelection._joint_routing_assignment_column_from_route(route, exact)
            @test relaxed_rc <= exact_rc + 1e-9
        end
    end

    @testset "single passenger: improving destination upgrades reward, doesn't double count" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
        ]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_joint_routing_assignment_pricing_labels(pricing_data), 1)

        label1 = only(extend_joint_routing_assignment_pricing_label(label0, 2, pricing_data))
        @test StationSelection._sum_layer_weights(pricing_data, label1.activated_reward_layers) ≈ 4.0

        label2 = only(extend_joint_routing_assignment_pricing_label(label1, 3, pricing_data))
        total = StationSelection._sum_layer_weights(pricing_data, label2.activated_reward_layers)
        @test total ≈ 10.0
        @test !isapprox(total, 14.0)

        assignments, tau, reduced_cost = StationSelection._joint_routing_assignment_column_from_route(
            label2.route, pricing_data; label_reduced_cost=label2.reduced_cost,
        )
        @test assignments == [(1, 1, 3)]
        @test reduced_cost ≈ label2.reduced_cost
    end

    @testset "single passenger: worse destination later activates nothing" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 10.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 4.0),
        ]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_joint_routing_assignment_pricing_labels(pricing_data), 1)

        label1 = only(extend_joint_routing_assignment_pricing_label(label0, 2, pricing_data))
        @test StationSelection._sum_layer_weights(pricing_data, label1.activated_reward_layers) ≈ 10.0

        label2 = only(extend_joint_routing_assignment_pricing_label(label1, 3, pricing_data))
        @test label2.activated_reward_layers == label1.activated_reward_layers
        @test StationSelection._sum_layer_weights(pricing_data, label2.activated_reward_layers) ≈ 10.0
    end

    @testset "alternative origins: only the larger certified reward is retained" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 2, 3, 100.0, 10.0),
        ]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_joint_routing_assignment_pricing_labels(pricing_data), 1)
        label1 = only(extend_joint_routing_assignment_pricing_label(label0, 2, pricing_data))
        label2 = only(extend_joint_routing_assignment_pricing_label(label1, 3, pricing_data))
        @test StationSelection._sum_layer_weights(pricing_data, label2.activated_reward_layers) ≈ 10.0
    end

    @testset "two passengers on the same OD pair are counted independently" begin
        travel = line_travel_cost(2)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(2, 1, 2, 100.0, 7.0),
        ]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_joint_routing_assignment_pricing_labels(pricing_data), 1)
        label1 = only(extend_joint_routing_assignment_pricing_label(label0, 2, pricing_data))
        @test StationSelection._sum_layer_weights(pricing_data, label1.activated_reward_layers) ≈ 11.0

        assignments, _tau, _rc = StationSelection._joint_routing_assignment_column_from_route(
            label1.route, pricing_data; label_reduced_cost=label1.reduced_cost,
        )
        @test Set(assignments) == Set([(1, 1, 2), (2, 1, 2)])
    end

    @testset "multiple assignments for one passenger: only the running maximum counts" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 3.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 8.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 5.0),  # worse than the upgrade already seen
        ]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label = only_at(initial_joint_routing_assignment_pricing_labels(pricing_data), 1)
        running = Float64[]
        for next_node in (2, 3, 4)
            label = only(extend_joint_routing_assignment_pricing_label(label, next_node, pricing_data))
            push!(running, StationSelection._sum_layer_weights(pricing_data, label.activated_reward_layers))
        end
        @test running ≈ [3.0, 8.0, 8.0]
    end

    @testset "ride-time infeasible assignment activates nothing" begin
        travel = line_travel_cost(3)
        candidates = [PassengerAssignmentCandidate(1, 1, 3, 1.5, 10.0)]  # ride limit 1.5 < travel(1,3) = 2
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_joint_routing_assignment_pricing_labels(pricing_data), 1)
        label1 = only(extend_joint_routing_assignment_pricing_label(label0, 2, pricing_data))
        label2 = only(extend_joint_routing_assignment_pricing_label(label1, 3, pricing_data))
        @test isempty(label2.activated_reward_layers)
    end

    @testset "visiting an origin after the pickup cutoff does not reset its age" begin
        travel = line_travel_cost(3)
        candidates = [PassengerAssignmentCandidate(1, 2, 3, 100.0, 10.0)]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=0.5,
        )
        label0 = JointRoutingAssignmentPricingLabel(
            1, [1], 0.0, Dict(1 => 0.0), RewardLayerBitset(), 0.0, 0.0, 1,
        )
        late_visit = only(extend_joint_routing_assignment_pricing_label(label0, 2, pricing_data))
        @test late_visit.time == 1.0
        @test !haskey(late_visit.station_age, 2)

        not_certified = only(extend_joint_routing_assignment_pricing_label(late_visit, 3, pricing_data))
        @test isempty(not_certified.activated_reward_layers)
    end

    @testset "dominance" begin
        travel = line_travel_cost(3)
        # Two distinct reward levels for passenger 1 so layer subset checks are non-trivial.
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
        ]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        layer_low = pricing_data.assignment_layer_mask[(1, 1, 2)]   # {1}
        layer_high = pricing_data.assignment_layer_mask[(1, 1, 3)]  # {1, 2}

        mklabel(current, time, station_age, layers, rc) =
            JointRoutingAssignmentPricingLabel(
                current, [current], time, station_age, layers, time, rc, 1,
            )

        @testset "dominates when time/rc/layers/station-ages are all no worse" begin
            a = mklabel(2, 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
            b = mklabel(2, 2.0, Dict(1 => 2.0), RewardLayerBitset(), 0.0)
            @test StationSelection._dominates_joint_routing_assignment_label(a, b, pricing_data.layer_weight, false)
            @test !StationSelection._dominates_joint_routing_assignment_label(b, a, pricing_data.layer_weight, false)
        end

        @testset "extra activated layers must be paid for out of the reduced-cost gap" begin
            # a holds {1,2} (worth 10), b holds {1} (worth 4), so a's compensation
            # w(A_a \ A_b) is the incremental layer 2, worth 10 - 4 = 6. At equal
            # reduced cost a cannot cover it: a shared suffix that certifies layer 2
            # pays b 6 and pays a nothing, so b can still overtake a.
            a = mklabel(2, 1.0, Dict{Int, Float64}(), layer_high, -1.0)
            b = mklabel(2, 1.0, Dict{Int, Float64}(), layer_low, -1.0)
            @test !StationSelection._dominates_joint_routing_assignment_label(a, b, pricing_data.layer_weight, false)
            # b's mask (low) IS a subset of a's mask (high), so its compensation is
            # zero and b dominates a once it is no worse on time/rc/ages.
            @test StationSelection._dominates_joint_routing_assignment_label(b, a, pricing_data.layer_weight, false)

            # Give a a reduced-cost advantage that *does* cover the 6, and the same
            # pair now dominates in the direction the plain subset rule could never
            # express. 6.5 > 6 clears it; 5.5 < 6 does not.
            a_cheap = mklabel(2, 1.0, Dict{Int, Float64}(), layer_high, -7.5)
            @test StationSelection._dominates_joint_routing_assignment_label(a_cheap, b, pricing_data.layer_weight, false)
            a_not_cheap_enough = mklabel(2, 1.0, Dict{Int, Float64}(), layer_high, -6.5)
            @test !StationSelection._dominates_joint_routing_assignment_label(a_not_cheap_enough, b, pricing_data.layer_weight, false)
        end

        @testset "a live origin the candidate lacks breaks dominance" begin
            a = mklabel(2, 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -1.0)
            b = mklabel(2, 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
            # a has no live origins at all, b still has station 1 live -- a must not
            # claim to dominate b, since b retains an option a has already lost.
            @test !StationSelection._dominates_joint_routing_assignment_label(a, b, pricing_data.layer_weight, false)
        end

        @testset "worse station age breaks dominance" begin
            a = mklabel(2, 1.0, Dict(1 => 5.0), RewardLayerBitset(), -1.0)
            b = mklabel(2, 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
            @test !StationSelection._dominates_joint_routing_assignment_label(a, b, pricing_data.layer_weight, false)
        end

        @testset "plain and bitset dominance implementations agree (randomized)" begin
            rng = MersenneTwister(42)
            nodes = [1, 2, 3, 4]
            node_index = Dict(n => i for (i, n) in enumerate(nodes))
            n_nodes = length(nodes)
            # Labels below activate layer ids drawn from 1:5; uneven weights make the
            # compensation term discriminate rather than collapse to a bit count.
            layer_weight = [1.0, 2.5, 0.5, 4.0, 3.0]

            function rand_label(rng, current)
                n_ages = rand(rng, 0:3)
                station_age = Dict{Int, Float64}()
                for s in rand(rng, nodes, n_ages)
                    station_age[s] = rand(rng) * 5
                end
                n_layers_active = rand(rng, 0:3)
                layers = RewardLayerBitset(rand(rng, 1:5, n_layers_active))
                t = rand(rng) * 5
                rc = rand(rng) * 10 - 5
                route_length = rand(rng, 1:4)
                return JointRoutingAssignmentPricingLabel(
                    current, [current], t, station_age, layers, t, rc, route_length,
                )
            end

            for _ in 1:200
                current = rand(rng, nodes)
                a = rand_label(rng, current)
                b = rand_label(rng, current)
                a_bs = StationSelection._make_joint_routing_assignment_label_bitsets(a, node_index, n_nodes)
                b_bs = StationSelection._make_joint_routing_assignment_label_bitsets(b, node_index, n_nodes)
                for bounded in (true, false)
                    plain = StationSelection._dominates_joint_routing_assignment_label(a, b, layer_weight, bounded)
                    bitset = StationSelection._dominates_joint_routing_assignment_label(a, b, a_bs, b_bs, layer_weight, bounded)
                    @test plain == bitset
                end
            end
        end
    end

    @testset "remaining-reward bound is admissible (brute force over small random instances)" begin
        rng = MersenneTwister(7)
        n = 4
        nodes = collect(1:n)
        travel = line_travel_cost(n)
        n_nodes = length(nodes)

        checked = 0
        trial = 0
        while checked < 15 && trial < 100
            trial += 1
            candidates = PassengerAssignmentCandidate[]
            for p in 1:2, j in nodes, k in nodes
                j == k && continue
                rand(rng) < 0.6 && continue
                reward = rand(rng) * 10
                ride_limit = rand(rng) * 5 + travel[(j, k)]
                push!(candidates, PassengerAssignmentCandidate(p, j, k, ride_limit, reward))
            end
            isempty(candidates) && continue

            pricing_data = create_joint_routing_assignment_pricing_data(
                1, nodes, travel, candidates;
                route_regularization_weight=0.1, max_wait_time=3.0,
            )
            isempty(pricing_data.opportunities) && continue

            search_index = StationSelection._build_joint_routing_assignment_search_index(pricing_data)
            bound_workspace = StationSelection._create_joint_routing_assignment_bound_workspace()

            labels0 = initial_joint_routing_assignment_pricing_labels(pricing_data)
            isempty(labels0) && continue
            label = rand(rng, labels0)
            for _ in 1:rand(rng, 0:2)
                next_nodes = StationSelection._joint_routing_assignment_candidate_next_nodes(label, pricing_data)
                isempty(next_nodes) && break
                label = only(extend_joint_routing_assignment_pricing_label(label, rand(rng, next_nodes), pricing_data))
            end

            label_bs = StationSelection._make_joint_routing_assignment_label_bitsets(
                label, search_index.node_index, n_nodes,
            )
            bound = StationSelection._joint_routing_assignment_remaining_reward_bound(
                label, label_bs, pricing_data, search_index, bound_workspace,
            )

            # The bound is a plain sum of reachable reward (no travel discount --
            # see `_joint_routing_assignment_remaining_reward_bound`'s docstring), so
            # it is necessarily >= the actual best *net* gain any completion achieves
            # (net of the travel cost paid to collect it). The invariant it has to
            # satisfy is exactly the one the search relies on: `label.rc - bound`
            # never exceeds the reduced cost of any descendant.
            best_gain = Ref(0.0)
            function dfs!(l, depth)
                depth <= 0 && return
                for nd in nodes
                    haskey(travel, (l.current, nd)) || continue
                    child = only(extend_joint_routing_assignment_pricing_label(l, nd, pricing_data))
                    best_gain[] = max(best_gain[], label.reduced_cost - child.reduced_cost)
                    dfs!(child, depth - 1)
                end
            end
            dfs!(label, 3)

            @test bound >= best_gain[] - 1e-9
            checked += 1
        end
        @test checked > 0
    end

    @testset "label search finds the brute-force-optimal reduced cost" begin
        nodes = [1, 2, 3]
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 8.0),
            PassengerAssignmentCandidate(2, 2, 3, 100.0, 5.0),
        ]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0, max_stops=3,
        )

        function brute_force_best_reduced_cost()
            best = Inf
            function visit!(route::Vector{Int})
                if length(route) >= 2
                    assignments, _tau, rc = StationSelection._joint_routing_assignment_column_from_route(route, pricing_data)
                    isempty(assignments) || (best = min(best, rc))
                end
                length(route) >= 3 && return
                for nd in nodes
                    nd == route[end] && continue
                    haskey(travel, (route[end], nd)) || continue
                    visit!(vcat(route, nd))
                end
            end
            for start in nodes
                visit!([start])
            end
            return best
        end

        brute_best = brute_force_best_reduced_cost()

        columns, _exhausted, _stats = joint_routing_assignment_pricing_by_label_setting(
            pricing_data, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10, n_candidates=10, time_limit=10.0,
        )
        search_best = isempty(columns) ? Inf : minimum(c.metadata["reduced_cost"] for c in columns)
        @test isapprox(search_best, brute_best; atol=1e-6)
    end

    @testset "unbounded max_stops: terminates and matches a bounded-but-generous search" begin
        # The real target is no stop limit at all. This is finite even so: every
        # extension adds travel > 0 to `time` and to every live clock, no new clock
        # is created past `max_wait_time`, and clocks are pruned once they cannot
        # certify -- so route length is implicitly capped by roughly
        # (max_wait + max_ride_limit) / min_travel. This checks that it (a) halts,
        # and (b) finds the same optimum as a search whose explicit cap is set well
        # above that implicit bound.
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 6.0, 12.0),
            PassengerAssignmentCandidate(2, 2, 4, 6.0, 9.0),
            PassengerAssignmentCandidate(3, 1, 4, 8.0, 7.0),
        ]
        mk(max_stops) = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=0.5, max_wait_time=3.0,
            max_stops=max_stops,
        )

        unbounded = mk(typemax(Int))
        @test unbounded.max_stops == typemax(Int)
        @test unbounded.bounded_max_stops == false

        cols_unb, exhausted_unb, _st = joint_routing_assignment_pricing_by_label_setting(
            unbounded, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=60.0,
        )
        @test exhausted_unb            # halted on its own, not by timeout
        @test !isempty(cols_unb)

        # generous explicit cap, far above the implicit one
        generous = mk(30)
        cols_gen, exhausted_gen, _st2 = joint_routing_assignment_pricing_by_label_setting(
            generous, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=60.0,
        )
        @test exhausted_gen
        best_unb = minimum(c.metadata["reduced_cost"] for c in cols_unb)
        best_gen = minimum(c.metadata["reduced_cost"] for c in cols_gen)
        @test isapprox(best_unb, best_gen; atol=1e-6)
    end

    @testset "optimized bound/dominance still match brute force under revisits" begin
        # Guards the bound rewrite (per-origin masks) and the sparse-age dominance
        # rewrite against the exhaustive optimum on an instance whose optimum needs
        # a revisit, which is where age bookkeeping is easiest to get wrong.
        nodes = [1, 2, 3, 4]
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 2, 4, 100.0, 11.0),
            PassengerAssignmentCandidate(2, 3, 1, 100.0, 9.0),
            PassengerAssignmentCandidate(3, 2, 1, 100.0, 6.0),
        ]
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=0.5, max_wait_time=20.0,
            max_stops=4,
        )

        best_bf = Inf
        route = Int[]
        function dfs!()
            if length(route) >= 2
                a, _t, rc = StationSelection._joint_routing_assignment_column_from_route(
                    copy(route), pricing_data,
                )
                isempty(a) || (best_bf = min(best_bf, rc))
            end
            length(route) >= 4 && return
            for nd in nodes
                !isempty(route) && nd == route[end] && continue
                push!(route, nd)
                dfs!()
                pop!(route)
            end
        end
        for s in nodes
            push!(route, s); dfs!(); pop!(route)
        end

        cols, exhausted, _st = joint_routing_assignment_pricing_by_label_setting(
            pricing_data, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        @test exhausted
        @test isapprox(minimum(c.metadata["reduced_cost"] for c in cols), best_bf; atol=1e-6)
    end

    @testset "reduced-cost consistency for extracted columns" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 12.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
        ]
        reward_lookup = Dict((c.p, c.origin, c.destination) => c.reward for c in candidates)
        pricing_data = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=4,
        )
        columns, _exhausted, _stats = joint_routing_assignment_pricing_by_label_setting(
            pricing_data, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=5, n_candidates=5, time_limit=10.0,
        )
        @test !isempty(columns)
        for column in columns
            expected_reward = sum(reward_lookup[a] for a in column.assignments)
            expected_rc = pricing_data.route_regularization_weight * (column.tau + pricing_data.repositioning_time) - expected_reward
            @test column.metadata["reduced_cost"] ≈ expected_rc atol = 1e-6
            @test column.metadata["reduced_cost"] < -1e-6
        end
    end

    @testset "label search finds the brute-force-optimal reduced cost (randomized)" begin
        # End-to-end soundness check: for a random small instance, does the
        # full label-setting search (dominance included) agree with exhaustive
        # enumeration of every valid route up to max_stops? Kept as a
        # permanent regression test -- this is exactly the kind of check that
        # would catch a future dominance rule that discards a label the
        # search actually needed.
        function brute_force_best_reduced_cost(nodes, travel, pricing_data, max_stops)
            best = Ref(Inf)
            function visit!(route::Vector{Int})
                if length(route) >= 2
                    assignments, _tau, rc = StationSelection._joint_routing_assignment_column_from_route(route, pricing_data)
                    isempty(assignments) || (best[] = min(best[], rc))
                end
                length(route) >= max_stops && return
                for nd in nodes
                    nd == route[end] && continue
                    haskey(travel, (route[end], nd)) || continue
                    visit!(vcat(route, nd))
                end
            end
            for start in nodes
                visit!([start])
            end
            return best[]
        end

        rng = MersenneTwister(2026)
        n_trials = 300
        n_checked = 0
        for _ in 1:n_trials
            n = rand(rng, 3:5)
            nodes = collect(1:n)
            travel = line_travel_cost(n)
            n_cand = rand(rng, 2:5)
            cands = PassengerAssignmentCandidate[]
            for p in 1:n_cand
                j, k = rand(rng, nodes, 2)
                j == k && continue
                push!(cands, PassengerAssignmentCandidate(
                    p, j, k, Float64(rand(rng, 1:6)), Float64(rand(rng, 1:10)),
                ))
            end
            isempty(cands) && continue
            max_stops = rand(rng, 2:4)
            pd = create_joint_routing_assignment_pricing_data(
                1, nodes, travel, cands;
                route_regularization_weight=Float64(rand(rng, [0.0, 0.5, 1.0])),
                max_wait_time=Float64(rand(rng, 1:4)),
                max_stops=max_stops,
            )
            isempty(pd.opportunities) && continue
            n_checked += 1

            brute = brute_force_best_reduced_cost(nodes, travel, pd, max_stops)
            ctx = StationSelection.JointRoutingAssignmentSearchContext(pd)
            labels, exhausted, _stats = StationSelection._run_label_setting(
                ctx; time_limit=30.0, reduced_cost_tol=0.0, use_reduced_cost_pruning=false,
            )
            @test exhausted
            search_best = isempty(labels) ? Inf : minimum(
                StationSelection._pricing_candidate_from_label(ctx, l).reduced_cost for l in labels
            )
            ok = (brute == Inf && search_best == Inf) || isapprox(search_best, brute; atol=1e-6)
            @test ok
        end
        @test n_checked > 0
    end

    @testset "dominance is preserved under any common one-step extension (randomized)" begin
        # This is the property dominance-based pruning actually relies on: the
        # search discards a dominated label the instant it is inserted, so if
        # `a` dominates `b` now but a shared future move could make that stop
        # being true, discarding `b` was unsound. Every label here comes from
        # an actual random walk through the real extend function
        # (`_extend_joint_routing_assignment_pricing_label`), never
        # hand-constructed, so every one automatically satisfies whatever
        # invariants the real search maintains -- a hand-built label can
        # violate invariants the real search would never produce, which reads
        # as a false "violation" that has nothing to do with the dominance
        # rule itself.
        function random_walk_labels(rng, seed_label, pd, nodes, depth)
            labels = [seed_label]
            label = seed_label
            for _ in 1:depth
                candidates = [nd for nd in nodes if nd != label.current && haskey(pd.travel_cost, (label.current, nd))]
                isempty(candidates) && break
                next_node = rand(rng, candidates)
                label = StationSelection._extend_joint_routing_assignment_pricing_label(label, next_node, pd)
                push!(labels, label)
            end
            return labels
        end

        rng = MersenneTwister(20260827)
        n_trials = 300
        n_checked = 0
        for _ in 1:n_trials
            n = rand(rng, 3:6)
            nodes = collect(1:n)
            travel = line_travel_cost(n)
            n_cand = rand(rng, 1:5)
            cands = PassengerAssignmentCandidate[]
            for p in 1:n_cand
                j, k = rand(rng, nodes, 2)
                j == k && continue
                push!(cands, PassengerAssignmentCandidate(
                    p, j, k, Float64(rand(rng, 1:6)), Float64(rand(rng, 1:10)),
                ))
            end
            isempty(cands) && continue
            pd = create_joint_routing_assignment_pricing_data(
                1, nodes, travel, cands;
                route_regularization_weight=Float64(rand(rng, [0.0, 0.5, 1.0])),
                max_wait_time=Float64(rand(rng, 1:4)),
                max_stops=typemax(Int),
            )
            isempty(pd.opportunities) && continue

            seeds = initial_joint_routing_assignment_pricing_labels(pd)
            isempty(seeds) && continue

            by_current = Dict{Int, Vector{Any}}()
            for _ in 1:6
                seed_label = rand(rng, seeds)
                depth = rand(rng, 0:4)
                for label in random_walk_labels(rng, seed_label, pd, nodes, depth)
                    push!(get!(() -> Any[], by_current, label.current), label)
                end
            end

            for (current, labels) in by_current
                length(labels) < 2 && continue
                for i in eachindex(labels), j in eachindex(labels)
                    i == j && continue
                    a, b = labels[i], labels[j]
                    StationSelection._dominates_joint_routing_assignment_label(a, b, pd.layer_weight, false) || continue
                    for next_node in nodes
                        next_node == current && continue
                        haskey(travel, (current, next_node)) || continue
                        n_checked += 1
                        a2 = StationSelection._extend_joint_routing_assignment_pricing_label(a, next_node, pd)
                        b2 = StationSelection._extend_joint_routing_assignment_pricing_label(b, next_node, pd)
                        @test StationSelection._dominates_joint_routing_assignment_label(a2, b2, pd.layer_weight, false)
                    end
                end
            end
        end
        @test n_checked > 0
    end
end
