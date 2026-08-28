@testset "Passenger free-assignment station-simple (elementary) pricing" begin
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

    # Best reduced cost over all ELEMENTARY routes (no repeated station). Since
    # station-simple prices over exactly this universe, its optimum must match this,
    # which may be strictly worse than the revisit-tolerant optimum.
    function brute_force_best_elementary_rc(nodes, travel, pricing_data)
        best = Inf
        function visit!(route)
            if length(route) >= 2
                a, _t, rc = StationSelection._joint_routing_assignment_column_from_route(route, pricing_data)
                isempty(a) || (best = min(best, rc))
            end
            for nd in nodes
                nd in route && continue
                haskey(travel, (route[end], nd)) || continue
                visit!(vcat(route, nd))
            end
        end
        for s in nodes
            visit!([s])
        end
        return best
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
    joint_routing_assignment_pricing_by_station_simple_label_setting(pricing_data, existing_columns; kwargs...) =
        run_joint_routing_assignment_pricing_driver(
            StationSelection.JointRoutingAssignmentStationSimpleSearchContext, pricing_data, existing_columns; kwargs...,
        )

    @testset "extension forbids revisits and grows the visited set" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
        ]
        pd = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(StationSelection._initial_joint_routing_assignment_station_simple_labels(pd), 1)
        @test label0.visited == Set([1])

        label1 = StationSelection._extend_joint_routing_assignment_station_simple_label(label0, 2, pd)
        @test label1.visited == Set([1, 2])
        @test label1.route == [1, 2]
        @test StationSelection._sum_layer_weights(pd, label1.activated_reward_layers) ≈ 4.0

        # Revisiting a station on the route is a hard error.
        @test_throws ArgumentError StationSelection._extend_joint_routing_assignment_station_simple_label(label1, 1, pd)
    end

    @testset "candidate generation excludes already-visited nodes" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 1, 100.0, 8.0),  # would make 1 "useful" again
        ]
        pd = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(StationSelection._initial_joint_routing_assignment_station_simple_labels(pd), 1)
        label1 = StationSelection._extend_joint_routing_assignment_station_simple_label(label0, 2, pd)
        cands = StationSelection._joint_routing_assignment_station_simple_candidate_next_nodes(label1, pd)
        @test 1 ∉ cands      # visited, never a candidate despite p2's (2,1) opportunity
        @test 2 ∉ cands      # current, also visited
        @test 3 ∈ cands
    end

    @testset "every returned column is an elementary route" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 12.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
            PassengerAssignmentCandidate(3, 2, 1, 100.0, 5.0),
        ]
        pd = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=4,
        )
        cols, _exhausted, _stats = joint_routing_assignment_pricing_by_station_simple_label_setting(
            pd, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=50, n_candidates=1000, time_limit=30.0,
        )
        @test !isempty(cols)
        for c in cols
            @test length(unique(c.route)) == length(c.route)  # no station repeated
        end
    end

    @testset "reduced-cost consistency for extracted columns" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 12.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
        ]
        reward_lookup = Dict((c.p, c.origin, c.destination) => c.reward for c in candidates)
        pd = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=4,
        )
        # The driver's route replay asserts label rc == recomputed rc internally; here
        # we also check it against an independent per-assignment recomputation.
        cols, _exhausted, _stats = joint_routing_assignment_pricing_by_station_simple_label_setting(
            pd, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=5, n_candidates=5, time_limit=10.0,
        )
        @test !isempty(cols)
        for c in cols
            expected_reward = sum(reward_lookup[a] for a in c.assignments)
            expected_rc = pd.route_regularization_weight * (c.tau + pd.repositioning_time) - expected_reward
            @test c.metadata["reduced_cost"] ≈ expected_rc atol = 1e-6
            @test c.metadata["reduced_cost"] < -1e-6
        end
    end

    @testset "attains the brute-force elementary optimum" begin
        nodes = [1, 2, 3, 4]
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 8.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
            PassengerAssignmentCandidate(3, 1, 4, 100.0, 7.0),
        ]
        pd = create_joint_routing_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=4,
        )
        brute_best = brute_force_best_elementary_rc(nodes, travel, pd)
        cols, exhausted, _stats = joint_routing_assignment_pricing_by_station_simple_label_setting(
            pd, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        @test exhausted
        @test isapprox(minimum(c.metadata["reduced_cost"] for c in cols), brute_best; atol=1e-6)
    end

    @testset "agrees with the revisit-tolerant pricer when the optimum is elementary" begin
        # Route 1->2->3 collects every reward on a single elementary pass; a revisit
        # can only add travel, so the two pricers must reach the same optimum.
        nodes = [1, 2, 3]
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 8.0),
            PassengerAssignmentCandidate(2, 2, 3, 100.0, 5.0),
        ]
        pd = create_joint_routing_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=3,
        )
        rev_cols, _e1, _s1 = joint_routing_assignment_pricing_by_label_setting(
            pd, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        ss_cols, _e2, _s2 = joint_routing_assignment_pricing_by_station_simple_label_setting(
            pd, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        rev_best = minimum(c.metadata["reduced_cost"] for c in rev_cols)
        ss_best = minimum(c.metadata["reduced_cost"] for c in ss_cols)
        @test isapprox(rev_best, ss_best; atol=1e-6)
    end

    @testset "restriction is visible where a revisit strictly helps" begin
        # nodes [1,2]: p1 wants 1->2, p2 wants 2->1. Serving both needs the route
        # 1->2->1, which revisits 1. The revisit-tolerant pricer collects both
        # (reward 20); an elementary route can serve at most one (reward 10).
        nodes = [1, 2]
        travel = line_travel_cost(2)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 1, 100.0, 10.0),
        ]
        pd = create_joint_routing_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=3,
        )

        rev_cols, _e1, _s1 = joint_routing_assignment_pricing_by_label_setting(
            pd, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        ss_cols, _e2, _s2 = joint_routing_assignment_pricing_by_station_simple_label_setting(
            pd, JointRoutingAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )

        # The revisit-tolerant pricer serves both via 1->2->1.
        @test any(length(c.assignments) == 2 for c in rev_cols)
        rev_best = minimum(c.metadata["reduced_cost"] for c in rev_cols)

        # Station-simple returns only elementary routes, each serving one passenger.
        for c in ss_cols
            @test length(unique(c.route)) == length(c.route)
            @test length(c.assignments) == 1
        end
        ss_best = minimum(c.metadata["reduced_cost"] for c in ss_cols)

        # It attains the best ELEMENTARY reduced cost, which is strictly worse
        # (less negative) than the revisit-tolerant optimum -- the documented
        # column-universe restriction.
        @test isapprox(ss_best, brute_force_best_elementary_rc(nodes, travel, pd); atol=1e-6)
        @test ss_best > rev_best + 1e-6
    end

    @testset "dominance: subset-visited rule with compensated layers" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
        ]
        pd = create_joint_routing_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        node_index = Dict(n => i for (i, n) in enumerate(pd.nodes))
        n_nodes = length(pd.nodes)

        mklabel(current, visited, time, station_age, layers, rc) =
            StationSelection.JointRoutingAssignmentStationSimpleLabel(
                current, collect(visited), BitSet(visited), time, station_age, layers, time, rc, length(visited),
            )
        ages(label) = StationSelection._make_joint_routing_assignment_station_simple_ages(label, node_index)
        dominates(x, y) = StationSelection._dominates_joint_routing_assignment_station_simple_label(
            x, y, ages(x), ages(y), pd.layer_weight,
        )

        # Equal visited, strictly no worse on time/rc/age -> dominates.
        a = mklabel(2, [1, 2], 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
        b = mklabel(2, [1, 2], 2.0, Dict(1 => 2.0), RewardLayerBitset(), 0.0)
        @test dominates(a, b)
        @test !dominates(b, a)

        # Subset visited: a lean route dominates a wandered one that is otherwise
        # no worse -- the cross-domination the exact-visited rule could not express.
        lean = mklabel(2, [2], 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -1.0)
        wandered = mklabel(2, [1, 2], 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -1.0)
        @test dominates(lean, wandered)     # {2} ⊆ {1,2}
        @test !dominates(wandered, lean)    # {1,2} ⊄ {2}

        # A superset-visited label cannot dominate even with a better reduced cost:
        # it has forbidden itself a station the lean label can still use.
        cheap_wandered = mklabel(2, [1, 2], 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -5.0)
        @test !dominates(cheap_wandered, lean)

        # Live-clock (age) resource: the dominator must hold every live clock the
        # dominated one has. An extra clock is fine; a missing one breaks dominance.
        has_clock = mklabel(2, [2], 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
        no_clock = mklabel(2, [2], 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -1.0)
        @test dominates(has_clock, no_clock)
        @test !dominates(no_clock, has_clock)
    end

    @testset "label search finds the brute-force-optimal reduced cost (randomized)" begin
        # End-to-end soundness check against exhaustive enumeration of every
        # elementary route -- see the revisit-tolerant pricer's twin in
        # `test_joint_routing_assignment_pricing.jl` for why this is kept as a
        # permanent regression test.
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
            pd = create_joint_routing_assignment_pricing_data(
                1, nodes, travel, cands;
                route_regularization_weight=Float64(rand(rng, [0.0, 0.5, 1.0])),
                max_wait_time=Float64(rand(rng, 1:4)),
                max_stops=typemax(Int),
            )
            isempty(pd.opportunities) && continue
            n_checked += 1

            brute = brute_force_best_elementary_rc(nodes, travel, pd)
            ctx = StationSelection.JointRoutingAssignmentStationSimpleSearchContext(pd)
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
        # Same property, same rationale as the revisit-tolerant pricer's twin
        # in `test_joint_routing_assignment_pricing.jl`: every label here is
        # produced by an actual random walk through the real extend function,
        # never hand-constructed.
        function random_walk_ss_labels(rng, seed_label, pd, nodes, depth)
            labels = [seed_label]
            label = seed_label
            for _ in 1:depth
                candidates = [
                    nd for nd in nodes
                    if nd != label.current && nd ∉ label.visited && haskey(pd.travel_cost, (label.current, nd))
                ]
                isempty(candidates) && break
                next_node = rand(rng, candidates)
                label = StationSelection._extend_joint_routing_assignment_station_simple_label(label, next_node, pd)
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

            node_index = Dict(nd => i for (i, nd) in enumerate(pd.nodes))
            ages(label) = StationSelection._make_joint_routing_assignment_station_simple_ages(label, node_index)
            dominates(x, y) = StationSelection._dominates_joint_routing_assignment_station_simple_label(
                x, y, ages(x), ages(y), pd.layer_weight,
            )

            seeds = StationSelection._initial_joint_routing_assignment_station_simple_labels(pd)
            isempty(seeds) && continue

            by_current = Dict{Int, Vector{Any}}()
            for _ in 1:6
                seed_label = rand(rng, seeds)
                depth = rand(rng, 0:min(4, n - 1))
                for label in random_walk_ss_labels(rng, seed_label, pd, nodes, depth)
                    push!(get!(() -> Any[], by_current, label.current), label)
                end
            end

            for (current, labels) in by_current
                length(labels) < 2 && continue
                for i in eachindex(labels), j in eachindex(labels)
                    i == j && continue
                    a, b = labels[i], labels[j]
                    dominates(a, b) || continue
                    for next_node in nodes
                        next_node in b.visited && continue
                        next_node in a.visited && continue
                        haskey(travel, (current, next_node)) || continue
                        n_checked += 1
                        a2 = StationSelection._extend_joint_routing_assignment_station_simple_label(a, next_node, pd)
                        b2 = StationSelection._extend_joint_routing_assignment_station_simple_label(b, next_node, pd)
                        @test dominates(a2, b2)
                    end
                end
            end
        end
        @test n_checked > 0
    end
end
