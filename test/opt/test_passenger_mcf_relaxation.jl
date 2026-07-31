@testset "Passenger free-assignment MCF relaxation" begin
    using Random
    using Gurobi

    # One Gurobi.Env reused across this whole testset -- constructing several per
    # process is what produced a silent 30-minute hang in an earlier run.
    MCF_ENV = Gurobi.Env()

    function line_travel_cost(n::Int; scale::Float64=1.0)
        costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:n, j in 1:n
            i == j && continue
            costs[(i, j)] = scale * abs(i - j)
        end
        return costs
    end

    "The exact pricer's optimum over the same `pricing_data`, run to exhaustion."
    function exact_best_rc(pd; time_limit::Float64=60.0)
        cols, exhausted, _stats = passenger_free_assignment_pricing_by_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=typemax(Int) ÷ 2,
            n_candidates=typemax(Int) ÷ 2, time_limit=time_limit,
            reduced_cost_tol=1e-9,
        )
        @test exhausted
        isempty(cols) && return Inf
        return minimum(Float64(c.metadata["reduced_cost"]) for c in cols)
    end

    fine_config(; kwargs...) = PassengerMCFRelaxationConfig(;
        enabled=true, lp_time_limit_sec=60.0, kwargs...,
    )

    # ── the invariant that matters ────────────────────────────────────────────
    # A relaxation may be loose, but it may NEVER exceed the exact optimum: that
    # is precisely the direction in which a wrong bound would wrongly certify.
    @testset "bound never exceeds the exact pricing optimum" begin
        @testset "hand-built instances" begin
            cases = [
                # (nodes, travel, candidates, beta, max_wait, max_stops)
                ([1, 2, 3], line_travel_cost(3), [
                    PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
                    PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
                ], 1.0, 10.0, 4),
                ([1, 2, 3, 4], line_travel_cost(4), [
                    PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
                    PassengerAssignmentCandidate(1, 1, 4, 100.0, 12.0),
                    PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
                    PassengerAssignmentCandidate(3, 2, 1, 100.0, 5.0),
                ], 0.5, 10.0, 4),
                # Every reward tiny relative to travel: the optimum is "do nothing",
                # so the exact pricer returns no column at all.
                ([1, 2, 3], line_travel_cost(3; scale=50.0), [
                    PassengerAssignmentCandidate(1, 1, 2, 500.0, 0.5),
                ], 1.0, 100.0, 4),
            ]
            for (nodes, travel, candidates, beta, wait, stops) in cases
                pd = create_passenger_free_assignment_pricing_data(
                    1, nodes, travel, candidates;
                    route_regularization_weight=beta, max_wait_time=wait, max_stops=stops,
                )
                bound, _certified, stats = passenger_free_assignment_mcf_lower_bound(
                    pd, MCF_ENV; config=fine_config(), reduced_cost_tol=1e-9,
                )
                @test stats.reason == :ok || stats.reason == :no_opportunities
                @test bound <= exact_best_rc(pd) + 1e-6
            end
        end

        @testset "the revisit case the elementary pricer cannot express" begin
            # 2 stations, p1 goes 1->2 and p2 goes 2->1: the optimum needs `1,2,1`.
            travel = line_travel_cost(2)
            candidates = [
                PassengerAssignmentCandidate(1, 1, 2, 100.0, 10.0),
                PassengerAssignmentCandidate(2, 2, 1, 100.0, 10.0),
            ]
            pd = create_passenger_free_assignment_pricing_data(
                1, [1, 2], travel, candidates;
                route_regularization_weight=1.0, max_wait_time=50.0, max_stops=4,
                max_visits_per_node=3,
            )
            bound, _certified, _stats = passenger_free_assignment_mcf_lower_bound(
                pd, MCF_ENV; config=fine_config(), reduced_cost_tol=1e-9,
            )
            exact = exact_best_rc(pd)
            @test exact < -1e-6            # the revisiting route really is improving
            @test bound <= exact + 1e-6    # and the relaxation sees at least that much
        end

        @testset "randomised instances" begin
            rng = MersenneTwister(20260731)
            for trial in 1:12
                n = rand(rng, 3:5)
                nodes = collect(1:n)
                travel = Dict{Tuple{Int, Int}, Float64}()
                coords = [(rand(rng) * 10, rand(rng) * 10) for _ in 1:n]
                for i in 1:n, j in 1:n
                    i == j && continue
                    dx = coords[i][1] - coords[j][1]
                    dy = coords[i][2] - coords[j][2]
                    # Metric costs, floored away from zero so the grid stays sane.
                    travel[(i, j)] = max(1.0, round(sqrt(dx^2 + dy^2); digits=2))
                end
                candidates = PassengerAssignmentCandidate[]
                for p in 1:rand(rng, 1:4), _ in 1:rand(rng, 1:3)
                    o = rand(rng, nodes)
                    d = rand(rng, filter(!=(o), nodes))
                    push!(candidates, PassengerAssignmentCandidate(
                        p, o, d, 2.0 * travel[(o, d)], round(rand(rng) * 20; digits=2),
                    ))
                end
                isempty(candidates) && continue
                pd = create_passenger_free_assignment_pricing_data(
                    1, nodes, travel, candidates;
                    route_regularization_weight=round(0.5 + rand(rng); digits=2),
                    max_wait_time=15.0, max_stops=5, max_visits_per_node=2,
                )
                isempty(pd.opportunities) && continue
                bound, _certified, stats = passenger_free_assignment_mcf_lower_bound(
                    pd, MCF_ENV; config=fine_config(), reduced_cost_tol=1e-9,
                )
                stats.reason == :ok || continue
                exact = exact_best_rc(pd)
                @test bound <= exact + 1e-6
            end
        end
    end

    # ── the certificate is only useful if it is sound ─────────────────────────
    @testset "certifying implies the exact search finds nothing" begin
        rng = MersenneTwister(31072026)
        n_certified = 0
        for _ in 1:12
            nodes = [1, 2, 3, 4]
            travel = line_travel_cost(4; scale=round(1.0 + 9.0 * rand(rng); digits=2))
            candidates = PassengerAssignmentCandidate[]
            for p in 1:3
                o = rand(rng, nodes)
                d = rand(rng, filter(!=(o), nodes))
                push!(candidates, PassengerAssignmentCandidate(
                    p, o, d, 2.0 * travel[(o, d)], round(rand(rng) * 12; digits=2),
                ))
            end
            pd = create_passenger_free_assignment_pricing_data(
                1, nodes, travel, candidates;
                route_regularization_weight=1.0, max_wait_time=40.0, max_stops=5,
                max_visits_per_node=2,
            )
            isempty(pd.opportunities) && continue
            _bound, certified, stats = passenger_free_assignment_mcf_lower_bound(
                pd, MCF_ENV; config=fine_config(), reduced_cost_tol=1e-6,
            )
            stats.reason == :ok || continue
            if certified
                n_certified += 1
                cols, exhausted, _s = passenger_free_assignment_pricing_by_label_setting(
                    pd, PassengerFreeAssignmentRouteColumn[];
                    next_column_id=1, max_new_columns=typemax(Int) ÷ 2,
                    n_candidates=typemax(Int) ÷ 2, time_limit=60.0,
                    reduced_cost_tol=1e-6,
                )
                @test exhausted
                @test isempty(cols)
            end
        end
        # A certificate that never fires would make the test above vacuous.
        @test n_certified > 0
    end

    @testset "no positive-reward opportunity certifies without an LP" begin
        travel = line_travel_cost(3)
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, [PassengerAssignmentCandidate(1, 1, 2, 100.0, 0.0)];
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        @test isempty(pd.opportunities)
        bound, certified, stats = passenger_free_assignment_mcf_lower_bound(
            pd, MCF_ENV; config=fine_config(),
        )
        @test certified
        @test bound == 0.0
        @test stats.reason == :no_opportunities
    end

    # ── tightness on cases where the LP has nothing to fractionalise ──────────
    @testset "exact on a single-passenger two-station instance" begin
        travel = line_travel_cost(2; scale=3.0)
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2], travel, [PassengerAssignmentCandidate(1, 1, 2, 100.0, 10.0)];
            route_regularization_weight=1.0, max_wait_time=20.0, max_stops=3,
        )
        bound, certified, _stats = passenger_free_assignment_mcf_lower_bound(
            pd, MCF_ENV; config=fine_config(), reduced_cost_tol=1e-9,
        )
        # Route [1,2]: tau = 3, reward 10, beta = 1 => rc = -7.
        @test bound ≈ -7.0 atol = 1e-6
        @test !certified
        @test bound ≈ exact_best_rc(pd) atol = 1e-6
    end

    # ── network construction ──────────────────────────────────────────────────
    @testset "network construction and the time-step clamp" begin
        travel = line_travel_cost(3; scale=2.0)   # shortest arc is 2.0
        candidates = [PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0)]
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0, max_stops=4,
        )

        # A step longer than the shortest arc would floor that arc to zero and
        # break the DAG, so it is clamped down rather than honoured. Station 2
        # certifies nothing and is dropped, so the shortest *modelled* arc is
        # `travel(1, 3) = 4.0`, not the raw shortest arc of 2.0.
        net, reason = StationSelection._build_passenger_mcf_network(
            pd, PassengerMCFRelaxationConfig(; enabled=true, time_step=100.0),
        )
        @test reason == :ok
        @test net.time_step ≈ 4.0

        # A finer request is honoured as-is.
        net_fine, reason_fine = StationSelection._build_passenger_mcf_network(
            pd, PassengerMCFRelaxationConfig(; enabled=true, time_step=0.5),
        )
        @test reason_fine == :ok
        @test net_fine.time_step ≈ 0.5
        @test net_fine.n_layers > net.n_layers

        # Only opportunity endpoints are modelled; station 2 certifies nothing.
        @test Set(net.nodes) == Set([1, 3])

        # Every travel arc strictly advances time -- otherwise the suffix flows
        # could circulate within a layer and manufacture reward.
        for a in net.travel_arcs
            @test net.state_time[net.arc_head[a]] > net.state_time[net.arc_tail[a]]
        end
        # Source arcs start at t = 0, and every state can reach the sink.
        for a in net.source_arcs
            @test net.state_time[net.arc_head[a]] == 0
        end
        @test length(net.sink_arcs) == length(net.state_node)
    end

    @testset "the size guard declines to certify" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 8.0),
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=20.0, max_stops=5,
        )
        bound, certified, stats = passenger_free_assignment_mcf_lower_bound(
            pd, MCF_ENV; config=PassengerMCFRelaxationConfig(; enabled=true, max_arcs=1),
        )
        @test stats.reason == :too_large
        @test bound == -Inf
        @test !certified
    end

    # Finer boarding buckets tighten each commodity's ride deadline but also hand
    # the LP one independent injection per bucket, which pulls the other way --
    # so the bound is NOT monotone in the bucket count and the test asserts only
    # what actually holds: both settings stay valid.
    @testset "more boarding buckets stay valid" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 4.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 4, 4.0, 9.0),
            PassengerAssignmentCandidate(3, 3, 1, 6.0, 7.0),
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=6.0, max_stops=5,
            max_visits_per_node=2,
        )
        coarse, _c1, s1 = passenger_free_assignment_mcf_lower_bound(
            pd, MCF_ENV; config=fine_config(n_boarding_buckets=1), reduced_cost_tol=1e-9,
        )
        fine, _c2, s2 = passenger_free_assignment_mcf_lower_bound(
            pd, MCF_ENV; config=fine_config(n_boarding_buckets=8), reduced_cost_tol=1e-9,
        )
        @test s1.reason == :ok && s2.reason == :ok
        @test s2.n_commodities >= s1.n_commodities
        exact = exact_best_rc(pd)
        @test coarse <= exact + 1e-6
        @test fine <= exact + 1e-6
    end

    @testset "max_stops and max_visits caps are honoured" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 3, 100.0, 10.0),
        ]
        # `max_stops = 2` allows a single arc, so at most one of the two
        # passengers can be certified.
        pd_tight = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=20.0, max_stops=2,
        )
        @test pd_tight.bounded_max_stops
        tight, _ct, st = passenger_free_assignment_mcf_lower_bound(
            pd_tight, MCF_ENV; config=fine_config(), reduced_cost_tol=1e-9,
        )
        @test st.reason == :ok
        @test tight <= exact_best_rc(pd_tight) + 1e-6

        pd_loose = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=20.0, max_stops=4,
        )
        loose, _cl, sl = passenger_free_assignment_mcf_lower_bound(
            pd_loose, MCF_ENV; config=fine_config(), reduced_cost_tol=1e-9,
        )
        @test sl.reason == :ok
        @test loose <= tight + 1e-6   # a longer route budget can only help pricing
        @test loose <= exact_best_rc(pd_loose) + 1e-6
    end

    @testset "config validation" begin
        @test_throws ArgumentError PassengerMCFRelaxationConfig(; n_boarding_buckets=0)
        @test_throws ArgumentError PassengerMCFRelaxationConfig(; lp_time_limit_sec=0.0)
        @test_throws ArgumentError PassengerMCFRelaxationConfig(; max_arcs=0)
        @test !PassengerMCFRelaxationConfig().enabled   # off by default

        disabled_pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2], line_travel_cost(2),
            [PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0)];
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        bound, certified, stats = passenger_free_assignment_mcf_lower_bound(disabled_pd, MCF_ENV)
        @test bound == -Inf
        @test !certified
        @test stats.reason == :disabled
    end
end
