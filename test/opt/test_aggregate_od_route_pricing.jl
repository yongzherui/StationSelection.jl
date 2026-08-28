@testset "AggregateODRouteModel label-setting pricing" begin
    using JuMP

    function line_travel_cost(n::Int)
        costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:n, j in 1:n
            i == j && continue
            costs[(i, j)] = abs(i - j)
        end
        return costs
    end

    function line_pricing_data(;
            scenario::Int=1,
            active_pairs=[(1, 3), (2, 4)],
            max_wait_time=10.0,
            detour_factor=1.5,
            max_stops=5,
            bounded_max_stops=true,
            compensated_dominance=true,
        )
        return RouteCoveringPricingData(
            scenario,
            [1, 2, 3, 4],
            line_travel_cost(4),
            Tuple{Int, Int}.(active_pairs),
            1.0,
            0.0,
            max_wait_time,
            detour_factor,
            max_stops,
            bounded_max_stops,
            compensated_dominance,
        )
    end

    function label_at_current(labels, current)
        return only(filter(label -> label.current == current, labels))
    end

    # Stand-in for the pre-`round.jl` per-pricer driver functions
    # (`aggregate_od_route_pricing_by_label_setting` and friends), removed when pricing
    # was consolidated into `round.jl`'s generic two-phase `_run_pricing_round` (see that
    # file's docstring). `darp`/`darp_modified` (joint_routing_assignment/) kept a copy of
    # this exact shape (`driver.jl` in each of their directories) for standalone
    # benchmarking; route_covering's `exact`/`station_simple` did not, so tests that want
    # a bare, no-master-model comparison run replicate it locally instead of resurrecting
    # production API for it.
    function run_route_covering_pricing_driver(
        ctx_ctor, pricing_data, existing_columns::AbstractVector{AggregateODRouteColumn}, duals;
        next_column_id::Int=1,
        max_new_columns::Int=typemax(Int) ÷ 2,
        n_candidates::Int=typemax(Int) ÷ 2,
        time_limit::Float64=30.0,
        reduced_cost_tol::Float64=1e-6,
        profile::Bool=false,
    )
        ctx = ctx_ctor(pricing_data, duals)

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
        columns = AggregateODRouteColumn[
            StationSelection._pricing_make_column(ctx, next_column_id + offset - 1, candidate)
            for (offset, candidate) in enumerate(truncated)
        ]
        return columns, exhausted, stats
    end

    aggregate_od_route_pricing_by_label_setting(pricing_data, existing_columns, duals; kwargs...) =
        run_route_covering_pricing_driver(
            StationSelection.RouteCoveringSearchContext, pricing_data, existing_columns, duals; kwargs...,
        )
    aggregate_od_route_pricing_by_station_simple_label_setting(pricing_data, existing_columns, duals; kwargs...) =
        run_route_covering_pricing_driver(
            StationSelection.RouteCoveringStationSimpleSearchContext, pricing_data, existing_columns, duals; kwargs...,
        )

    @testset "exhaustive enumeration requires a finite max_stops" begin
        @test StationSelection._resolve_aggregate_od_route_max_stops(7) == 7
        @test_throws ArgumentError StationSelection._resolve_aggregate_od_route_max_stops(typemax(Int))
    end

    @testset "pricing max_stops resolution allows fully unbounded search" begin
        @test StationSelection._resolve_aggregate_od_route_pricing_max_stops(7) == 7
        # unlike enumeration's resolver, unbounded max_stops is a legitimate pricing
        # configuration (dominance/reduced-cost bound the search instead of a
        # route-length ceiling), not an error.
        @test StationSelection._resolve_aggregate_od_route_pricing_max_stops(typemax(Int)) == typemax(Int)
    end

    @testset "initial labels remember pickup station age" begin
        pricing_data = line_pricing_data()
        duals = RouteCoveringPricingDuals(Dict((1, 3) => 10.0, (2, 4) => 8.0))
        initial_1 = label_at_current(initial_route_covering_pricing_labels(pricing_data, duals), 1)

        @test isempty(initial_1.served_pairs)
        @test initial_1.station_age == Dict(1 => 0.0)
        @test initial_1.reduced_cost == 0.0
    end

    @testset "pickup eligibility ends at max_wait_time" begin
        duals = RouteCoveringPricingDuals(Dict((2, 4) => 10.0))

        at_cutoff = line_pricing_data(
            active_pairs=[(2, 4)],
            max_wait_time=1.0,
            detour_factor=2.0,
            max_stops=3,
        )
        initial_1 = RouteCoveringPricingLabel(
            1, [1], 0.0, Dict(1 => 0.0), Set{Tuple{Int, Int}}(), 0.0, 0.0, 1,
        )
        pickup_at_cutoff = only(extend_route_covering_pricing_label(initial_1, 2, at_cutoff, duals))
        @test pickup_at_cutoff.time == 1.0
        @test pickup_at_cutoff.station_age[2] == 0.0
        served_at_cutoff = only(extend_route_covering_pricing_label(pickup_at_cutoff, 4, at_cutoff, duals))
        @test (2, 4) in served_at_cutoff.served_pairs

        after_cutoff = line_pricing_data(
            active_pairs=[(2, 4)],
            max_wait_time=0.5,
            detour_factor=2.0,
            max_stops=3,
        )
        initial_1_late = RouteCoveringPricingLabel(
            1, [1], 0.0, Dict(1 => 0.0), Set{Tuple{Int, Int}}(), 0.0, 0.0, 1,
        )
        late_visit = only(extend_route_covering_pricing_label(initial_1_late, 2, after_cutoff, duals))
        @test late_visit.time == 1.0
        @test !haskey(late_visit.station_age, 2)
        not_served = only(extend_route_covering_pricing_label(late_visit, 4, after_cutoff, duals))
        @test (2, 4) ∉ not_served.served_pairs
    end

    @testset "extension certifies destination visits and updates reduced cost" begin
        pricing_data = line_pricing_data()
        duals = RouteCoveringPricingDuals(Dict((1, 3) => 10.0, (2, 4) => 8.0))
        initial_1 = label_at_current(initial_route_covering_pricing_labels(pricing_data, duals), 1)
        child_3 = only(extend_route_covering_pricing_label(initial_1, 3, pricing_data, duals))

        @test child_3.current == 3
        @test child_3.route == [1, 3]
        @test child_3.time == 2.0
        @test child_3.tau == 2.0
        @test child_3.served_pairs == Set([(1, 3)])
        @test child_3.reduced_cost == -8.0
    end

    @testset "expired opportunities are pruned before certification" begin
        pricing_data = line_pricing_data(detour_factor=1.0)
        duals = RouteCoveringPricingDuals(Dict((1, 3) => 10.0))
        initial_1 = label_at_current(initial_route_covering_pricing_labels(pricing_data, duals), 1)
        expired_child = only(extend_route_covering_pricing_label(initial_1, 4, pricing_data, duals))

        @test (1, 3) ∉ expired_child.served_pairs
        @test !haskey(expired_child.station_age, 1)
    end

    @testset "dominance respects time reduced cost served and station ages" begin
        good = RouteCoveringPricingLabel(
            2,
            [2],
            1.0,
            Dict(2 => 1.0),
            Set{Tuple{Int, Int}}(),
            0.0,
            1.0,
            1,
        )
        worse = RouteCoveringPricingLabel(
            2,
            [2],
            2.0,
            Dict(2 => 2.0),
            Set([(1, 3)]),
            0.0,
            2.0,
            1,
        )
        different_node = RouteCoveringPricingLabel(
            3,
            [3],
            2.0,
            Dict(3 => 0.0),
            Set{Tuple{Int, Int}}(),
            0.0,
            2.0,
            1,
        )
        longer_but_otherwise_better = RouteCoveringPricingLabel(
            2,
            [1, 2],
            0.5,
            Dict(2 => 0.5),
            Set{Tuple{Int, Int}}(),
            0.0,
            0.5,
            2,
        )

        @test StationSelection._dominates_route_covering_label(good, worse, true)
        @test !StationSelection._dominates_route_covering_label(worse, good, true)
        @test !StationSelection._dominates_route_covering_label(good, different_node, true)
        @test !StationSelection._dominates_route_covering_label(longer_but_otherwise_better, worse, true)
        @test StationSelection._dominates_route_covering_label(longer_but_otherwise_better, worse, false)

        pair_index = Dict((1, 3) => 1, (2, 4) => 2)
        node_index = Dict(1 => 1, 2 => 2, 3 => 3, 4 => 4)
        good_bs = StationSelection._make_route_covering_label_bitsets(good, pair_index, 2, node_index, 4)
        worse_bs = StationSelection._make_route_covering_label_bitsets(worse, pair_index, 2, node_index, 4)
        longer_bs = StationSelection._make_route_covering_label_bitsets(longer_but_otherwise_better, pair_index, 2, node_index, 4)
        @test StationSelection._dominates_route_covering_label(good, worse, good_bs, worse_bs, true)
        @test !StationSelection._dominates_route_covering_label(worse, good, worse_bs, good_bs, true)
        @test !StationSelection._dominates_route_covering_label(longer_but_otherwise_better, worse, longer_bs, worse_bs, true)
        @test StationSelection._dominates_route_covering_label(longer_but_otherwise_better, worse, longer_bs, worse_bs, false)
    end

    @testset "compensated dominance allows extra served pairs within reduced-cost budget" begin
        within_budget = RouteCoveringPricingLabel(
            2, [2], 1.0, Dict(2 => 1.0), Set([(1, 3)]), 0.0, -6.0, 1,
        )
        over_budget = RouteCoveringPricingLabel(
            2, [2], 1.0, Dict(2 => 1.0), Set([(1, 3)]), 0.0, -2.0, 1,
        )
        baseline = RouteCoveringPricingLabel(
            2, [2], 1.0, Dict(2 => 1.0), Set{Tuple{Int, Int}}(), 0.0, 0.0, 1,
        )
        pair_weight = Dict((1, 3) => 5.0, (2, 4) => 3.0)

        # plain subset test: the extra pair always blocks domination, regardless of
        # how favorable the reduced-cost gap is.
        @test !StationSelection._dominates_route_covering_label(within_budget, baseline, true; compensated_dominance=false)

        # compensated: dominates iff the reduced-cost gap covers the extra pair's weight
        @test StationSelection._dominates_route_covering_label(
            within_budget, baseline, true; pair_weight=pair_weight, compensated_dominance=true,
        )
        @test !StationSelection._dominates_route_covering_label(
            over_budget, baseline, true; pair_weight=pair_weight, compensated_dominance=true,
        )

        pair_index = Dict((1, 3) => 1, (2, 4) => 2)
        node_index = Dict(1 => 1, 2 => 2, 3 => 3, 4 => 4)
        weight = [5.0, 3.0]
        within_bs = StationSelection._make_route_covering_label_bitsets(within_budget, pair_index, 2, node_index, 4)
        over_bs = StationSelection._make_route_covering_label_bitsets(over_budget, pair_index, 2, node_index, 4)
        baseline_bs = StationSelection._make_route_covering_label_bitsets(baseline, pair_index, 2, node_index, 4)

        @test StationSelection._dominates_route_covering_label(
            within_budget, baseline, within_bs, baseline_bs, true; weight=weight, compensated_dominance=true,
        )
        @test !StationSelection._dominates_route_covering_label(
            over_budget, baseline, over_bs, baseline_bs, true; weight=weight, compensated_dominance=true,
        )
        @test !StationSelection._dominates_route_covering_label(
            within_budget, baseline, within_bs, baseline_bs, true; weight=weight, compensated_dominance=false,
        )
    end

    @testset "aggregate OD route station-age bitsets" begin
        label = RouteCoveringPricingLabel(
            2,
            [1, 2],
            1.0,
            Dict(1 => 1.0, 2 => 0.0),
            Set([(1, 3)]),
            1.0,
            -2.0,
            2,
        )
        pair_index = Dict((1, 3) => 1, (2, 4) => 2)
        node_index = Dict(1 => 1, 2 => 2, 3 => 3, 4 => 4)
        bs = StationSelection._make_route_covering_label_bitsets(label, pair_index, 2, node_index, 4)
        @test 1 in bs.served_bits
        @test bs.age_idx == Int32[node_index[1], node_index[2]]
        @test bs.age_val == [1.0, 0.0]
        @test bs.age_mask == 0x03
    end

    @testset "candidate generation can open fresh origins before pickup cutoff" begin
        pricing_data = line_pricing_data(
            active_pairs=[(3, 4)],
            max_wait_time=10.0,
            detour_factor=3.0,
            max_stops=4,
        )
        duals = RouteCoveringPricingDuals(Dict((3, 4) => 10.0))
        label = RouteCoveringPricingLabel(
            2,
            [1, 2],
            1.0,
            Dict{Int, Float64}(),
            Set{Tuple{Int, Int}}(),
            1.0,
            1.0,
            2,
        )

        candidates = StationSelection._route_covering_candidate_next_nodes(label, pricing_data, duals)
        @test 3 in candidates
    end

    @testset "pricing returns improving columns for one scenario" begin
        pricing_data = line_pricing_data(active_pairs=[(1, 3), (3, 4), (1, 4)])
        existing = AggregateODRouteColumn[
            AggregateODRouteColumn(1, [(1, 3)], 2.0),
            AggregateODRouteColumn(2, [(3, 4)], 1.0),
            AggregateODRouteColumn(3, [(1, 4)], 3.0),
        ]
        duals = RouteCoveringPricingDuals(Dict((1, 4) => 10.0, (1, 3) => 10.0, (3, 4) => 10.0))

        columns, exhausted, stats = aggregate_od_route_pricing_by_label_setting(
            pricing_data,
            existing,
            duals;
            next_column_id=10,
            max_new_columns=5,
            n_candidates=5,
            time_limit=5.0,
        )

        @test exhausted
        @test stats.labels_generated > 0
        @test !isempty(columns)
        @test any(column -> Set(column.od_pairs) == Set([(1, 3), (3, 4), (1, 4)]), columns)
        @test all(column -> column.metadata["scenario"] == 1, columns)
    end

    @testset "pricing stops early after enough candidates" begin
        pricing_data = line_pricing_data(active_pairs=[(1, 3), (3, 4), (1, 4)])
        existing = AggregateODRouteColumn[
            AggregateODRouteColumn(1, [(1, 3)], 2.0),
            AggregateODRouteColumn(2, [(3, 4)], 1.0),
            AggregateODRouteColumn(3, [(1, 4)], 3.0),
        ]
        duals = RouteCoveringPricingDuals(Dict((1, 4) => 10.0, (1, 3) => 10.0, (3, 4) => 10.0))

        columns, exhausted, stats = aggregate_od_route_pricing_by_label_setting(
            pricing_data,
            existing,
            duals;
            next_column_id=10,
            max_new_columns=1,
            n_candidates=1,
            time_limit=5.0,
        )

        @test !exhausted
        @test length(columns) == 1
        @test stats.labels_generated > 0
    end

    @testset "station-simple label-setting pricing" begin
        @testset "initial labels only open positive-dual origins" begin
            pricing_data = line_pricing_data(active_pairs=[(1, 3), (2, 4)])
            duals = RouteCoveringPricingDuals(Dict((1, 3) => 10.0))
            labels = StationSelection._initial_route_covering_station_simple_labels(pricing_data, duals)

            label_1 = only(filter(label -> label.current == 1, labels))
            label_2 = only(filter(label -> label.current == 2, labels))
            @test label_1.live_origin_age == Dict(1 => 0.0)
            @test isempty(label_2.live_origin_age)
            @test label_1.visited == Set([1])
            @test isempty(label_1.served_pairs)
            @test label_1.reduced_cost == 0.0
        end

        @testset "extension cannot revisit a station" begin
            pricing_data = line_pricing_data(active_pairs=[(1, 3)])
            duals = RouteCoveringPricingDuals(Dict((1, 3) => 10.0))
            label = only(
                filter(
                    label -> label.current == 1,
                    StationSelection._initial_route_covering_station_simple_labels(pricing_data, duals),
                ),
            )
            @test_throws ArgumentError StationSelection._extend_route_covering_station_simple_label(
                label, 1, pricing_data, duals,
            )
        end

        @testset "extension certifies destination visits and updates reduced cost" begin
            pricing_data = line_pricing_data(active_pairs=[(1, 3), (2, 4)])
            duals = RouteCoveringPricingDuals(Dict((1, 3) => 10.0, (2, 4) => 8.0))
            initial_1 = only(
                filter(
                    label -> label.current == 1,
                    StationSelection._initial_route_covering_station_simple_labels(pricing_data, duals),
                ),
            )
            child_3 = StationSelection._extend_route_covering_station_simple_label(initial_1, 3, pricing_data, duals)

            @test child_3.current == 3
            @test child_3.route == [1, 3]
            @test child_3.visited == Set([1, 3])
            @test child_3.time == 2.0
            @test child_3.tau == 2.0
            @test child_3.served_pairs == Set([(1, 3)])
            @test child_3.reduced_cost == -8.0
        end

        @testset "expired opportunities are pruned before certification" begin
            pricing_data = line_pricing_data(active_pairs=[(1, 3)], detour_factor=1.0)
            duals = RouteCoveringPricingDuals(Dict((1, 3) => 10.0))
            initial_1 = only(
                filter(
                    label -> label.current == 1,
                    StationSelection._initial_route_covering_station_simple_labels(pricing_data, duals),
                ),
            )
            expired_child = StationSelection._extend_route_covering_station_simple_label(initial_1, 4, pricing_data, duals)

            @test (1, 3) ∉ expired_child.served_pairs
            @test !haskey(expired_child.live_origin_age, 1)
        end

        @testset "dominance requires an exact visited match and live-origin domination" begin
            same_visited_better = RouteCoveringStationSimpleLabel(
                2, [1, 2], Set([1, 2]), 1.0, Dict(1 => 1.0), Set{Tuple{Int, Int}}(), 1.0, -2.0,
            )
            same_visited_worse = RouteCoveringStationSimpleLabel(
                2, [1, 2], Set([1, 2]), 2.0, Dict(1 => 2.0), Set{Tuple{Int, Int}}(), 1.0, -1.0,
            )
            different_visited = RouteCoveringStationSimpleLabel(
                2, [3, 2], Set([3, 2]), 1.0, Dict(3 => 1.0), Set{Tuple{Int, Int}}(), 1.0, -2.0,
            )
            different_current = RouteCoveringStationSimpleLabel(
                3, [1, 3], Set([1, 3]), 1.0, Dict(1 => 1.0), Set{Tuple{Int, Int}}(), 1.0, -2.0,
            )

            node_index = Dict(1 => 1, 2 => 2, 3 => 3, 4 => 4)
            better_bs = StationSelection._make_route_covering_station_simple_bitsets(same_visited_better, node_index)
            worse_bs = StationSelection._make_route_covering_station_simple_bitsets(same_visited_worse, node_index)
            different_visited_bs = StationSelection._make_route_covering_station_simple_bitsets(different_visited, node_index)
            different_current_bs = StationSelection._make_route_covering_station_simple_bitsets(different_current, node_index)

            @test StationSelection._dominates_route_covering_station_simple_label(
                same_visited_better, same_visited_worse, better_bs, worse_bs,
            )
            @test !StationSelection._dominates_route_covering_station_simple_label(
                same_visited_worse, same_visited_better, worse_bs, better_bs,
            )
            @test !StationSelection._dominates_route_covering_station_simple_label(
                same_visited_better, different_visited, better_bs, different_visited_bs,
            )
            @test !StationSelection._dominates_route_covering_station_simple_label(
                same_visited_better, different_current, better_bs, different_current_bs,
            )
        end

        @testset "candidate generation never offers a visited station" begin
            pricing_data = line_pricing_data(active_pairs=[(1, 3), (3, 4)], detour_factor=3.0, max_wait_time=10.0)
            duals = RouteCoveringPricingDuals(Dict((1, 3) => 10.0, (3, 4) => 10.0))
            label = RouteCoveringStationSimpleLabel(
                1, [1], Set([1]), 0.0, Dict(1 => 0.0), Set{Tuple{Int, Int}}(), 0.0, 0.0,
            )
            candidates = StationSelection._route_covering_station_simple_candidate_next_nodes(label, pricing_data, duals)
            @test 1 ∉ candidates
            @test 3 in candidates
        end

        @testset "pricing returns improving columns whose routes never repeat a station" begin
            pricing_data = line_pricing_data(active_pairs=[(1, 3), (3, 4), (1, 4)])
            existing = AggregateODRouteColumn[
                AggregateODRouteColumn(1, [(1, 3)], 2.0),
                AggregateODRouteColumn(2, [(3, 4)], 1.0),
                AggregateODRouteColumn(3, [(1, 4)], 3.0),
            ]
            duals = RouteCoveringPricingDuals(Dict((1, 4) => 10.0, (1, 3) => 10.0, (3, 4) => 10.0))

            columns, exhausted, stats = aggregate_od_route_pricing_by_station_simple_label_setting(
                pricing_data,
                existing,
                duals;
                next_column_id=10,
                max_new_columns=5,
                n_candidates=5,
                time_limit=5.0,
            )

            @test exhausted
            @test stats.labels_generated > 0
            @test !isempty(columns)
            @test any(column -> Set(column.od_pairs) == Set([(1, 3), (3, 4), (1, 4)]), columns)
            @test all(columns) do column
                route = column.metadata["route"]
                length(unique(route)) == length(route)
            end
        end
    end

    @testset "dominance under a single station extension (regression: station-clock pruning must not couple to served_pairs)" begin
        # Fuzz-discovered regression case for `_prune_irrelevant_route_covering_station_ages`
        # (`route_covering/data.jl`): it used to drop a station's live clock
        # once every pair reachable from it was already in `served_pairs`.
        # `served_pairs` is per-label and legitimately differs between two
        # labels compensated dominance allows to compare (one may have served
        # strictly more than the other, charged against its reduced-cost
        # lead) -- so the same physical station could get pruned from one
        # label but not the other purely because of which pairs each had
        # already banked, not because of any real physical/ride-limit
        # difference. That can flip a domination that held *before* a shared
        # one-station extension into one that no longer holds *after* it --
        # exactly the property dominance-based pruning depends on to discard
        # a label for good. See `_prune_irrelevant_route_covering_station_ages`'s
        # own docstring for the full argument and its analogue in
        # `joint_routing_assignment/data.jl`'s `_joint_routing_assignment_age_is_useful`
        # (fixed for the same reason in commit f644a7c).
        travel = line_travel_cost(3)
        active_pairs = [(2, 1), (1, 3), (3, 1)]
        sigma = Dict((2, 1) => 9.0, (1, 3) => 7.0, (3, 1) => 4.0)
        pd = RouteCoveringPricingData(
            1, [1, 2, 3], travel, active_pairs,
            0.5, 0.0, 3.0, 1.5, typemax(Int), false, true,
        )
        duals = RouteCoveringPricingDuals(sigma)
        pair_weight = Dict(pair => max(0.0, get(sigma, pair, 0.0)) for pair in active_pairs)

        seeds = StationSelection.initial_route_covering_pricing_labels(pd, duals)
        seed3 = only(filter(l -> l.current == 3, seeds))

        # `a`: 3 -> 2 -> 1, banking both (2,1) and (3,1) along the way.
        a1 = StationSelection._extend_route_covering_pricing_label(seed3, 2, pd, duals)
        a = StationSelection._extend_route_covering_pricing_label(a1, 1, pd, duals)
        # `b`: 3 -> 1 directly, banking only (3,1) -- strictly less than `a`,
        # but `a`'s reduced-cost lead is enough to compensate for the gap.
        b = StationSelection._extend_route_covering_pricing_label(seed3, 1, pd, duals)

        @test a.current == b.current == 1
        @test StationSelection._dominates_route_covering_label(
            a, b, false; pair_weight=pair_weight, compensated_dominance=true,
        )

        # A single common extension to node 2 -- one more station, nothing
        # hand-constructed -- must preserve that domination.
        a2 = StationSelection._extend_route_covering_pricing_label(a, 2, pd, duals)
        b2 = StationSelection._extend_route_covering_pricing_label(b, 2, pd, duals)
        @test StationSelection._dominates_route_covering_label(
            a2, b2, false; pair_weight=pair_weight, compensated_dominance=true,
        )
    end

    @testset "dominance is preserved under any common one-step extension (randomized)" begin
        # Same property and methodology as the passenger pricer's twin
        # (`test_joint_routing_assignment_pricing.jl`): every label here comes
        # from an actual random walk through the real extend function
        # (`_extend_route_covering_pricing_label`), never hand-constructed, so
        # every one automatically satisfies whatever invariants the real
        # search maintains.
        using Random

        function random_walk_route_covering_labels(rng, seed_label, pd, duals, nodes, depth)
            labels = [seed_label]
            label = seed_label
            for _ in 1:depth
                candidates = [nd for nd in nodes if nd != label.current && haskey(pd.travel_cost, (label.current, nd))]
                isempty(candidates) && break
                next_node = rand(rng, candidates)
                label = StationSelection._extend_route_covering_pricing_label(label, next_node, pd, duals)
                push!(labels, label)
            end
            return labels
        end

        rng = MersenneTwister(20260828)
        n_trials = 300
        n_checked = 0
        for _ in 1:n_trials
            n = rand(rng, 3:6)
            nodes = collect(1:n)
            travel = line_travel_cost(n)
            n_pairs = rand(rng, 1:5)
            active_pairs = Tuple{Int, Int}[]
            sigma = Dict{Tuple{Int, Int}, Float64}()
            for _ in 1:n_pairs
                j, k = rand(rng, nodes, 2)
                j == k && continue
                pair = (j, k)
                pair in active_pairs || push!(active_pairs, pair)
                sigma[pair] = Float64(rand(rng, 1:10))
            end
            isempty(active_pairs) && continue

            pd = RouteCoveringPricingData(
                1, nodes, travel, active_pairs,
                Float64(rand(rng, [0.0, 0.5, 1.0])),
                0.0,
                Float64(rand(rng, 1:4)),
                Float64(rand(rng, [1.0, 1.5, 2.0])),
                typemax(Int),
                false,
                true,
            )
            duals = RouteCoveringPricingDuals(sigma)

            seeds = StationSelection.initial_route_covering_pricing_labels(pd, duals)
            isempty(seeds) && continue

            by_current = Dict{Int, Vector{Any}}()
            for _ in 1:6
                seed_label = rand(rng, seeds)
                depth = rand(rng, 0:4)
                for label in random_walk_route_covering_labels(rng, seed_label, pd, duals, nodes, depth)
                    push!(get!(() -> Any[], by_current, label.current), label)
                end
            end

            pair_weight = Dict(pair => max(0.0, get(sigma, pair, 0.0)) for pair in active_pairs)

            for (current, labels) in by_current
                length(labels) < 2 && continue
                for i in eachindex(labels), j in eachindex(labels)
                    i == j && continue
                    a, b = labels[i], labels[j]
                    StationSelection._dominates_route_covering_label(
                        a, b, false; pair_weight=pair_weight, compensated_dominance=true,
                    ) || continue
                    for next_node in nodes
                        next_node == current && continue
                        haskey(travel, (current, next_node)) || continue
                        n_checked += 1
                        a2 = StationSelection._extend_route_covering_pricing_label(a, next_node, pd, duals)
                        b2 = StationSelection._extend_route_covering_pricing_label(b, next_node, pd, duals)
                        @test StationSelection._dominates_route_covering_label(
                            a2, b2, false; pair_weight=pair_weight, compensated_dominance=true,
                        )
                    end
                end
            end
        end
        @test n_checked > 0
    end
end
