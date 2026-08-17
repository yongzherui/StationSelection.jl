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
        return AggregateODRoutePricingData(
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
        duals = AggregateODRoutePricingDuals(Dict((1, 3) => 10.0, (2, 4) => 8.0))
        initial_1 = label_at_current(initial_aggregate_od_route_pricing_labels(pricing_data, duals), 1)

        @test isempty(initial_1.served_pairs)
        @test initial_1.station_age == Dict(1 => 0.0)
        @test initial_1.reduced_cost == 0.0
    end

    @testset "pickup eligibility ends at max_wait_time" begin
        duals = AggregateODRoutePricingDuals(Dict((2, 4) => 10.0))

        at_cutoff = line_pricing_data(
            active_pairs=[(2, 4)],
            max_wait_time=1.0,
            detour_factor=2.0,
            max_stops=3,
        )
        initial_1 = AggregateODRoutePricingLabel(
            1, [1], 0.0, Dict(1 => 0.0), Set{Tuple{Int, Int}}(), 0.0, 0.0, 1,
        )
        pickup_at_cutoff = only(extend_aggregate_od_route_pricing_label(initial_1, 2, at_cutoff, duals))
        @test pickup_at_cutoff.time == 1.0
        @test pickup_at_cutoff.station_age[2] == 0.0
        served_at_cutoff = only(extend_aggregate_od_route_pricing_label(pickup_at_cutoff, 4, at_cutoff, duals))
        @test (2, 4) in served_at_cutoff.served_pairs

        after_cutoff = line_pricing_data(
            active_pairs=[(2, 4)],
            max_wait_time=0.5,
            detour_factor=2.0,
            max_stops=3,
        )
        initial_1_late = AggregateODRoutePricingLabel(
            1, [1], 0.0, Dict(1 => 0.0), Set{Tuple{Int, Int}}(), 0.0, 0.0, 1,
        )
        late_visit = only(extend_aggregate_od_route_pricing_label(initial_1_late, 2, after_cutoff, duals))
        @test late_visit.time == 1.0
        @test !haskey(late_visit.station_age, 2)
        not_served = only(extend_aggregate_od_route_pricing_label(late_visit, 4, after_cutoff, duals))
        @test (2, 4) ∉ not_served.served_pairs
    end

    @testset "extension certifies destination visits and updates reduced cost" begin
        pricing_data = line_pricing_data()
        duals = AggregateODRoutePricingDuals(Dict((1, 3) => 10.0, (2, 4) => 8.0))
        initial_1 = label_at_current(initial_aggregate_od_route_pricing_labels(pricing_data, duals), 1)
        child_3 = only(extend_aggregate_od_route_pricing_label(initial_1, 3, pricing_data, duals))

        @test child_3.current == 3
        @test child_3.route == [1, 3]
        @test child_3.time == 2.0
        @test child_3.tau == 2.0
        @test child_3.served_pairs == Set([(1, 3)])
        @test child_3.reduced_cost == -8.0
    end

    @testset "expired opportunities are pruned before certification" begin
        pricing_data = line_pricing_data(detour_factor=1.0)
        duals = AggregateODRoutePricingDuals(Dict((1, 3) => 10.0))
        initial_1 = label_at_current(initial_aggregate_od_route_pricing_labels(pricing_data, duals), 1)
        expired_child = only(extend_aggregate_od_route_pricing_label(initial_1, 4, pricing_data, duals))

        @test (1, 3) ∉ expired_child.served_pairs
        @test !haskey(expired_child.station_age, 1)
    end

    @testset "dominance respects time reduced cost served and station ages" begin
        good = AggregateODRoutePricingLabel(
            2,
            [2],
            1.0,
            Dict(2 => 1.0),
            Set{Tuple{Int, Int}}(),
            0.0,
            1.0,
            1,
        )
        worse = AggregateODRoutePricingLabel(
            2,
            [2],
            2.0,
            Dict(2 => 2.0),
            Set([(1, 3)]),
            0.0,
            2.0,
            1,
        )
        different_node = AggregateODRoutePricingLabel(
            3,
            [3],
            2.0,
            Dict(3 => 0.0),
            Set{Tuple{Int, Int}}(),
            0.0,
            2.0,
            1,
        )
        longer_but_otherwise_better = AggregateODRoutePricingLabel(
            2,
            [1, 2],
            0.5,
            Dict(2 => 0.5),
            Set{Tuple{Int, Int}}(),
            0.0,
            0.5,
            2,
        )

        @test StationSelection._dominates_aggregate_od_route_label(good, worse, true)
        @test !StationSelection._dominates_aggregate_od_route_label(worse, good, true)
        @test !StationSelection._dominates_aggregate_od_route_label(good, different_node, true)
        @test !StationSelection._dominates_aggregate_od_route_label(longer_but_otherwise_better, worse, true)
        @test StationSelection._dominates_aggregate_od_route_label(longer_but_otherwise_better, worse, false)

        pair_index = Dict((1, 3) => 1, (2, 4) => 2)
        node_index = Dict(1 => 1, 2 => 2, 3 => 3, 4 => 4)
        good_bs = StationSelection._make_aggregate_od_route_label_bitsets(good, pair_index, 2, node_index, 4)
        worse_bs = StationSelection._make_aggregate_od_route_label_bitsets(worse, pair_index, 2, node_index, 4)
        longer_bs = StationSelection._make_aggregate_od_route_label_bitsets(longer_but_otherwise_better, pair_index, 2, node_index, 4)
        @test StationSelection._dominates_aggregate_od_route_label(good, worse, good_bs, worse_bs, true)
        @test !StationSelection._dominates_aggregate_od_route_label(worse, good, worse_bs, good_bs, true)
        @test !StationSelection._dominates_aggregate_od_route_label(longer_but_otherwise_better, worse, longer_bs, worse_bs, true)
        @test StationSelection._dominates_aggregate_od_route_label(longer_but_otherwise_better, worse, longer_bs, worse_bs, false)
    end

    @testset "compensated dominance allows extra served pairs within reduced-cost budget" begin
        within_budget = AggregateODRoutePricingLabel(
            2, [2], 1.0, Dict(2 => 1.0), Set([(1, 3)]), 0.0, -6.0, 1,
        )
        over_budget = AggregateODRoutePricingLabel(
            2, [2], 1.0, Dict(2 => 1.0), Set([(1, 3)]), 0.0, -2.0, 1,
        )
        baseline = AggregateODRoutePricingLabel(
            2, [2], 1.0, Dict(2 => 1.0), Set{Tuple{Int, Int}}(), 0.0, 0.0, 1,
        )
        pair_weight = Dict((1, 3) => 5.0, (2, 4) => 3.0)

        # plain subset test: the extra pair always blocks domination, regardless of
        # how favorable the reduced-cost gap is.
        @test !StationSelection._dominates_aggregate_od_route_label(within_budget, baseline, true; compensated_dominance=false)

        # compensated: dominates iff the reduced-cost gap covers the extra pair's weight
        @test StationSelection._dominates_aggregate_od_route_label(
            within_budget, baseline, true; pair_weight=pair_weight, compensated_dominance=true,
        )
        @test !StationSelection._dominates_aggregate_od_route_label(
            over_budget, baseline, true; pair_weight=pair_weight, compensated_dominance=true,
        )

        pair_index = Dict((1, 3) => 1, (2, 4) => 2)
        node_index = Dict(1 => 1, 2 => 2, 3 => 3, 4 => 4)
        weight = [5.0, 3.0]
        within_bs = StationSelection._make_aggregate_od_route_label_bitsets(within_budget, pair_index, 2, node_index, 4)
        over_bs = StationSelection._make_aggregate_od_route_label_bitsets(over_budget, pair_index, 2, node_index, 4)
        baseline_bs = StationSelection._make_aggregate_od_route_label_bitsets(baseline, pair_index, 2, node_index, 4)

        @test StationSelection._dominates_aggregate_od_route_label(
            within_budget, baseline, within_bs, baseline_bs, true; weight=weight, compensated_dominance=true,
        )
        @test !StationSelection._dominates_aggregate_od_route_label(
            over_budget, baseline, over_bs, baseline_bs, true; weight=weight, compensated_dominance=true,
        )
        @test !StationSelection._dominates_aggregate_od_route_label(
            within_budget, baseline, within_bs, baseline_bs, true; weight=weight, compensated_dominance=false,
        )
    end

    @testset "aggregate OD route station-age bitsets" begin
        label = AggregateODRoutePricingLabel(
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
        bs = StationSelection._make_aggregate_od_route_label_bitsets(label, pair_index, 2, node_index, 4)
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
        duals = AggregateODRoutePricingDuals(Dict((3, 4) => 10.0))
        label = AggregateODRoutePricingLabel(
            2,
            [1, 2],
            1.0,
            Dict{Int, Float64}(),
            Set{Tuple{Int, Int}}(),
            1.0,
            1.0,
            2,
        )

        candidates = StationSelection._aggregate_od_route_candidate_next_nodes(label, pricing_data, duals)
        @test 3 in candidates
    end

    @testset "pricing returns improving columns for one scenario" begin
        pricing_data = line_pricing_data(active_pairs=[(1, 3), (3, 4), (1, 4)])
        existing = AggregateODRouteColumn[
            AggregateODRouteColumn(1, [(1, 3)], 2.0),
            AggregateODRouteColumn(2, [(3, 4)], 1.0),
            AggregateODRouteColumn(3, [(1, 4)], 3.0),
        ]
        duals = AggregateODRoutePricingDuals(Dict((1, 4) => 10.0, (1, 3) => 10.0, (3, 4) => 10.0))

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
        duals = AggregateODRoutePricingDuals(Dict((1, 4) => 10.0, (1, 3) => 10.0, (3, 4) => 10.0))

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
            duals = AggregateODRoutePricingDuals(Dict((1, 3) => 10.0))
            labels = StationSelection._initial_aggregate_od_route_station_simple_labels(pricing_data, duals)

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
            duals = AggregateODRoutePricingDuals(Dict((1, 3) => 10.0))
            label = only(
                filter(
                    label -> label.current == 1,
                    StationSelection._initial_aggregate_od_route_station_simple_labels(pricing_data, duals),
                ),
            )
            @test_throws ArgumentError StationSelection._extend_aggregate_od_route_station_simple_label(
                label, 1, pricing_data, duals,
            )
        end

        @testset "extension certifies destination visits and updates reduced cost" begin
            pricing_data = line_pricing_data(active_pairs=[(1, 3), (2, 4)])
            duals = AggregateODRoutePricingDuals(Dict((1, 3) => 10.0, (2, 4) => 8.0))
            initial_1 = only(
                filter(
                    label -> label.current == 1,
                    StationSelection._initial_aggregate_od_route_station_simple_labels(pricing_data, duals),
                ),
            )
            child_3 = StationSelection._extend_aggregate_od_route_station_simple_label(initial_1, 3, pricing_data, duals)

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
            duals = AggregateODRoutePricingDuals(Dict((1, 3) => 10.0))
            initial_1 = only(
                filter(
                    label -> label.current == 1,
                    StationSelection._initial_aggregate_od_route_station_simple_labels(pricing_data, duals),
                ),
            )
            expired_child = StationSelection._extend_aggregate_od_route_station_simple_label(initial_1, 4, pricing_data, duals)

            @test (1, 3) ∉ expired_child.served_pairs
            @test !haskey(expired_child.live_origin_age, 1)
        end

        @testset "dominance requires an exact visited match and live-origin domination" begin
            same_visited_better = AggregateODRouteStationSimpleLabel(
                2, [1, 2], Set([1, 2]), 1.0, Dict(1 => 1.0), Set{Tuple{Int, Int}}(), 1.0, -2.0,
            )
            same_visited_worse = AggregateODRouteStationSimpleLabel(
                2, [1, 2], Set([1, 2]), 2.0, Dict(1 => 2.0), Set{Tuple{Int, Int}}(), 1.0, -1.0,
            )
            different_visited = AggregateODRouteStationSimpleLabel(
                2, [3, 2], Set([3, 2]), 1.0, Dict(3 => 1.0), Set{Tuple{Int, Int}}(), 1.0, -2.0,
            )
            different_current = AggregateODRouteStationSimpleLabel(
                3, [1, 3], Set([1, 3]), 1.0, Dict(1 => 1.0), Set{Tuple{Int, Int}}(), 1.0, -2.0,
            )

            node_index = Dict(1 => 1, 2 => 2, 3 => 3, 4 => 4)
            better_bs = StationSelection._make_aggregate_od_route_station_simple_bitsets(same_visited_better, node_index)
            worse_bs = StationSelection._make_aggregate_od_route_station_simple_bitsets(same_visited_worse, node_index)
            different_visited_bs = StationSelection._make_aggregate_od_route_station_simple_bitsets(different_visited, node_index)
            different_current_bs = StationSelection._make_aggregate_od_route_station_simple_bitsets(different_current, node_index)

            @test StationSelection._dominates_aggregate_od_route_station_simple_label(
                same_visited_better, same_visited_worse, better_bs, worse_bs,
            )
            @test !StationSelection._dominates_aggregate_od_route_station_simple_label(
                same_visited_worse, same_visited_better, worse_bs, better_bs,
            )
            @test !StationSelection._dominates_aggregate_od_route_station_simple_label(
                same_visited_better, different_visited, better_bs, different_visited_bs,
            )
            @test !StationSelection._dominates_aggregate_od_route_station_simple_label(
                same_visited_better, different_current, better_bs, different_current_bs,
            )
        end

        @testset "candidate generation never offers a visited station" begin
            pricing_data = line_pricing_data(active_pairs=[(1, 3), (3, 4)], detour_factor=3.0, max_wait_time=10.0)
            duals = AggregateODRoutePricingDuals(Dict((1, 3) => 10.0, (3, 4) => 10.0))
            label = AggregateODRouteStationSimpleLabel(
                1, [1], Set([1]), 0.0, Dict(1 => 0.0), Set{Tuple{Int, Int}}(), 0.0, 0.0,
            )
            candidates = StationSelection._aggregate_od_route_station_simple_candidate_next_nodes(label, pricing_data, duals)
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
            duals = AggregateODRoutePricingDuals(Dict((1, 4) => 10.0, (1, 3) => 10.0, (3, 4) => 10.0))

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
end
