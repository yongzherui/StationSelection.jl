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
            max_visits_per_node=2,
            bounded_max_stops=true,
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
            max_visits_per_node,
            bounded_max_stops,
        )
    end

    function label_at_current(labels, current)
        return only(filter(label -> label.current == current, labels))
    end

    @testset "unbounded max_stops uses the finite visit ceiling" begin
        @test StationSelection._resolve_aggregate_od_route_max_stops(
            typemax(Int), 3, 4,
        ) == 12
        @test StationSelection._resolve_aggregate_od_route_max_stops(7, 3, 4) == 7
        @test_throws ArgumentError StationSelection._resolve_aggregate_od_route_max_stops(
            typemax(Int), typemax(Int), 4,
        )
    end

    @testset "pricing max_stops resolution allows fully unbounded search" begin
        @test StationSelection._resolve_aggregate_od_route_pricing_max_stops(
            typemax(Int), 3, 4,
        ) == 12
        @test StationSelection._resolve_aggregate_od_route_pricing_max_stops(7, 3, 4) == 7
        # unlike enumeration's resolver, both limits unbounded is a legitimate
        # pricing configuration (dominance/reduced-cost bound the search instead
        # of a route-length ceiling), not an error.
        @test StationSelection._resolve_aggregate_od_route_pricing_max_stops(
            typemax(Int), typemax(Int), 4,
        ) == typemax(Int)
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

    @testset "enumeration matches independent bounded brute force" begin
        stations = DataFrame(
            id=[1, 2, 3],
            lon=[0.0, 1.0, 2.0],
            lat=[0.0, 0.0, 0.0],
        )
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 2],
            end_station_id=[3, 3],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
        )
        walking_costs = Dict((i, j) => (i == j ? 0.0 : 100.0) for i in 1:3, j in 1:3)
        routing_costs = Dict((i, j) => Float64(abs(i - j)) for i in 1:3, j in 1:3)
        data = create_station_selection_data(
            stations,
            requests,
            walking_costs;
            routing_costs=routing_costs,
        )
        active_pairs = [(1, 3), (2, 3)]

        function brute_signature_costs(max_wait_time::Float64, max_stops::Int, max_visits::Int)
            best = Dict{Tuple{Vararg{Tuple{Int, Int}}}, Float64}()
            function visit!(route::Vector{Int}, times::Vector{Float64})
                if length(route) >= 2
                    served = Tuple{Int, Int}[]
                    for pair in active_pairs
                        j, k = pair
                        feasible = any(
                            route[p] == j && times[p] <= max_wait_time + 1e-9 &&
                            route[q] == k &&
                            times[q] - times[p] <= 2.0 * routing_costs[(j, k)] + 1e-9
                            for p in 1:(length(route) - 1) for q in (p + 1):length(route)
                        )
                        feasible && push!(served, pair)
                    end
                    if !isempty(served)
                        signature = Tuple(sort!(served))
                        best[signature] = min(get(best, signature, Inf), times[end])
                    end
                end
                length(route) >= max_stops && return
                for next_node in 1:3
                    next_node == route[end] && continue
                    count(==(next_node), route) < max_visits || continue
                    visit!(vcat(route, next_node), vcat(times, times[end] + routing_costs[(route[end], next_node)]))
                end
            end
            for start in 1:3
                visit!([start], [0.0])
            end
            return best
        end

        for max_wait_time in (0.5, 1.0, 10.0), max_stops in (3, 4)
            model = AggregateODRouteModel(
                3;
                assignment_policy=NearestOpenAggregateODAssignmentPolicy(),
                max_walking_distance=0.0,
                max_wait_time=max_wait_time,
                detour_factor=2.0,
                max_stops=max_stops,
                max_visits_per_node=2,
                repositioning_time=0.0,
            )
            columns = enumerate_aggregate_od_route_columns(
                model,
                data;
                max_routes=10_000,
                time_limit_sec=5.0,
            )
            actual = Dict(
                Tuple(sort(column.od_pairs)) => column.tau
                for column in columns
            )
            @test actual == brute_signature_costs(max_wait_time, max_stops, 2)
        end
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
        @test bs.station_age[node_index[1]] == 1.0
        @test bs.station_age[node_index[2]] == 0.0
        @test isinf(bs.station_age[node_index[3]])
    end

    @testset "label-setting candidate generation enforces max visits per node" begin
        pricing_data = line_pricing_data(
            active_pairs=[(1, 2)],
            detour_factor=3.0,
            max_stops=4,
            max_visits_per_node=1,
        )
        duals = AggregateODRoutePricingDuals(Dict((1, 2) => 10.0))
        label = AggregateODRoutePricingLabel(
            1,
            [1, 2, 1],
            2.0,
            Dict(1 => 0.0),
            Set{Tuple{Int, Int}}(),
            2.0,
            -1.0,
            3,
        )

        candidates = StationSelection._aggregate_od_route_candidate_next_nodes(label, pricing_data, duals)
        @test isempty(candidates)
        relaxed_candidates = StationSelection._aggregate_od_route_candidate_next_nodes(
            label,
            pricing_data,
            duals;
            max_visits_per_node=2,
        )
        @test relaxed_candidates == [2]
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

    @testset "scenario pricing uses only scenario-specific duals" begin
        stations = DataFrame(id=[1, 2, 3, 4], lon=[0.0, 1.0, 2.0, 3.0], lat=[0.0, 0.0, 0.0, 0.0])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 2],
            end_station_id=[4, 3],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 10)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:4, j in 1:4
            walking_costs[(i, j)] = i == j ? 0.0 : 100.0
            routing_costs[(i, j)] = abs(i - j)
        end
        data = create_station_selection_data(
            stations,
            requests,
            walking_costs;
            routing_costs=routing_costs,
            scenarios=[
                ("2024-01-01 08:00:00", "2024-01-01 09:00:00"),
                ("2024-01-01 10:00:00", "2024-01-01 11:00:00"),
            ],
        )
        model = AggregateODRouteModel(
            4;
            max_walking_distance=1000.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=4,
            max_wait_time=100.0,
            max_new_columns=5,
            n_candidates=5,
        )
        mapping = create_map(model, data)
        duals = AggregateODRouteCoverageDuals(
            Dict{NTuple{3, Int}, Float64}(),
            Dict((1, 4, 1) => 10.0, (2, 4, 1) => 10.0, (2, 3, 2) => 0.0),
        )
        build_result = BuildResult(JuMP.Model(), mapping, nothing, nothing, Dict{String, Any}())
        build_result.model[:aggregate_od_route_station_budget] = model.l
        build_result.model[:aggregate_od_route_route_regularization_weight] = model.route_regularization_weight
        build_result.model[:aggregate_od_route_repositioning_time] = model.repositioning_time
        build_result.model[:aggregate_od_route_max_wait_time] = model.max_wait_time
        build_result.model[:aggregate_od_route_detour_factor] = model.detour_factor
        build_result.model[:aggregate_od_route_max_stops] = model.max_stops
        build_result.model[:aggregate_od_route_max_visits_per_node] = model.max_visits_per_node
        build_result.model[:aggregate_od_route_max_new_columns] = model.max_new_columns
        build_result.model[:aggregate_od_route_n_candidates] = model.n_candidates
        build_result.model[:aggregate_od_route_pricing_time_limit_sec] = model.pricing_time_limit_sec
        build_result.model[:aggregate_od_route_reduced_cost_tol] = model.reduced_cost_tol
        build_result.model[:aggregate_od_route_relax_integrality] = true

        columns = generate_aggregate_od_route_columns(build_result, duals, data)

        @test !isempty(columns)
        @test any(column -> column.metadata["scenario"] == 1, columns)
        @test all(column -> column.metadata["scenario"] != 2, columns)
    end

    @testset "priced columns can be added to the restricted master" begin
        gurobi_available = try
            using Gurobi
            true
        catch
            false
        end
        if !gurobi_available
            @warn "Gurobi not available, skipping add-column aggregate OD route test"
            @test true
            return
        end

        stations = DataFrame(id=[1, 2, 3], lon=[0.0, 1.0, 2.0], lat=[0.0, 0.0, 0.0])
        requests = DataFrame(
            id=[1],
            start_station_id=[1],
            end_station_id=[3],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:3, j in 1:3
            walking_costs[(i, j)] = i == j ? 0.0 : 100.0
            routing_costs[(i, j)] = abs(i - j)
        end
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            3;
            max_walking_distance=1000.0,
            route_regularization_weight=2.0,
            repositioning_time=5.0,
            max_stops=3,
            max_wait_time=100.0,
            relax_integrality=true,
        )
        build_result = build_model(model, data; optimizer_env=Gurobi.Env())
        column = AggregateODRouteColumn(999, [(1, 2), (2, 3)], 2.0; metadata=Dict{String, Any}("scenario" => 1))

        add_aggregate_od_route_column!(build_result, column)

        theta = build_result.model[:theta_compat][(999, 1)]
        con = first(build_result.model[:aggregate_od_route_coverage_by_pair_s][(1, 2, 1)])
        @test JuMP.coefficient(JuMP.objective_function(build_result.model), theta) == 14.0
        @test JuMP.normalized_coefficient(con, theta) == 1.0

        cheaper = AggregateODRouteColumn(1000, [(1, 2), (2, 3)], 1.0; metadata=Dict{String, Any}("scenario" => 1))
        _theta, action = add_or_update_aggregate_od_route_column!(build_result, cheaper)
        @test action == :replaced
        @test build_result.mapping.columns[findfirst(c -> c.id == 999, build_result.mapping.columns)].tau == 1.0
        @test JuMP.coefficient(JuMP.objective_function(build_result.model), theta) == 12.0

        worse = AggregateODRouteColumn(1001, [(1, 2), (2, 3)], 3.0; metadata=Dict{String, Any}("scenario" => 1))
        _theta2, action2 = add_or_update_aggregate_od_route_column!(build_result, worse)
        @test action2 == :skipped
    end

    @testset "restricted master solves from simple StationSelectionData" begin
        gurobi_available = try
            using Gurobi
            true
        catch
            false
        end
        if !gurobi_available
            @warn "Gurobi not available, skipping AggregateODRouteModel RMP solve test"
            @test true
            return
        end

        stations = DataFrame(id=[1, 2, 3], lon=[0.0, 1.0, 2.0], lat=[0.0, 0.0, 0.0])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 1],
            end_station_id=[3, 3],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 5)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:3, j in 1:3
            walking_costs[(i, j)] = i == j ? 0.0 : 100.0
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            2;
            max_walking_distance=1000.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
        )
        result = run_opt(data, model, DirectSolver(optimizer_env=Gurobi.Env(), silent=true))

        @test result.termination_status == MOI.OPTIMAL
        @test !isnothing(result.objective_value)
        @test result.counts.variables["station_selection"] == 3
        @test !haskey(result.counts.variables, "od_activation")
        @test result.counts.variables["aggregate_od_route_theta"] > 0
        @test result.counts.constraints["aggregate_od_route_coverage"] > 0

        m = result.model
        mapping = result.mapping
        y = value.(m[:y])
        x = m[:x]
        theta = m[:theta_compat]

        for s in 1:n_scenarios(data)
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (pair_idx, (j, k)) in enumerate(valid_pairs)
                    x_val = value(x[s][od_idx][pair_idx])
                    x_val <= 1e-7 && continue

                    @test y[j] >= 1e-6
                    @test y[k] >= 1e-6
                    covered = sum(
                        value(theta[(column_id, s)])
                        for column_id in get(mapping.columns_by_pair, (j, k), Int[]);
                        init=0.0,
                    )
                    @test covered + 1e-6 >= x_val
                end
            end
        end
    end

    @testset "column generation returns logs and final integer result" begin
        gurobi_available = try
            using Gurobi
            true
        catch
            false
        end
        if !gurobi_available
            @warn "Gurobi not available, skipping AggregateODRouteModel CG test"
            @test true
            return
        end

        stations = DataFrame(id=[1, 2, 3, 4], lon=[0.0, 1.0, 2.0, 3.0], lat=[0.0, 0.0, 0.0, 0.0])
        requests = DataFrame(
            id=[1],
            start_station_id=[1],
            end_station_id=[4],
            request_time=[DateTime(2024, 1, 1, 8)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:4, j in 1:4
            walking_costs[(i, j)] = i == j ? 0.0 : 100.0
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        model = AggregateODRouteModel(
            4;
            max_walking_distance=1000.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=4,
            max_wait_time=100.0,
            max_new_columns=5,
            n_candidates=5,
        )

        tmpdir = mktempdir()
        cg_log = joinpath(tmpdir, "cg.csv")
        col_log = joinpath(tmpdir, "columns.csv")
        dual_log = joinpath(tmpdir, "duals.csv")
        cg_result = run_aggregate_od_route_column_generation(
            model,
            data;
            verbose=false,
            max_cg_iters=2,
            pricing_time_limit_sec=5.0,
            cg_log_path=cg_log,
            column_log_path=col_log,
            dual_log_path=dual_log,
        )

        @test cg_result isa AggregateODRouteColumnGenerationResult
        @test cg_result.final_result.termination_status == MOI.OPTIMAL
        @test cg_result.n_cg_iters >= 1
        @test !isempty(cg_result.iteration_rows)
        @test !isempty(cg_result.generated_columns)
        @test isfile(cg_log)
        @test isfile(col_log)
        @test isfile(dual_log)
        @test occursin("iteration", read(cg_log, String))
        @test occursin("reduced_cost", read(col_log, String))
        @test occursin("sigma", read(dual_log, String))
    end

    @testset "nearest-open aggregate assignment and route covering API" begin
        gurobi_available = try
            using Gurobi
            true
        catch
            false
        end
        if !gurobi_available
            @warn "Gurobi not available, skipping nearest-open aggregate OD route tests"
            @test true
            return
        end

        stations = DataFrame(id=[1, 2, 3], lon=[0.0, 1.0, 2.0], lat=[0.0, 0.0, 0.0])
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 1],
            end_station_id=[3, 3],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 5)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:3, j in 1:3
            walking_costs[(i, j)] = abs(i - j)
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)

        nearest = AggregateODRouteModel(
            2;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(),
            max_walking_distance=10.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
            max_new_columns=2,
            n_candidates=2,
            pricing_time_limit_sec=2.0,
        )
        result = run_opt(
            data,
            nearest,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                decomposition=BendersY(),
                max_iterations=10,
                inner_solver=ColumnGenerationSolver(
                    config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                    max_iterations=20,
                    max_columns_per_iteration=2,
                    n_candidates=2,
                    pricing_time_limit_sec=2.0,
                ),
            ),
        )

        @test result.termination_status == MOI.OPTIMAL
        @test result.metadata["solve_method"] == "benders"
        @test result.metadata["benders_decomposition"] == "BendersY"
        @test result.metadata["benders_cut_mode"] == "MultiCut(scenario)"
        @test result.metadata["benders_iterations"] >= 1
        @test result.metadata["optimality_cuts_added"] >= 1
        @test result.metadata["inner_cg_iterations"] >= 1
        @test result.metadata["feasibility_cut_style"] == "big_m_nearest"
        @test result.metadata["benders_outer_gap_warning_tol"] == 0.03
        @test result.metadata["benders_outer_gap_within_warning_tol"] isa Bool
        y = value.(result.model[:y])
        x = result.model[:x]
        mapping = result.mapping
        for s in 1:n_scenarios(data)
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                pairs = get_valid_jk_pairs(mapping, o, d)
                open_pairs = [pair for pair in pairs if y[pair[1]] > 0.5 && y[pair[2]] > 0.5]
                isempty(open_pairs) && continue
                nearest_pair = first(sort(open_pairs, by=pair -> (
                    get_walking_cost(data, o, pair[1]) + get_walking_cost(data, pair[2], d),
                    pair[1],
                    pair[2],
                )))
                assigned_idx = findfirst(pair_idx -> value(x[s][od_idx][pair_idx]) > 1e-6, eachindex(pairs))
                @test pairs[assigned_idx] == nearest_pair
            end
        end

        @test BendersSolver().decomposition isa BendersY
        @test BendersSolver().cut_mode isa MultiCut
        @test BendersSolver().inner_solver isa ColumnGenerationSolver
        @test BendersSolver().cut_derivation == :zero_completion
        @test BendersSolver().outer_gap_warning_tol == 0.03
        @test BendersSolver().max_reprice_rounds == 10_000
        @test_throws ArgumentError BendersSolver(outer_gap_warning_tol=-0.01)
        @test StationSelection._outer_gap_absolute(12.0, 10.0) == 2.0
        @test StationSelection._outer_gap_relative(12.0, 10.0) == 0.2
        diagnostic_solver = BendersSolver(cut_derivation=:standard, reprice_subproblem=false)
        @test_logs (:warn, r"diagnostics only") begin
            @test StationSelection._warn_if_uncertified_standard_cut(diagnostic_solver)
        end
        @test !StationSelection._warn_if_uncertified_standard_cut(BendersSolver())
        @test BendersSolver(inner_solver=DirectSolver(silent=true)).inner_solver isa DirectSolver
        xy_result = run_opt(
            data,
            nearest,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                decomposition=BendersXY(),
                max_iterations=10,
                inner_solver=ColumnGenerationSolver(
                    config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                    max_iterations=20,
                    max_columns_per_iteration=2,
                    n_candidates=2,
                    pricing_time_limit_sec=2.0,
                ),
            ),
        )
        @test xy_result.termination_status == MOI.OPTIMAL
        @test xy_result.metadata["solve_method"] == "benders"
        @test xy_result.metadata["benders_decomposition"] == "BendersXY"
        @test xy_result.metadata["benders_cut_mode"] == "MultiCut(scenario)"
        @test xy_result.metadata["benders_iterations"] >= 1
        @test xy_result.metadata["optimality_cuts_added"] >= 1
        @test xy_result.metadata["inner_cg_iterations"] >= 1

        @test_throws ArgumentError run_opt(
            data,
            AggregateODRouteModel(
                2;
                max_walking_distance=10.0,
                route_regularization_weight=1.0,
                repositioning_time=0.0,
                max_stops=3,
                max_wait_time=100.0,
                max_new_columns=2,
                n_candidates=2,
                pricing_time_limit_sec=2.0,
            ),
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                decomposition=BendersY(),
                max_iterations=10,
            ),
        )

        benders_log_dir = mktempdir()
        unrestricted = AggregateODRouteModel(
            2;
            max_walking_distance=10.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
            max_new_columns=2,
            n_candidates=2,
            pricing_time_limit_sec=2.0,
        )
        unrestricted_result = run_opt(
            data,
            unrestricted,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                decomposition=BendersXY(),
                max_iterations=10,
                log_dir=benders_log_dir,
                inner_solver=ColumnGenerationSolver(
                    config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                    max_iterations=20,
                    max_columns_per_iteration=2,
                    n_candidates=2,
                    pricing_time_limit_sec=2.0,
                    log_dir=benders_log_dir,
                ),
            ),
        )
        @test unrestricted_result.termination_status == MOI.OPTIMAL
        @test unrestricted_result.metadata["solve_method"] == "benders"
        @test unrestricted_result.metadata["benders_decomposition"] == "BendersXY"
        @test unrestricted_result.metadata["selected_assignment_count"] == 1
        @test isfile(joinpath(benders_log_dir, "aggregate_od_route_benders_iterations.csv"))
        @test any(
            f -> occursin(r"aggregate_od_route_benders_subiter\d+_cg_iterations\.csv", f),
            readdir(benders_log_dir),
        )

        # A walking distance wide enough that pickup and dropoff candidate
        # sets fully coincide ({1,2,3} on both sides) must still build: the
        # off-diagonal (j != k) pair list is necessarily a strict subset of
        # the full n x n Cartesian product (the diagonal j==k is never a
        # real station pair -- see WALK_ONLY_PAIR), so `_check_big_m_nearest_pair_consistency!`
        # compares against the *off-diagonal* Cartesian product, not the
        # full one. (Previously this threw ArgumentError because the
        # validator required literal full Cartesian coverage including the
        # diagonal, which can never hold once j != k is enforced -- that was
        # a validator bug, not a real modeling restriction.)
        wide_big_m = AggregateODRouteModel(
            2;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=10.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
        )
        wide_big_m_build = StationSelection.build_model(
            wide_big_m,
            data;
            optimizer_env=Gurobi.Env(),
        )
        @test wide_big_m_build isa StationSelection.BuildResult

        tight_big_m = AggregateODRouteModel(
            2;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=0.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
            max_new_columns=2,
            n_candidates=2,
            pricing_time_limit_sec=2.0,
        )
        tight_direct = run_opt(
            data,
            tight_big_m,
            DirectSolver(
                optimizer_env=Gurobi.Env(),
                silent=true,
                max_enumerated_routes=1000,
                max_enumeration_time_sec=5.0,
            ),
        )
        tight_benders = run_opt(
            data,
            tight_big_m,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                decomposition=BendersXY(),
                max_iterations=10,
                inner_solver=ColumnGenerationSolver(
                    config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                    max_iterations=20,
                    max_columns_per_iteration=2,
                    n_candidates=2,
                    pricing_time_limit_sec=2.0,
                ),
            ),
        )
        @test tight_direct.objective_value ≈ tight_benders.objective_value

        direct_result = run_opt(
            data,
            nearest,
            DirectSolver(
                optimizer_env=Gurobi.Env(),
                silent=true,
                max_enumerated_routes=1000,
                max_enumeration_time_sec=5.0,
            ),
        )
        @test direct_result.termination_status == MOI.OPTIMAL
        @test direct_result.metadata["solve_method"] == "route_enumeration"
        @test direct_result.metadata["enumerated_routes"] > 0

        expensive_routing = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:3, j in 1:3
            expensive_routing[(i, j)] = i == j ? 0.0 : 100.0
        end
        expensive_data = create_station_selection_data(
            stations,
            requests[1:1, :],
            walking_costs;
            routing_costs=expensive_routing,
        )
        expensive_nearest = AggregateODRouteModel(
            2;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(),
            max_walking_distance=10.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=2,
            max_wait_time=1_000.0,
        )
        expensive_columns = enumerate_aggregate_od_route_columns(
            expensive_nearest,
            expensive_data;
            max_routes=100,
            time_limit_sec=5.0,
        )
        @test any(column -> (1, 3) in column.od_pairs, expensive_columns)

        @test_throws ArgumentError run_opt(
            data,
            nearest,
            DirectSolver(
                optimizer_env=Gurobi.Env(),
                silent=true,
                max_enumerated_routes=1,
                max_enumeration_time_sec=5.0,
            ),
        )

        route_covering = RouteCoveringProblem(
            2,
            [1, 3],
            Dict((1, 1, 3) => (1, 3));
            max_walking_distance=10.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
            max_new_columns=2,
            n_candidates=2,
            pricing_time_limit_sec=2.0,
        )
        route_result = run_opt(
            data,
            route_covering,
            ColumnGenerationSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true),
                max_iterations=1,
                max_columns_per_iteration=2,
                n_candidates=2,
                pricing_time_limit_sec=2.0,
            ),
        )

        @test route_result.termination_status == MOI.OPTIMAL
        @test route_result.counts.constraints["fixed_open_stations"] == data.n_stations
        @test get_valid_jk_pairs(route_result.mapping, 1, 3) == [(1, 3)]

        route_direct_result = run_opt(
            data,
            route_covering,
            DirectSolver(
                optimizer_env=Gurobi.Env(),
                silent=true,
                max_enumerated_routes=1000,
                max_enumeration_time_sec=5.0,
            ),
        )
        @test route_direct_result.termination_status == MOI.OPTIMAL
        @test route_direct_result.metadata["solve_method"] == "route_enumeration"
    end
end
