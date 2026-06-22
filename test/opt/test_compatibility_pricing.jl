@testset "CompatibilitySetModel label-setting pricing" begin
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
        )
        return CompatibilityPricingData(
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
        )
    end

    function label_at_current(labels, current)
        return only(filter(label -> label.current == current, labels))
    end

    @testset "initial labels create live pickup opportunities" begin
        pricing_data = line_pricing_data()
        duals = CompatibilityPricingDuals(Dict((1, 3) => 10.0, (2, 4) => 8.0))
        initial_1 = label_at_current(initial_compatibility_pricing_labels(pricing_data, duals), 1)

        @test isempty(initial_1.served_pairs)
        @test haskey(initial_1.live_opportunity_expiry, (1, 3))
        @test initial_1.live_opportunity_expiry[(1, 3)] == 3.0
        @test initial_1.reduced_cost == 0.0
    end

    @testset "extension certifies destination visits and updates reduced cost" begin
        pricing_data = line_pricing_data()
        duals = CompatibilityPricingDuals(Dict((1, 3) => 10.0, (2, 4) => 8.0))
        initial_1 = label_at_current(initial_compatibility_pricing_labels(pricing_data, duals), 1)
        child_3 = only(extend_compatibility_pricing_label(initial_1, 3, pricing_data, duals))

        @test child_3.current == 3
        @test child_3.route == [1, 3]
        @test child_3.time == 2.0
        @test child_3.tau == 2.0
        @test child_3.served_pairs == Set([(1, 3)])
        @test child_3.reduced_cost == -8.0
    end

    @testset "expired opportunities are pruned before certification" begin
        pricing_data = line_pricing_data(detour_factor=1.0)
        duals = CompatibilityPricingDuals(Dict((1, 3) => 10.0))
        initial_1 = label_at_current(initial_compatibility_pricing_labels(pricing_data, duals), 1)
        expired_child = only(extend_compatibility_pricing_label(initial_1, 4, pricing_data, duals))

        @test (1, 3) ∉ expired_child.served_pairs
        @test !haskey(expired_child.live_opportunity_expiry, (1, 3))
    end

    @testset "dominance respects time reduced cost served and live states" begin
        good = CompatibilityPricingLabel(
            2,
            [2],
            1.0,
            Set{Tuple{Int, Int}}(),
            Dict((2, 4) => 5.0),
            0.0,
            1.0,
        )
        worse = CompatibilityPricingLabel(
            2,
            [2],
            2.0,
            Set([(1, 3)]),
            Dict((2, 4) => 4.0),
            0.0,
            2.0,
        )
        different_node = CompatibilityPricingLabel(
            3,
            [3],
            2.0,
            Set{Tuple{Int, Int}}(),
            Dict((2, 4) => 4.0),
            0.0,
            2.0,
        )

        @test StationSelection._dominates_compatibility_label(good, worse)
        @test !StationSelection._dominates_compatibility_label(worse, good)
        @test !StationSelection._dominates_compatibility_label(good, different_node)

        pair_index = Dict((1, 3) => 1, (2, 4) => 2)
        good_bs = StationSelection._make_compatibility_label_bitsets(good, pair_index, 2)
        worse_bs = StationSelection._make_compatibility_label_bitsets(worse, pair_index, 2)
        @test StationSelection._dominates_compatibility_label(good, worse, good_bs, worse_bs)
        @test !StationSelection._dominates_compatibility_label(worse, good, worse_bs, good_bs)
    end

    @testset "compatibility bitset operations" begin
        bs = StationSelection.CompatibilityPairBitset(256)
        bs = StationSelection._compatibility_setbit(bs, 1)
        bs = StationSelection._compatibility_setbit(bs, 130)
        sup = StationSelection._compatibility_setbit(bs, 200)
        @test issubset(bs, sup)
        @test !issubset(sup, bs)

        large = StationSelection.CompatibilityPairBitset(512)
        large = StationSelection._compatibility_setbit(large, 300)
        large = StationSelection._compatibility_setbit(large, 400)
        @test !issubset(large, bs)
    end

    @testset "pricing returns improving columns for one scenario" begin
        pricing_data = line_pricing_data(active_pairs=[(1, 3), (3, 4), (1, 4)])
        existing = CompatibilityColumn[
            CompatibilityColumn(1, [(1, 3)], 2.0),
            CompatibilityColumn(2, [(3, 4)], 1.0),
            CompatibilityColumn(3, [(1, 4)], 3.0),
        ]
        duals = CompatibilityPricingDuals(Dict((1, 4) => 10.0, (1, 3) => 10.0, (3, 4) => 10.0))

        columns, exhausted, stats = compatibility_pricing_by_label_setting(
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
        model = CompatibilitySetModel(
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
        duals = CompatibilityCoverageDuals(
            Dict{NTuple{3, Int}, Float64}(),
            Dict((1, 4, 1) => 10.0, (2, 3, 2) => 0.0),
        )
        build_result = BuildResult(JuMP.Model(), mapping, nothing, nothing, Dict{String, Any}())
        build_result.model[:compatibility_station_budget] = model.l
        build_result.model[:compatibility_route_regularization_weight] = model.route_regularization_weight
        build_result.model[:compatibility_repositioning_time] = model.repositioning_time
        build_result.model[:compatibility_max_wait_time] = model.max_wait_time
        build_result.model[:compatibility_detour_factor] = model.detour_factor
        build_result.model[:compatibility_max_stops] = model.max_stops
        build_result.model[:compatibility_max_visits_per_node] = model.max_visits_per_node
        build_result.model[:compatibility_max_new_columns] = model.max_new_columns
        build_result.model[:compatibility_n_candidates] = model.n_candidates
        build_result.model[:compatibility_pricing_time_limit_sec] = model.pricing_time_limit_sec
        build_result.model[:compatibility_reduced_cost_tol] = model.reduced_cost_tol
        build_result.model[:compatibility_relax_integrality] = true

        columns = generate_compatibility_columns(build_result, duals, data)

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
            @warn "Gurobi not available, skipping add-column compatibility test"
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
        model = CompatibilitySetModel(
            3;
            max_walking_distance=1000.0,
            route_regularization_weight=2.0,
            repositioning_time=5.0,
            max_stops=3,
            max_wait_time=100.0,
            relax_integrality=true,
        )
        build_result = build_model(model, data; optimizer_env=Gurobi.Env())
        column = CompatibilityColumn(999, [(1, 2), (2, 3)], 2.0; metadata=Dict{String, Any}("scenario" => 1))

        add_compatibility_column!(build_result, column)

        theta = build_result.model[:theta_compat][(999, 1)]
        con = build_result.model[:compatibility_coverage_constraints][(1, 2, 1)]
        @test JuMP.coefficient(JuMP.objective_function(build_result.model), theta) == 14.0
        @test JuMP.normalized_coefficient(con, theta) == 1.0

        cheaper = CompatibilityColumn(1000, [(1, 2), (2, 3)], 1.0; metadata=Dict{String, Any}("scenario" => 1))
        _theta, action = add_or_update_compatibility_column!(build_result, cheaper)
        @test action == :replaced
        @test build_result.mapping.columns[findfirst(c -> c.id == 999, build_result.mapping.columns)].tau == 1.0
        @test JuMP.coefficient(JuMP.objective_function(build_result.model), theta) == 12.0

        worse = CompatibilityColumn(1001, [(1, 2), (2, 3)], 3.0; metadata=Dict{String, Any}("scenario" => 1))
        _theta2, action2 = add_or_update_compatibility_column!(build_result, worse)
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
            @warn "Gurobi not available, skipping CompatibilitySetModel RMP solve test"
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
        model = CompatibilitySetModel(
            2;
            max_walking_distance=1000.0,
            route_regularization_weight=1.0,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=100.0,
        )
        result = run_opt(model, data; optimizer_env=Gurobi.Env(), silent=true)

        @test result.termination_status == MOI.OPTIMAL
        @test !isnothing(result.objective_value)
        @test result.counts.variables["station_selection"] == 3
        @test result.counts.variables["od_activation"] > 0
        @test result.counts.variables["compatibility_theta"] > 0
        @test result.counts.constraints["compatibility_coverage"] > 0

        m = result.model
        mapping = result.mapping
        y = value.(m[:y])
        x = m[:x]
        u = m[:u]
        theta = m[:theta_compat]

        for s in 1:n_scenarios(data)
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                demand = mapping.Q_s[s][(o, d)]
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (pair_idx, (j, k)) in enumerate(valid_pairs)
                    x_val = value(x[s][od_idx][pair_idx])
                    x_val <= 1e-7 && continue

                    @test y[j] >= 1e-6
                    @test y[k] >= 1e-6
                    @test value(u[(j, k, s)]) + 1e-6 >= x_val / demand
                end
            end

            for (j, k) in mapping.active_jk_s[s]
                u_val = value(u[(j, k, s)])
                u_val <= 1e-7 && continue
                covered = sum(
                    value(theta[(column_id, s)])
                    for column_id in get(mapping.columns_by_pair, (j, k), Int[]);
                    init=0.0,
                )
                @test covered + 1e-6 >= u_val
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
            @warn "Gurobi not available, skipping CompatibilitySetModel CG test"
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
        model = CompatibilitySetModel(
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
        cg_result = run_compatibility_column_generation(
            model,
            data;
            verbose=false,
            max_cg_iters=2,
            pricing_time_limit_sec=5.0,
            cg_log_path=cg_log,
            column_log_path=col_log,
            dual_log_path=dual_log,
        )

        @test cg_result isa CompatibilityColumnGenerationResult
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
end
