@testset "AggregateODRouteModel :direct_ly (γ-chain, no x) nearest-open coverage" begin
    # --- Solver-free correctness check ------------------------------------------------
    # At binary y, the γ-chain recursion is exactly γ_r = γ_{r-1} OR (y_j AND y_k) (an OR of
    # "has some fully-open pair been found among ranks 1..r"), so x_r := γ_r - γ_{r-1} is 1
    # exactly for the FIRST (cheapest, jointly-ranked) fully-open pair and 0 elsewhere. This
    # simulates that recursion directly in plain Julia (no JuMP/Gurobi) rather than relying on
    # solving an LP, since at binary y the chain's four inequalities + final equality pin γ
    # uniquely (verified by hand: γ_r's lower and upper bounds coincide at every step given
    # γ_{r-1} is already binary -- see the box upper_bound=1.0, which is load-bearing once
    # γ_{r-1}=1 is already reached, since the row-based upper bound alone permits up to 2).
    @testset "γ-chain x_r = γ_r - γ_{r-1} matches nearest-open indicator on every binary y pattern" begin
        # Jointly-ranked (by combined cost) feasible pairs for one request, tie-broken by
        # (j,k) id, mirroring _rank_aggregate_od_route_pairs_by_assignment_cost exactly.
        stations = [10, 20, 30, 40]
        pairs = [(10, 40), (20, 40), (30, 40), (10, 20)]   # arbitrary feasible (j,k) pairs
        costs = [5.0, 3.0, 3.0, 5.0]                        # tie between rank-2/rank-3 (cost 3.0)
        order = sortperm(collect(eachindex(pairs)); by=i -> (costs[i], pairs[i]))
        ranked_pairs = pairs[order]
        # cost-3.0 tie: (20,40) < (30,40) by first element; cost-5.0 tie: (10,20) < (10,40)
        # by second element (tuple comparison, not id-of-whole-pair).
        @test ranked_pairs == [(20, 40), (30, 40), (10, 20), (10, 40)]

        function gamma_diffs(y::Dict{Int, Bool})
            R = length(ranked_pairs)
            gamma = falses(R)
            prev = false
            for (r, (j, k)) in enumerate(ranked_pairs)
                gamma[r] = prev || (y[j] && y[k])
                prev = gamma[r]
            end
            x = falses(R)
            prevg = false
            for r in 1:R
                x[r] = gamma[r] && !prevg
                prevg = gamma[r]
            end
            return x
        end

        n = length(stations)
        checked_at_least_one_open_case = false
        for pattern in 0:(2^n - 1)
            y = Dict{Int, Bool}(s => Bool((pattern >> (idx - 1)) & 1) for (idx, s) in enumerate(stations))
            any_pair_fully_open = any(((j, k),) -> y[j] && y[k], ranked_pairs)
            any_pair_fully_open || continue
            checked_at_least_one_open_case = true

            x = gamma_diffs(y)
            @test count(x) == 1  # exactly one rank fires
            first_open_rank = findfirst(((j, k),) -> y[j] && y[k], ranked_pairs)
            @test findfirst(x) == first_open_rank
        end
        @test checked_at_least_one_open_case
    end

    # --- Gurobi-based checks -----------------------------------------------------------
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping :direct_ly Gurobi-based checks"
        @test true
        return
    end

    # Identical geometry to the existing nearest_open_alignment_fixture
    # (test_aggregate_od_route_nearest_open_alignment.jl), rebuilt locally so this file has no
    # include-order dependency. Unlike the first (per-pair-only) attempt at :direct_ly, walking
    # cost is fully supported now (via the γ-chain's own differences), so this uses the SAME
    # walk_cost_weight the other styles use and should match their known optimum exactly.
    function direct_ly_fixture()
        stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
        requests = DataFrame(
            id=[1, 2],
            start_station_id=[1, 2],
            end_station_id=[5, 4],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            walking_costs[(i, j)] = 100.0
        end
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 2)] = 3.0
        walking_costs[(4, 5)] = 3.0
        walking_costs[(5, 5)] = 0.0
        walking_costs[(2, 2)] = 0.0
        walking_costs[(4, 4)] = 0.0
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    end

    function direct_ly_model(style::Symbol)
        return AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
    end

    @testset "allow_walk_only guard" begin
        bad_model = AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:direct_ly),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            max_stops=3,
            allow_walk_only=true,
        )
        data = direct_ly_fixture()
        @test_throws ArgumentError build_model(bad_model, data; optimizer_env=Gurobi.Env())
    end

    @testset "final IP matches DirectSolver ground truth and :big_m_nearest" begin
        data = direct_ly_fixture()
        model = direct_ly_model(:direct_ly)

        ground_truth = run_opt(
            data, model,
            DirectSolver(
                optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
            ),
        )
        @test ground_truth.termination_status == MOI.OPTIMAL
        y_gt = value.(ground_truth.model[:y])
        @test Set(j for j in eachindex(y_gt) if y_gt[j] > 0.5) == Set([1, 2, 4, 5])

        cg_direct_ly = run_aggregate_od_route_column_generation(
            model, data;
            optimizer_env=Gurobi.Env(), verbose=false,
            max_cg_iters=200, max_new_columns=20, n_candidates=20,
            ip_time_limit_sec=30.0, mip_gap=0.0, silent=true,
        )
        @test cg_direct_ly.cg_stop_reason == :optimality_proven
        @test cg_direct_ly.final_result.termination_status == MOI.OPTIMAL
        @test isapprox(cg_direct_ly.final_result.objective_value, ground_truth.objective_value; atol=1e-6)
        y_cg = value.(cg_direct_ly.final_result.model[:y])
        @test Set(j for j in eachindex(y_cg) if y_cg[j] > 0.5) == Set([1, 2, 4, 5])

        # Cross-check against the existing, already-trusted :big_m_nearest encoding on the
        # identical instance -- should match exactly now that walking cost is representable.
        cg_big_m = run_aggregate_od_route_column_generation(
            direct_ly_model(:big_m_nearest), data;
            optimizer_env=Gurobi.Env(), verbose=false,
            max_cg_iters=200, max_new_columns=20, n_candidates=20,
            ip_time_limit_sec=30.0, mip_gap=0.0, silent=true,
        )
        @test isapprox(cg_direct_ly.final_result.objective_value, cg_big_m.final_result.objective_value; atol=1e-6)
    end

    @testset "LP relaxation bound vs :big_m_nearest (same instance)" begin
        data = direct_ly_fixture()

        cg_direct_ly = run_aggregate_od_route_column_generation(
            direct_ly_model(:direct_ly), data;
            optimizer_env=Gurobi.Env(), verbose=false,
            max_cg_iters=200, max_new_columns=20, n_candidates=20,
            ip_time_limit_sec=30.0, mip_gap=0.0, silent=true,
        )
        cg_big_m = run_aggregate_od_route_column_generation(
            direct_ly_model(:big_m_nearest), data;
            optimizer_env=Gurobi.Env(), verbose=false,
            max_cg_iters=200, max_new_columns=20, n_candidates=20,
            ip_time_limit_sec=30.0, mip_gap=0.0, silent=true,
        )
        @test cg_direct_ly.cg_stop_reason == :optimality_proven
        @test cg_big_m.cg_stop_reason == :optimality_proven
        @info ":direct_ly vs :big_m_nearest LP bound (tiny fixture)" cg_direct_ly.lp_bound cg_big_m.lp_bound
        true_opt = cg_direct_ly.final_result.objective_value
        @test cg_direct_ly.lp_bound <= true_opt + 1e-6
        @test cg_big_m.lp_bound <= true_opt + 1e-6
        # Unlike the earlier per-pair-only attempt (0.3 vs 0.4, a real gap), the γ-chain's
        # conservation law should make this LP relaxation comparably tight to :big_m_nearest's
        # on this tiny instance -- check they're close, not just individually valid.
        @test isapprox(cg_direct_ly.lp_bound, cg_big_m.lp_bound; atol=1e-6, rtol=0.05)
    end
end
