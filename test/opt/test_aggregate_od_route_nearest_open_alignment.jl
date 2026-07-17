@testset "AggregateODRouteModel nearest-open cross-solver alignment" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping AggregateODRouteModel nearest-open alignment tests"
        @test true
        return
    end

    # Synthetic 5-station fixture, hand-designed so that:
    #   - request A (o=1, d=5) has two disjoint candidate clusters, each with 2 members:
    #     pickup candidates {1 (cost 0), 2 (cost 3)}, dropoff candidates {4 (cost 3), 5 (cost 0)}.
    #   - request B (o=2, d=4) has a *singleton* candidate pair (2,4), which forces stations 2
    #     and 4 open unconditionally (otherwise the whole model is infeasible).
    #   - station 3 is a pure decoy: unreachable (walking cost 100) from every request anchor.
    # With l=4 (exactly 4 of 5 stations built), stations 2 and 4 are forced open by request B,
    # and opening station 1 and 5 (instead of the decoy, station 3) strictly reduces request A's
    # walking cost (0 vs 3 on each side) with no offsetting cost elsewhere (route_regularization_weight
    # is kept small relative to the walking-cost gap) — so the unique optimum closes station 3 and
    # opens {1,2,4,5}, giving request A a genuine two-candidate choice on *both* sides
    # simultaneously: this is what exercises `:big_m_nearest`'s independent pickup/dropoff selectors
    # (and `:pair_chain`'s joint-pair ranking) non-trivially.
    function nearest_open_alignment_fixture()
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
        walking_costs[(1, 1)] = 0.0   # request A: station 1 is the nearest pickup
        walking_costs[(1, 2)] = 3.0   # request A: station 2 is a farther pickup alternative
        walking_costs[(4, 5)] = 3.0   # request A: station 4 is a farther dropoff alternative
        walking_costs[(5, 5)] = 0.0   # request A: station 5 is the nearest dropoff
        walking_costs[(2, 2)] = 0.0   # request B: station 2 is its only feasible pickup
        walking_costs[(4, 4)] = 0.0   # request B: station 4 is its only feasible dropoff

        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end

        data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
        return data
    end

    function nearest_open_alignment_model(style::Symbol; kwargs...)
        return AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
            kwargs...,
        )
    end

    # For any solved OptResult, verify every positive x assignment goes to the true
    # nearest-open (j,k) pair (by joint walking cost) among the pairs whose endpoints are
    # both open in that same result -- independent of which constraint style produced it.
    function assert_nearest_open_assignments(data::StationSelectionData, result; atol=1e-6)
        m, mapping = result.model, result.mapping
        y_val = value.(m[:y])
        y_open = Set(j for j in eachindex(y_val) if y_val[j] > 0.5)
        for s in 1:n_scenarios(data)
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                isempty(valid_pairs) && continue
                request = (s, o, d)
                open_pairs = filter(p -> p[1] in y_open && p[2] in y_open, valid_pairs)
                isempty(open_pairs) && continue
                nearest = first(StationSelection._ranked_request_pairs(data, request, open_pairs))
                for (pair_idx, pair) in enumerate(valid_pairs)
                    xv = value(m[:x][s][od_idx][pair_idx])
                    xv > atol && @test pair == nearest
                end
            end
        end
    end

    # Shared cross-solver comparison, reused by both the synthetic and the real-data fixture
    # below. `model_for(style)` must build a *fresh* AggregateODRouteModel each call (station
    # count / l / other params fixed, only assignment_policy varies).
    #
    # `enumerate_aggregate_od_route_columns` was previously implemented by reusing the CG
    # label-setting pricer's dominance-pruned search with synthetic uniform duals; that
    # dominance rule is only valid for finding *a* negative-reduced-cost column during real
    # pricing, not for exhaustive enumeration, and it silently discarded complete, cheaper,
    # multi-stop consolidated routes -- see notes/2026-07-14_nearest_open_solver_alignment.md.
    # It has been replaced with an independent bounded-DFS enumerator
    # (src/opt/optimize/aggregate_od_route_enumeration.jl) that shares no code with the pricer.
    #
    # FORMERLY a known bug, now FIXED: BendersY could converge to a genuinely
    # suboptimal-but-correctly-costed y -- a premature-convergence bug where the optimality cut
    # derived from a *restricted* column pool (`cg_result.generated_columns`, specific to one
    # y_hat) was not actually a valid global underestimator of the true value function at other
    # y, so the master accepted the first y it tried as "optimal" after a single cut. Fixed by
    # `BendersSolver(reprice_subproblem=true)` (see notes/2026-07-15_bendersy_stale_cut_soundness.md):
    # each subproblem LP now re-prices against its own covering duals instead of trusting the
    # restricted pool. `run_cross_solver_alignment_checks` always passes `reprice_subproblem=true`
    # for BendersY, so `benders_y_expected_broken` should be `false` everywhere now; kept as a
    # parameter (rather than deleted) so a future fixture that finds a *new* gap can still mark
    # itself `@test_broken` explicitly instead of silently failing. BendersXY was never affected
    # by this bug (it fixes both y and x jointly each iteration, so its per-iteration CG priming
    # is always re-derived for the actual candidate being evaluated).
    function run_cross_solver_alignment_checks(
        data::StationSelectionData, model_for::Function; benders_y_expected_broken::Bool,
        styles::Tuple=(:pair_chain, :big_m_nearest),
    )
        for style in styles
            @testset "style=$style" begin
                model = model_for(style)

                ground_truth = run_opt(
                    data, model,
                    DirectSolver(
                        optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                        max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
                    ),
                )
                @test ground_truth.termination_status == MOI.OPTIMAL
                @test ground_truth.metadata["solve_method"] == "route_enumeration"
                assert_nearest_open_assignments(data, ground_truth)

                cg_direct = run_aggregate_od_route_column_generation(
                    model, data;
                    optimizer_env=Gurobi.Env(), verbose=false,
                    max_cg_iters=200, max_new_columns=20, n_candidates=20,
                    ip_time_limit_sec=30.0, mip_gap=0.0, silent=true,
                )
                @test cg_direct.cg_stop_reason == :optimality_proven
                @test cg_direct.final_result.termination_status == MOI.OPTIMAL
                @test isapprox(cg_direct.final_result.objective_value, ground_truth.objective_value; atol=1e-6)
                assert_nearest_open_assignments(data, cg_direct.final_result)

                inner_cg = ColumnGenerationSolver(
                    config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                    max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
                    final_ip_time_limit_sec=30.0,
                )

                # reprice_subproblem=true closes BendersY's premature-convergence /
                # stale-cut soundness gap (notes/2026-07-15_bendersy_stale_cut_soundness.md):
                # each subproblem LP re-prices against its own covering duals instead of
                # trusting a column pool only proven complete for a narrower formulation.
                # Verified exact on 4 independent fixtures in that note (synthetic +
                # 3 Zhuzhou max_stops variants) -- confirming it here too.
                benders_y = run_opt(
                    data, model,
                    BendersSolver(
                        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                        decomposition=BendersY(), inner_solver=inner_cg, max_iterations=50,
                        reprice_subproblem=true,
                    ),
                )
                @test benders_y.termination_status == MOI.OPTIMAL
                if benders_y_expected_broken
                    @test_broken isapprox(benders_y.objective_value, ground_truth.objective_value; atol=1e-6)
                else
                    @test isapprox(benders_y.objective_value, ground_truth.objective_value; atol=1e-6)
                end
                assert_nearest_open_assignments(data, benders_y)

                benders_xy = run_opt(
                    data, model,
                    BendersSolver(
                        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                        decomposition=BendersXY(), inner_solver=inner_cg, max_iterations=50,
                    ),
                )
                @test benders_xy.termination_status == MOI.OPTIMAL
                @test isapprox(benders_xy.objective_value, ground_truth.objective_value; atol=1e-6)
                assert_nearest_open_assignments(data, benders_xy)
            end
        end
    end

    @testset "synthetic 5-station fixture" begin
        data = nearest_open_alignment_fixture()
        run_cross_solver_alignment_checks(data, nearest_open_alignment_model; benders_y_expected_broken=false)
    end

    # Same 5-station fixture as above, plus a third request (C) whose origin
    # and destination are physically identical (station 3, the decoy from
    # the other two requests) -- exercises the direct-walking
    # (WALK_ONLY_PAIR) coupling introduced for :big_m_nearest +
    # allow_walk_only=true. Only :big_m_nearest supports this (:pair_chain
    # -- joint station-*pair* ranking -- still rejects allow_walk_only
    # entirely, see assert_no_walk_only_pairs call sites), so this fixture
    # is checked with styles=(:big_m_nearest,) only, unlike every other
    # fixture in this file which checks both styles.
    function nearest_open_walk_only_fixture()
        data = nearest_open_alignment_fixture()
        stations = data.stations
        # Request C: o=d=3 (both endpoints are the decoy station itself, so
        # its only candidate on either side -- once opened -- is station 3;
        # direct walk distance is 0, trivially within 2*max_walking_distance).
        requests = DataFrame(
            id=[1, 2, 3],
            start_station_id=[1, 2, 3],
            end_station_id=[5, 4, 3],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1), DateTime(2024, 1, 1, 8, 2)],
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
        walking_costs[(3, 3)] = 0.0
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:5, j in 1:5
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    end

    function nearest_open_walk_only_model(style::Symbol; kwargs...)
        return AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
            allow_walk_only=true,
            kwargs...,
        )
    end

    @testset "synthetic 5-station fixture with direct walking (:big_m_nearest only)" begin
        data = nearest_open_walk_only_fixture()
        run_cross_solver_alignment_checks(
            data, nearest_open_walk_only_model; benders_y_expected_broken=false, styles=(:big_m_nearest,),
        )
    end

    # Reuse the hand-crafted real-coordinate benchmark data under ../Data/test2_zone_proximity
    # (sibling to this package) rather than only synthetic fixtures. Subset to stations
    # {3,4,5,7,8} ("M","B","p1","p3","p4") and requests with origin in {5,7,8}, destination=4
    # ("B"): station 4 is isolated (>1400m from anything else) so it's always a forced,
    # singleton dropoff candidate; the three origins are mutually close (<300m) to each other
    # and to station 3, giving each request 2-3 real disjoint-from-dropoff pickup candidates --
    # a genuine, non-hand-tuned nearest-open ranking exercise on real geometry.
    data_root = joinpath(@__DIR__, "..", "..", "..", "Data", "test2_zone_proximity", "close_to_B", "seed_01")
    if !isdir(data_root)
        @warn "Data/test2_zone_proximity not found next to StationSelection.jl; skipping real-data alignment fixture" data_root
        @test true
    else
        @testset "real-data 5-station fixture (test2_zone_proximity/close_to_B)" begin
            keep_station_ids = Set([3, 4, 5, 7, 8])
            stations = read_candidate_stations(joinpath(data_root, "station.csv"))
            stations = stations[in.(stations.id, Ref(keep_station_ids)), :]
            requests = read_customer_requests(
                joinpath(data_root, "order.csv");
                start_time="2026-01-01 00:00:00", end_time="2026-01-02 00:00:00",
            )
            requests = requests[
                in.(requests.origin_station_id, Ref(Set([5, 7, 8]))) .&
                (requests.destination_station_id .== 4),
                :,
            ]
            walking_costs = compute_station_pairwise_costs(stations)
            routing_costs = read_routing_costs_from_segments(joinpath(data_root, "segment.csv"), stations)
            data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)

            model_for(style) = AggregateODRouteModel(
                4;
                assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
                max_walking_distance=200.0,
                route_regularization_weight=0.1,
                repositioning_time=0.0,
                max_stops=3,
                max_wait_time=10000.0,
                detour_factor=2.0,
            )
            run_cross_solver_alignment_checks(data, model_for; benders_y_expected_broken=false)
        end
    end

    @testset "Benders subproblem (RouteCoveringProblem CG) matches exhaustive enumeration" begin
        data = nearest_open_alignment_fixture()
        # assignment_policy is irrelevant here -- only used to read Omega_s/valid_jk_pairs.
        probe_model = nearest_open_alignment_model(:pair_chain)
        mapping = StationSelection.create_map(probe_model, data)
        requests, _demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)

        open_stations = [1, 2, 4, 5]
        y_hat = zeros(Float64, data.n_stations)
        for j in open_stations
            y_hat[j] = 1.0
        end
        assignments, infeasible = StationSelection._fixed_assignments_from_y(data, requests, feasible_pairs, y_hat)
        @test isempty(infeasible)
        @test assignments[(1, 1, 5)] == (1, 5)
        @test assignments[(1, 2, 4)] == (2, 4)

        free_model = AggregateODRouteModel(
            4;
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )

        inner_cg = ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        )
        wrapped_benders = BendersSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            decomposition=BendersY(), inner_solver=inner_cg, max_iterations=50,
        )
        subproblem_result = StationSelection._solve_fixed_route_covering_by_cg(
            data, free_model, assignments, wrapped_benders, nothing, open_stations,
        )
        @test subproblem_result.cg_stop_reason == :optimality_proven
        @test subproblem_result.final_result.termination_status == MOI.OPTIMAL

        route_problem = StationSelection._route_covering_problem_from_assignments(free_model, assignments, open_stations)
        enumerated_columns = enumerate_aggregate_od_route_columns(route_problem, data; max_routes=1000, time_limit_sec=10.0)
        exhaustive_problem = StationSelection._copy_with_initial_columns(route_problem, enumerated_columns)
        exhaustive_result = run_opt(
            data, exhaustive_problem,
            DirectSolver(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        )
        @test exhaustive_result.termination_status == MOI.OPTIMAL
        @test isapprox(
            subproblem_result.final_result.objective_value, exhaustive_result.objective_value; atol=1e-6
        )

        nearest_route_problem = RouteCoveringProblem(
            4,
            open_stations,
            assignments;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
        nearest_build = StationSelection.build_model(
            nearest_route_problem,
            data;
            optimizer_env=Gurobi.Env(),
        )
        @test nearest_build.counts.constraints["fixed_open_stations"] == data.n_stations
        @test !haskey(nearest_build.counts.constraints, "nearest_open_assignment")

        nearest_direct = run_opt(
            data,
            nearest_route_problem,
            DirectSolver(
                optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                max_enumerated_routes=1000, max_enumeration_time_sec=10.0,
            ),
        )
        @test nearest_direct.termination_status == MOI.OPTIMAL
        @test nearest_direct.metadata["solve_method"] == "route_enumeration"
        @test isapprox(nearest_direct.objective_value, exhaustive_result.objective_value; atol=1e-6)
    end
end
