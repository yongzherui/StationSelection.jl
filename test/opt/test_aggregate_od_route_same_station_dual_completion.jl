@testset "AggregateODRouteModel restricted dual-completion: same-station assigned pairs" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping same-station dual-completion regression tests"
        @test true
        return
    end

    # Regression fixture for a real gap: `_certified_route_covering_pi`/`_zero_extended_pi`
    # (shared by BendersY/BendersYZ/BendersYZH's :zero_completion and :restricted_mw_fixed_pi
    # modes) assumed every request's *assigned* pair has a route-covering coverage row -- true
    # only when the assignment needs a vehicle route. A same-station pair (here, request 3's
    # o=d=3, cheapest resolution (3,3)) gets no coverage row anywhere in the codebase
    # (`requires_no_vehicle_route`), so there is no `pi` dual to certify for it -- the correct
    # completion-LP treatment is to omit the `pi` term for such a row (equivalently, zero-extend
    # it to 0.0), not to throw or KeyError. Before the fix, this silently degraded every
    # non-`:standard` cut for any group containing such a request to a fallback `:standard` cut
    # every iteration (caught by the existing try/catch, so no crash -- just silently defeating
    # the point of the non-standard modes whenever a same-station assignment is optimal, which
    # `allow_same_station=true` being the model default now makes routine, not rare).
    #
    # 6th station (station 6) is a pure decoy with no demand touching it, added specifically so
    # this fixture has more than one feasible `y` -- request 3's own singleton pinned-open
    # candidate (station 3) alone would leave only one feasible `y`, which can only test cut
    # *tightness*, not global *validity* (the actual question: does folding a `pi=0` row into the
    # completion LP still produce a cut that's a valid underestimator everywhere, not just at the
    # point it was derived from).
    function fixture()
        stations = DataFrame(id=collect(1:6), lon=Float64.(1:6), lat=zeros(6))
        requests = DataFrame(
            id=[1, 2, 3],
            start_station_id=[1, 2, 3],
            end_station_id=[5, 4, 3],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1), DateTime(2024, 1, 1, 8, 2)],
        )
        walking_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:6, j in 1:6
            walking_costs[(i, j)] = 100.0
        end
        walking_costs[(1, 1)] = 0.0
        walking_costs[(1, 2)] = 3.0
        walking_costs[(4, 5)] = 3.0
        walking_costs[(5, 5)] = 0.0
        walking_costs[(2, 2)] = 0.0
        walking_costs[(4, 4)] = 0.0
        walking_costs[(3, 3)] = 0.0   # request 3 (o=3,d=3): cheapest resolution is same-station (3,3)
        routing_costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:6, j in 1:6
            routing_costs[(i, j)] = abs(i - j) + 1.0
        end
        return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    end

    function fixture_model()
        return AggregateODRouteModel(
            5;   # l=5 of 6 -- forces exactly one closure; stations 2,3,4 are singleton-pinned
                 # open (request B/C have only one candidate each), so the only real choice is
                 # which of {1, 5, 6} to close -- closing the decoy (6) is optimal, closing 1 or 5
                 # is feasible but strictly worse (falls back to the costlier candidate 2 or 4).
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            walk_cost_weight=0.37,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
    end

    data = fixture()
    model = fixture_model()
    optimizer_env = Gurobi.Env()

    mapping = StationSelection.create_map(model, data)
    requests, demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)
    request_3 = only(r for r in requests if r[2] == 3 && r[3] == 3)

    function resolve(y_hat::Vector{Float64})
        assignments, infeasible = StationSelection._fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
            allow_same_station=true,
        )
        return assignments, infeasible
    end

    y_bar = ones(6); y_bar[6] = 0.0   # close the decoy -- the true optimum
    assignments_bar, infeasible_bar = resolve(y_bar)
    @test isempty(infeasible_bar)
    @test assignments_bar[request_3] == (3, 3)
    @test StationSelection.requires_no_vehicle_route(assignments_bar[request_3])
    open_stations_bar = StationSelection._open_station_values(y_bar)

    solver = BendersSolver(
        config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        decomposition=BendersY(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        ),
    )

    @testset "certification tolerates a same-station assigned pair (no throw, no KeyError)" begin
        certified, Q_bar = StationSelection._certified_qbar(
            data, model, solver, requests, assignments_bar, open_stations_bar,
        )
        @test certified.exact
        @test isfinite(Q_bar)
        pi_full = StationSelection._zero_extended_pi(requests, feasible_pairs, assignments_bar, certified.pi_by_request)
        @test pi_full[(request_3, (3, 3))] == 0.0
    end

    # True value at every feasible y, via the certified (repriced) BendersY subproblem --
    # independent of anything the cut-derivation code under test produces.
    function true_lp_value(y_hat::Vector{Float64})
        assignments, infeasible = resolve(y_hat)
        isempty(infeasible) || return nothing
        open_stations = StationSelection._open_station_values(y_hat)
        cg_result = StationSelection._solve_fixed_route_covering_by_cg(
            data, model, assignments, solver, nothing, open_stations,
        )
        v_hat, _rho, _pool, _n_new, _rounds, exhausted, _delta =
            StationSelection._solve_nearest_open_y_subproblem_lp_with_repricing(
                data, model, mapping, requests, demand, feasible_pairs,
                cg_result.generated_columns, y_hat, optimizer_env, true,
            )
        @test exhausted
        return v_hat
    end

    all_y = Vector{Float64}[]
    for closed in (1, 5, 6)
        y = ones(6); y[closed] = 0.0
        push!(all_y, y)
    end
    true_values = Dict(y => true_lp_value(y) for y in all_y)
    Q_bar_truth = true_values[y_bar]
    # Closing the decoy is strictly cheaper than closing 1 or 5 (both fall back to a
    # positive-cost candidate for request A) -- confirms the fixture is set up as intended.
    @test all(true_values[y] > Q_bar_truth + 1e-9 for y in all_y if y != y_bar)

    core = StationSelection._y_master_core_point(data, model, requests, optimizer_env, true)

    @testset "restricted completion LP is tight and globally valid with a pi=0 same-station row" begin
        for objective_mode in (:zero, :maximize_core)
            mw = StationSelection._restricted_mw_optimality_cut(
                data, model, solver, requests, feasible_pairs, y_bar, assignments_bar, open_stations_bar,
                core.y, optimizer_env, objective_mode,
            )
            @test mw.status == :ok
            @test isapprox(mw.Q_bar, Q_bar_truth; atol=1e-5)

            cut_at_ybar = mw.cut_constant + sum(get(mw.beta, j, 0.0) * y_bar[j] for j in 1:6)
            @test isapprox(cut_at_ybar, mw.Q_bar; atol=1e-4)

            # Global validity, including at the two feasible points where request 3's same-station
            # pi=0 dual credit is *not* the same as at y_bar (still same assignment (3,3) here since
            # station 3 is pinned open in every feasible y, but the pickup/dropoff duals for the
            # *other* requests differ, so this still exercises the completed cut away from its
            # derivation point).
            for y in all_y
                v = true_values[y]
                cut_val = mw.cut_constant + sum(get(mw.beta, j, 0.0) * y[j] for j in 1:6)
                @test cut_val <= v + 1e-4
            end
        end
    end

    ground_truth = run_opt(
        data, model,
        DirectSolver(
            optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
            max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
        ),
    )
    @test ground_truth.termination_status == MOI.OPTIMAL
    @test isapprox(ground_truth.objective_value, Q_bar_truth; atol=1e-6)

    @testset "end-to-end match across decompositions/cut_derivation modes" begin
        for (decomposition, name) in ((BendersY(), :y), (BendersYZ(), :yz), (BendersYZH(), :yzh))
            modes = name == :yzh ? (:standard, :zero_completion) : (:standard, :zero_completion, :restricted_mw_fixed_pi)
            for cut_derivation in modes
                reprice = cut_derivation == :standard
                @testset "$name / $cut_derivation" begin
                    result = run_opt(
                        data, model,
                        BendersSolver(
                            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                            decomposition=decomposition,
                            inner_solver=ColumnGenerationSolver(
                                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                                max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
                                final_ip_time_limit_sec=30.0,
                            ),
                            max_iterations=50, reprice_subproblem=reprice, cut_derivation=cut_derivation,
                        ),
                    )
                    @test result.termination_status == MOI.OPTIMAL
                    @test isapprox(result.objective_value, ground_truth.objective_value; atol=1e-6)
                end
            end
        end
    end
end
