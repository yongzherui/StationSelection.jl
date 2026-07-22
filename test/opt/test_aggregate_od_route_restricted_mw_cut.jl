@testset "AggregateODRouteModel BendersY restricted-MW cut" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping BendersY restricted-MW cut tests"
        @test true
        return
    end

    # Same hand-designed synthetic 5-station fixture as
    # test/opt/test_aggregate_od_route_nearest_open_alignment.jl (see that file's docstring for
    # the full construction rationale): l=4 of 5 stations, request A (o=1,d=5) has two genuine
    # candidates on each side, request B (o=2,d=4) pins stations 2 and 4 open unconditionally,
    # station 3 is a pure decoy. Small enough (C(5,4)=5 feasible binary y) to exhaustively check
    # cut validity against every feasible y, per the spec's tiny-instance validation requirement.
    function mw_cut_fixture()
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

    function mw_cut_model()
        return AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0,
            route_regularization_weight=0.1,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
    end

    data = mw_cut_fixture()
    model = mw_cut_model()
    optimizer_env = Gurobi.Env()

    mapping = StationSelection.create_map(model, data)
    requests, demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)

    ground_truth_solver = BendersSolver(
        config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        decomposition=BendersY(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        ),
    )

    # Ground truth for one candidate y: the *full* nearest-open assignment/routing LP value.
    # First primes a converged, exact route pool via CG on the fixed-assignment route-covering
    # problem (exactly how production BendersY seeds `shared_pool` before repricing), then
    # certifies/extends it against the *broader* full-subproblem LP's own dual structure via
    # genuine repricing -- "solve the full integrated assignment-routing LP" from the spec's
    # tiny-instance validation section, independently of anything BendersY itself produces.
    function true_lp_value(y_hat::Vector{Float64})
        assignments, infeasible = StationSelection._fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
        )
        isempty(infeasible) || return (nothing, assignments)
        open_stations = StationSelection._open_station_values(y_hat)
        cg_result = StationSelection._solve_fixed_route_covering_by_cg(
            data, model, assignments, ground_truth_solver, nothing, open_stations,
        )
        v_hat, _rho, _pool, _n_new, _rounds, exhausted, _delta =
            StationSelection._solve_nearest_open_y_subproblem_lp_with_repricing(
                data, model, mapping, requests, demand, feasible_pairs,
                cg_result.generated_columns, y_hat, optimizer_env, true,
            )
        @test exhausted
        return (v_hat, assignments)
    end

    # Every feasible binary y (close exactly one of the five stations).
    all_y = Vector{Float64}[]
    for closed in 1:5
        y = ones(5)
        y[closed] = 0.0
        push!(all_y, y)
    end
    true_values = Dict{Vector{Float64}, Union{Nothing, Float64}}()
    for y in all_y
        v, _assignments = true_lp_value(y)
        true_values[y] = v
    end
    # Requests B pins stations 2 and 4 open; closing either must be infeasible.
    @test isnothing(true_values[[1.0, 0.0, 1.0, 1.0, 1.0]])
    @test isnothing(true_values[[1.0, 1.0, 1.0, 0.0, 1.0]])
    @test !isnothing(true_values[[0.0, 1.0, 1.0, 1.0, 1.0]])
    @test !isnothing(true_values[[1.0, 1.0, 0.0, 1.0, 1.0]])
    @test !isnothing(true_values[[1.0, 1.0, 1.0, 1.0, 0.0]])

    y_bar = [1.0, 1.0, 0.0, 1.0, 1.0]   # closes the decoy station 3 -- the true optimum
    Q_bar_truth = true_values[y_bar]

    solver = BendersSolver(
        config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        decomposition=BendersY(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        ),
    )

    @testset "core point (Section B)" begin
        core = StationSelection._y_master_core_point(data, model, requests, optimizer_env, true)
        @test core.delta > 1e-6
        @test isapprox(sum(core.y), model.l; atol=1e-6)
        @test all(0.0 - 1e-6 <= v <= 1.0 + 1e-6 for v in core.y)
        # Stations 2 and 4 are structurally forced open by request B's singleton candidate sets.
        @test 2 in core.fixed_one
        @test 4 in core.fixed_one
    end

    core = StationSelection._y_master_core_point(data, model, requests, optimizer_env, true)

    assignments_bar, infeasible_bar = StationSelection._fixed_assignments_from_y(
        data, requests, feasible_pairs, y_bar;
        style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
    )
    @test isempty(infeasible_bar)
    open_stations_bar = StationSelection._open_station_values(y_bar)

    @testset "certified route-covering duals (Sections C-D)" begin
        certified = StationSelection._certified_route_covering_pi(
            data, model, assignments_bar, open_stations_bar, requests, solver,
        )
        @test certified.exact
        pi_full = StationSelection._zero_extended_pi(requests, feasible_pairs, assignments_bar, certified.pi_by_request)
        # Route-dual feasibility over the certified pool: sum_{p,j,k} a[r,p,j,k]*pi[p,j,k] <= g[r] + tol,
        # where a[r,p,j,k]=1 iff route r serves the pair request p is actually assigned to.
        for column in certified.pool
            credit = sum(
                get(pi_full, (request, assignments_bar[request]), 0.0)
                for request in requests
                if assignments_bar[request] in column.od_pairs;
                init=0.0,
            )
            g_r = aggregate_od_route_column_objective_coefficient(
                model.route_regularization_weight, model.repositioning_time, column,
            )
            @test credit <= g_r + 1e-6
        end
    end

    @testset "restricted completion LP: tightness, validity, and MW >= zero-completion at y_core" begin
        mw = StationSelection._restricted_mw_optimality_cut(
            data, model, solver, requests, feasible_pairs, y_bar, assignments_bar, open_stations_bar,
            core.y, optimizer_env, :maximize_core,
        )
        @test mw.status == :ok
        @test isapprox(mw.Q_bar, Q_bar_truth; atol=1e-5)

        # Tightness at y_bar.
        cut_at_ybar = mw.cut_constant + sum(get(mw.beta, j, 0.0) * y_bar[j] for j in 1:5)
        @test isapprox(cut_at_ybar, mw.Q_bar; atol=1e-4)

        # Global validity: cut_constant + beta'y <= Q(y) + tol for every feasible binary y.
        for y in all_y
            v = true_values[y]
            isnothing(v) && continue
            cut_val = mw.cut_constant + sum(get(mw.beta, j, 0.0) * y[j] for j in 1:5)
            @test cut_val <= v + 1e-4
        end

        # Restricted MW completion should be no worse, at y_core, than the zero-completion
        # baseline using the same fixed pi_full.
        @test !isnothing(mw.phi_core_baseline)
        @test mw.phi_core >= mw.phi_core_baseline - 1e-4

        zero_mw = StationSelection._restricted_mw_optimality_cut(
            data, model, solver, requests, feasible_pairs, y_bar, assignments_bar, open_stations_bar,
            core.y, optimizer_env, :zero,
        )
        @test zero_mw.status == :ok
        cut_at_ybar_zero = zero_mw.cut_constant + sum(get(zero_mw.beta, j, 0.0) * y_bar[j] for j in 1:5)
        @test isapprox(cut_at_ybar_zero, zero_mw.Q_bar; atol=1e-4)
        for y in all_y
            v = true_values[y]
            isnothing(v) && continue
            cut_val = zero_mw.cut_constant + sum(get(zero_mw.beta, j, 0.0) * y[j] for j in 1:5)
            @test cut_val <= v + 1e-4
        end
    end

    @testset "end-to-end BendersY convergence under each cut_derivation mode" begin
        ground_truth = run_opt(
            data, model,
            DirectSolver(
                optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
            ),
        )
        @test ground_truth.termination_status == MOI.OPTIMAL

        for cut_derivation in (:standard, :zero_completion, :restricted_mw_fixed_pi)
            @testset "cut_derivation=$cut_derivation" begin
                result = run_opt(
                    data, model,
                    BendersSolver(
                        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                        decomposition=BendersY(),
                        inner_solver=ColumnGenerationSolver(
                            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
                            final_ip_time_limit_sec=30.0,
                        ),
                        max_iterations=50,
                        reprice_subproblem=true,
                        cut_derivation=cut_derivation,
                    ),
                )
                @test result.termination_status == MOI.OPTIMAL
                @test isapprox(result.objective_value, ground_truth.objective_value; atol=1e-6)
                @test result.metadata["cut_derivation"] == string(cut_derivation)
            end
        end
    end

    @testset "unsupported configurations throw" begin
        pair_chain_model = AggregateODRouteModel(
            4; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:pair_chain),
            max_walking_distance=5.0, route_regularization_weight=0.1, repositioning_time=0.0,
            max_stops=3, max_wait_time=1000.0, detour_factor=2.0,
        )
        @test_throws ArgumentError run_opt(
            data, pair_chain_model,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=BendersY(), cut_derivation=:restricted_mw_fixed_pi,
            ),
        )
    end
end
