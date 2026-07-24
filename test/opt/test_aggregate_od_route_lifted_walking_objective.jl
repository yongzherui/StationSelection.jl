@testset "AggregateODRouteModel lifted_walking_objective" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping lifted_walking_objective tests"
        @test true
        return
    end

    # Same hand-designed synthetic 5-station fixture as
    # test_aggregate_od_route_restricted_mw_cut.jl / test_aggregate_od_route_yz_restricted_mw_cut.jl:
    # l=4 of 5 stations, request A (o=1,d=5) has two genuine candidates on each side, request B
    # (o=2,d=4) pins stations 2 and 4 open unconditionally, station 3 is a pure decoy. Small enough
    # (C(5,4)=5 feasible binary y) to exhaustively check every feasible y.
    function lifted_fixture()
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

    function lifted_model()
        return AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0,
            route_regularization_weight=3.0,
            walk_cost_weight=0.37,
            repositioning_time=0.0,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
    end

    data = lifted_fixture()
    model = lifted_model()
    subproblem_model = StationSelection._unit_weighted_routing_model(model)
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

    # Certified full nearest-open assignment/routing LP value at a fixed, already-feasible y and
    # its `assignments`, against whichever `weight_model` is passed -- the real model gives
    # Q_old(y) (walking + β-weighted routing combined, today's behavior); `subproblem_model` gives
    # Q_route_new(y) (unweighted routing only, what the lifted subproblem now certifies).
    function true_lp_value_feasible(weight_model::AggregateODRouteModel, y_hat::Vector{Float64}, assignments)
        open_stations = StationSelection._open_station_values(y_hat)
        cg_result = StationSelection._solve_fixed_route_covering_by_cg(
            data, weight_model, assignments, ground_truth_solver, nothing, open_stations,
        )
        v_hat, _rho, _pool, _n_new, _rounds, exhausted, _delta =
            StationSelection._solve_nearest_open_y_subproblem_lp_with_repricing(
                data, weight_model, mapping, requests, demand, feasible_pairs,
                cg_result.generated_columns, y_hat, optimizer_env, true,
            )
        @test exhausted
        return v_hat
    end

    # Every feasible binary y (close exactly one of the five stations); requests B pins 2 and 4.
    all_y = Vector{Float64}[]
    for closed in 1:5
        y = ones(5)
        y[closed] = 0.0
        push!(all_y, y)
    end

    @testset "Test A: fixed-y objective equivalence (BendersY)" begin
        for y in all_y
            assignments, infeasible = StationSelection._fixed_assignments_from_y(
                data, requests, feasible_pairs, y;
                style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
            )
            isempty(infeasible) || continue   # infeasible y (closes station 2 or 4); nothing to compare
            q_old = true_lp_value_feasible(model, y, assignments)
            q_route_new = true_lp_value_feasible(subproblem_model, y, assignments)
            c_walk = StationSelection._lifted_walking_cost(data, model, assignments)
            @test isapprox(q_old, c_walk + model.route_regularization_weight * q_route_new; atol=1e-6)
        end
    end

    @testset "Test A: fixed-y objective equivalence (BendersYZ)" begin
        function derive_z_hat(weight_model::AggregateODRouteModel, y_hat::Vector{Float64})
            zm = Model(() -> Gurobi.Optimizer(optimizer_env))
            set_silent(zm)
            @variable(zm, 0 <= y[1:5] <= 1)
            for j in 1:5
                fix(y[j], y_hat[j]; force=true)
            end
            StationSelection._add_nearest_open_master_z!(
                zm, data, y, requests, feasible_pairs, weight_model.max_walking_distance,
                weight_model.allow_walk_only, weight_model.assignment_policy.feasibility_cut_style,
            )
            optimize!(zm)
            primal_status(zm) == MOI.FEASIBLE_POINT || return nothing
            return Dict{StationSelection._AggregateODRouteEndpointChainKey, Vector{Float64}}(
                key => round.(value.(vars)) for (key, vars) in zm[:nearest_endpoint_chain_cache]
            )
        end
        function yz_lp_value(weight_model::AggregateODRouteModel, y_hat::Vector{Float64}, assignments)
            z_hat = derive_z_hat(weight_model, y_hat)
            open_stations = StationSelection._open_station_values(y_hat)
            cg_result = StationSelection._solve_fixed_route_covering_by_cg(
                data, weight_model, assignments, ground_truth_solver, nothing, open_stations,
            )
            v_hat, _rho, _pool, _n_new, _rounds, exhausted, _delta =
                StationSelection._solve_yz_route_subproblem_lp_with_repricing(
                    data, weight_model, mapping, requests, feasible_pairs,
                    cg_result.generated_columns, z_hat, optimizer_env, true,
                )
            @test exhausted
            return v_hat
        end
        for y in all_y
            assignments, infeasible = StationSelection._fixed_assignments_from_y(
                data, requests, feasible_pairs, y;
                style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
            )
            isempty(infeasible) || continue
            q_old = yz_lp_value(model, y, assignments)
            q_route_new = yz_lp_value(subproblem_model, y, assignments)
            c_walk = StationSelection._lifted_walking_cost(data, model, assignments)
            @test isapprox(q_old, c_walk + model.route_regularization_weight * q_route_new; atol=1e-6)
        end
    end

    @testset "Test B: integrated optimum equivalence" begin
        ground_truth = run_opt(
            data, model,
            DirectSolver(
                optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
            ),
        )
        @test ground_truth.termination_status == MOI.OPTIMAL

        for decomposition in (BendersY(), BendersYZ())
            for cut_derivation in (:standard, :zero_completion)
                @testset "$(typeof(decomposition)), cut_derivation=$cut_derivation, lifted=$lifted" for lifted in (false, true)
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
                            max_iterations=50,
                            reprice_subproblem=true,
                            cut_derivation=cut_derivation,
                            lifted_walking_objective=lifted,
                        ),
                    )
                    @test result.termination_status == MOI.OPTIMAL
                    @test isapprox(result.objective_value, ground_truth.objective_value; atol=1e-6)
                end
            end
        end
    end

    @testset "unsupported configurations throw" begin
        @test_throws ArgumentError BendersSolver(decomposition=BendersXY(), lifted_walking_objective=true)
        @test_throws ArgumentError BendersSolver(decomposition=BendersYZH(), lifted_walking_objective=true)

        pair_chain_model = AggregateODRouteModel(
            4; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:pair_chain),
            max_walking_distance=5.0, route_regularization_weight=3.0, repositioning_time=0.0,
            max_stops=3, max_wait_time=1000.0, detour_factor=2.0,
        )
        @test_throws ArgumentError run_opt(
            data, pair_chain_model,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=BendersY(), lifted_walking_objective=true,
            ),
        )
    end
end
