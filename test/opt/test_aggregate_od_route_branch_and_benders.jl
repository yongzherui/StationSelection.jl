@testset "AggregateODRouteModel BranchAndBendersSolver" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping BranchAndBendersSolver tests"
        @test true
        return
    end

    # Same hand-designed synthetic 5-station fixture as
    # test_aggregate_od_route_lifted_walking_objective.jl: l=4 of 5 stations, request A (o=1,d=5)
    # has two genuine candidates on each side, request B (o=2,d=4) pins stations 2 and 4 open
    # unconditionally, station 3 is a pure decoy. Single scenario, so cut_ids=[1] and the full
    # `requests` list is exactly cut_id=1's group.
    function lifted_lb_fixture()
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

    function lifted_lb_model()
        return AggregateODRouteModel(
            4;
            assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
            max_walking_distance=5.0,
            route_regularization_weight=3.0,
            walk_cost_weight=0.37,
            repositioning_time=1.5,
            max_stops=3,
            max_wait_time=1000.0,
            detour_factor=2.0,
        )
    end

    data = lifted_lb_fixture()
    model = lifted_lb_model()

    @testset "BendersYZ integrated optimum equivalence across cut_derivation/lifted_walking_objective" begin
        ground_truth = run_opt(
            data, model,
            DirectSolver(
                optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
            ),
        )
        @test ground_truth.termination_status == MOI.OPTIMAL

        for cut_derivation in (:standard, :zero_completion, :restricted_mw_fixed_pi)
            for lifted_walking in (false, true)
                @testset "cut_derivation=$cut_derivation, lifted_walking_objective=$lifted_walking" begin
                    result = run_opt(
                        data, model,
                        BendersSolver(
                            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                            decomposition=BendersYZ(),
                            inner_solver=ColumnGenerationSolver(
                                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                                max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
                                final_ip_time_limit_sec=30.0,
                            ),
                            max_iterations=50,
                            reprice_subproblem=true,
                            cut_derivation=cut_derivation,
                            lifted_walking_objective=lifted_walking,
                        ),
                    )
                    @test result.termination_status == MOI.OPTIMAL
                    @test isapprox(result.objective_value, ground_truth.objective_value; atol=1e-6)
                end
            end
        end
    end

    @testset "BranchAndBendersSolver certified Y/YZ callback modes" begin
        @test BranchAndBendersSolver().decomposition isa BendersYZ

        function branch_solver(
            decomposition; initial_cuts=BranchBendersCut[], initial_rounds=0,
            cut_derivation=:standard,
        )
            return BranchAndBendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=decomposition,
                cut_derivation=cut_derivation,
                inner_solver=ColumnGenerationSolver(
                    config=SolverConfig(silent=true, mip_gap=0.0),
                    max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
                    final_ip_time_limit_sec=30.0,
                ),
                initial_cuts=initial_cuts,
                initial_benders_cut_rounds=initial_rounds,
            )
        end

        result_y = run_opt(data, model, branch_solver(BendersY()))
        result_yz = run_opt(data, model, branch_solver(BendersYZ()))
        result_yz_mw = run_opt(
            data, model,
            branch_solver(BendersYZ(); cut_derivation=:restricted_mw_fixed_pi),
        )
        @test result_y.termination_status == MOI.OPTIMAL
        @test result_yz.termination_status == MOI.OPTIMAL
        @test result_yz_mw.termination_status == MOI.OPTIMAL
        @test isapprox(result_y.objective_value, result_yz.objective_value; atol=1e-6)
        @test isapprox(result_yz_mw.objective_value, result_yz.objective_value; atol=1e-6)
        @test result_yz_mw.metadata["branch_benders_cut_derivation"] ==
              "restricted_mw_fixed_pi"
        @test result_yz_mw.metadata["branch_benders_mw_completion_seconds"] >= 0.0
        @test_throws ArgumentError BranchAndBendersSolver(
            decomposition=BendersY(), cut_derivation=:restricted_mw_fixed_pi,
        )
        @test result_y.metadata["branch_benders_open_stations"] ==
              result_yz.metadata["branch_benders_open_stations"]
        @test isapprox(
            result_yz.metadata["branch_benders_certified_ub"],
            result_yz.metadata["branch_benders_global_lb"];
            atol=1e-6,
        )
        @test result_yz.metadata["branch_benders_threads"] == 1
        @test result_yz.metadata["branch_benders_unique_y"] >= 1
        @test result_yz.metadata["branch_benders_cuts_submitted"] >= 1

        # Every stored cut is tight at the binary station set whose oracle result generated it.
        generated_yz = result_yz.metadata["branch_benders_generated_cuts"]
        @test !isempty(generated_yz)
        @test all(cut -> cut.decomposition == :yz, generated_yz)
        @test isapprox(result_yz.objective_value, 20.61; atol=1e-6)

        # Tightness at generation and sampled global validity across every other integer
        # candidate that Gurobi exposed during these solves.
        for solved in (result_y, result_yz, result_yz_mw)
            oracle_results = solved.metadata["branch_benders_oracle_results"]
            for generating in oracle_results, cut in values(generating.cuts)
                generated_rhs = StationSelection._branch_benders_cut_rhs(
                    cut, generating.y, generating.z,
                )
                @test isapprox(
                    generated_rhs, generating.recourse[cut.block_id]; atol=1e-6, rtol=1e-6,
                )
                for other in oracle_results
                    rhs = StationSelection._branch_benders_cut_rhs(cut, other.y, other.z)
                    @test rhs <= other.recourse[cut.block_id] + 1e-6
                end
            end
        end
        # Restricted-MW cuts must also be valid against the independently repriced exact
        # recourse values exposed by the standard YZ oracle, including candidates that the MW
        # master may cut off before presenting them as incumbents.
        for generating in result_yz_mw.metadata["branch_benders_oracle_results"],
            cut in values(generating.cuts),
            other in result_yz.metadata["branch_benders_oracle_results"]
            rhs = StationSelection._branch_benders_cut_rhs(cut, other.y, other.z)
            if rhs > other.recourse[cut.block_id] + 1e-6
                common_keys = intersect(Set(keys(generating.z)), Set(keys(other.z)))
                max_z_distance = maximum(
                    abs(generating.z[key][i] - other.z[key][i])
                    for key in common_keys for i in eachindex(generating.z[key]); init=0.0,
                )
                generating_z_fractionality = maximum(
                    min(abs(v), abs(1.0-v)) for vals in values(generating.z) for v in vals;
                    init=0.0,
                )
                candidate_z_fractionality = maximum(
                    min(abs(v), abs(1.0-v)) for vals in values(other.z) for v in vals;
                    init=0.0,
                )
                @info "restricted-MW cross-candidate violation" generating_y=generating.y_key candidate_y=other.y_key block_id=cut.block_id rhs=rhs exact_recourse=other.recourse[cut.block_id] alpha=cut.alpha beta=cut.beta max_z_distance=max_z_distance generating_z_fractionality=generating_z_fractionality candidate_z_fractionality=candidate_z_fractionality generating_z=generating.z candidate_z=other.z
            end
            @test rhs <= other.recourse[cut.block_id] + 1e-6
        end

        # Independently re-price the route dual block used by zero completion.  MW and zero
        # completion fix the same pi; this explicitly proves whether that pi admits another
        # negative-reduced-cost route instead of relying only on the originating CG stop flag.
        diag_state = Base.only(filter(
            state -> state.y_key == (1, 2, 3, 4),
            result_yz_mw.metadata["branch_benders_oracle_results"],
        ))
        diag_mapping = create_map(model, data)
        diag_requests, _diag_demand, diag_pairs =
            StationSelection._aggregate_od_route_benders_requests(diag_mapping)
        diag_assignments, diag_infeasible = StationSelection._fixed_assignments_from_y(
            data, diag_requests, diag_pairs, diag_state.y;
            style=model.assignment_policy.feasibility_cut_style,
            max_walking_distance=model.max_walking_distance,
            allow_walk_only=model.allow_walk_only,
            allow_same_station=true,
        )
        @test isempty(diag_infeasible)
        diag_unit_model = StationSelection._unit_weighted_routing_model(model)
        diag_solver = branch_solver(BendersYZ(); cut_derivation=:restricted_mw_fixed_pi)
        diag_proxy = StationSelection._branch_benders_proxy_solver(diag_solver, Gurobi.Env())
        diag_cg = StationSelection._solve_fixed_route_covering_by_cg(
            data, diag_unit_model, diag_assignments, diag_proxy, nothing,
            StationSelection._open_station_values(diag_state.y),
        )
        @test diag_cg.cg_stop_reason == :optimality_proven
        diag_certified, _diag_qbar = StationSelection._certified_qbar(
            data, diag_unit_model, diag_cg, diag_requests, diag_assignments,
        )
        diag_pi = StationSelection._zero_extended_pi(
            diag_requests, diag_pairs, diag_assignments, diag_certified.pi_by_request,
        )

        # Build the zero-objective completion from the identical certified routing dual block.
        # If this cut also fails globally, the defect precedes the MW core-point objective and
        # lies in completion feasibility/algebra or the mapping into the master chains.
        diag_core_untyped = StationSelection._yz_joint_core_point(
            data, diag_unit_model, diag_requests, Gurobi.Env(), true,
        ).z
        # Match the concrete dictionary produced by the Branch-and-Benders callback. This
        # guards against invariant Dict signatures hidden inside the completion helpers.
        diag_core = Dict{StationSelection._AggregateODRouteEndpointChainKey, Vector{Float64}}(
            key => values for (key, values) in diag_core_untyped
        )
        zero_result = StationSelection._restricted_yz_optimality_cut(
            data, diag_unit_model, diag_proxy, diag_requests, diag_pairs,
            diag_state.z, diag_assignments,
            StationSelection._open_station_values(diag_state.y), diag_core,
            Gurobi.Env(), :zero; certified=diag_certified, Q_bar=_diag_qbar,
        )
        @test zero_result.status == :ok
        completion_lp = StationSelection._yz_completion_lp(
            data, diag_unit_model, diag_requests, diag_pairs, diag_state.z, diag_core,
            diag_pi, _diag_qbar, :zero, Gurobi.Env(), true,
        )
        optimize!(completion_lp.model)
        @test primal_status(completion_lp.model) == MOI.FEASIBLE_POINT
        alpha_values = Dict(k => value(v) for (k, v) in completion_lp.alpha)
        rhoO_values = Dict(k => value(v) for (k, v) in completion_lp.rhoO)
        rhoD_values = Dict(k => value(v) for (k, v) in completion_lp.rhoD)
        sigma_values = Dict(k => value(v) for (k, v) in completion_lp.sigma)
        x_dual_residuals = NamedTuple[]
        for p in diag_requests, pair in diag_pairs[p]
            StationSelection.is_walk_only_pair(pair) && continue
            lhs = alpha_values[p] - rhoO_values[(p, pair)] -
                rhoD_values[(p, pair)] + sigma_values[(p, pair)] - diag_pi[(p, pair)]
            cost = StationSelection._assignment_pair_cost(
                data, p, pair; weight=diag_unit_model.walk_cost_weight,
            )
            push!(x_dual_residuals, (request=p, pair=pair, residual=lhs - cost))
        end
        worst_x = argmax(row -> row.residual, x_dual_residuals)
        route_dual_residuals = NamedTuple[]
        for column in diag_certified.pool, s in 1:n_scenarios(data)
            dual_sum = sum(
                get(diag_pi, (p, pair), 0.0)
                for p in diag_requests if p[1] == s
                for pair in diag_pairs[p]
                if !StationSelection.requires_no_vehicle_route(pair) && pair in column.od_pairs;
                init=0.0,
            )
            cost = StationSelection.aggregate_od_route_column_objective_coefficient(
                diag_unit_model.route_regularization_weight,
                diag_unit_model.repositioning_time, column,
            )
            push!(route_dual_residuals, (column_id=column.id, scenario=s, residual=dual_sum-cost))
        end
        worst_route = argmax(row -> row.residual, route_dual_residuals)
        reconstructed_constant = sum(values(alpha_values); init=0.0) -
            sum(values(sigma_values); init=0.0)
        @info "zero-completion full-dual audit" worst_x=worst_x worst_route=worst_route reconstructed_constant=reconstructed_constant returned_constant=zero_result.cut_constant constant_error=reconstructed_constant-zero_result.cut_constant
        @test worst_x.residual <= 1e-7
        @test worst_route.residual <= 1e-7
        @test isapprox(reconstructed_constant, zero_result.cut_constant; atol=1e-8)

        mw_completion_lp = StationSelection._yz_completion_lp(
            data, diag_unit_model, diag_requests, diag_pairs, diag_state.z, diag_core,
            diag_pi, _diag_qbar, :maximize_core, Gurobi.Env(), true,
        )
        optimize!(mw_completion_lp.model)
        mw_term = termination_status(mw_completion_lp.model)
        mw_primal = primal_status(mw_completion_lp.model)
        mw_dual = dual_status(mw_completion_lp.model)
        mw_alpha = Dict(k => value(v) for (k, v) in mw_completion_lp.alpha)
        mw_rhoO = Dict(k => value(v) for (k, v) in mw_completion_lp.rhoO)
        mw_rhoD = Dict(k => value(v) for (k, v) in mw_completion_lp.rhoD)
        mw_sigma = Dict(k => value(v) for (k, v) in mw_completion_lp.sigma)
        mw_x_residuals = NamedTuple[]
        for p in diag_requests, pair in diag_pairs[p]
            StationSelection.is_walk_only_pair(pair) && continue
            lhs = mw_alpha[p] - mw_rhoO[(p, pair)] - mw_rhoD[(p, pair)] +
                mw_sigma[(p, pair)] - diag_pi[(p, pair)]
            cost = StationSelection._assignment_pair_cost(
                data, p, pair; weight=diag_unit_model.walk_cost_weight,
            )
            push!(mw_x_residuals, (request=p, pair=pair, residual=lhs-cost))
        end
        mw_worst_x = argmax(row -> row.residual, mw_x_residuals)
        mw_tightness_value = value(mw_completion_lp.phi_zhat_expr)
        mw_tightness_error = mw_tightness_value - _diag_qbar
        mw_max_abs_dual = maximum(abs, vcat(
            collect(values(mw_alpha)), collect(values(mw_rhoO)),
            collect(values(mw_rhoD)), collect(values(mw_sigma)),
        ))
        mw_objective = objective_value(mw_completion_lp.model)
        mw_objective_bound = objective_bound(mw_completion_lp.model)
        @info "maximize-core completion audit" termination_status=mw_term primal_status=mw_primal dual_status=mw_dual objective=mw_objective objective_bound=mw_objective_bound qbar=_diag_qbar tightness_value=mw_tightness_value tightness_error=mw_tightness_error worst_x=mw_worst_x worst_route=worst_route max_abs_dual=mw_max_abs_dual
        @test mw_term == MOI.OPTIMAL
        @test mw_primal == MOI.FEASIBLE_POINT
        @test mw_worst_x.residual <= 1e-7
        @test worst_route.residual <= 1e-7
        @test abs(mw_tightness_error) <= 1e-7
        zero_cut = BranchBendersCut(
            :yz, 1, zero_result.cut_constant, zero_result.beta, zero_result.Q_bar,
        )
        for other in result_yz.metadata["branch_benders_oracle_results"]
            rhs = StationSelection._branch_benders_cut_rhs(zero_cut, other.y, other.z)
            if rhs > other.recourse[1] + 1e-6
                @info "zero-completion cross-candidate violation" generating_y=diag_state.y_key candidate_y=other.y_key rhs=rhs exact_recourse=other.recourse[1] alpha=zero_cut.alpha beta=zero_cut.beta
            end
            @test rhs <= other.recourse[1] + 1e-6
        end
        diag_sigma = Dict{NTuple{3, Int}, Float64}()
        for ((request, pair), pi) in diag_pi
            StationSelection.requires_no_vehicle_route(pair) && continue
            s = request[1]
            key = (pair[1], pair[2], s)
            diag_sigma[key] = get(diag_sigma, key, 0.0) + pi
        end
        for s in 1:n_scenarios(data)
            pricing_duals = StationSelection._scenario_pricing_duals(
                StationSelection.AggregateODRouteCoverageDuals(Dict{Any, Float64}(), diag_sigma), s,
            )
            pricing_data = create_aggregate_od_route_pricing_data(
                diag_unit_model, data, diag_mapping, s, pricing_duals,
            )
            priced, exhausted, pricing_stats = aggregate_od_route_pricing_by_label_setting(
                pricing_data, diag_certified.pool, pricing_duals;
                next_column_id=isempty(diag_certified.pool) ? 1 :
                    maximum(column.id for column in diag_certified.pool) + 1,
                reduced_cost_tol=diag_solver.pricing_tolerance,
                max_new_columns=20, n_candidates=20,
                time_limit=diag_solver.inner_solver.pricing_time_limit_sec,
                max_visits_per_node=diag_unit_model.max_visits_per_node,
            )
            best_reduced_cost = isempty(priced) ? missing : minimum(
                Float64(get(column.metadata, "reduced_cost", Inf)) for column in priced
            )
            @info "zero-completion routing-dual repricing audit" scenario=s n_new_columns=length(priced) exhausted=exhausted best_reduced_cost=best_reduced_cost labels_generated=pricing_stats.labels_generated
            @test exhausted
            @test isempty(priced)
        end

        # Pre-solve rounds use the same oracle/cut path and preserve the exact optimum.
        warm = run_opt(data, model, branch_solver(BendersYZ(); initial_rounds=1))
        @test warm.termination_status == MOI.OPTIMAL
        @test isapprox(warm.objective_value, result_yz.objective_value; atol=1e-6)

        # Test the exact cache primitive directly: a repeated key returns the stored result and
        # cannot execute its oracle closure a second time.
        cache = Dict{Tuple{Vararg{Int}}, Int}()
        stats = StationSelection._BranchBendersStats()
        oracle_calls = Ref(0)
        first, first_cached = StationSelection._branch_benders_cache_get!(
            cache, (1, 3), stats, () -> (oracle_calls[] += 1; 17),
        )
        second, second_cached = StationSelection._branch_benders_cache_get!(
            cache, (1, 3), stats, () -> (oracle_calls[] += 1; 99),
        )
        @test (first, first_cached) == (17, false)
        @test (second, second_cached) == (17, true)
        @test oracle_calls[] == 1
        @test stats.cache_hits == 1

        # Cuts harvested from a prior run are ordinary globally valid master constraints.
        preloaded = run_opt(data, model, branch_solver(BendersYZ(); initial_cuts=generated_yz))
        @test preloaded.termination_status == MOI.OPTIMAL
        @test isapprox(preloaded.objective_value, result_yz.objective_value; atol=1e-6)
    end
end
