@testset "AggregateODRouteModel BendersYZ restricted-MW cut" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping BendersYZ restricted-MW cut tests"
        @test true
        return
    end

    # Same hand-designed synthetic 5-station fixture as
    # test/opt/test_aggregate_od_route_restricted_mw_cut.jl (BendersY's analogous test): l=4 of 5
    # stations, request A (o=1,d=5) has two genuine candidates on each side, request B (o=2,d=4)
    # pins stations 2 and 4 open unconditionally, station 3 is a pure decoy.
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

    # Derive z_hat from a candidate y_hat exactly the way `solve_benders_yz_subproblem` does
    # internally (a throwaway LP using the same `_add_nearest_open_master_z!` the real master
    # uses), so the fixture doesn't depend on any internal-key knowledge beyond that.
    function derive_z_hat(y_hat::Vector{Float64})
        zm = Model(() -> Gurobi.Optimizer(optimizer_env))
        set_silent(zm)
        @variable(zm, 0 <= y[1:5] <= 1)
        for j in 1:5
            fix(y[j], y_hat[j]; force=true)
        end
        StationSelection._add_nearest_open_master_z!(
            zm, data, y, requests, feasible_pairs, model.max_walking_distance, model.allow_walk_only,
            model.assignment_policy.feasibility_cut_style,
        )
        optimize!(zm)
        primal_status(zm) == MOI.FEASIBLE_POINT || return nothing
        return Dict{StationSelection._AggregateODRouteEndpointChainKey, Vector{Float64}}(
            key => round.(value.(vars)) for (key, vars) in zm[:nearest_endpoint_chain_cache]
        )
    end

    ground_truth_solver = BendersSolver(
        config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        decomposition=BendersYZ(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        ),
    )

    # Ground truth for one candidate y: primes a converged, exact route pool via CG on the
    # fixed-assignment route-covering problem (exactly how production BendersYZ seeds its pool
    # before repricing), then certifies/extends it against the broader fixed-z subproblem LP's
    # own dual structure via genuine repricing -- independently of anything the cut-derivation
    # code under test produces.
    function true_lp_value(y_hat::Vector{Float64})
        assignments, infeasible = StationSelection._fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
        )
        isempty(infeasible) || return (nothing, nothing)
        z_hat = derive_z_hat(y_hat)
        open_stations = StationSelection._open_station_values(y_hat)
        cg_result = StationSelection._solve_fixed_route_covering_by_cg(
            data, model, assignments, ground_truth_solver, nothing, open_stations,
        )
        v_hat, _rho, _pool, _n_new, _rounds, exhausted, _delta =
            StationSelection._solve_yz_route_subproblem_lp_with_repricing(
                data, model, mapping, requests, feasible_pairs, cg_result.generated_columns, z_hat, optimizer_env, true,
            )
        @test exhausted
        return (v_hat, z_hat)
    end

    all_y = Vector{Float64}[]
    for closed in 1:5
        y = ones(5)
        y[closed] = 0.0
        push!(all_y, y)
    end
    true_values = Dict{Vector{Float64}, Union{Nothing, Float64}}()
    z_hats = Dict{Vector{Float64}, Any}()
    for y in all_y
        v, z_hat = true_lp_value(y)
        true_values[y] = v
        z_hats[y] = z_hat
    end
    @test isnothing(true_values[[1.0, 0.0, 1.0, 1.0, 1.0]])
    @test isnothing(true_values[[1.0, 1.0, 1.0, 0.0, 1.0]])
    @test !isnothing(true_values[[0.0, 1.0, 1.0, 1.0, 1.0]])
    @test !isnothing(true_values[[1.0, 1.0, 0.0, 1.0, 1.0]])
    @test !isnothing(true_values[[1.0, 1.0, 1.0, 1.0, 0.0]])

    y_bar = [1.0, 1.0, 0.0, 1.0, 1.0]   # closes the decoy station 3 -- the true optimum
    z_bar = z_hats[y_bar]
    Q_bar_truth = true_values[y_bar]

    solver = BendersSolver(
        config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        decomposition=BendersYZ(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        ),
    )

    @testset "joint (y,z) core point" begin
        core_point = StationSelection._yz_joint_core_point(data, model, requests, optimizer_env, true)
        @test core_point.delta > 1e-6
        @test isapprox(sum(core_point.y), model.l; atol=1e-6)
        @test all(0.0 - 1e-6 <= v <= 1.0 + 1e-6 for v in core_point.y)
        # Stations 2 and 4 are structurally forced open by request B's singleton candidate sets --
        # same structural fact `_y_master_core_point`'s analogous BendersY test checks.
        @test 2 in core_point.fixed_one
        @test 4 in core_point.fixed_one
        @test !isempty(core_point.z)
        for (key, vals) in core_point.z
            @test isapprox(sum(vals), 1.0; atol=1e-6)
            @test all(-1e-6 <= v <= 1.0 + 1e-6 for v in vals)
        end
    end

    core = StationSelection._yz_joint_core_point(data, model, requests, optimizer_env, true).z

    assignments_bar, infeasible_bar = StationSelection._fixed_assignments_from_y(
        data, requests, feasible_pairs, y_bar;
        style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
    )
    @test isempty(infeasible_bar)
    open_stations_bar = StationSelection._open_station_values(y_bar)

    @testset "restricted completion LP: tightness, validity, and MW >= zero-completion at z_core" begin
        mw = StationSelection._restricted_yz_optimality_cut(
            data, model, solver, requests, feasible_pairs, z_bar, assignments_bar, open_stations_bar,
            core, optimizer_env, :maximize_core,
        )
        @test mw.status == :ok
        @test isapprox(mw.Q_bar, Q_bar_truth; atol=1e-5)

        cut_at_zbar = mw.cut_constant + sum(mw.beta[key] * z_bar[key[1]][key[2]] for key in keys(mw.beta))
        @test isapprox(cut_at_zbar, mw.Q_bar; atol=1e-4)

        # Global validity: cut_constant + beta'z <= Q(z) + tol at every feasible (y,z) pair.
        for y in all_y
            v = true_values[y]
            isnothing(v) && continue
            z_hat = z_hats[y]
            cut_val = mw.cut_constant + sum(
                get(mw.beta, key, 0.0) * z_hat[key[1]][key[2]] for key in keys(mw.beta)
            )
            @test cut_val <= v + 1e-4
        end

        @test !isnothing(mw.phi_core_baseline)
        @test mw.phi_core >= mw.phi_core_baseline - 1e-4

        zero_mw = StationSelection._restricted_yz_optimality_cut(
            data, model, solver, requests, feasible_pairs, z_bar, assignments_bar, open_stations_bar,
            core, optimizer_env, :zero,
        )
        @test zero_mw.status == :ok
        cut_at_zbar_zero = zero_mw.cut_constant + sum(
            zero_mw.beta[key] * z_bar[key[1]][key[2]] for key in keys(zero_mw.beta)
        )
        @test isapprox(cut_at_zbar_zero, zero_mw.Q_bar; atol=1e-4)
        for y in all_y
            v = true_values[y]
            isnothing(v) && continue
            z_hat = z_hats[y]
            cut_val = zero_mw.cut_constant + sum(
                get(zero_mw.beta, key, 0.0) * z_hat[key[1]][key[2]] for key in keys(zero_mw.beta)
            )
            @test cut_val <= v + 1e-4
        end
    end

    @testset "end-to-end BendersYZ convergence under each cut_derivation mode" begin
        ground_truth = run_opt(
            data, model,
            DirectSolver(
                optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
            ),
        )
        @test ground_truth.termination_status == MOI.OPTIMAL

        for (cut_derivation, reprice) in ((:standard, true), (:zero_completion, false), (:restricted_mw_fixed_pi, false))
            @testset "cut_derivation=$cut_derivation, reprice_subproblem=$reprice" begin
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
                        reprice_subproblem=reprice,
                        cut_derivation=cut_derivation,
                    ),
                )
                @test result.termination_status == MOI.OPTIMAL
                @test isapprox(result.objective_value, ground_truth.objective_value; atol=1e-6)
            end
        end
    end

    @testset "unsupported configurations throw" begin
        endpoint_chain_model = AggregateODRouteModel(
            4; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:endpoint_chain),
            max_walking_distance=5.0, route_regularization_weight=0.1, repositioning_time=0.0,
            max_stops=3, max_wait_time=1000.0, detour_factor=2.0,
        )
        @test_throws ArgumentError run_opt(
            data, endpoint_chain_model,
            BendersSolver(
                config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                decomposition=BendersYZ(), cut_derivation=:restricted_mw_fixed_pi,
            ),
        )
    end
end
