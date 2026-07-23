@testset "AggregateODRouteModel BendersYZH zero-completion cut" begin
    gurobi_available = try
        using Gurobi
        true
    catch
        false
    end
    if !gurobi_available
        @warn "Gurobi not available, skipping BendersYZH zero-completion cut tests"
        @test true
        return
    end

    # Same hand-designed synthetic 5-station fixture as the BendersY/BendersYZ restricted-MW-cut
    # tests: l=4 of 5 stations, request A (o=1,d=5) has two genuine candidates on each side,
    # request B (o=2,d=4) pins stations 2 and 4 open unconditionally, station 3 is a pure decoy.
    function zc_fixture()
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

    function zc_model()
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

    data = zc_fixture()
    model = zc_model()
    optimizer_env = Gurobi.Env()

    mapping = StationSelection.create_map(model, data)
    requests, demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)
    physical_pairs, occurrences, feasible_pairs_by_p = StationSelection._aggregate_od_route_benders_physical_pairs(mapping)

    # Derive h_hat from a candidate y_hat exactly the way BendersYZH's real master would resolve
    # it, via a throwaway LP using the same `_add_nearest_open_master_h!` the real master uses.
    function derive_h_hat(y_hat::Vector{Float64})
        hm = Model(() -> Gurobi.Optimizer(optimizer_env))
        set_silent(hm)
        @variable(hm, 0 <= y[1:5] <= 1)
        for j in 1:5
            fix(y[j], y_hat[j]; force=true)
        end
        h = StationSelection._add_nearest_open_master_h!(
            hm, data, y, physical_pairs, feasible_pairs_by_p, model.max_walking_distance, model.allow_walk_only,
            model.assignment_policy.feasibility_cut_style,
        )
        @objective(hm, Min, 0.0)
        optimize!(hm)
        primal_status(hm) == MOI.FEASIBLE_POINT || return nothing
        return Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64}(
            key => round(value(var)) for (key, var) in h
        )
    end

    solver = BendersSolver(
        config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        decomposition=BendersYZH(),
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
            max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
            final_ip_time_limit_sec=30.0,
        ),
    )

    # Ground truth: certified (repriced) BendersYZH subproblem value for one candidate y.
    function true_lp_value(y_hat::Vector{Float64})
        h_hat = derive_h_hat(y_hat)
        isnothing(h_hat) && return (nothing, nothing)
        assignments, infeasible = StationSelection._fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
        )
        isempty(infeasible) || return (nothing, h_hat)
        open_stations = StationSelection._open_station_values(y_hat)
        cg_result = StationSelection._solve_fixed_route_covering_by_cg(
            data, model, assignments, solver, nothing, open_stations,
        )
        v_hat, _rho, _pool, _n_new, _rounds, exhausted, _delta =
            StationSelection._solve_yzh_route_subproblem_lp_with_repricing(
                data, model, mapping, requests, feasible_pairs_by_p,
                cg_result.generated_columns, h_hat, optimizer_env, true,
            )
        @test exhausted
        return (v_hat, h_hat)
    end

    all_y = Vector{Float64}[]
    for closed in 1:5
        y = ones(5)
        y[closed] = 0.0
        push!(all_y, y)
    end
    true_values = Dict{Vector{Float64}, Union{Nothing, Float64}}()
    h_hats = Dict{Vector{Float64}, Union{Nothing, Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64}}}()
    for y in all_y
        v, h_hat = true_lp_value(y)
        true_values[y] = v
        h_hats[y] = h_hat
    end
    @test isnothing(true_values[[1.0, 0.0, 1.0, 1.0, 1.0]])
    @test isnothing(true_values[[1.0, 1.0, 1.0, 0.0, 1.0]])
    @test !isnothing(true_values[[0.0, 1.0, 1.0, 1.0, 1.0]])
    @test !isnothing(true_values[[1.0, 1.0, 0.0, 1.0, 1.0]])
    @test !isnothing(true_values[[1.0, 1.0, 1.0, 1.0, 0.0]])

    y_bar = [1.0, 1.0, 0.0, 1.0, 1.0]   # closes the decoy station 3 -- the true optimum
    h_bar = h_hats[y_bar]
    Q_bar_truth = true_values[y_bar]

    assignments_bar, infeasible_bar = StationSelection._fixed_assignments_from_y(
        data, requests, feasible_pairs, y_bar;
        style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
    )
    @test isempty(infeasible_bar)
    open_stations_bar = StationSelection._open_station_values(y_bar)

    @testset "zero-completion rho: tightness and global validity" begin
        Q_bar, rho, certified = StationSelection._zero_completion_yzh_rho(
            data, model, solver, requests, feasible_pairs_by_p, assignments_bar, open_stations_bar,
        )
        @test certified.exact
        @test isapprox(Q_bar, Q_bar_truth; atol=1e-5)

        # Tight by construction at h_bar.
        cut_at_hbar = Q_bar + sum(rho[key] * (h_bar[key] - get(h_bar, key, 0.0)) for key in keys(rho))
        @test isapprox(cut_at_hbar, Q_bar; atol=1e-9)

        # Global validity: Q_bar + rho'(h-h_bar) <= Q(h) + tol at every feasible (y,h) pair.
        for y in all_y
            v = true_values[y]
            isnothing(v) && continue
            h_hat = h_hats[y]
            cut_val = Q_bar + sum(rho[key] * (get(h_hat, key, 0.0) - h_bar[key]) for key in keys(rho))
            @test cut_val <= v + 1e-4
        end
    end

    @testset "end-to-end BendersYZH convergence under each cut_derivation mode" begin
        ground_truth = run_opt(
            data, model,
            DirectSolver(
                optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
                max_enumerated_routes=2000, max_enumeration_time_sec=20.0,
            ),
        )
        @test ground_truth.termination_status == MOI.OPTIMAL

        for (cut_derivation, reprice) in ((:standard, true), (:zero_completion, false))
            @testset "cut_derivation=$cut_derivation, reprice_subproblem=$reprice" begin
                result = run_opt(
                    data, model,
                    BendersSolver(
                        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
                        decomposition=BendersYZH(),
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
        @test_throws ArgumentError BendersSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            decomposition=BendersYZH(), cut_derivation=:restricted_mw_fixed_pi,
        )
    end
end
