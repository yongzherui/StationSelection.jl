@testset "AggregateODRouteBaseFormulation + CGSolver" begin
    instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
    data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
    problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
    # DirectMIPSolver's exhaustive enumeration requires a finite max_stops (it has no other
    # way to terminate a plain DFS); CG's label-setting pricer has no such requirement, but
    # a shared, finite formulation keeps the two solvers' results directly comparable below.
    formulation = AggregateODRouteBaseFormulation(max_stops = 4)

    @testset "build_model honors solver.initial_columns as the seed pool" begin
        default_build = StationSelection.build_model(problem, formulation, CGSolver())
        @test default_build.counts.extras["seed_columns_added"] > 0
        @test !any(is_binary(v) for v in default_build.model[:y])

        empty_solver = CGSolver(initial_columns = StationSelection.AggregateODRouteColumn[])
        empty_build = StationSelection.build_model(problem, formulation, empty_solver)
        @test empty_build.counts.extras["seed_columns_added"] == 0
        @test isempty(empty_build.model[:aggregate_od_route_base_theta])
    end

    @testset "reduced-cost dual sign: slack route_link row has non-negative sigma" begin
        build_result = StationSelection.build_model(problem, formulation, CGSolver())
        m = build_result.model
        StationSelection._apply_solver_config!(m, CGSolver().config)
        optimize!(m)
        @test termination_status(m) == MOI.OPTIMAL

        duals = StationSelection.extract_aggregate_od_route_base_duals(m)
        @test all(all(v -> v >= -1e-6, values(d.sigma)) for d in values(duals))
    end

    @testset "integer_recovery_build rebuilds y/x/x_walk/theta as binary and stays feasible" begin
        solver = CGSolver()
        build_result = StationSelection.build_model(problem, formulation, solver)
        m = build_result.model
        StationSelection._apply_solver_config!(m, solver.config)
        optimize!(m)
        @test termination_status(m) == MOI.OPTIMAL
        discovered_count = length(m[:aggregate_od_route_base_theta])
        @test discovered_count > 0
        @test !any(is_binary(v) for v in m[:y])

        ip_build_result = StationSelection.integer_recovery_build(build_result, build_result.mapping, m)
        ip_m = ip_build_result.model
        @test ip_m !== m

        y = ip_m[:y]
        theta = ip_m[:aggregate_od_route_base_theta]
        x = ip_m[:x]
        x_walk = ip_m[:x_walk]
        @test length(theta) == discovered_count
        @test all(is_binary(v) for v in y)
        @test all(is_binary(v) for v in values(theta))
        @test all(is_binary(v) for v in values(x))
        @test all(is_binary(v) for v in values(x_walk))

        StationSelection._apply_solver_config!(ip_m, solver.config)
        optimize!(ip_m)
        @test termination_status(ip_m) == MOI.OPTIMAL
        @test sum(value.(y)) == problem.k
    end

    @testset "CGSolver LP bound and integer recovery agree with DirectMIPSolver's exact optimum" begin
        direct_result = run_opt(problem, formulation, DirectMIPSolver())
        @test direct_result.termination_status == SOLVE_OPTIMAL

        result_lp = run_opt(problem, formulation, CGSolver())
        @test result_lp.termination_status == SOLVE_OPTIMAL
        @test result_lp.objective_value <= direct_result.objective_value + 1e-4

        result_ip = run_opt(problem, formulation, CGSolver(recover_integer_solution = true))
        @test result_ip.termination_status == SOLVE_OPTIMAL
        @test result_ip.metadata["cg_converged"] == true
        @test result_ip.metadata["cg_lp_objective_value"] <= result_ip.objective_value + 1e-6
        @test isapprox(result_ip.metadata["cg_lp_objective_value"], result_lp.objective_value; atol = 1e-6)
        @test isapprox(result_ip.objective_value, direct_result.objective_value; atol = 1e-4)
    end

    @testset "recover_integer_solution=false leaves the LP-relaxed domains untouched" begin
        result = run_opt(problem, formulation, CGSolver())
        @test result.termination_status == SOLVE_OPTIMAL
        @test result.metadata["cg_integer_recovery"] == false
        m = result.model
        @test !any(is_binary(v) for v in m[:y])
    end

    @testset "dispatch.jl disambiguates Base vs JointRoutingAssignment CGSolver hooks" begin
        base_result = run_opt(problem, formulation, CGSolver())
        @test base_result.termination_status == SOLVE_OPTIMAL

        joint_result = run_opt(
            problem, AggregateODRouteJointRoutingAssignmentFormulation(), CGSolver(),
        )
        @test joint_result.termination_status == SOLVE_OPTIMAL
        @test haskey(joint_result.model.obj_dict, :joint_routing_assignment_theta)
        @test !haskey(base_result.model.obj_dict, :joint_routing_assignment_theta)
        @test haskey(base_result.model.obj_dict, :aggregate_od_route_base_theta)
    end
end
