@testset "CGSolver integer recovery" begin
    instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
    data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
    problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
    formulation = AggregateODRouteJointRoutingAssignmentFormulation()

    @testset "build_model honors solver.initial_columns as the seed pool" begin
        default_build = StationSelection.build_model(problem, formulation, CGSolver())
        @test default_build.counts.extras["seed_columns_added"] > 0

        empty_solver = CGSolver(initial_columns = StationSelection.JointRoutingAssignmentRouteColumn[])
        empty_build = StationSelection.build_model(problem, formulation, empty_solver)
        @test empty_build.counts.extras["seed_columns_added"] == 0
        @test isempty(empty_build.model[:joint_routing_assignment_theta])
    end

    @testset "integer_recovery_build rebuilds y/theta/x_walk as binary and stays feasible" begin
        solver = CGSolver()
        build_result = StationSelection.build_model(problem, formulation, solver)
        m = build_result.model
        StationSelection._apply_solver_config!(m, solver.config)
        optimize!(m)
        @test termination_status(m) == MOI.OPTIMAL
        seeded_count = length(m[:joint_routing_assignment_columns])
        @test seeded_count > 0
        @test !any(is_binary(v) for v in m[:y])

        ip_build_result = StationSelection.integer_recovery_build(build_result, build_result.mapping, m)
        ip_m = ip_build_result.model
        @test ip_m !== m

        y = ip_m[:y]
        theta = ip_m[:joint_routing_assignment_theta]
        x_walk = ip_m[:x_walk]
        @test length(theta) == seeded_count
        @test all(is_binary(v) for v in y)
        @test all(is_binary(v) for v in values(theta))
        @test all(is_binary(v) for v in values(x_walk))

        StationSelection._apply_solver_config!(ip_m, solver.config)
        optimize!(ip_m)
        @test termination_status(ip_m) == MOI.OPTIMAL
        @test sum(value.(y)) == problem.k
        @test all(isapprox(value(v), round(value(v)); atol = 1e-6) for v in y)
        @test all(isapprox(value(v), round(value(v)); atol = 1e-6) for v in values(theta))
        @test all(isapprox(value(v), round(value(v)); atol = 1e-6) for v in values(x_walk))
    end

    @testset "CGSolver recovers a feasible integer solution at or above the LP bound" begin
        solver_lp = CGSolver()
        result_lp = run_opt(problem, formulation, solver_lp)
        @test result_lp.termination_status == MOI.OPTIMAL

        solver_ip = CGSolver(recover_integer_solution = true)
        result_ip = run_opt(problem, formulation, solver_ip)
        @test result_ip.termination_status == MOI.OPTIMAL
        @test result_ip.metadata["cg_integer_recovery"] == true
        @test result_ip.metadata["cg_converged"] == true
        @test result_ip.metadata["cg_lp_objective_value"] <= result_ip.objective_value + 1e-6
        @test isapprox(result_ip.metadata["cg_lp_objective_value"], result_lp.objective_value; atol = 1e-6)

        m = result_ip.model
        @test all(is_binary(v) for v in m[:y])
        @test all(is_binary(v) for v in values(m[:joint_routing_assignment_theta]))
        @test all(is_binary(v) for v in values(m[:x_walk]))
    end

    @testset "recover_integer_solution=false leaves the LP-relaxed domains untouched" begin
        solver = CGSolver()
        result = run_opt(problem, formulation, solver)
        @test result.termination_status == MOI.OPTIMAL
        @test result.metadata["cg_integer_recovery"] == false
        @test !haskey(result.metadata, "cg_lp_objective_value")
        m = result.model
        @test !any(is_binary(v) for v in m[:y])
    end
end
