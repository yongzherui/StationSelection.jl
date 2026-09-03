@testset "Joint CG: :station_simple pricing mode and warm-start phasing" begin
    instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
    data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
    problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
    formulation() = AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4)
    ss_formulation() = AggregateODRouteJointRoutingAssignmentFormulation(
        max_stops = 4, pricing_mode = :station_simple,
    )

    @testset "pricing_mode=:station_simple is accepted by the formulation" begin
        @test ss_formulation().pricing_mode === :station_simple
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(
            pricing_mode = :not_a_pricer,
        )
    end

    @testset "a standalone :station_simple run reports OPTIMAL, scoped" begin
        # OPTIMAL keeps its usual meaning here -- pricing exhausted the universe it was
        # searching. What differs is the SCOPE of that claim, and the scope has to be
        # readable off the result rather than inferred from the formulation, because a
        # consumer filtering on status alone cannot otherwise tell the two apart.
        result = run_opt(problem, ss_formulation(), CGSolver())
        @test result.termination_status == SOLVE_OPTIMAL
        @test result.metadata["cg_optimality_scope"] == "elementary_routes_only"
        @test result.metadata["cg_pricing_universe_restricted"] === true
        @test result.metadata["cg_final_pricing_mode"] === :station_simple
        @test result.objective_value !== nothing
    end

    @testset "an :exact run still certifies (guard does not over-fire)" begin
        result = run_opt(problem, formulation(), CGSolver())
        @test result.termination_status == SOLVE_OPTIMAL
        @test result.metadata["cg_optimality_scope"] == "full_route_universe"
        @test result.metadata["cg_pricing_universe_restricted"] === false
        @test result.metadata["cg_warm_start_pricing_mode"] === nothing
        @test result.metadata["cg_warm_start_iterations"] == 0
    end

    @testset "warm start reaches the SAME certified optimum as pure :exact" begin
        # The whole justification for the feature. Phase 1 harvests elementary columns
        # cheaply; phase 2 runs the full pricer over the same pool and is what certifies.
        # If the handoff were wrong -- pool dropped, `converged` left set from phase 1, or
        # the mode not actually switched -- this is where it shows up.
        exact = run_opt(problem, formulation(), CGSolver(recover_integer_solution = true))
        warm = run_opt(problem, formulation(), CGSolver(
            recover_integer_solution = true, warm_start_pricing_mode = :station_simple,
        ))

        @test exact.termination_status == SOLVE_OPTIMAL
        @test warm.termination_status == SOLVE_OPTIMAL
        @test warm.objective_value ≈ exact.objective_value atol = 1e-6

        @test warm.metadata["cg_warm_start_pricing_mode"] === :station_simple
        @test warm.metadata["cg_final_pricing_mode"] === :exact
        @test warm.metadata["cg_pricing_universe_restricted"] === false
        # Phase 2 is what certifies, so the scope is the full universe even though phase 1
        # priced in the restricted one.
        @test warm.metadata["cg_optimality_scope"] == "full_route_universe"
        # The handoff actually happened, i.e. phase 1 ran and then ended.
        @test warm.metadata["cg_warm_start_iterations"] >= 1
        @test warm.metadata["cg_stop_reason"] == "converged"
        # And the model really is in the full-universe mode afterwards.
        @test warm.model[:joint_routing_assignment_pricing_mode] === :exact
    end

    @testset "the two OPTIMALs are distinguishable by scope, not by status" begin
        # Both statuses read OPTIMAL; `cg_optimality_scope` is the only thing separating a
        # full-universe optimum from an elementary-only one. This is the check that would
        # fail if a future change dropped the key or stopped populating it.
        ss = run_opt(problem, ss_formulation(), CGSolver())
        ex = run_opt(problem, formulation(), CGSolver())
        @test ss.termination_status == ex.termination_status == SOLVE_OPTIMAL
        @test ss.metadata["cg_optimality_scope"] != ex.metadata["cg_optimality_scope"]
        # The restriction can only ever cost objective value, never gain it (minimisation
        # over a subset of columns), so the elementary optimum is >= the full one.
        @test ss.objective_value >= ex.objective_value - 1e-6
    end

    @testset "iteration log records which phase each iteration priced in" begin
        warm = run_opt(problem, formulation(), CGSolver(
            warm_start_pricing_mode = :station_simple,
        ))
        log = warm.metadata["cg_iteration_log"]
        @test !isempty(log)
        @test all(haskey(row, :pricing_mode) for row in log)
        modes = unique(row.pricing_mode for row in log)
        @test "station_simple" in modes
        # Rows stay homogeneous NamedTuples -- benchmarks concatenate them into a DataFrame.
        @test length(unique(keys(row) for row in log)) == 1
    end

    @testset "warm start is rejected where it cannot mean anything" begin
        # Same mode both phases: a no-op handoff that would silently halve max_iterations.
        @test_throws ArgumentError run_opt(problem, formulation(), CGSolver(
            warm_start_pricing_mode = :exact,
        ))
        # Base has no selectable pricer, so there is nothing to hand off to. Rejected
        # rather than ignored, so a warm start can never quietly not happen.
        @test_throws ArgumentError run_opt(
            problem, AggregateODRouteBaseFormulation(max_stops = 4),
            CGSolver(warm_start_pricing_mode = :station_simple),
        )
    end
end
