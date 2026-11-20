@testset "Joint M3: heuristic capacity by column splitting (vehicle_capacity)" begin
    using DataFrames
    using Dates
    using JuMP

    # Stations (metres): 1 (0,0), 2 (1500,1100) a detour, 3 (1500,0), 4 (3000,0); 5 (1500,-400) is not a
    # candidate, so its rider walks 286 s to station 3. Driving 10 m/s, walking 1.4 m/s; every other walk is over
    # the 720 s limit, so riders board at their own station (walk 0) except the one from 5.
    xy = [(0.0, 0.0), (1500.0, 1100.0), (1500.0, 0.0), (3000.0, 0.0), (1500.0, -400.0)]
    n = length(xy)
    dist(i, j) = hypot(xy[i][1] - xy[j][1], xy[i][2] - xy[j][2])
    drive = [round(dist(i, j) / 10.0) for i in 1:n, j in 1:n]
    walk = [dist(i, j) / 1.4 for i in 1:n, j in 1:n]
    t0 = DateTime(2025, 5, 16, 17, 40)

    # (origin, destination, passengers) per request; all request at the scenario start
    reqs = vcat(fill((1, 4, 3), 4), fill((2, 4, 2), 4), fill((1, 3, 1), 3), [(5, 4, 1)])
    function problem_for(reqs)
        stations = DataFrame(id = collect(1:n), lon = zeros(n), lat = zeros(n), candidate = [true, true, true, true, false])
        req = DataFrame(order_id = collect(1:length(reqs)), origin_station_id = [r[1] for r in reqs],
                        destination_station_id = [r[2] for r in reqs], start_station_id = [r[1] for r in reqs],
                        end_station_id = [r[2] for r in reqs], request_time = fill(t0, length(reqs)),
                        origin_idx = [r[1] for r in reqs], dest_idx = [r[2] for r in reqs], pax = [r[3] for r in reqs])
        data = StationSelectionData(stations, n, Dict(i => i for i in 1:n), collect(1:n), walk, drive,
                                    [ScenarioData("s", t0, t0 + Minute(10), req)])
        return data, StationSelectionProblem(data, 4; walking_limit_s = 720.0, door_to_door_limit_ratio = 3.0)
    end
    formulation = AggregateODRouteJointRoutingAssignmentFormulation(
        route_regularization_weight = 10.0, walk_cost_weight = 0.1, repositioning_time = 20.0,
        pickup_deadline_s = 900.0, ride_time_limit_ratio = 2.0, max_stops = 10)
    m3(cap) = CGPricingConfig(mode = :exact_no_anticipation, first_boarding = :after_last_walker, vehicle_capacity = cap)
    solver(cap) = CGSolver(config = SolverOptions(silent = true), pricing = m3(cap), recover_integer_solution = true)

    function chosen(res)
        m = res.model
        cols = m[:joint_routing_assignment_columns]
        return [cols[id] for (id, v) in m[:joint_routing_assignment_theta] if value(v) > 0.5]
    end

    @testset "config" begin
        @test m3(18).vehicle_capacity == 18
        @test isnothing(m3(nothing).vehicle_capacity)
        @test_throws ArgumentError m3(0)
        @test_throws ArgumentError CGPricingConfig(mode = :exact, vehicle_capacity = 18)
        @test_throws ArgumentError CGPricingConfig(mode = :exact_no_anticipation, first_boarding = :start_now,
                                                   vehicle_capacity = 18)
    end

    @testset "the formulation's exact vehicle_capacity cannot be combined with M3" begin
        _, problem = problem_for(reqs)
        capf = AggregateODRouteJointRoutingAssignmentFormulation(
            route_regularization_weight = 10.0, walk_cost_weight = 0.1, repositioning_time = 20.0,
            pickup_deadline_s = 900.0, ride_time_limit_ratio = 2.0, max_stops = 10, vehicle_capacity = 18)
        @test_throws ArgumentError build_model(problem, capf, solver(nothing))
    end

    @testset "M3 check: a subsequence of a feasible route is not always feasible" begin
        data, problem = problem_for(reqs)
        br = build_model(problem, formulation, solver(18))
        m, mapping = br.model, br.mapping
        p(o, d) = findfirst(==((o, d)), mapping.Omega_s[1])
        pax = joint_split_group_pax(data, mapping)
        @test pax[1][p(1, 4)] == 12 && pax[1][p(2, 4)] == 8 && pax[1][p(5, 4)] == 1
        item(o, d, j, k, a, b) = (p = p(o, d), j = j, k = k, a = a, b = b, w = pax[1][p(o, d)])
        route = [1, 2, 3, 4]
        # via the detour the vehicle reaches 3 at 296 s, after the rider from 5 (286 s), and 4 at 446 s, inside their
        # door-to-door limit (3 x 155 = 465 s) ...
        items = [item(1, 4, 1, 4, 1, 4), item(2, 4, 2, 4, 2, 4), item(5, 4, 3, 4, 3, 4)]
        @test StationSelection._joint_split_m3_check(m, data, mapping, 1, route, items; keep_all = true)[1]
        # ... skipping 2 it gets there at 150 s, before them: the piece {(1,4), (5,4)} on [1, 3, 4] must be refused
        ok, stops = StationSelection._joint_split_m3_check(m, data, mapping, 1, route, items[[1, 3]])
        @test stops == [1, 3, 4] && !ok
        # without the walker the same subsequence is fine
        @test StationSelection._joint_split_m3_check(m, data, mapping, 1, route, items[[1]])[1]
    end

    @testset "demand_groups = :rider, demand_weighting = :pax" begin
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(pickup_deadline_s = 900.0,
                                                                                    demand_groups = :od)
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(pickup_deadline_s = 900.0,
                                                                                    demand_weighting = :riders)
        riderf = AggregateODRouteJointRoutingAssignmentFormulation(
            route_regularization_weight = 10.0, walk_cost_weight = 0.1, repositioning_time = 20.0,
            pickup_deadline_s = 900.0, ride_time_limit_ratio = 2.0, max_stops = 10,
            demand_groups = :rider, demand_weighting = :pax)
        data, problem = problem_for(reqs)
        mapping = build_model(problem, riderf, solver(18)).mapping
        @test length(mapping.Omega_s[1]) == length(reqs)                      # one group per request
        @test count(==((1, 4)), mapping.Omega_s[1]) == 4
        @test sort(mapping.Q_s[1]) == sort([r[3] for r in reqs])               # Q = that request's passengers
        @test sort(vcat(mapping.group_rows_s[1]...)) == collect(1:length(reqs))
        # OD pair groups: same requests, one group per OD pair, Q = requests (:count) or passengers (:pax)
        odm = build_model(problem, formulation, solver(18)).mapping
        @test length(odm.Omega_s[1]) == 4 && odm.Q_s[1][findfirst(==((1, 4)), odm.Omega_s[1])] == 4
        # One OD pair with 7 orders of 3 (21 passengers): grouped by OD pair it can only ride as one group of
        # 21 (the heuristic lets a lone group exceed capacity); grouped by rider, every route is within 18.
        reqs21 = vcat(fill((1, 4, 3), 7), fill((2, 4, 2), 2))
        data21, problem21 = problem_for(reqs21)
        byod = run_opt(problem21, formulation, solver(18))
        byrider = run_opt(problem21, riderf, solver(18))
        @test byrider.termination_status in (SOLVE_OPTIMAL, SOLVE_FEASIBLE)
        load(res) = maximum(joint_split_column_max_load(c, joint_split_group_pax(data21, res.mapping)) for c in chosen(res))
        @test load(byod) == 21
        @test load(byrider) <= 18
        st = byrider.metadata["cg_split_stats"]
        @test get(st, "check_fail_priced", 0) == 0 && get(st, "check_fail_seed", 0) == 0
        # Benders groups by OD pair only
        @test_throws ArgumentError StationSelection._joint_routing_assignment_shared_fields(riderf)
    end


    @testset "end to end: plan within capacity, self-test clean" begin
        data, problem = problem_for(reqs)
        free = run_opt(problem, formulation, solver(nothing))
        capd = run_opt(problem, formulation, solver(18))
        @test free.termination_status in (SOLVE_OPTIMAL, SOLVE_FEASIBLE)
        @test capd.termination_status in (SOLVE_OPTIMAL, SOLVE_FEASIBLE)
        pax = joint_split_group_pax(data, capd.mapping)
        # this instance pools past 18 when uncapacitated, so the split has work to do
        @test maximum(joint_split_column_max_load(c, pax) for c in chosen(free)) > 18
        @test isnothing(free.metadata["cg_vehicle_capacity"])
        @test all(joint_split_column_max_load(c, pax) <= 18 for c in chosen(capd))
        st = capd.metadata["cg_split_stats"]
        @test capd.metadata["cg_vehicle_capacity"] == 18
        @test get(st, "check_fail_priced", 0) == 0 && get(st, "check_fail_seed", 0) == 0
        @test get(st, "over_capacity_columns", 0) > 0 && get(st, "split_ok", 0) > 0
        @test get(st, "mip_columns_dropped", 0) > 0        # recovery counts reach the metadata
        @test capd.objective_value >= free.objective_value - 1e-6
    end
end
