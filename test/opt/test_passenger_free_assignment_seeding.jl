@testset "Passenger free-assignment two-stop seeding" begin
    using Gurobi

    # One Gurobi.Env reused across this whole testset -- constructing several per
    # process has previously caused a silent multi-minute stall on this cluster.
    SEED_ENV = Gurobi.Env()

    function tiny_instance(; n_stations::Int=5, l::Int=3, detour_factor::Float64=2.0)
        stations = DataFrame(
            id=collect(1:n_stations),
            lon=Float64.(0:(n_stations - 1)),
            lat=zeros(n_stations),
        )
        requests = DataFrame(
            id=[1, 2, 3],
            origin_station_id=[1, 2, 1],
            destination_station_id=[n_stations, n_stations - 1, n_stations - 1],
            request_time=[
                DateTime(2024, 1, 1, 8),
                DateTime(2024, 1, 1, 8, 1),
                DateTime(2024, 1, 1, 8, 2),
            ],
        )
        walking = Dict{Tuple{Int, Int}, Float64}()
        routing = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:n_stations, j in 1:n_stations
            walking[(i, j)] = 10.0 * abs(i - j)
            routing[(i, j)] = Float64(abs(i - j))
        end
        data = create_station_selection_data(stations, requests, walking; routing_costs=routing)
        model = AggregateODRouteModel(
            l;
            route_regularization_weight = 1.0,
            walk_cost_weight            = 0.1,
            repositioning_time          = 0.0,
            max_walking_distance        = 1000.0,
            max_wait_time               = 100.0,
            detour_factor               = detour_factor,
            max_stops                   = 3,
        )
        return data, model
    end

    md_for(data, model) = create_passenger_free_assignment_master_data(
        model, data, create_map(model, data),
    )

    @testset "seeds are well-formed two-stop routes" begin
        data, model = tiny_instance()
        md = md_for(data, model)
        seeds = passenger_free_assignment_two_stop_seed_columns(md)
        @test !isempty(seeds)

        scenario_of = Dict(p.id => p.scenario for p in md.passengers)
        for c in seeds
            @test length(c.route) == 2
            j, k = c.route
            @test j != k
            @test c.tau ≈ md.travel_cost[(j, k)]
            @test !isempty(c.assignments)
            s = Int(c.metadata["scenario"])
            for (p, jj, kk) in c.assignments
                # a two-stop route offers exactly one (pickup, dropoff) option
                @test (jj, kk) == (j, k)
                @test scenario_of[p] == s
                @test (j, k) in md.feasible_assignments[p]
            end
        end

        # One column per (scenario, j, k), not per (passenger, j, k).
        keys_seen = [(Int(c.metadata["scenario"]), c.route[1], c.route[2]) for c in seeds]
        @test length(unique(keys_seen)) == length(keys_seen)
        @test all(c -> c.id > 0, seeds)
        @test length(unique(c.id for c in seeds)) == length(seeds)
    end

    @testset "seeds certify EVERY feasible (p, j, k)" begin
        # The claim the big-M rests on: ride_limit = detour_factor * travel(j,k)
        # and replaying [j,k] ages the pickup by exactly travel(j,k), so with
        # detour_factor >= 1 no feasible assignment is left uncovered.
        data, model = tiny_instance()
        md = md_for(data, model)
        seeds = passenger_free_assignment_two_stop_seed_columns(md)

        covered = Set{Tuple{Int, Int, Int}}()
        for c in seeds, a in c.assignments
            push!(covered, a)
        end
        expected = Set{Tuple{Int, Int, Int}}()
        for p in md.passengers, (j, k) in md.feasible_assignments[p.id]
            j == k && continue
            haskey(md.travel_cost, (j, k)) || continue
            push!(expected, (p.id, j, k))
        end
        @test !isempty(expected)
        @test covered == expected
    end

    @testset "detour_factor >= 1 is enforced, so coverage is unconditional" begin
        # The coverage claim above needs travel(j,k) <= ride_limit = detour_factor
        # * travel(j,k), i.e. detour_factor >= 1. That is not an assumption the
        # seeder gets to make on its own -- the model constructor enforces it, so
        # the uncoverable configuration is unreachable. (The seeder still applies
        # the age test explicitly, in case this guard ever moves.)
        @test_throws ArgumentError tiny_instance(; detour_factor=0.5)
    end

    @testset "seeding leaves the optimum untouched" begin
        # Seeding only pre-populates columns the pricer could generate itself, so
        # a certified LP bound and the MIP objective must be identical.
        data, model = tiny_instance()
        common = (
            optimizer_env=SEED_ENV, max_cg_iters=500, n_candidates=10,
            max_new_columns=10, pricing_time_limit_sec=30.0,
            certification_time_limit_sec=60.0, ip_time_limit_sec=60.0,
            verbose=false,
        )
        off = run_passenger_free_assignment_column_generation(
            model, data; seed_two_stop_routes=false, common...,
        )
        on = run_passenger_free_assignment_column_generation(
            model, data; seed_two_stop_routes=true, common...,
        )

        @test off.cg_stop_reason == :optimality_proven
        @test on.cg_stop_reason == :optimality_proven
        @test on.lp_bound ≈ off.lp_bound rtol = 1e-6
        @test on.mip_objective ≈ off.mip_objective rtol = 1e-6

        @test get(on.final_result.metadata, "seed_two_stop_columns", 0) > 0
        @test get(off.final_result.metadata, "seed_two_stop_columns", -1) == 0
    end

    @testset "station-simple warm start reaches the same certified optimum" begin
        # Warm start prices elementary routes first, then switches to the exact
        # revisit-tolerant pricer once the elementary universe is exhausted. It must
        # certify the SAME LP optimum and MIP objective as a pure revisit-tolerant
        # run: the elementary phase only pre-populates columns, and the exact phase
        # still runs to its own exhaustion before optimality is claimed.
        data, model = tiny_instance()
        common = (
            optimizer_env=SEED_ENV, max_cg_iters=500, n_candidates=10,
            max_new_columns=10, pricing_time_limit_sec=30.0,
            certification_time_limit_sec=60.0, ip_time_limit_sec=60.0,
            verbose=false,
        )
        plain = run_passenger_free_assignment_column_generation(model, data; common...)
        warm = run_passenger_free_assignment_column_generation(
            model, data; station_simple_warm_start=true, common...,
        )

        @test plain.cg_stop_reason == :optimality_proven
        @test warm.cg_stop_reason == :optimality_proven
        @test warm.lp_bound ≈ plain.lp_bound rtol = 1e-6
        @test warm.mip_objective ≈ plain.mip_objective rtol = 1e-6

        # The run must actually go through BOTH pricer phases: elementary first, then
        # the revisit-tolerant pricer after the warm-start switch.
        phases = Set(r.pricer for r in warm.iteration_rows)
        @test "station_simple" in phases
        @test "revisit" in phases
    end

    @testset "L=2 harvester retains exact certification and optimum" begin
        data, model = tiny_instance()
        common = (
            optimizer_env=SEED_ENV, max_cg_iters=500, n_candidates=10,
            max_new_columns=10, pricing_time_limit_sec=30.0,
            certification_time_limit_sec=60.0, ip_time_limit_sec=60.0,
            verbose=false,
        )
        plain = run_passenger_free_assignment_column_generation(
            model, data; station_simple_warm_start=false, common...,
        )
        coarse = run_passenger_free_assignment_column_generation(
            model, data; reward_coarsening_levels=2,
            station_simple_warm_start=false, common...,
        )
        @test coarse.cg_stop_reason == :optimality_proven
        @test coarse.lp_bound ≈ plain.lp_bound rtol = 1e-6
        @test coarse.mip_objective ≈ plain.mip_objective rtol = 1e-6
        @test any(r.pricer == "reward_coarsened_L2" for r in coarse.iteration_rows)
        @test any(r.phase == "certification" && r.pricer == "revisit" for r in coarse.iteration_rows)
        @test_throws ArgumentError run_passenger_free_assignment_column_generation(
            model, data; reward_coarsening_levels=2, use_station_simple=true,
            optimizer_env=SEED_ENV, verbose=false,
        )
    end


    @testset "station-simple warm start is the exact default" begin
        data, model = tiny_instance()
        result = run_passenger_free_assignment_column_generation(
            model, data;
            optimizer_env=SEED_ENV, max_cg_iters=500, n_candidates=10,
            max_new_columns=10, pricing_time_limit_sec=30.0,
            certification_time_limit_sec=60.0, ip_time_limit_sec=60.0,
            verbose=false,
        )
        @test result.cg_stop_reason == :optimality_proven
        @test result.final_result.metadata["station_simple_warm_start"] === true
        phases = Set(row.pricer for row in result.iteration_rows)
        @test "station_simple" in phases
        @test "revisit" in phases
    end

    @testset "seeded start never pays the unserved penalty" begin
        # The point of seeding: v[p] is inert from iteration 1, so the opening LP
        # is a real service cost rather than a big-M artefact.
        data, model = tiny_instance()
        result = run_passenger_free_assignment_column_generation(
            model, data;
            optimizer_env=SEED_ENV, max_cg_iters=500, n_candidates=10,
            max_new_columns=10, pricing_time_limit_sec=30.0,
            certification_time_limit_sec=60.0, ip_time_limit_sec=60.0,
            seed_two_stop_routes=true, verbose=false,
        )
        @test isempty(result.unserved_passengers)
        @test !isempty(result.iteration_rows)
        # An LP still carrying slack would be at least one unserved_penalty tall.
        md = md_for(data, model)
        @test result.iteration_rows[1].lp_bound < md.unserved_penalty
    end
end
