@testset "Joint: vehicle capacity (:exact_capacitated, capacitated :darp)" begin
    using Random
    using DataFrames
    using Dates

    # Brute force on small random instances: every route (revisits allowed, <= max_stops) x
    # every way to credit riders that keeps every arc within capacity. Both capacitated pricers
    # must reach its minimum reduced cost exactly (scripts/check_exact_capacitated.jl runs the
    # same comparison at scale).
    function instance(rng; n, P, maxopts)
        xy = [rand(rng, 2) .* 1000 for _ in 1:n]
        travel = Dict{Tuple{Int,Int},Float64}()
        for i in 1:n, j in 1:n
            i == j || (travel[(i, j)] = round(hypot((xy[i] .- xy[j])...)))
        end
        # shortest-path closure: real travel times are shortest paths, and the pricers only stop at
        # pickup/dropoff stations, so a rounded-Euclidean shortcut via another station is not a route
        for k in 1:n, i in 1:n, j in 1:n
            (i == j || i == k || j == k) && continue
            travel[(i, k)] + travel[(k, j)] < travel[(i, j)] && (travel[(i, j)] = travel[(i, k)] + travel[(k, j)])
        end
        df = rand(rng, (1.2, 1.5, 2.0, 3.0))
        cands = StationSelection.PassengerAssignmentCandidate[]
        load = Dict{Int,Int}()
        for p in 1:P
            load[p] = rand(rng, 1:3)
            pairs = Set{Tuple{Int,Int}}()
            while length(pairs) < rand(rng, 1:maxopts)
                j, k = rand(rng, 1:n), rand(rng, 1:n)
                j == k || push!(pairs, (j, k))
            end
            for (j, k) in pairs
                push!(cands, StationSelection.PassengerAssignmentCandidate(p, j, k, df * travel[(j, k)], round(rand(rng) * 900 + 50)))
            end
        end
        W = rand(rng, (400.0, 800.0, 1500.0, Inf))
        rho = rand(rng, (0.0, 20.0))
        return collect(1:n), travel, cands, load, W, rho
    end

    function brute(nodes, travel, cands, load, W, rho, cap, maxstops)
        byp = Dict{Int,Vector{StationSelection.PassengerAssignmentCandidate}}()
        for c in cands; push!(get!(byp, c.p, StationSelection.PassengerAssignmentCandidate[]), c); end
        ps = sort!(collect(keys(byp)))
        best = Inf; best_route = Int[]
        function evaluate(route)
            L = length(route)
            t = zeros(L)
            for i in 2:L; t[i] = t[i-1] + travel[(route[i-1], route[i])]; end
            opts = Dict(p => [(a, b, c.reward) for c in byp[p] for a in 1:L, b in 1:L
                              if a < b && route[a] == c.origin && route[b] == c.destination &&
                                 t[a] <= W + 1e-9 && t[b] - t[a] <= c.ride_limit + 1e-9] for p in ps)
            arc = zeros(Int, max(L - 1, 0))
            bestR = 0.0
            function rec(i, R)
                if i > length(ps)
                    bestR = max(bestR, R); return
                end
                p = ps[i]
                rec(i + 1, R)
                for (a, b, r) in opts[p]
                    all(e -> arc[e] + load[p] <= cap, a:(b-1)) || continue
                    for e in a:(b-1); arc[e] += load[p]; end
                    rec(i + 1, R + r)
                    for e in a:(b-1); arc[e] -= load[p]; end
                end
            end
            rec(1, 0.0)
            return (t[end] + rho) - bestR
        end
        function dfs(route)
            v = evaluate(route)
            v < best - 1e-9 && (best = v; best_route = copy(route))
            length(route) >= maxstops && return
            for nx in nodes
                nx == route[end] && continue
                push!(route, nx); dfs(route); pop!(route)
            end
        end
        for s in nodes; dfs([s]); end
        return best, best_route
    end


    rng = MersenneTwister(20261116)
    for trial in 1:20
        n = rand(rng, 3:5); P = rand(rng, 2:6); maxstops = rand(rng, 3:5)
        nodes, travel, cands, load, W, rho = instance(rng; n, P, maxopts = 3)
        kw = (route_regularization_weight = 1.0, pickup_deadline_s = W, repositioning_time = rho, max_stops = maxstops)
        for cap in (1, 2, 4)
            kept = [c for c in cands if load[c.p] <= cap]
            want, _ = brute(nodes, travel, kept, load, W, rho, cap, maxstops)
            for prune_rho in (0.0, 1.0)
                pd = StationSelection.create_joint_routing_assignment_capacitated_pricing_data(1, nodes, travel, cands;
                         capacity = cap, load = load, stop_at_first_column = false, kw...)
                ctx = StationSelection.JointRoutingAssignmentCapacitatedSearchContext(pd; rho = prune_rho)
                _, exhausted, stats = StationSelection._run_label_setting(ctx; time_limit = 60.0, reduced_cost_tol = 1e-6)
                @test exhausted
                got = stats.best_capacitated_rc
                @test want < -1e-6 ? isapprox(got, want; atol = 1e-6) : !(got < -1e-6)
            end
            isempty(kept) && continue
            dd = create_joint_routing_assignment_darp_pricing_data(1, nodes, travel, kept; capacity = cap, load = load, kw...)
            dctx = StationSelection.JointRoutingAssignmentDarpSearchContext(dd)
            dl, dex, _ = StationSelection._run_label_setting(dctx; time_limit = 60.0, reduced_cost_tol = 1e-6)
            @test dex
            dgot = minimum((c.reduced_cost for c in (StationSelection._pricing_candidate_from_label(dctx, l) for l in dl)
                            if c !== nothing); init = Inf)
            @test want < -1e-6 ? isapprox(dgot, want; atol = 1e-6) : !(dgot < -1e-6)
        end
    end

    @testset "CG with a capacity: columns respect it; a capacity that never binds = :exact" begin
        n = 7
        ods = [(1, 7), (2, 7), (1, 6), (2, 6), (3, 7), (2, 5)]
        stations = DataFrame(id = collect(1:n), lon = zeros(n), lat = zeros(n))
        requests = DataFrame(id = collect(1:length(ods)), start_station_id = first.(ods), end_station_id = last.(ods),
                             request_time = fill(DateTime(2026, 1, 1, 8, 0), length(ods)))
        cost = Dict((i, j) => Float64(abs(i - j)) for i in 1:n for j in 1:n)
        data = create_station_selection_data(stations, requests, cost; routing_costs = cost,
                                             scenarios = [("2026-01-01 08:00:00", "2026-01-01 09:00:00")])
        problem = StationSelectionProblem(data, 4; max_walking_distance = 1.5)
        f(cap) = AggregateODRouteJointRoutingAssignmentFormulation(pickup_deadline_s = Inf, route_regularization_weight = 1.0, walk_cost_weight = 0.1,
            repositioning_time = 0.0, detour_factor = 2.0, max_stops = 5, vehicle_capacity = cap)
        solve(cap) = run_opt(problem, f(cap), CGSolver(config = SolverOptions(silent = true), max_iterations = 200,
                                                       reduced_cost_tol = 1e-7, recover_integer_solution = true))
        base = solve(nothing)
        @test base.termination_status == SOLVE_OPTIMAL
        big = solve(1000)
        @test big.termination_status == SOLVE_OPTIMAL
        @test isapprox(big.objective_value, base.objective_value; atol = 1e-6)
        for cap in (1, 2)
            r = solve(cap)
            @test r.termination_status == SOLVE_OPTIMAL
            @test r.objective_value >= base.objective_value - 1e-6
            for col in values(r.model[:joint_routing_assignment_columns])
                haskey(col.metadata, "max_load") && @test col.metadata["max_load"] <= cap
            end
        end
        @test_throws ArgumentError run_opt(problem, f(2), DirectMIPSolver(config = SolverOptions(silent = true)))
        @test_throws ArgumentError AggregateODRouteJointRoutingAssignmentFormulation(pickup_deadline_s = Inf, vehicle_capacity = 0)
    end
end
