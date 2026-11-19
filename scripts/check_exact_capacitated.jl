# Exactness check for :exact_capacitated on random small pricing instances.
#   brute force: every route (revisits allowed, <= max_stops stops) x every capacity-feasible way
#   to credit riders (each rider: skip, or one (j,k) option boarded at any visit of j inside the
#   pickup window and dropped at a later visit of k within its ride limit) -> min reduced cost.
#   pricer: min reduced cost over the labels the search returns.
# Also: capacity large enough never to bind must reproduce :exact (uncapacitated, with upgrades).
using StationSelection, Random, Printf
const SS = StationSelection

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
    cands = SS.PassengerAssignmentCandidate[]
    load = Dict{Int,Int}()
    for p in 1:P
        load[p] = rand(rng, 1:3)
        pairs = Set{Tuple{Int,Int}}()
        while length(pairs) < rand(rng, 1:maxopts)
            j, k = rand(rng, 1:n), rand(rng, 1:n)
            j == k || push!(pairs, (j, k))
        end
        for (j, k) in pairs
            push!(cands, SS.PassengerAssignmentCandidate(p, j, k, df * travel[(j, k)], round(rand(rng) * 900 + 50)))
        end
    end
    W = rand(rng, (400.0, 800.0, 1500.0, Inf))
    rho = rand(rng, (0.0, 20.0))
    return collect(1:n), travel, cands, load, W, rho
end

function brute(nodes, travel, cands, load, W, rho, cap, maxstops)
    byp = Dict{Int,Vector{SS.PassengerAssignmentCandidate}}()
    for c in cands; push!(get!(byp, c.p, SS.PassengerAssignmentCandidate[]), c); end
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

function pricer_min(ctx)
    labels, exhausted, stats = SS._run_label_setting(ctx; time_limit = 120.0, reduced_cost_tol = 1e-6)
    exhausted || error("search not exhausted")
    if ctx isa SS.JointRoutingAssignmentCapacitatedSearchContext
        return stats.best_capacitated_rc, stats
    end
    return isempty(labels) ? Inf : minimum(l.reduced_cost for l in labels), stats
end

function main()
    rng = MersenneTwister(parse(Int, get(ARGS, 1, "1")))
    ntrials = parse(Int, get(ARGS, 2, "200"))
    bad = 0; checked = 0; improving = 0
    for trial in 1:ntrials
        n = rand(rng, 3:parse(Int, get(ENV, "CHK_NMAX", "5"))); P = rand(rng, 2:parse(Int, get(ENV, "CHK_PMAX", "6"))); maxstops = rand(rng, 3:6)
        nodes, travel, cands, load, W, rho = instance(rng; n, P, maxopts = 3)
        kw = (route_regularization_weight = 1.0, pickup_deadline_s = W, repositioning_time = rho, max_stops = maxstops)
        only = parse(Int, get(ENV, "CHK_ONLY", "0"))
        if only > 0 && trial == only
            kept = cands
            bd = SS.create_joint_routing_assignment_pricing_data(1, nodes, travel, kept; compensated_dominance = true, kw...)
            ctx = SS.JointRoutingAssignmentSearchContext(bd)
            labels, ex, _ = SS._run_label_setting(ctx; time_limit = 120.0, reduced_cost_tol = 1e-6)
            bl = labels[argmin([l.reduced_cost for l in labels])]
            println("exact best ", bl.reduced_cost, " route ", bl.route)
            want, wroute = brute(nodes, travel, kept, load, W, rho, 1000, maxstops)
            println("brute best ", want, " route ", wroute, " W=", W, " rho=", rho, " maxstops=", maxstops)
            println("replay of brute route: ", SS._joint_routing_assignment_column_from_route(wroute, bd))
            println("travel: ", travel)
            println("cands: ", kept)
        end
        for cap in (1, 2, 3, 4, 1000)
            for comp in (true, false)
                if only > 0
                    sinit = get(ENV, "CHK_SINIT", "mix"); mode = sinit == "mix" ? rand(rng, ("empty", "random", "all")) : sinit
                    mode == "random" && [rand(rng) < 0.5 for _ in 1:P]; rand(rng, 1:3)
                    continue
                end
                sinit = get(ENV, "CHK_SINIT", "mix")
                mode = sinit == "mix" ? rand(rng, ("empty", "random", "all")) : sinit
                crit = mode == "empty" ? Bool[] : mode == "all" ? fill(true, P) : [rand(rng) < 0.5 for _ in 1:P]
                pd = SS.create_joint_routing_assignment_capacitated_pricing_data(1, nodes, travel, cands;
                         capacity = cap, load = load, compensated_dominance = comp, critical = crit,
                         stop_at_first_column = false, growth_routes = rand(rng, 1:3),
                         initial_critical = mode == "empty" ? :empty : :all, kw...)
                got, _ = pricer_min(SS.JointRoutingAssignmentCapacitatedSearchContext(pd; rho = parse(Float64, get(ENV, "CAP_RHO", "1"))))
                kept = [c for c in cands if load[c.p] <= cap]
                want, wroute = brute(nodes, travel, kept, load, W, rho, cap, maxstops)
                checked += 1
                ok = want < -1e-6 ? isapprox(got, want; atol = 1e-6) : !(got < -1e-6)
                want < -1e-6 && (improving += 1)
                if !ok
                    bad += 1
                    @printf("MISMATCH trial %d cap %d comp %s S0=%s: pricer %.3f brute %.3f (route %s) n=%d P=%d stops=%d W=%s\n",
                            trial, cap, comp, mode, got, want, string(wroute), n, P, maxstops, string(W))
                end
                if comp
                    # :darp with the same capacity (boarding commits to (j, k), onboard set explicit):
                    # an independent formulation, checked against brute force too
                    dd = SS.create_joint_routing_assignment_darp_pricing_data(1, nodes, travel, kept;
                             capacity = cap == 1000 ? typemax(Int) : cap, load = load, kw...)
                    dctx = SS.JointRoutingAssignmentDarpSearchContext(dd)
                    dl, dex, _ = SS._run_label_setting(dctx; time_limit = 120.0, reduced_cost_tol = 1e-6)
                    dex || error("darp search not exhausted")
                    dgot = minimum((c.reduced_cost for c in (SS._pricing_candidate_from_label(dctx, l) for l in dl) if c !== nothing); init = Inf)
                    dok = want < -1e-6 ? isapprox(dgot, want; atol = 1e-6) : !(dgot < -1e-6)
                    dok || (bad += 1; @printf("DARP MISMATCH trial %d cap %d: darp %.3f brute %.3f (route %s)\n", trial, cap, dgot, want, string(wroute)))
                end
                if cap == 1000 && comp
                    bd = SS.create_joint_routing_assignment_pricing_data(1, nodes, travel, kept; compensated_dominance = true, kw...)
                    ex, _ = pricer_min(SS.JointRoutingAssignmentSearchContext(bd))
                    same = (ex < -1e-6 || got < -1e-6) ? isapprox(ex, got; atol = 1e-6) : true
                    same || (bad += 1; @printf("EXACT MISMATCH trial %d: :exact %.3f capacitated(1000) %.3f\n", trial, ex, got))
                end
            end
        end
    end
    @printf("checked %d (%d with an improving route), mismatches %d\n", checked, improving, bad)
end
main()
