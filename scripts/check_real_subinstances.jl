# Exactness check of :exact_capacitated on pieces of the real instance: at the master duals after
# `iters` CG iterations (0 = seed master), each scenario's real stations, rewards, loads, ride
# limits, travel times and route weight are cut down to a few random stations and riders, and
# the pricer's minimum reduced cost is compared with brute force (every route x every
# capacity-feasible crediting; same definition as check_exact_capacitated.jl).
# usage: julia --project=env_cap check_real_subinstances.jl <instance.toml> <iters,...> <trials per scenario> [seed]
const ROOT = "/home/yongzr/2025-09-JacqWang-Microtransit"
include(joinpath(ROOT, "ExplorationCG.jl", "src", "ExplorationCG.jl")); using .ExplorationCG
const E = ExplorationCG; const SS = E.SS
using JuMP, Random, Printf
inst = E.load_instance(ARGS[1], ROOT)
iters = parse.(Int, split(ARGS[2], ","))
ntr = parse(Int, ARGS[3])
rng = MersenneTwister(parse(Int, get(ARGS, 4, "1")))
f = inst.form
form = SS.AggregateODRouteJointRoutingAssignmentFormulation(; route_regularization_weight = f.route_regularization_weight,
    walk_cost_weight = f.walk_cost_weight, repositioning_time = f.repositioning_time, max_wait_time = f.max_wait_time,
    detour_factor = f.detour_factor, max_stops = f.max_stops, demand_weighting = f.demand_weighting, vehicle_capacity = 4)

function brute(nodes, travel, cands, load, W, rho, w, cap, maxstops)
    byp = Dict{Int, Vector{SS.PassengerAssignmentCandidate}}()
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
        return w * (t[end] + rho) - bestR
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

ENV["CAP_RHO"] = get(ENV, "CAP_RHO", "1")
bad = 0; checked = 0; improving = 0
done_iters = 0
solver = SS.CGSolver(config = SS.SolverOptions(silent = true, threads = Threads.nthreads()),
                     pricing = SS.CGPricingConfig(), max_iterations = 1, pricing_time_limit_sec = 600.0,
                     certifying_pricing_time_limit_sec = 600.0, total_time_limit_sec = 3000.0,
                     parallel_scenario_pricing = true, recover_integer_solution = false)
br = SS.build_model(inst.problem, form, solver)
m = br.model; set_silent(m)
for it in sort(iters)
    global done_iters, bad, checked, improving
    while done_iters < it          # one CG iteration at a time on the same model
        SS.optimize_model(br, solver); done_iters += 1
    end
    optimize!(m)
    duals = SS.extract_joint_routing_assignment_duals(m)
    delete!(JuMP.object_dictionary(m), :joint_routing_assignment_capacitated_critical)
    m[:joint_routing_assignment_pricing_mode] = :exact_capacitated
    m[:joint_routing_assignment_vehicle_capacity] = 4
    @printf("duals after %d iterations: master LP %.2f\n", it, objective_value(m)); flush(stdout)
    for s in 1:inst.n_scenarios
        built = SS._pricing_build_scenario_context(form, br.mapping, s, m, duals)
        built === nothing && continue
        pd0 = built[1].pricing_data; b0 = pd0.base
        allc = [SS.PassengerAssignmentCandidate(o.p, o.origin, o.destination, o.ride_limit, o.reward) for o in b0.opportunities]
        for trial in 1:ntr
            n = rand(rng, 4:6); maxstops = rand(rng, 3:(n <= 5 ? 6 : 5))
            used = unique(vcat([c.origin for c in allc], [c.destination for c in allc]))
            nodes = sort(shuffle(rng, used)[1:min(n, length(used))])
            sub = [c for c in allc if c.origin in nodes && c.destination in nodes]
            riders = unique(c.p for c in sub)
            keep = Set(shuffle(rng, riders)[1:min(length(riders), rand(rng, 3:8))])
            cands = [c for c in sub if c.p in keep]
            isempty(cands) && continue
            travel = Dict((u, v) => b0.travel_cost[(u, v)] for u in nodes for v in nodes if u != v)
            load = Dict(p => pd0.load[p] for p in keep)
            for cap in (2, 4), comp in (true, false)
                kept = [c for c in cands if load[c.p] <= cap]
                isempty(kept) && continue
                maxp = maximum(c.p for c in cands)
                pd = SS.create_joint_routing_assignment_capacitated_pricing_data(1, nodes, travel, cands;
                         capacity = cap, load = load, compensated_dominance = comp, critical = fill(true, maxp),
                         stop_at_first_column = false, route_regularization_weight = b0.route_regularization_weight,
                         max_wait_time = b0.max_wait_time, repositioning_time = b0.repositioning_time, max_stops = maxstops)
                ctx = SS.JointRoutingAssignmentCapacitatedSearchContext(pd)
                labels, ex, st = SS._run_label_setting(ctx; time_limit = 300.0, reduced_cost_tol = 1e-6)
                ex || error("not exhausted")
                got = st.best_capacitated_rc
                want, wroute = brute(nodes, travel, kept, load, b0.max_wait_time, b0.repositioning_time,
                                     b0.route_regularization_weight, cap, maxstops)
                checked += 1
                tol = 1e-6 * max(1.0, abs(want))
                ok = want < -1e-6 ? isapprox(got, want; atol = tol) : !(got < -1e-6)
                want < -1e-6 && (improving += 1)
                if !ok
                    bad += 1
                    @printf("MISMATCH it %d s %d trial %d cap %d comp %s: pricer %.4f brute %.4f route %s nodes %s stops %d\n",
                            it, s, trial, cap, comp, got, want, string(wroute), string(nodes), maxstops)
                end
            end
        end
        @printf("  scenario %d: checked so far %d (%d improving), mismatches %d\n", s, checked, improving, bad); flush(stdout)
    end
end
@printf("checked %d (%d with an improving route), mismatches %d\n", checked, improving, bad)
