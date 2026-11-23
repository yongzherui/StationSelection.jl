# Pricing at late-CG duals. Runs capacity-4 CG (no integer recovery) for N iterations (to
# convergence by default), re-solves the master, then times one pricing round per scenario:
# :exact, and :exact_capacitated at CAP_RHO = 1 and 0; then profiles the slowest capacitated one.
# usage: DUALS_IN=<file> julia --project=env_cap prof_fixed.jl <instance.toml> [unused] [limit=120]
const ROOT = "/home/yongzr/2025-09-JacqWang-Microtransit"
include(joinpath(ROOT, "ExplorationCG.jl", "src", "ExplorationCG.jl")); using .ExplorationCG
const E = ExplorationCG; const SS = E.SS
using JuMP, Printf, Profile, Dates
inst = E.load_instance(ARGS[1], ROOT)
N = parse(Int, get(ARGS, 2, "1000")); limit = parse(Float64, get(ARGS, 3, "120"))
f = inst.form
form = SS.AggregateODRouteJointRoutingAssignmentFormulation(; route_regularization_weight = f.route_regularization_weight,
    walk_cost_weight = f.walk_cost_weight, repositioning_time = f.repositioning_time, max_wait_time = f.max_wait_time,
    detour_factor = f.detour_factor, max_stops = f.max_stops, demand_weighting = f.demand_weighting, vehicle_capacity = 4)
ENV["CAP_RHO"] = "1"
using Serialization
solver = SS.CGSolver(pricing = SS.CGPricingConfig())
br = SS.build_model(inst.problem, form, solver)
m = br.model
duals = deserialize(ENV["DUALS_IN"])
println("loaded duals from ", ENV["DUALS_IN"])
delete!(JuMP.object_dictionary(m), :joint_routing_assignment_capacitated_critical)
function run(mode, cap, s, rho; lim = limit, prof = false)
    ENV["CAP_RHO"] = string(rho)
    m[:joint_routing_assignment_pricing_mode] = mode
    m[:joint_routing_assignment_vehicle_capacity] = cap
    built = SS._pricing_build_scenario_context(form, br.mapping, s, m, duals)
    built === nothing && return 0.0
    ctx = built[1]
    t = @elapsed (labels, ex, st) = prof ? (@profile SS._run_label_setting(ctx; time_limit = lim, reduced_cost_tol = 1e-6)) :
        SS._run_label_setting(ctx; time_limit = lim, reduced_cost_tol = 1e-6)
    best = ctx isa SS.JointRoutingAssignmentCapacitatedSearchContext ? st.best_capacitated_rc :
        minimum((l.reduced_cost for l in labels); init = Inf)
    isdefined(SS, :CAP_COUNTS) && (c = SS.CAP_COUNTS; println("  counts riders_with_options ", c[1], " potential_ok ", c[2], " safe_ok ", c[3], " forced ", c[4],
        " dfs_actions ", c[5], " dfs_nodes ", c[6], " leaf_cut ", c[7]); fill!(c, 0))
    isdefined(SS, :CAP_DOM) && (d = SS.CAP_DOM; println("  dom calls ", d[1], " fail time ", d[2], " support ", d[3], " stops ", d[4], " budget ", d[5],
        " ages ", d[6], " loads ", d[7], " noncrit ", d[8], " crit ", d[9], " ok ", d[10], " ok_cross_mask ", d[11]); fill!(d, 0))
    @printf("%-20s rho %s s=%d sec %.2f exhausted %s labels %d live %d pruned %s best %.3f\n", string(mode, "/", cap), rho, s, t, ex,
            st.labels_generated, st.max_live_labels, get(st, :pruned_by_bound, "-"), best)
    flush(stdout)
    return t
end
times = Dict{Int, Float64}()
only1 = get(ENV, "FIXED_ONLY_RHO1", "0") == "1"
for s in 1:inst.n_scenarios
    only1 || run(:exact, nothing, s, 0)
    times[s] = run(:exact_capacitated, 4, s, 1)
    only1 || run(:exact_capacitated, 4, s, 0)
end
only1 && exit(0)
sw = argmax(times)
Profile.init(n = 10^7, delay = 0.002)
run(:exact_capacitated, 4, sw, 1; prof = true)
Profile.print(format = :flat, sortedby = :count, mincount = 100, noisefloor = 2.0)
