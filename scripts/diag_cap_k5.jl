# Rider data and root bounds for one scenario at the seed-master duals (capacity 4).
# usage: julia --project=env_cap diag_cap_k5.jl <instance.toml> [scenario=1]
const ROOT = "/home/yongzr/2025-09-JacqWang-Microtransit"
include(joinpath(ROOT, "ExplorationCG.jl", "src", "ExplorationCG.jl")); using .ExplorationCG
const E = ExplorationCG; const SS = E.SS
using JuMP, Printf
inst = E.load_instance(ARGS[1], ROOT)
s0 = parse(Int, get(ARGS, 2, "1"))
f = inst.form
form = SS.AggregateODRouteJointRoutingAssignmentFormulation(; route_regularization_weight = f.route_regularization_weight,
    walk_cost_weight = f.walk_cost_weight, repositioning_time = f.repositioning_time, max_wait_time = f.max_wait_time,
    detour_factor = f.detour_factor, max_stops = f.max_stops, demand_weighting = f.demand_weighting, vehicle_capacity = 4)
br = SS.build_model(inst.problem, form, SS.CGSolver(pricing = SS.CGPricingConfig()))
m = br.model; set_silent(m); optimize!(m)
duals = SS.extract_joint_routing_assignment_duals(m)
m[:joint_routing_assignment_pricing_mode] = :exact_capacitated
m[:joint_routing_assignment_vehicle_capacity] = 4
ctx = SS._pricing_build_scenario_context(form, br.mapping, s0, m, duals)[1]
pd = ctx.pricing_data; base = pd.base
println("w = ", base.route_regularization_weight, " max_wait ", base.max_wait_time, " max_stops ", base.max_stops,
        " nodes ", length(base.nodes), " opps ", length(base.opportunities), " riders ", length(pd.max_reward))
println("p  Q  maxr   nopts  min_d  max_ride_limit  best r-(w/C)Qd")
for p in sort(collect(keys(pd.max_reward)); by = p -> -pd.max_reward[p])
    os = pd.opps_by_p[p]
    ds = [pd.sp[pd.node_idx[o.origin], pd.node_idx[o.destination]] for o in os]
    v = maximum(o.reward - base.route_regularization_weight / 4 * pd.load[p] * d for (o, d) in zip(os, ds))
    @printf("%3d %d %9.1f %4d %7.0f %7.0f %9.1f\n", p, pd.load[p], pd.max_reward[p], length(os), minimum(ds),
            maximum(o.ride_limit for o in os), v)
end
for l in SS.initial_joint_routing_assignment_capacitated_pricing_labels(pd)[1:min(end, 5)]
    bs = SS._make_joint_routing_assignment_capacitated_label_bitsets(l, ctx)
    @printf("root label at %d: rc %.1f bound %.1f priority %.1f\n", l.current, l.reduced_cost, bs.bound, l.reduced_cost - bs.bound)
end
const ROUTES = Dict{Vector{Int}, Int}(); const DEPTH = zeros(Int, 20); const NCRED = zeros(Int, 40)
const PRI = Float64[]
function SS._pricing_on_label_inserted(ctx::SS.JointRoutingAssignmentCapacitatedSearchContext, label)
    ROUTES[label.route] = get(ROUTES, label.route, 0) + 1
    DEPTH[label.route_length] += 1
    NCRED[length(label.credited) + 1] += 1
    return nothing
end
labels, ex, st = SS._run_label_setting(ctx; time_limit = 60.0, reduced_cost_tol = 1e-6)
println("stats ", st)
println("inserted labels ", sum(values(ROUTES)), " distinct routes ", length(ROUTES))
println("by depth ", DEPTH[1:11])
println("by #credited ", NCRED[1:12])
cnt = sort(collect(values(ROUTES)); rev = true)
println("labels per route: top ", cnt[1:min(end, 15)], " median ", cnt[div(end + 1, 2)])
top = sort(collect(ROUTES); by = x -> -x[2])[1:5]
for (r, c) in top
    println("  route ", r, " x", c)
end
best = sort(labels; by = l -> SS._capacitated_route_result(ctx, l.route)[3])[1:min(end, 5)]
for l in best
    r = SS._capacitated_route_result(ctx, l.route)
    println("route ", l.route, " time ", round(l.time), " cap rc ", round(r[3]; digits = 1), " riders ", r[1], " max_load ", r[5])
end
