# @code_warntype on the capacitated pricer's hot functions (seed-master duals, scenario 1).
const ROOT = "/home/yongzr/2025-09-JacqWang-Microtransit"
include(joinpath(ROOT, "ExplorationCG.jl", "src", "ExplorationCG.jl")); using .ExplorationCG
const E = ExplorationCG; const SS = E.SS
using JuMP, InteractiveUtils
inst = E.load_instance(ARGS[1], ROOT)
f = inst.form
form = SS.AggregateODRouteJointRoutingAssignmentFormulation(; route_regularization_weight = f.route_regularization_weight,
    walk_cost_weight = f.walk_cost_weight, repositioning_time = f.repositioning_time, max_wait_time = f.max_wait_time,
    detour_factor = f.detour_factor, max_stops = f.max_stops, demand_weighting = f.demand_weighting, vehicle_capacity = 4)
br = SS.build_model(inst.problem, form, SS.CGSolver(pricing = SS.CGPricingConfig()))
m = br.model; set_silent(m); optimize!(m)
duals = SS.extract_joint_routing_assignment_duals(m)
m[:joint_routing_assignment_pricing_mode] = :exact_capacitated
m[:joint_routing_assignment_vehicle_capacity] = 4
ctx = SS._pricing_build_scenario_context(form, br.mapping, 1, m, duals)[1]
pd = ctx.pricing_data
l = SS.initial_joint_routing_assignment_capacitated_pricing_labels(pd)[1]
acts = SS._capacitated_candidate_actions(l, pd; ctx = ctx, thr = -1e-6)
a = acts[1]
child = SS._extend_joint_routing_assignment_capacitated_label(l, a, pd)
bs = SS._make_joint_routing_assignment_capacitated_label_bitsets(child, ctx)
function show_bad(f, types)
    io = IOBuffer()
    code_warntype(io, f, types)
    s = String(take!(io))
    lines = filter(x -> occursin(r"::Any|Core\.Box|::Union\{", x), split(s, "\n"))
    println("== ", f, " : ", length(lines), " flagged")
    for x in lines[1:min(end, 25)]; println("   ", strip(x)); end
end
show_bad(SS._capacitated_candidate_actions, Tuple{typeof(l), typeof(pd)})
show_bad(SS._extend_joint_routing_assignment_capacitated_label, Tuple{typeof(l), typeof(a), typeof(pd)})
show_bad(SS._make_joint_routing_assignment_capacitated_label_bitsets, Tuple{typeof(l), typeof(ctx)})
show_bad(SS._capacitated_remaining_bound, Tuple{typeof(ctx), typeof(l), Vector{Int32}, Vector{Float64}, Vector{Int}, Vector{Float64}})
show_bad(SS._capacitated_scan!, Tuple{SS.CapacitatedScan, typeof(ctx), typeof(l), Vector{Int32}, Vector{Float64}})
show_bad(SS._capacitated_scan_eval!, Tuple{SS.CapacitatedScan, typeof(ctx), BitSet, Vector{Int}})
show_bad(SS._capacitated_dominates, Tuple{SS.JointRoutingAssignmentCapacitatedDominanceFilters, typeof(bs), SS.JointRoutingAssignmentCapacitatedDominanceFilters, typeof(bs), typeof(pd), SS.JointRoutingAssignmentCapacitatedDominanceRules{true, true}})
show_bad(SS._pricing_candidate_next_nodes, Tuple{typeof(ctx), typeof(l)})
show_bad(SS._capacitated_reach_ages, Tuple{typeof(l), Dict{Int, Float64}, typeof(pd), Int})
show_bad(SS._capacitated_rider_potential, Tuple{typeof(pd), Int, Int, Float64, Dict{Int, Float64}, Float64})
show_bad(SS._capacitated_subsets!, Tuple{Vector{SS.JointRoutingAssignmentCapacitatedAction}, typeof(l), typeof(pd), typeof(ctx), Float64, Int, Float64, Dict{Int, Float64}, Dict{Int, Int}, Vector{Tuple{Int, Int, Float64}}, Vector{Int}, Dict{Int, Vector{Tuple{Int, Float64}}}, Bool})
