# One pricing round at the seed-master duals on an instance: :exact vs :exact_capacitated (cap 4),
# per scenario, then a flat profile of the capacitated search on one scenario.
# usage: julia --project=env_cap prof_cap_k5.jl <instance.toml> [limit=90] [scenario=1]
const ROOT = "/home/yongzr/2025-09-JacqWang-Microtransit"
include(joinpath(ROOT, "ExplorationCG.jl", "src", "ExplorationCG.jl")); using .ExplorationCG
const E = ExplorationCG; const SS = E.SS
using JuMP, Printf, Profile
inst = E.load_instance(ARGS[1], ROOT)
limit = parse(Float64, get(ARGS, 2, "90")); s0 = parse(Int, get(ARGS, 3, "1"))
f = inst.form
capform(cap) = SS.AggregateODRouteJointRoutingAssignmentFormulation(; route_regularization_weight = f.route_regularization_weight,
    walk_cost_weight = f.walk_cost_weight, repositioning_time = f.repositioning_time, max_wait_time = f.max_wait_time,
    detour_factor = f.detour_factor, max_stops = f.max_stops, demand_weighting = f.demand_weighting, vehicle_capacity = cap)
form = capform(4)
br = SS.build_model(inst.problem, form, SS.CGSolver(pricing = SS.CGPricingConfig()))
m = br.model; set_silent(m); optimize!(m)
println("seed LP ", objective_value(m))
duals = SS.extract_joint_routing_assignment_duals(m)
function run(mode, cap, s; lim = limit, prof = false)
    m[:joint_routing_assignment_pricing_mode] = mode
    m[:joint_routing_assignment_vehicle_capacity] = cap
    delete!(JuMP.object_dictionary(m), :joint_routing_assignment_capacitated_critical)
    built = SS._pricing_build_scenario_context(form, br.mapping, s, m, duals)
    built === nothing && return nothing
    ctx = built[1]
    t = @elapsed (labels, ex, st) = prof ? (@profile SS._run_label_setting(ctx; time_limit = lim, reduced_cost_tol = 1e-6)) :
        SS._run_label_setting(ctx; time_limit = lim, reduced_cost_tol = 1e-6)
    best = ctx isa SS.JointRoutingAssignmentCapacitatedSearchContext ? st.best_capacitated_rc :
        minimum((l.reduced_cost for l in labels); init = Inf)
    @printf("%-18s s=%d sec %.1f exhausted %s labels %d (%.0f/s) live %d best %.2f passes %s\n", string(mode, "/", cap), s, t, ex,
            st.labels_generated, st.labels_generated / t, st.max_live_labels, best, get(st, :dssr_passes, 1))
    flush(stdout)
end
run(:exact, nothing, s0; lim = 5.0)  # compile
run(:exact_capacitated, 4, s0; lim = 5.0)
for s in 1:inst.n_scenarios
    run(:exact, nothing, s)
    run(:exact_capacitated, 4, s)
end
Profile.init(n = 10^7, delay = 0.005)
run(:exact_capacitated, 4, s0; lim = 60.0, prof = true)
Profile.print(format = :flat, sortedby = :count, mincount = 200, maxdepth = 200, noisefloor = 2.0)
