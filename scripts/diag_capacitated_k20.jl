# Where does :exact_capacitated spend its time? Fixed 913,533 set, capacity-4 model, seed duals
# (= what CG iteration 1 faces). Per scenario: :exact vs cap4 search stats (60 s limit), and
# the branching of cap4 measured on random walks from the seed labels.
const ROOT = "/home/yongzr/2025-09-JacqWang-Microtransit"
include(joinpath(ROOT, "ExplorationCG.jl", "src", "ExplorationCG.jl")); using .ExplorationCG
const E = ExplorationCG; const SS = E.SS
using JuMP, JSON, Printf, Random, Statistics
out = ARGS[1]
inst = E.load_instance(joinpath(ROOT, "experiments/2026-11-13_exploration_cg/config/instance_k20_10min.toml"), ROOT)
f = inst.form
form = SS.AggregateODRouteJointRoutingAssignmentFormulation(; route_regularization_weight = f.route_regularization_weight,
    walk_cost_weight = f.walk_cost_weight, repositioning_time = f.repositioning_time, max_wait_time = f.max_wait_time,
    detour_factor = f.detour_factor, max_stops = f.max_stops, demand_weighting = f.demand_weighting, vehicle_capacity = 4)
BEST = [14, 16, 22, 36, 40, 44, 54, 100, 117, 121, 133, 135, 154, 176, 190, 202, 203, 204, 207, 209]
d = E.fixed_data(inst.data, sort([inst.data.station_id_to_array_idx[i] for i in BEST]))
prob = SS.StationSelectionProblem(d, 20; max_walking_distance = inst.maxwalk, door_to_door_ratio = inst.d2d)
br = SS.build_model(prob, form, SS.CGSolver(pricing = SS.CGPricingConfig()))
m = br.model; set_silent(m); optimize!(m)
println("seed LP ", objective_value(m)); flush(stdout)
duals = SS.extract_joint_routing_assignment_duals(m)
S = length(d.scenarios)
rows = Any[]; lk = ReentrantLock()
Threads.@threads for s in 1:S
    r = Dict{String,Any}("scenario" => s)
    for (name, mode, cap) in (("exact", :exact, nothing), ("cap1000", :exact_capacitated, 1000), ("cap4", :exact_capacitated, 4))
        mm = Dict(:mode => mode)
        # per-thread copy of the mode switch: build contexts directly instead of mutating m
        local ctx
        lock(lk) do
            m[:joint_routing_assignment_pricing_mode] = mode
            m[:joint_routing_assignment_vehicle_capacity] = cap
            built = SS._pricing_build_scenario_context(form, br.mapping, s, m, duals)
            ctx = built === nothing ? nothing : built[1]
        end
        ctx === nothing && continue
        t = @elapsed labels, ex, st = SS._run_label_setting(ctx; time_limit = 60.0, reduced_cost_tol = 1e-6, profile = true)
        r[name] = Dict("sec" => t, "exhausted" => ex, "generated" => st.labels_generated,
                       "rejected" => st.labels_rejected_by_dominance, "removed" => st.labels_removed_by_dominance,
                       "max_live" => st.max_live_labels, "t_cand" => st.t_candidates_sec, "t_ext" => st.t_extension_sec,
                       "t_dom" => st.t_dominance_sec, "t_queue" => st.t_queue_sec,
                       "improving" => count(l -> l.reduced_cost < -1e-6, labels))
        if name == "cap4"
            pd = ctx.pricing_data
            rng = MersenneTwister(s); nn = Int[]; na = Int[]; nr = Int[]; nfree = 0; ntot = 0
            init = SS.initial_joint_routing_assignment_capacitated_pricing_labels(pd)
            for w in 1:300
                isempty(init) && break
                lab = init[rand(rng, 1:length(init))]
                for step in 1:10
                    nodes = SS._capacitated_candidate_next_nodes(lab, pd.base)
                    for n in nodes
                        tt = SS._joint_routing_assignment_travel(pd.base, lab.current, n)
                        ntot += 1; nfree += SS._capacitated_cannot_bind(lab, Dict(c => a + tt for (c, a) in lab.station_age), pd)
                    end
                    acts = SS._capacitated_candidate_actions(lab, pd)
                    isempty(acts) && break
                    push!(nn, length(nodes)); push!(na, length(acts))
                    a = acts[rand(rng, 1:length(acts))]
                    lab = SS._extend_joint_routing_assignment_capacitated_label(lab, a, pd)
                end
            end
            r["walk"] = Dict("mean_next_nodes" => isempty(nn) ? 0 : mean(nn), "mean_actions" => isempty(na) ? 0 : mean(na),
                             "max_actions" => isempty(na) ? 0 : maximum(na), "free_frac" => ntot == 0 ? 0 : nfree / ntot, "riders" => length(pd.max_reward),
                             "loads" => sort(collect(values(pd.load))))
        end
    end
    lock(lk) do; push!(rows, r); end
end
sort!(rows, by = r -> r["scenario"])
for r in rows
    haskey(r, "cap4") || continue
    e, c, w, k = r["exact"], r["cap4"], r["walk"], r["cap1000"]
    @printf("s%2d free %.2f | cap1000 %5.1fs ex=%d gen %9d | ", r["scenario"], w["free_frac"], k["sec"], k["exhausted"], k["generated"])
    @printf("s%2d riders %2d | exact %5.1fs ex=%d gen %8d | cap4 %5.1fs ex=%d gen %9d rej %9d live %8d cand %.0f ext %.0f dom %.0f | next %.1f actions %.1f (max %d)\n",
        r["scenario"], w["riders"], e["sec"], e["exhausted"], e["generated"], c["sec"], c["exhausted"], c["generated"], c["rejected"],
        c["max_live"], c["t_cand"], c["t_ext"], c["t_dom"], w["mean_next_nodes"], w["mean_actions"], w["max_actions"])
end
open(joinpath(out, "diag.json"), "w") do io; JSON.print(io, rows); end
println("END")
