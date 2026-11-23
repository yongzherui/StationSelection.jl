# Speed / scale of :exact_capacitated on the k20 instance (60 ten-minute scenarios).
#   A  fixed-set CG on the 913,533 set: :exact (uncapacitated) vs capacity 4
#   B  one free-station pricing round at the same master duals, per scenario:
#      :exact vs :exact_capacitated at capacity 1000 (never binds) and 4
#   C  free-station CG at capacity 4 with the exact capacitated pricer, 2 h budget
# usage: julia --project=env_cap bench_exact_capacitated_k20.jl <A|B|C> <out_dir>
const ROOT = "/home/yongzr/2025-09-JacqWang-Microtransit"
include(joinpath(ROOT, "ExplorationCG.jl", "src", "ExplorationCG.jl")); using .ExplorationCG
const E = ExplorationCG; const SS = E.SS
using JuMP, JSON, Printf, Dates
stage, out = ARGS[1], ARGS[2]; mkpath(out)
inst = E.load_instance(joinpath(ROOT, "experiments/2026-11-13_exploration_cg/config/instance_k20_10min.toml"), ROOT)
f = inst.form
capform(cap) = SS.AggregateODRouteJointRoutingAssignmentFormulation(; route_regularization_weight = f.route_regularization_weight,
    walk_cost_weight = f.walk_cost_weight, repositioning_time = f.repositioning_time, max_wait_time = f.max_wait_time,
    detour_factor = f.detour_factor, max_stops = f.max_stops, demand_weighting = f.demand_weighting, vehicle_capacity = cap)
log(x) = (println("[", Dates.format(now(), "HH:MM:SS"), "] ", x); flush(stdout))
const BEST = [14, 16, 22, 36, 40, 44, 54, 100, 117, 121, 133, 135, 154, 176, 190, 202, 203, 204, 207, 209]

if stage == "A"
    set_idx = sort([inst.data.station_id_to_array_idx[i] for i in BEST])
    rows = Any[]
    which = split(get(ENV, "BENCH_A_PRICERS", "exact,cap4"), ",")
    for (name, form) in filter(x -> x[1] in which, (("exact", f), ("cap4", capform(4))))
        t0 = time()
        cb(row) = log("A $name it " * JSON.json(Dict("iteration" => row.iteration, "t" => round(time() - t0),
                        "lp" => row.master_objective, "accepted" => row.columns_accepted)))
        d = E.fixed_data(inst.data, set_idx)
        prob = SS.StationSelectionProblem(d, 20; max_walking_distance = inst.maxwalk, door_to_door_ratio = inst.d2d)
        mp = SS.create_aggregate_od_route_map(prob, form, d)
        ngroups = sum(count(>(0), mp.Q_s[s]) for s in keys(mp.Q_s))
        solver = SS.CGSolver(config = SS.SolverOptions(silent = true, mip_gap = 1e-4, time_limit_sec = 600.0, threads = 16),
                             pricing = SS.CGPricingConfig(), pricing_time_limit_sec = 1800.0,
                             certifying_pricing_time_limit_sec = 1800.0, total_time_limit_sec = 3600.0,
                             parallel_scenario_pricing = true, recover_integer_solution = true, iteration_callback = cb)
        t = @elapsed res = SS.run_opt(prob, form, solver)
        md = res.metadata
        r = Dict("pricer" => name, "groups" => ngroups, "sec" => t, "converged" => get(md, "cg_converged", false),
                 "stop" => string(get(md, "cg_stop_reason", "")), "iterations" => get(md, "cg_iterations", 0),
                 "lp" => get(md, "cg_lp_objective_value", NaN), "integer" => res.objective_value,
                 "routes" => length(res.model[:joint_routing_assignment_columns]))
        log("A " * JSON.json(r)); push!(rows, r)
    end
    open(joinpath(out, "A_fixed_set.json"), "w") do io; JSON.print(io, rows, 2); end

elseif stage == "B"
    form = capform(4)
    solver = SS.CGSolver(pricing = SS.CGPricingConfig())
    br = SS.build_model(inst.problem, form, solver)
    m = br.model; set_silent(m); optimize!(m)
    log("seed master LP $(objective_value(m)) (capacity 4 instance: groups > 4 requests dropped)")
    duals = SS.extract_joint_routing_assignment_duals(m)
    S = length(inst.data.scenarios)
    rows = Vector{Any}(undef, 0); lk = ReentrantLock()
    for (name, mode, cap) in filter(x -> x[1] in split(get(ENV, "BENCH_B_PRICERS", "exact,cap1000,cap4"), ","),
                                   (("exact", :exact, nothing), ("cap1000", :exact_capacitated, 1000), ("cap4", :exact_capacitated, 4)))
        m[:joint_routing_assignment_pricing_mode] = mode
        m[:joint_routing_assignment_vehicle_capacity] = cap
        delete!(JuMP.object_dictionary(m), :joint_routing_assignment_capacitated_critical)
        t0 = time()
        Threads.@threads for s in 1:S
            built = SS._pricing_build_scenario_context(form, br.mapping, s, m, duals)
            built === nothing && continue
            ctx = built[1]
            t = @elapsed labels, exhausted, st = SS._run_label_setting(ctx; time_limit = parse(Float64, get(ENV, "BENCH_B_LIMIT", "300")), reduced_cost_tol = 1e-6)
            imp = count(l -> l.reduced_cost < -1e-6, labels)
            capctx = ctx isa SS.JointRoutingAssignmentCapacitatedSearchContext
            if capctx
                imp = count(l -> SS._capacitated_route_result(ctx, l.route)[3] < -1e-6, labels)
            end
            r = Dict("pricer" => name, "scenario" => s, "sec" => t, "exhausted" => exhausted,
                     "labels" => st.labels_generated, "improving" => imp,
                     "passes" => get(st, :dssr_passes, 1), "n_critical" => get(st, :n_critical, 0),
                     "min_rc" => capctx ? st.best_capacitated_rc : (isempty(labels) ? nothing : minimum(l.reduced_cost for l in labels)))
            lock(lk) do; push!(rows, r); end
        end
        sub = filter(r -> r["pricer"] == name, rows)
        log(@sprintf("B %-8s wall %.0f s | scenario sec: sum %.0f, max %.0f | exhausted %d/%d | labels %.3g | improving routes %d | passes max %d | critical sum %d | scen w/ improving %d",
            name, time() - t0, sum(r["sec"] for r in sub), maximum(r["sec"] for r in sub),
            count(r -> r["exhausted"], sub), length(sub), sum(r["labels"] for r in sub), sum(r["improving"] for r in sub),
            maximum(r["passes"] for r in sub), sum(r["n_critical"] for r in sub), count(r -> r["improving"] > 0, sub)))
    end
    open(joinpath(out, "B_pricing_round.json"), "w") do io; JSON.print(io, rows); end

elseif stage == "C"
    form = capform(4)
    t0 = time()
    rows = Any[]
    cb(row) = (r = Dict("iteration" => row.iteration, "t" => time() - t0, "lp" => row.master_objective,
                        "accepted" => row.columns_accepted); push!(rows, r); log("C " * JSON.json(r)))
    solver = SS.CGSolver(config = SS.SolverOptions(silent = true, mip_gap = 0.01, time_limit_sec = 1800.0, threads = 16),
                         pricing = SS.CGPricingConfig(), max_iterations = 100_000,
                         pricing_time_limit_sec = 900.0, certifying_pricing_time_limit_sec = 1800.0,
                         total_time_limit_sec = 7200.0, parallel_scenario_pricing = true,
                         recover_integer_solution = true, iteration_callback = cb)
    res = SS.run_opt(inst.problem, form, solver)
    md = res.metadata
    s = Dict("converged" => get(md, "cg_converged", false), "stop" => string(get(md, "cg_stop_reason", "")),
             "iterations" => get(md, "cg_iterations", 0), "lp" => get(md, "cg_lp_objective_value", NaN),
             "integer" => res.objective_value, "sec" => time() - t0,
             "set" => E.ids_of(inst, findall(>(0.5), [value(res.model[:y][i]) for i in 1:inst.data.n_stations])))
    log("C DONE " * JSON.json(s))
    open(joinpath(out, "C_cg.json"), "w") do io; JSON.print(io, Dict("summary" => s, "iterations" => rows), 2); end
end
log("END $stage")
