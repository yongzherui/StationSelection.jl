# Free-station (or fixed-set) CG with :exact (no capacity) vs :exact_capacitated at capacities given.
# usage: julia --project=env_cap bench_cap_cg.jl <instance.toml> <out_dir> [caps=none,1000,4] [fixed=11,100,...]
#   env: BENCH_TOTAL (s, default 3600), BENCH_PRICING (s per round, default 1800), BENCH_GAP (default 1e-4)
const ROOT = "/home/yongzr/2025-09-JacqWang-Microtransit"
include(joinpath(ROOT, "ExplorationCG.jl", "src", "ExplorationCG.jl")); using .ExplorationCG
const E = ExplorationCG; const SS = E.SS
using JuMP, JSON, Printf, Dates
cfg, out = ARGS[1], ARGS[2]; mkpath(out)
caps = split(get(ARGS, 3, "none,1000,4"), ",")
fixed = length(ARGS) >= 4 ? parse.(Int, split(ARGS[4], ",")) : nothing
inst = E.load_instance(cfg, ROOT)
f = inst.form
capform(cap) = SS.AggregateODRouteJointRoutingAssignmentFormulation(; route_regularization_weight = f.route_regularization_weight,
    walk_cost_weight = f.walk_cost_weight, repositioning_time = f.repositioning_time, max_wait_time = f.max_wait_time,
    detour_factor = f.detour_factor, max_stops = f.max_stops, demand_weighting = f.demand_weighting, vehicle_capacity = cap)
log(x) = (println("[", Dates.format(now(), "HH:MM:SS"), "] ", x); flush(stdout))
total = parse(Float64, get(ENV, "BENCH_TOTAL", "3600")); pricing = parse(Float64, get(ENV, "BENCH_PRICING", "1800"))
gap = parse(Float64, get(ENV, "BENCH_GAP", "1e-4"))
log("instance $(cfg): $(inst.n_scenarios) scenarios, k $(inst.k), threads $(Threads.nthreads()), fixed $(fixed)")
colload(m, col) = Int(get(col.metadata, "max_load", 0))
rows = Any[]
for c in caps
    cap = c == "none" ? nothing : parse(Int, c)
    form = capform(cap)
    prob = if fixed === nothing
        inst.problem
    else
        idx = sort([inst.data.station_id_to_array_idx[i] for i in fixed])
        SS.StationSelectionProblem(E.fixed_data(inst.data, idx), length(idx); max_walking_distance = inst.maxwalk,
                                   door_to_door_ratio = inst.d2d)
    end
    t0 = time()
    its = Any[]
    cb(row) = (r = Dict("iteration" => row.iteration, "t" => round(time() - t0; digits = 1), "lp" => row.master_objective,
                        "accepted" => row.columns_accepted); push!(its, r); log("$c it " * JSON.json(r)))
    solver = SS.CGSolver(config = SS.SolverOptions(silent = true, mip_gap = gap, time_limit_sec = 1800.0, threads = Threads.nthreads()),
                         pricing = SS.CGPricingConfig(), max_iterations = 100_000,
                         pricing_time_limit_sec = pricing, certifying_pricing_time_limit_sec = pricing,
                         total_time_limit_sec = total, parallel_scenario_pricing = true,
                         recover_integer_solution = true, iteration_callback = cb)
    res = SS.run_opt(prob, form, solver)
    md = res.metadata
    st = get(JuMP.object_dictionary(res.model), :label_setting_pricing_stats, Any[])
    search = sum((s.search_sec for s in st); init = 0.0)
    passes = [get(s, :dssr_passes, 1) for s in st]
    ncrit = [get(s, :n_critical, 0) for s in st]
    crit = get(JuMP.object_dictionary(res.model), :joint_routing_assignment_capacitated_critical, nothing)
    y = fixed === nothing ? E.ids_of(inst, findall(>(0.5), [value(res.model[:y][i]) for i in 1:inst.data.n_stations])) : fixed
    r = Dict("cap" => c, "sec" => time() - t0, "converged" => get(md, "cg_converged", false),
             "stop" => string(get(md, "cg_stop_reason", "")), "iterations" => get(md, "cg_iterations", 0),
             "lp" => get(md, "cg_lp_objective_value", NaN), "integer" => res.objective_value,
             "routes" => length(res.model[:joint_routing_assignment_columns]), "set" => y,
             "search_sec_sum" => search, "search_calls" => length(st),
             "search_sec_max" => maximum((s.search_sec for s in st); init = 0.0),
             "dssr_passes_max" => maximum(passes; init = 0), "dssr_passes_sum" => sum(passes; init = 0),
             "n_critical_final" => crit === nothing ? nothing : Dict(string(k) => count(v) for (k, v) in crit),
             "n_groups" => Dict(string(s) => length(v) for (s, v) in (crit === nothing ? Dict() : crit)),
             "max_load" => maximum((colload(res.model, col) for col in values(res.model[:joint_routing_assignment_columns])); init = 0),
             "iters" => its)
    log("RESULT " * JSON.json(filter(kv -> kv[1] != "iters", r)))
    push!(rows, r)
    open(joinpath(out, "result.json"), "w") do io; JSON.print(io, rows, 2); end
end
log("END")
