# Run one Study 4 exact/warm-start comparison arm and record column provenance.
using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
f = split(strip(ARGS[1]), '\t')
length(f) == 14 || error("expected 14 fields, got $(length(f))")
job_id, cell_id, arm = parse(Int, f[1]), f[2], Symbol(f[3])
n, p, s, seed = parse.(Int, f[4:7])
max_stops, n_threads, K, guide_routes = parse.(Int, f[8:11])
pricing_limit, certifying_limit, total_limit = parse.(Float64, f[12:14])
arm in (:exact, :station_simple, :cluster_guide) || error("unknown arm $arm")
Threads.nthreads() == n_threads || error(
    "job requests $n_threads Julia threads, process has $(Threads.nthreads())",
)

problem, k, instance_meta = benchmark_problem(@__DIR__, "STUDY4", n, p, s, seed)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops,
)
pricing = arm === :exact ? CGPricingConfig(mode=:exact) :
    arm === :station_simple ? CGPricingConfig(mode=:exact, warm_start_mode=:station_simple) :
    CGPricingConfig(mode=:exact, warm_start_mode=:cluster_guide,
                    relaxed_cluster_count=K, relaxed_cluster_guide_routes=guide_routes)
solver = benchmark_cg_solver(
    pricing_limit; recover_integer_solution=true,
    certifying_pricing_time_limit_sec=certifying_limit,
    total_time_limit_sec=total_limit, threads=1,
    parallel_scenario_pricing=true, pricing=pricing,
)
result = run_opt(problem, formulation, solver)
metrics = benchmark_cg_metrics(result, :joint_routing_assignment_columns)
outdir = benchmark_output_dir(@__DIR__, "STUDY4", "study4_warm_start_pricers")

pool = collect(values(result.model[:joint_routing_assignment_columns]))
priced = filter(c -> haskey(c.metadata, "pricing_mode"), pool)
source_count(mode) = count(c -> get(c.metadata, "pricing_mode", :seed) === mode, pool)
pricing_stats = result.metadata["cg_pricing_stats"]
thread_ids = sort!(unique(Int(r.thread_id) for r in pricing_stats if haskey(r, :thread_id)))
guide_stats = result.metadata["cg_relaxed_cluster_guide_stats"]
guide_thread_ids = sort!(unique(Int(r.thread_id) for r in guide_stats if haskey(r, :thread_id)))
summary = DataFrame((
    job_id=[job_id], cell_id=[cell_id], arm=[String(arm)], n_stations=[n], n_pairs=[p], n_scenarios=[s],
    seed=[seed], k=[k], n_pairs_actual=[sum(instance_meta.pairs_per_scenario)],
    max_stops=[max_stops], relaxed_cluster_count=[arm === :cluster_guide ? K : missing],
    guide_routes=[arm === :cluster_guide ? guide_routes : missing],
    julia_threads=[Threads.nthreads()], pricing_thread_ids=[join(thread_ids, ";")],
    parallel_threads_used=[length(thread_ids)], guide_thread_ids=[join(guide_thread_ids, ";")],
    guide_parallel_threads_used=[length(guide_thread_ids)],
    termination_status=[string(result.termination_status)],
    optimality_scope=[result.metadata["cg_optimality_scope"]], objective_value=[result.objective_value],
    runtime_sec=[metrics.runtime_sec], cg_iterations=[metrics.cg_iterations],
    warm_start_iterations=[result.metadata["cg_warm_start_iterations"]],
    warm_start_sec=[result.metadata["cg_warm_start_sec"]], n_columns=[length(pool)],
    cluster_guide_columns=[source_count(:cluster_guide)],
    station_simple_columns=[source_count(:station_simple)], exact_columns=[source_count(:exact)],
))
CSV.write(joinpath(outdir, "job_$(lpad(job_id, 2, '0'))_$(arm).csv"), summary)

columns = DataFrame((
    job_id=fill(job_id, length(priced)), cell_id=fill(cell_id, length(priced)),
    arm=fill(String(arm), length(priced)),
    column_id=[c.id for c in priced], pricing_mode=[String(c.metadata["pricing_mode"]) for c in priced],
    scenario=[Int(c.metadata["scenario"]) for c in priced],
    route_length=[length(c.route) for c in priced],
    unique_stations=[length(unique(c.route)) for c in priced],
    revisits=[length(c.route) - length(unique(c.route)) for c in priced],
    assignments=[length(c.assignments) for c in priced], tau=[c.tau for c in priced],
    reduced_cost=[get(c.metadata, "reduced_cost", missing) for c in priced],
))
columns_dir = joinpath(outdir, "columns")
mkpath(columns_dir)
CSV.write(joinpath(columns_dir, "job_$(lpad(job_id, 2, '0'))_$(arm)_columns.csv"), columns)
println(summary)
