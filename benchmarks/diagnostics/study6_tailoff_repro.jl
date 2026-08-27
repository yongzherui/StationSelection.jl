# Reproduce Study 6's iters=1000 tail-off and dump the per-iteration CG log.
#
# Hypothesis: pricing keeps returning columns that `add_columns!` de-duplicates away
# (`:skipped`), so the master never changes, the duals never change, and the loop spins
# to `max_iterations` because `cg_solver.jl` breaks only on an *empty* pricing result and
# never inspects how many columns were actually added.
#
# Usage: julia --project=<root> study6_tailoff_repro.jl <n_stations> <seed> [max_iterations]

using StationSelection
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

n_stations = parse(Int, ARGS[1])
seed       = parse(Int, ARGS[2])
max_iters  = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 1_000
n_pairs, n_scenarios, max_stops = 16, 3, 4

problem, k, meta = benchmark_problem(@__DIR__, "STUDY6", n_stations, n_pairs, n_scenarios, seed)
formulation = AggregateODRouteBaseFormulation(; BENCHMARK_BASELINE..., max_stops=max_stops)
solver = CGSolver(
    config=SolverOptions(silent=true, time_limit_sec=300.0, threads=1),
    max_iterations=max_iters, reduced_cost_tol=1e-6,
    pricing_time_limit_sec=900.0, recover_integer_solution=false,
)

println("=== repro n=$n_stations seed=$seed max_iterations=$max_iters ===")
t0 = time()
result = run_opt(problem, formulation, solver)
wall = time() - t0

md = result.metadata
println("wall=$(round(wall; digits=1))s  iterations=$(md["cg_iterations"])  ",
        "converged=$(md["cg_converged"])  exhausted=$(md["cg_pricing_exhausted"])")

log = DataFrame(md["cg_iteration_log"])
outdir = joinpath(@__DIR__, "out"); mkpath(outdir)
CSV.write(joinpath(outdir, "tailoff_n$(n_stations)_seed$(seed).csv"), log)

# The diagnostic: does the master objective ever move after the first few iterations,
# and does pricing keep claiming columns while the pool stays flat?
println("\niter  master_objective        columns_added  cumulative")
for r in eachrow(log[1:min(6, nrow(log)), :])
    println(lpad(r.iteration, 4), "  ", rpad(r.master_objective, 22), " ",
            lpad(r.columns_added, 8), " ", lpad(r.cumulative_columns_added, 12))
end
if nrow(log) > 12
    println("  ...")
    for r in eachrow(log[end-5:end, :])
        println(lpad(r.iteration, 4), "  ", rpad(r.master_objective, 22), " ",
                lpad(r.columns_added, 8), " ", lpad(r.cumulative_columns_added, 12))
    end
end

objs = collect(skipmissing(log.master_objective))
if length(objs) > 2
    tail = objs[2:end]
    println("\nmaster objective over iterations 2..end: ",
            "min=", minimum(tail), " max=", maximum(tail),
            "  spread=", maximum(tail) - minimum(tail))
    println("distinct objective values after iteration 1: ", length(unique(tail)))
end
println("distinct columns_added values: ", sort(unique(log.columns_added)))
