# Instrumented re-implementation of CGSolver's LP loop for the Study 6 tail-off case.
#
# Replicates cg_solver.jl's loop exactly, but additionally records what
# `add_columns!` RETURNS (the number actually added, which the real loop discards)
# and the signature of each priced column, to test whether the same already-present
# column is re-found every iteration.

using StationSelection
using JuMP
using CSV
using DataFrames
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const SS = StationSelection

n_stations = parse(Int, ARGS[1])
seed       = parse(Int, ARGS[2])
n_iters    = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 12
n_pairs, n_scenarios, max_stops = 16, 3, 4

problem, k, meta = benchmark_problem(@__DIR__, "STUDY6", n_stations, n_pairs, n_scenarios, seed)
formulation = AggregateODRouteBaseFormulation(; BENCHMARK_BASELINE..., max_stops=max_stops)
solver = CGSolver(
    config=SolverOptions(silent=true, time_limit_sec=300.0, threads=1),
    max_iterations=n_iters, reduced_cost_tol=1e-6,
    pricing_time_limit_sec=900.0, recover_integer_solution=false,
)

build_result = SS.build_model(problem, formulation, solver)
m = build_result.model
mapping = build_result.mapping
SS._apply_solver_config!(m, solver.config)

theta_count(model) = length(model[:aggregate_od_route_base_theta])

println("=== instrumented n=$n_stations seed=$seed ===")
println("iter |    master_obj |  priced | ADDED | theta_pool | pair_sigs of priced cols")
for it in 1:n_iters
    optimize!(m)
    st = termination_status(m)
    if st != MOI.OPTIMAL
        println("iteration $it: master status $st -- stopping"); break
    end
    obj = objective_value(m)
    duals = SS.extract_duals(build_result, mapping, m)
    cols = SS.price_columns(build_result, mapping, m, duals, solver)
    npriced = isnothing(cols) ? 0 : length(cols)
    if npriced == 0
        println("iteration $it: pricing returned nothing -- converged"); break
    end
    before = theta_count(m)
    added = SS.add_columns!(build_result, mapping, m, cols)   # <-- the discarded return value
    after = theta_count(m)
    sigs = join([string(c.od_pairs, "@s", Int(c.metadata["scenario"])) for c in cols][1:min(2, npriced)], " ")
    println(lpad(it, 4), " | ", lpad(round(obj; digits=4), 13), " | ", lpad(npriced, 7),
            " | ", lpad(added, 5), " | ", lpad(string(before, "->", after), 10), " | ", sigs)
end
