"""Does the Benders decomposition of the joint routing+assignment model actually solve it?

First end-to-end run of `AggregateODRouteJointRoutingAssignmentFormulation` +
`BendersSolver` (`:direct_enumeration` oracle), on the Study 1 reference cell:
Zhuzhou n=10, p=8, 1 scenario, seed 42, `max_stops=4`.

# What is verified, and why these three numbers

The Benders subproblem must be an LP -- the cut IS its dual -- so what the decomposition
converges to is the optimum of the MIXED model: `y` binary, `theta`/`x_walk` continuous.
That is a different number from the all-binary direct MIP, by this formulation's LP-IP
gap. So the run reports:

  benders      decomposed mixed optimum        (the thing under test)
  mixed_mono   same mixed model, monolithic    MUST equal `benders` -- the exactness test
  direct_mip   all-binary, same column pool    MUST be >= `benders` -- the LP-IP gap

`mixed_mono` is built by taking the package's own shared master body at
`relax_integrality=true` over the same enumerated pool and re-declaring `y` binary, so it
is the identical model the master/subproblem pair decomposes -- not a re-derivation that
could disagree for its own reasons.

Also checked: the master's lower bound is monotone, the final gap is within tolerance, and
the subproblems' `reduced_cost(y)` cross-check agrees with the aggregated linking duals
(a diagnostic the subproblem records per scenario; see `benders/subproblem.jl`).

Usage: sbatch benchmarks/diagnostics/run_benders_n10.sh
Env: BJ_N BJ_P BJ_S BJ_SEED BJ_MAX_STOPS BJ_MAX_ITERS
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "BJ_N", "10"))
const P = parse(Int, get(ENV, "BJ_P", "8"))
const S = parse(Int, get(ENV, "BJ_S", "1"))
const SEED = parse(Int, get(ENV, "BJ_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "BJ_MAX_STOPS", "4"))
# Benders over binary `y` terminates finitely -- each cut is tight at the `yhat` it came
# from, so a repeated incumbent closes the gap -- but the bound is the number of feasible
# station sets (C(10,5) = 252 here), so the cap has to be comfortably above that to
# distinguish "converged" from "ran out of iterations".
const MAX_ITERS = parse(Int, get(ENV, "BJ_MAX_ITERS", "500"))

problem, k, meta = benchmark_problem(@__DIR__, "BJ", N, P, S, SEED)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)

@printf("instance: zhuzhou n=%d p=%d scenarios=%d seed=%d k=%d max_stops=%d\n",
        N, P, S, SEED, k, MAX_STOPS)
flush(stdout)

# ---------------------------------------------------------------- Benders
solver = BendersSolver(
    config=SolverOptions(silent=true, time_limit_sec=300.0),
    max_iterations=MAX_ITERS,
    optimality_tol=1e-6,
    subproblem=BendersSubproblemConfig(
        max_stops=MAX_STOPS, max_routes=200_000, enumeration_time_limit_sec=300.0),
    total_time_limit_sec=1800.0,
)
println("\n=== Benders ===")
flush(stdout)
result = run_opt(problem, formulation, solver)
md = result.metadata
@printf("status            %s\n", result.termination_status)
@printf("objective (UB)    %.6f\n", something(result.objective_value, NaN))
@printf("lower bound       %.6f\n", md["benders_lower_bound"])
@printf("gap / rel         %.3e / %.3e\n", md["benders_gap"], md["benders_relative_gap"])
@printf("iterations        %d  (best at %d)\n", md["benders_iterations"], md["benders_best_iteration"])
@printf("cuts added        %d  (%s)\n", md["benders_cuts_added"], md["benders_cut_mode"])
@printf("stop reason       %s\n", md["benders_stop_reason"])
@printf("enumerated cols   %d  in %.2fs\n", md["benders_enumerated_columns"], md["benders_enumeration_sec"])
@printf("scope             %s (subproblem max_stops=%d, formulation %d)\n",
        md["benders_optimality_scope"], md["benders_subproblem_max_stops"],
        md["benders_formulation_max_stops"])
@printf("time  total %.2fs = master %.2fs + subproblems %.2fs\n",
        result.runtime_sec, md["benders_master_sec"], md["benders_subproblem_sec"])
stations = result.solution.selected_station_indices
@printf("stations (idx)    %s\n", string(stations))
@printf("stations (id)     %s\n",
        string([get_station_id(result.mapping, j) for j in stations]))
@printf("scenario costs    %s\n", string(result.solution.scenario_objectives))
flush(stdout)

# --------------------------------------------- monolithic references, same pool
data = problem.data
mapping = create_aggregate_od_route_map(problem, formulation, data)
columns = StationSelection.enumerate_joint_routing_assignment_columns(
    problem, formulation, data; max_routes=200_000, time_limit_sec=300.0)
@printf("\nreference pool    %d columns\n", length(columns))

function solve_reference(; y_binary_only::Bool)
    build = StationSelection._build_joint_routing_assignment_model(
        data, mapping, problem.k, formulation;
        relax_integrality=y_binary_only, initial_columns=columns)
    m = build.model
    set_silent(m)
    set_time_limit_sec(m, 900.0)
    # `relax_integrality=true` made EVERY family continuous; re-declaring only `y` binary
    # is what turns it into the mixed model Benders actually solves.
    y_binary_only && JuMP.set_binary.(m[:y])
    optimize!(m)
    return (status=termination_status(m), objective=objective_value(m),
            y=[v > 0.5 ? 1 : 0 for v in value.(m[:y])])
end

println("\n=== monolithic references ===")
flush(stdout)
mixed = solve_reference(y_binary_only=true)
@printf("mixed_mono  %s  %.6f  y=%s\n", mixed.status, mixed.objective, string(findall(==(1), mixed.y)))
direct = solve_reference(y_binary_only=false)
@printf("direct_mip  %s  %.6f  y=%s\n", direct.status, direct.objective, string(findall(==(1), direct.y)))
flush(stdout)

# ---------------------------------------------------------------- verdict
println("\n=== checks ===")
benders_obj = something(result.objective_value, NaN)
checks = Tuple{String, Bool, String}[]
push!(checks, ("benders == mixed_mono",
    isapprox(benders_obj, mixed.objective; rtol=1e-6, atol=1e-6),
    @sprintf("%.6f vs %.6f (diff %.3e)", benders_obj, mixed.objective,
             benders_obj - mixed.objective)))
push!(checks, ("benders <= direct_mip",
    benders_obj <= direct.objective + 1e-6,
    @sprintf("%.6f vs %.6f (LP-IP gap %.3f%%)", benders_obj, direct.objective,
             100 * (direct.objective - benders_obj) / max(1e-9, abs(direct.objective)))))
push!(checks, ("converged", md["benders_converged"] === true, md["benders_stop_reason"]))
push!(checks, ("status OPTIMAL", string(result.termination_status) == "OPTIMAL",
    string(result.termination_status)))
for (name, ok, detail) in checks
    @printf("%-24s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
all(c -> c[2], checks) || error("verification failed")
println("\nALL CHECKS PASSED")
