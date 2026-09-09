"""Is the Benders answer the right answer for the right MODEL? Cross-check against CGSolver.

`benders_brute_force_certificate.jl` validates the Benders ALGORITHM: it enumerates all
C(n,k) station sets, evaluates the second stage exactly, and audits every cut pointwise. But
it shares the model DEFINITION with the thing under test -- the same `AggregateODRouteMap`,
the same enumerated column pool, the same cost weights. If the pool is incomplete or the
enumeration is wrong, brute force and Benders agree and are wrong together.

`CGSolver` is the independent path, because it obtains its columns by a completely different
mechanism: label-setting pricing over the route universe
(`label_setting/joint_routing_assignment/exact/`), never enumeration. Same formulation, same
`max_stops`, same cost weights, different column source and a different master.

# The four numbers and what their relationships prove

    cg_lp        CGSolver's LP master  -- EVERYTHING relaxed, y included
    benders      the mixed optimum     -- y binary, theta/x_walk continuous
    mixed_mono   the same, monolithic over the ENUMERATED pool
    cg_ip        CGSolver's recovered integer solution -- all binary, over the CG pool
    direct_mip   all binary, over the ENUMERATED pool

Ordering that must hold if everything is right:

    cg_lp  <=  benders == mixed_mono  <=  direct_mip
    cg_lp  <=  cg_ip

`cg_lp <= benders` is the load-bearing one for the POOL question. `cg_lp` is a lower bound
on the true mixed optimum whatever the pool, because pricing searched the full route
universe and y is relaxed on top. So:

- if `cg_lp == direct_mip`, the enumerated pool is complete for the optimum AND the
  y-relaxation is tight there -- every number collapses to one, and the agreement is between
  two independent column sources. That is the strong outcome.
- if `cg_lp < benders`, it is AMBIGUOUS on its own: the slack could be the y-relaxation
  (legitimate) or a column the enumerator missed (a real bug). `cg_ip` vs `direct_mip`
  separates them -- both are all-binary optima over their respective pools, so
  `cg_ip < direct_mip` means the enumerated pool is genuinely missing something the pricer
  found, while `cg_ip == direct_mip` points at the relaxation.

# The precondition, and why it is not optional

None of this means anything unless CG actually CONVERGED: `cg_converged == true` with
`cg_optimality_scope == "full_route_universe"`. A budget-stopped CG run has an incomplete
pool by definition, so `cg_lp` is then not a valid lower bound on anything and a mismatch
would prove nothing while looking exactly like one of the two implementations having a bug
-- the most misleading possible outcome. Checked explicitly below before any comparison is
reported as meaningful.

Usage: sbatch benchmarks/diagnostics/run_benders_vs_cg.sh
Env: XC_N XC_P XC_S XC_SEED XC_MAX_STOPS XC_PRICING_LIMIT
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "XC_N", "10"))
const P = parse(Int, get(ENV, "XC_P", "8"))
const S = parse(Int, get(ENV, "XC_S", "1"))
const SEED = parse(Int, get(ENV, "XC_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "XC_MAX_STOPS", "4"))
const PRICING_LIMIT = parse(Float64, get(ENV, "XC_PRICING_LIMIT", "900.0"))
const TOL = 1e-6

problem, k, _meta = benchmark_problem(@__DIR__, "XC", N, P, S, SEED)
# ONE formulation object for every solver. This is the point of the Problem/Formulation/
# Solver split and it is what makes the comparison valid: three solvers, one model.
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)

@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d\n",
        N, P, S, SEED, k, MAX_STOPS)
flush(stdout)

# ------------------------------------------------------------------ CGSolver
println("\n=== CGSolver (:exact pricer, label-setting -- NOT enumeration) ===")
flush(stdout)
cg_solver = benchmark_cg_solver(PRICING_LIMIT; recover_integer_solution=true,
                                pricing=CGPricingConfig(mode=:exact))
cg = run_opt(problem, formulation, cg_solver)
cgmd = cg.metadata
cg_ip = something(cg.objective_value, NaN)
cg_lp = get(cgmd, "cg_lp_objective_value", NaN)
cg_converged = get(cgmd, "cg_converged", false) === true
cg_scope = get(cgmd, "cg_optimality_scope", "<none>")
@printf("status %s | LP %.6f | IP %.6f\n", cg.termination_status, cg_lp, cg_ip)
@printf("converged %s | scope %s | stop %s | iters %s | pool %s\n",
        cg_converged, cg_scope, get(cgmd, "cg_stop_reason", "?"),
        string(get(cgmd, "cg_iterations", "?")),
        string(get(cgmd, "cg_columns_total", get(cgmd, "cg_n_columns", "?"))))
flush(stdout)

# ------------------------------------------------------------------ Benders
println("\n=== BendersSolver (:direct_enumeration oracle) ===")
flush(stdout)
benders_solver = BendersSolver(
    config=SolverOptions(silent=true, time_limit_sec=300.0),
    max_iterations=500,
    subproblem=BendersSubproblemConfig(
        max_stops=MAX_STOPS, max_routes=200_000, enumeration_time_limit_sec=300.0))
bd = run_opt(problem, formulation, benders_solver)
bmd = bd.metadata
benders = something(bd.objective_value, NaN)
@printf("status %s | obj %.6f | iters %d cuts %d | pool %d | scope %s\n",
        bd.termination_status, benders, bmd["benders_iterations"], bmd["benders_cuts_added"],
        bmd["benders_enumerated_columns"], bmd["benders_optimality_scope"])
flush(stdout)

# ------------------------------------------- monolithic references (enumerated pool)
data = problem.data
mapping = create_aggregate_od_route_map(problem, formulation, data)
columns = StationSelection.enumerate_joint_routing_assignment_columns(
    problem, formulation, data; max_routes=200_000, time_limit_sec=300.0)

function solve_reference(; mixed::Bool)
    build = StationSelection._build_joint_routing_assignment_model(
        data, mapping, problem.k, formulation;
        relax_integrality=mixed, initial_columns=columns)
    m = build.model
    set_silent(m)
    set_time_limit_sec(m, 900.0)
    mixed && JuMP.set_binary.(m[:y])
    optimize!(m)
    return (status=termination_status(m), objective=objective_value(m))
end

println("\n=== monolithic over the ENUMERATED pool ===")
mixed_mono = solve_reference(mixed=true)
direct_mip = solve_reference(mixed=false)
@printf("pool %d columns | mixed_mono %s %.6f | direct_mip %s %.6f\n",
        length(columns), mixed_mono.status, mixed_mono.objective,
        direct_mip.status, direct_mip.objective)
flush(stdout)

# ---------------------------------------------------------------- checks
println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
push_check!(name, ok, detail) = push!(checks, (name, ok, detail))

# The precondition. Reported FIRST, because every comparison below is meaningless without
# it and a reader who sees only PASSes further down would be misled.
push_check!("CG converged (precondition)", cg_converged,
    "$(get(cgmd, "cg_stop_reason", "?")) / converged=$cg_converged")
push_check!("CG scope is full universe", cg_scope == "full_route_universe", cg_scope)
push_check!("references are optima",
    string(mixed_mono.status) == "OPTIMAL" && string(direct_mip.status) == "OPTIMAL",
    "mixed $(mixed_mono.status) / direct $(direct_mip.status)")

push_check!("cg_lp <= benders", cg_lp <= benders + TOL,
    @sprintf("%.6f <= %.6f (slack %.3e)", cg_lp, benders, benders - cg_lp))
push_check!("benders == mixed_mono",
    isapprox(benders, mixed_mono.objective; rtol=1e-6, atol=1e-6),
    @sprintf("%.6f vs %.6f", benders, mixed_mono.objective))
push_check!("benders <= direct_mip", benders <= direct_mip.objective + TOL,
    @sprintf("%.6f <= %.6f", benders, direct_mip.objective))
push_check!("cg_lp <= cg_ip", cg_lp <= cg_ip + TOL,
    @sprintf("%.6f <= %.6f", cg_lp, cg_ip))
# THE pool-completeness check: two independent column sources, both solved all-binary.
# A pricer-found column the enumerator missed shows up here and nowhere else.
push_check!("cg_ip == direct_mip (pool agreement)",
    isapprox(cg_ip, direct_mip.objective; rtol=1e-6, atol=1e-6),
    @sprintf("%.6f vs %.6f (diff %.3e)", cg_ip, direct_mip.objective,
             cg_ip - direct_mip.objective))

for (name, ok, detail) in checks
    @printf("%-38s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end

# Interpretation, printed rather than left to the reader, since the ambiguous case is the
# one that matters and it is easy to misread as a clean pass.
println()
if isapprox(cg_lp, direct_mip.objective; rtol=1e-6, atol=1e-6)
    println("VERDICT: cg_lp == direct_mip -- all four numbers collapse to one. The enumerated")
    println("         pool is complete for the optimum AND the y-relaxation is tight here, and")
    println("         the agreement is between two independent column sources (pricing vs")
    println("         enumeration). This is the strong outcome.")
elseif isapprox(cg_ip, direct_mip.objective; rtol=1e-6, atol=1e-6)
    @printf("VERDICT: cg_lp < benders by %.3e, but cg_ip == direct_mip, so the two pools agree\n",
            benders - cg_lp)
    println("         on the integer optimum -- the slack is the y-relaxation, not a missing")
    println("         column. Benders is solving the right model.")
else
    @printf("VERDICT: cg_ip (%.6f) != direct_mip (%.6f). Two all-binary optima over two pools\n",
            cg_ip, direct_mip.objective)
    println("         disagree, which means one pool is missing columns the other has. If cg_ip")
    println("         is LOWER, the enumerator missed a route the pricer found -- a real bug in")
    println("         the model definition Benders inherits, not in the decomposition.")
end

n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)
n_fail == 0 || error("verification failed")
println("ALL CHECKS PASSED")
