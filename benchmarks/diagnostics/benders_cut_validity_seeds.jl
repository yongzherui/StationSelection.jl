"""Are any Benders cuts INVALID? Pointwise audit at n=15 (exhaustive) and n=20 (targeted), many seeds.

# What had and had not been checked

Pointwise cut validity -- does each cut underestimate the true second-stage cost `Q_s(y)` at
EVERY master-feasible `y`, not just at the `yhat` it was derived at -- had only been checked
at **n=10** (86 sets, s=1 and s=3, worst violation 6.4e-07).

At n=15 and n=20 the only cut-side evidence was the pair of lower-bound invariants (monotone,
never above the final objective). Those are necessary but weak: an invalid cut that
overestimates `Q` somewhere the master never visits, or that binds only BELOW the optimum,
passes them. Objective agreement with `DirectMIPSolver`/CG/brute force is also not a cut
audit -- it says the loop landed in the right place, not that every cut it used was sound.

This script closes that at both sizes and across seeds.

# Audit domains

`C(15,8) = 6,435` and `C(20,10) = 184,756`, so:

- **n=15: exhaustive.** Every master-feasible set (4,437 of 6,435) evaluated, every cut
  checked at every one. A complete certificate, same strength as the n=10 one.
- **n=20: targeted + random.** Exhaustive is out (184,756 sets x 3 scenario LPs over a
  ~79k-column subproblem each). Instead the audit set is deliberately biased toward where an
  invalid cut would actually do damage:
    * the Benders optimum's own `y`
    * the `DirectMIPSolver` solution's `y`
    * the **entire swap-1 neighbourhood of the optimum** (k*(n-k) = 100 sets) -- an invalid
      cut that excluded the true optimum would almost certainly misprice a near-optimal
      neighbour, so this is where a violation is most likely and most consequential
    * uniform random master-feasible sets to fill the quota
  Random-only sampling would be a weaker test: 500 uniform draws from 184,756 sets touches
  0.3% of the space and is unlikely to land near the optimum at all.

# Reported per seed

Cuts are read as EXACT `ConstraintRef` rows via `m[:benders_cuts]`, never rebuilt from the
rounded dedup signatures in `m[:benders_cut_signatures]` -- the first version of this script
did the latter and reported a spurious `+1.353e-06` violation at n=15 seed 42, which is
rounding, not a cut defect (`4.8e-11` relative to an objective of 28384).

`benders` vs the shipped `DirectMIPSolver` arm vs `CGSolver` (independent column source, only
compared when it converged at full-universe scope), the LB invariants, and the audit's worst
violation over (cuts x audited sets). A positive worst violation beyond tolerance is an
invalid cut and fails the run.

Usage: sbatch benchmarks/diagnostics/run_benders_cut_validity.sh
Env: CV_N CV_SEEDS CV_P CV_S CV_MAX_STOPS CV_AUDIT_SAMPLES CV_EXHAUSTIVE_MAX
"""

using StationSelection
using JuMP
using Printf
using Random
using Combinatorics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "CV_N", "20"))
const SEEDS = [parse(Int, x) for x in split(get(ENV, "CV_SEEDS", "42,43,44,45,46,47,48,49,50,51"), ',')]
const P = parse(Int, get(ENV, "CV_P", "8"))
const S = parse(Int, get(ENV, "CV_S", "3"))
const MAX_STOPS = parse(Int, get(ENV, "CV_MAX_STOPS", "4"))
const AUDIT_SAMPLES = parse(Int, get(ENV, "CV_AUDIT_SAMPLES", "400"))
# Enumerate the whole master-feasible set when it is at most this large.
const EXHAUSTIVE_MAX = parse(Int, get(ENV, "CV_EXHAUSTIVE_MAX", "8000"))
const MAX_ROUTES = 20_000_000
const ENUM_LIMIT = 3600.0
const TOL = 1e-6

@printf("n=%d p=%d s=%d max_stops=%d | seeds %s | audit: exhaustive if C(n,k)<=%d else %d targeted+random\n",
        N, P, S, MAX_STOPS, string(SEEDS), EXHAUSTIVE_MAX, AUDIT_SAMPLES)
flush(stdout)

"""The master's feasible station sets, as a predicate plus the required-location data."""
function feasibility_data(problem, formulation)
    data = problem.data
    mapping = create_aggregate_od_route_map(problem, formulation, data)
    required = Set{Int}()
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
            push!(required, o); push!(required, d)
        end
    end
    cand = Dict(pt => Set(j for j in 1:N
                          if get_walking_cost(data, pt, j) <= mapping.max_walking_distance)
                for pt in required)
    return required, cand
end

is_feasible(combo, required, cand) =
    all(!isempty(intersect(cand[pt], combo)) for pt in required)

"""Audit domain: exhaustive when small, else optimum + swap-1 neighbourhood + random fill.

The swap-1 neighbourhood is the point of this, not the random fill: an invalid cut that
excluded the true optimum has to misprice something near it, so the neighbours of the
incumbent are where a violation is both most likely and most damaging. Uniform sampling from
184,756 sets would touch 0.3% of the space and rarely land anywhere near the optimum.
"""
function audit_domain(k, optimum_sets, required, cand, rng)
    all_sets = collect(combinations(1:N, k))
    feasible = filter(c -> is_feasible(c, required, cand), all_sets)
    if length(feasible) <= EXHAUSTIVE_MAX
        return feasible, "exhaustive ($(length(feasible)) of $(length(all_sets)))"
    end
    domain = Vector{Vector{Int}}()
    seen = Set{Vector{Int}}()
    push_set!(c) = (sc = sort(c); (sc in seen || !is_feasible(sc, required, cand)) ||
                    (push!(seen, sc); push!(domain, sc)))
    for opt in optimum_sets
        push_set!(opt)
        outside = setdiff(1:N, opt)
        for drop in opt, add in outside          # the full swap-1 neighbourhood
            push_set!(sort(vcat(setdiff(opt, [drop]), add)))
        end
    end
    n_targeted = length(domain)
    while length(domain) < AUDIT_SAMPLES
        push_set!(sort(randperm(rng, N)[1:k]))
    end
    return domain, "targeted+random ($(n_targeted) targeted, $(length(domain)) total)"
end

rows = Any[]
for seed in SEEDS
    @printf("\n%s\n=== n=%d seed=%d ===\n", repeat("=", 74), N, seed)
    flush(stdout)
    try
        problem, k, _meta = benchmark_problem(@__DIR__, "CV", N, P, S, seed)
        formulation = AggregateODRouteJointRoutingAssignmentFormulation(
            ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS)

        trace = Any[]
        solver = BendersSolver(
            config=SolverOptions(silent=true, time_limit_sec=600.0),
            max_iterations=2000,
            subproblem=BendersSubproblemConfig(max_stops=MAX_STOPS, max_routes=MAX_ROUTES,
                                               enumeration_time_limit_sec=ENUM_LIMIT),
            total_time_limit_sec=3600.0,
            iteration_callback=r -> push!(trace, r))
        bd = run_opt(problem, formulation, solver)
        benders = something(bd.objective_value, NaN)
        master = bd.model
        # The EXACT rows, not the rounded dedup signatures. Auditing a cut rebuilt from its
        # 6-decimal signature reports violations up to ~5e-7 per term that the real cut does
        # not have -- measured: a spurious +1.353e-06 at n=15 seed 42 (run 22420947).
        theta_vars = master[:benders_cut_variables]
        y_vars = master[:y]
        cuts = Tuple{Int, Float64, Vector{Tuple{Int, Float64}}}[]
        for (group, row) in master[:benders_cuts]
            # Normalized form is `Theta_g + sum_j coef_j y_j >= constant`; assert the Theta
            # coefficient so a future change to the row's shape cannot be misread silently.
            isapprox(JuMP.normalized_coefficient(row, theta_vars[group]), 1.0; atol=1e-12) ||
                error("unexpected Theta coefficient in cut row for group $group")
            coefs = Tuple{Int, Float64}[]
            for j in eachindex(y_vars)
                cj = JuMP.normalized_coefficient(row, y_vars[j])
                cj == 0.0 || push!(coefs, (j, cj))
            end
            push!(cuts, (group, JuMP.normalized_rhs(row), coefs))
        end
        lbs = Float64[r.lower_bound for r in trace]

        # independent shipped arm
        dsolver = DirectMIPSolver(config=SolverOptions(silent=true, time_limit_sec=1800.0))
        dbuild = build_model(problem, formulation, dsolver;
                             max_routes=MAX_ROUTES, time_limit_sec=ENUM_LIMIT)
        dres = StationSelection.optimize_model(dbuild, dsolver)
        direct = something(dres.objective_value, NaN)
        direct_y = findall(v -> v > 0.5, JuMP.value.(dbuild.model[:y]))

        # independent column source
        cg = run_opt(problem, formulation,
                     benchmark_cg_solver(900.0; recover_integer_solution=true,
                                         pricing=CGPricingConfig(mode=:exact)))
        cg_ok = get(cg.metadata, "cg_converged", false) === true &&
                get(cg.metadata, "cg_optimality_scope", "") == "full_route_universe"
        cg_ip = something(cg.objective_value, NaN)
        cg_lp = get(cg.metadata, "cg_lp_objective_value", NaN)

        # ---- the audit ----
        required, cand = feasibility_data(problem, formulation)
        rng = MersenneTwister(1234 + seed)
        benders_y = bd.solution.selected_station_indices
        domain, domain_label = audit_domain(k, [sort(benders_y), sort(direct_y)],
                                            required, cand, rng)
        worst = -Inf
        worst_at = Int[]
        n_evaluated = 0
        for combo in domain
            y = zeros(Float64, N); y[combo] .= 1.0
            per_scenario = try
                sub = StationSelection._solve_joint_routing_assignment_benders_subproblems(
                    master, y, solver)
                Float64[r.objective for r in sub.scenarios]
            catch
                continue   # second-stage infeasible at this y; Q undefined, nothing to audit
            end
            n_evaluated += 1
            total = sum(per_scenario)
            for (group, constant, coefficients) in cuts
                predicted = constant
                for (j, coefficient) in coefficients
                    j in combo && (predicted -= coefficient)
                end
                q = group == 0 ? total : per_scenario[group]
                if predicted - q > worst
                    worst = predicted - q
                    worst_at = combo
                end
            end
        end

        @printf("Benders %.6f | Direct %.6f | CG lp %.6f ip %.6f (converged %s)\n",
                benders, direct, cg_lp, cg_ip, cg_ok)
        @printf("iters %d cuts %d | audit %s -> %d evaluated | WORST CUT VIOLATION %+.3e%s\n",
                bd.metadata["benders_iterations"], bd.metadata["benders_cuts_added"],
                domain_label, n_evaluated, worst,
                worst > TOL ? "  <-- INVALID at $(worst_at)" : "")
        flush(stdout)
        push!(rows, (seed=seed, ok=true, benders=benders, direct=direct,
                     cg_lp=cg_lp, cg_ip=cg_ip, cg_ok=cg_ok,
                     iters=bd.metadata["benders_iterations"],
                     cuts=bd.metadata["benders_cuts_added"],
                     worst=worst, n_evaluated=n_evaluated, domain_label=domain_label,
                     lb_monotone=all(lbs[i] <= lbs[i+1] + 1e-9 for i in 1:length(lbs)-1),
                     lb_ok=all(lb <= benders + TOL for lb in lbs), error=nothing))
    catch err
        msg = sprint(showerror, err)
        @printf("!! seed %d FAILED: %s\n", seed, first(msg, 250))
        flush(stdout)
        push!(rows, (seed=seed, ok=false, error=msg))
    end
end

println("\n", repeat("=", 74))
@printf("%6s %13s %13s %6s %6s %9s %12s\n",
        "seed", "benders", "DirectMIP", "iters", "cuts", "audited", "worst viol")
for r in rows
    r.ok || (@printf("%6d  FAILED\n", r.seed); continue)
    @printf("%6d %13.2f %13.2f %6d %6d %9d %+12.3e\n",
            r.seed, r.benders, r.direct, r.iters, r.cuts, r.n_evaluated, r.worst)
end

println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
for r in rows
    r.ok || (push!(checks, ("seed $(r.seed): completed", false, first(r.error, 100))); continue)
    push!(checks, ("seed $(r.seed): no invalid cut", r.worst <= TOL,
        @sprintf("worst %+.3e over %d cuts x %d sets", r.worst, r.cuts, r.n_evaluated)))
    push!(checks, ("seed $(r.seed): benders == DirectMIP",
        isapprox(r.benders, r.direct; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f", r.benders, r.direct)))
    push!(checks, ("seed $(r.seed): LB monotone and <= optimum",
        r.lb_monotone && r.lb_ok, "monotone=$(r.lb_monotone) bounded=$(r.lb_ok)"))
    if r.cg_ok
        push!(checks, ("seed $(r.seed): cg_lp <= benders <= cg_ip",
            r.cg_lp <= r.benders + TOL && r.benders <= r.cg_ip + TOL,
            @sprintf("%.6f <= %.6f <= %.6f", r.cg_lp, r.benders, r.cg_ip)))
    end
end
for (name, ok, detail) in checks
    @printf("%-42s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
ok_rows = [r for r in rows if r.ok]
@printf("\n%d checks, %d failed | %d/%d seeds completed\n",
        length(checks), n_fail, length(ok_rows), length(SEEDS))
if !isempty(ok_rows)
    @printf("worst cut violation across ALL seeds: %+.3e (tolerance %.0e)\n",
            maximum(r.worst for r in ok_rows), TOL)
    @printf("cuts across seeds: min %d max %d (compare objectives across seeds, never cuts --\n",
            minimum(r.cuts for r in ok_rows), maximum(r.cuts for r in ok_rows))
    println("  the master is a MIP with many tied optima, so the cut sequence is not reproducible)")
end
n_fail == 0 || error("verification failed")
println("\nALL CHECKS PASSED")
