"""Why is the exact pricer's minimum reduced cost ~0 at a converged CG master?

`relaxed_cluster_certification_probe.jl` reported `exact_min_rc = -0.0000` at every arm,
which looks wrong: the label search only ever tracks routes that certify at least one
passenger (`_pricing_best_signature` returns `nothing` for a label with no activated reward
layers), so the empty/no-reward route is NOT in that minimum. A real, reward-carrying route
landing exactly on zero wants an explanation.

Two candidate explanations, and this script distinguishes them:

  (A) **Complementary slackness.** `theta` carries only a lower bound of 0, so any column
      the master actually USES (theta_j > 0) has reduced cost exactly zero at EVERY optimal
      dual, not merely at the basic one. The pricing universe contains those columns, so
      `min rc = 0` follows from optimality alone -- no degeneracy required, and no choice of
      optimal dual can move it. (An earlier version of this file argued via variables at an
      upper bound; theta has none.)

  (B) **Degeneracy / something else.** The minimum is attained by a route NOT in the pool,
      or the run never actually converged, or many structurally different routes pile up at
      exactly zero in a way that indicates a degenerate master.

The decisive test is (A)'s prediction: the argmin route's assignment signature is already a
key in `m[:joint_routing_assignment_column_signatures]`. This script reports that, plus the
minimum at full precision, the count of routes within tolerance of zero (dual degeneracy
shows up as many alternative optima), whether each search actually exhausted (a truncated
search reports a minimum that is only an upper bound on the true one), and a primal
degeneracy readout over the master's own variables.

Usage: sbatch benchmarks/diagnostics/run_min_rc_audit.sh
"""

using StationSelection
using JuMP
using Printf

const SS = StationSelection

n_stations = parse(Int, get(ENV, "AUDIT_N", "15"))
n_pairs = parse(Int, get(ENV, "AUDIT_P", "16"))
n_scenarios = parse(Int, get(ENV, "AUDIT_S", "3"))
seed = parse(Int, get(ENV, "AUDIT_SEED", "42"))

include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))
problem, k, _meta = benchmark_problem(@__DIR__, "AUDIT", n_stations, n_pairs, n_scenarios, seed)
@printf("instance: n=%d p=%d s=%d seed=%d (k=%d)\n\n", n_stations, n_pairs, n_scenarios, seed, k)

formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=10, pricing_mode=:exact,
)
solver = benchmark_cg_solver(
    120.0; recover_integer_solution=false, threads=1,
    certifying_pricing_time_limit_sec=600.0, total_time_limit_sec=1800.0,
)

result = run_opt(problem, formulation, solver)
md = result.metadata
println("── did CG actually converge? ──")
@printf("  termination      %s\n", string(result.termination_status))
@printf("  cg_converged     %s\n", string(get(md, "cg_converged", missing)))
@printf("  cg_stop_reason   %s\n", string(get(md, "cg_stop_reason", missing)))
@printf("  cg_iterations    %s\n", string(get(md, "cg_iterations", missing)))
@printf("  cg_certifying_rounds %s\n", string(get(md, "cg_certifying_rounds", missing)))
@printf("  lp objective     %.6f\n\n", something(result.objective_value, NaN))

m = result.model
mapping = result.mapping

# ── primal degeneracy readout on the master's own variables ─────────────────
theta = m[:joint_routing_assignment_theta]
theta_values = [JuMP.value(v) for v in values(theta)]
y_values = [JuMP.value(v) for v in m[:y]]
println("── master solution shape ──")
@printf("  theta columns          %d\n", length(theta_values))
@printf("  theta at 0 (<=1e-9)    %d\n", count(v -> v <= 1e-9, theta_values))
@printf("  theta at 1 (>=1-1e-9)  %d\n", count(v -> v >= 1 - 1e-9, theta_values))
@printf("  theta strictly fractional %d\n", count(v -> 1e-9 < v < 1 - 1e-9, theta_values))
@printf("  y fractional           %d of %d\n\n",
        count(v -> 1e-9 < v < 1 - 1e-9, y_values), length(y_values))

# ── basis status: the decisive degeneracy test ──────────────────────────────
# `min rc in [-tol, 0]` follows from optimality alone and implies nothing about
# degeneracy: a non-degenerate LP attains rc = 0 exactly at its BASIC columns. What would
# prove degeneracy is a column NONBASIC AT ITS LOWER BOUND carrying rc = 0 -- it can be
# pivoted in for free, i.e. an alternative optimum. So count the basis statuses and, below,
# report the status of every rc = 0 column found by the pricer.
basis_status = Dict{Int, Any}()
basis_available = true
try
    for (id, var) in m[:joint_routing_assignment_theta]
        basis_status[id] = MOI.get(m, MOI.VariableBasisStatus(), var)
    end
catch err
    global basis_available = false
    @printf("  (basis status unavailable: %s)\n\n", replace(sprint(showerror, err), '\n' => ' '))
end
if basis_available
    tally = Dict{Any, Int}()
    for st in values(basis_status)
        tally[st] = get(tally, st, 0) + 1
    end
    println("── theta basis status ──")
    for (st, n) in sort(collect(tally); by = kv -> string(kv[1]))
        @printf("  %-24s %d\n", string(st), n)
    end
    # Degenerate basic variables: basic AND sitting at a bound. In the LP relaxation theta
    # is declared with `lower_bound = 0.0` and NO upper bound (see
    # `add_joint_routing_assignment_column!`), so the ONLY bound is 0 -- a basic theta at
    # value 1 is an ordinary interior basic value, not a degenerate one. An earlier version
    # of this line also counted `value >= 1 - 1e-9`, which inflated the count by every
    # column the master actually uses.
    n_basic_at_bound = count(
        id -> basis_status[id] == MOI.BASIC &&
              JuMP.value(m[:joint_routing_assignment_theta][id]) <= 1e-9,
        collect(keys(basis_status)),
    )
    @printf("  BASIC at lower bound 0 (primal degeneracy) %d\n", n_basic_at_bound)
    @printf("  BASIC at a strictly positive value         %d\n\n",
            count(id -> basis_status[id] == MOI.BASIC &&
                        JuMP.value(m[:joint_routing_assignment_theta][id]) > 1e-9,
                  collect(keys(basis_status))))
end

# ── the pricing minimum at the final duals ──────────────────────────────────
build_result = BuildResult(m, mapping, nothing,
    ModelCounts(Dict{String,Int}(), Dict{String,Int}(), Dict{String,Int}()), Dict{String,Any}())
alpha, gamma_o, gamma_d = StationSelection.extract_duals(build_result, mapping, m)
data = m[:joint_routing_assignment_data]
pool_signatures = m[:joint_routing_assignment_column_signatures]
@printf("pool holds %d column signatures\n\n", length(pool_signatures))

println("── exact pricer minimum at the FINAL duals, per scenario ──")
for s in 1:length(mapping.scenarios)
    candidates = joint_routing_assignment_pricing_candidates(
        data, mapping, alpha, gamma_o, gamma_d,
        Float64(m[:joint_routing_assignment_walk_cost_weight]),
        Float64(m[:joint_routing_assignment_detour_factor]), s,
    )
    if isempty(candidates)
        @printf("scenario %d: no candidates (nothing to price)\n", s)
        continue
    end
    pricing = create_joint_routing_assignment_pricing_data(
        s, m[:joint_routing_assignment_nodes], m[:joint_routing_assignment_travel_cost],
        candidates;
        route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
        max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
        repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
        max_stops=Int(m[:joint_routing_assignment_max_stops]),
        compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
    )
    # Pruning OFF: with it on, a label whose completions cannot beat -tol is never
    # extended, so the reported minimum is only meaningful below the tolerance. This audit
    # is precisely about the value AT the tolerance.
    labels, exhausted, _stats = SS._run_label_setting(
        SS.JointRoutingAssignmentSearchContext(pricing);
        time_limit=300.0, reduced_cost_tol=1e-6, use_reduced_cost_pruning=false,
    )
    @printf("scenario %d: %d tracked labels, search exhausted = %s\n", s, length(labels), exhausted)
    isempty(labels) && continue
    sorted = sort(labels; by=l -> l.reduced_cost)
    @printf("  min reduced cost   %.12g\n", sorted[1].reduced_cost)
    @printf("  routes with rc <= 1e-6      %d\n", count(l -> l.reduced_cost <= 1e-6, labels))
    @printf("  routes with |rc| <= 1e-6    %d\n", count(l -> abs(l.reduced_cost) <= 1e-6, labels))
    @printf("  routes with rc <= 1e-3      %d\n", count(l -> l.reduced_cost <= 1e-3, labels))
    println("  five cheapest, decomposed, and matched against the master's own pool:")
    beta = Float64(m[:joint_routing_assignment_route_regularization_weight])
    reposition = Float64(m[:joint_routing_assignment_repositioning_time])
    theta_by_id = m[:joint_routing_assignment_theta]
    for label in sorted[1:min(5, length(sorted))]
        assignments, tau, rc, _pos = SS._joint_routing_assignment_column_from_route(
            label.route, pricing; label_reduced_cost=label.reduced_cost,
        )
        # The pool is keyed by (scenario, assignment signature) -- see
        # `add_joint_routing_assignment_column!`. Looking up the bare signature silently
        # misses every time, which is exactly the bug the first run of this audit had.
        signature = (s, SS._joint_routing_assignment_column_signature(assignments))
        pooled_id = get(pool_signatures, signature, nothing)
        pooled = if isnothing(pooled_id)
            "no"
        else
            @sprintf("id=%d theta=%.6f status=%s", pooled_id,
                     JuMP.value(theta_by_id[pooled_id]),
                     basis_available ? string(get(basis_status, pooled_id, "?")) : "?")
        end
        reward = beta * (tau + reposition) - rc
        # NOTE: @printf needs a LITERAL format string -- a `*` concatenation is a compile
        # error, not a runtime one, so it takes a job to find out.
        @printf("    rc=%+.6g = beta*travel %.4f + beta*repos %.4f - reward %.4f | %d assignments | in_pool: %s | route=%s\n",
                rc, beta * tau, beta * reposition, reward, length(assignments),
                pooled, string(label.route))
    end
    println()
end
