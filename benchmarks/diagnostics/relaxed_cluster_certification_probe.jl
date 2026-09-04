"""Does the relaxed-cluster relaxation ever certify at K < n, and how loose is it?

The correctness of the relaxation is settled by
`test/opt/test_joint_routing_assignment_relaxed_cluster_pricing.jl` (mutation-verified:
the bound fails if the reward aggregate or the metric closure is broken). What is NOT
settled is whether it is ever *tight enough to be useful*, and there is a structural
reason to doubt it:

  at CG convergence the exact pricer's minimum reduced cost sits essentially AT zero --
  that is what convergence means -- so ANY strictly positive relaxation slack on the best
  route drives the relaxed minimum below `-reduced_cost_tol` and the certificate fails.

A relaxation that is even slightly loose therefore certifies never, not sometimes. This
probe measures that directly and cheaply, before committing queue time to Study 9's sweep.

For each `K` it reports, per run: whether the relaxation ended the solve, how many attempts
it took, and the refuted/inconclusive split of the failures -- plus the *margin*, which is
the number that actually explains the outcome. The margin is measured at the FINAL duals
(the ones a certifying round faces) as

    exact_min_rc   -- the best real column's reduced cost, ~0 at convergence
    relaxed_min_rc -- the relaxation's minimum over the same duals

so `relaxed_min_rc` is how far below zero the slack pushes things, i.e. exactly how much
tighter the relaxation would have to be to certify. That is the diagnostic that says
whether a bigger `K` could ever close the gap or whether the approach is dead.

Usage: sbatch benchmarks/diagnostics/run_relaxed_cluster_probe.sh
"""

using StationSelection
using JuMP
using Printf
using Statistics

const SS = StationSelection

n_stations = parse(Int, get(ENV, "PROBE_N", "15"))
n_pairs = parse(Int, get(ENV, "PROBE_P", "16"))
n_scenarios = parse(Int, get(ENV, "PROBE_S", "3"))
seed = parse(Int, get(ENV, "PROBE_SEED", "42"))

include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))
problem, k, _meta = benchmark_problem(@__DIR__, "PROBE", n_stations, n_pairs, n_scenarios, seed)
@printf("instance: n=%d p=%d s=%d seed=%d (k=%d)\n\n", n_stations, n_pairs, n_scenarios, seed, k)

formulation(K) = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=10, pricing_mode=:exact,
    relaxed_cluster_count=K,
)
certification_mode = Symbol(get(ENV, "PROBE_CERT_MODE", "relaxed_cluster"))
solver(K) = benchmark_cg_solver(
    120.0; recover_integer_solution=false, threads=1,
    certifying_pricing_time_limit_sec=600.0, total_time_limit_sec=1800.0,
    certification_pricing_mode=(isnothing(K) ? nothing : certification_mode),
    certification_time_limit_sec=parse(Float64, get(ENV, "PROBE_CERT_LIMIT", "60.0")),
    certification_max_rounds=parse(Int, get(ENV, "PROBE_CERT_ROUNDS", "32")),
)
@printf("certification mode: %s\n\n", string(certification_mode))

"""
Minimum reduced cost the exact pricer and the relaxation each see at `m`'s CURRENT duals
-- the converged ones, which is the only regime where certification matters. Both searches
run to exhaustion with reduced-cost pruning OFF, so the reported minimum is the true one
even when it sits above the tolerance (with pruning on, a label that cannot beat the
tolerance is simply not extended, which is sound for a yes/no certificate but useless for
measuring how far off the relaxation is).
"""
function measure_margins(m, mapping, clustering)
    duals = StationSelection.extract_duals(
        BuildResult(m, mapping, nothing, ModelCounts(Dict{String,Int}(), Dict{String,Int}(), Dict{String,Int}()), Dict{String,Any}()),
        mapping, m,
    )
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    exact_min, relaxed_min = Inf, Inf
    for s in 1:length(mapping.scenarios)
        candidates = joint_routing_assignment_pricing_candidates(
            data, mapping, alpha, gamma_o, gamma_d,
            Float64(m[:joint_routing_assignment_walk_cost_weight]),
            Float64(m[:joint_routing_assignment_detour_factor]), s,
        )
        isempty(candidates) && continue
        shared = (
            route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
            max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
            repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
            max_stops=Int(m[:joint_routing_assignment_max_stops]),
            compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
        )
        exact = create_joint_routing_assignment_pricing_data(
            s, m[:joint_routing_assignment_nodes], m[:joint_routing_assignment_travel_cost],
            candidates; shared...,
        )
        if !isempty(exact.opportunities)
            labels, _ex, _ = SS._run_label_setting(
                SS.JointRoutingAssignmentSearchContext(exact);
                time_limit=120.0, reduced_cost_tol=1e-6, use_reduced_cost_pruning=false,
            )
            isempty(labels) || (exact_min = min(exact_min, minimum(l.reduced_cost for l in labels)))
        end
        relaxed = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
            s, clustering, m[:joint_routing_assignment_travel_cost], candidates; shared...,
        )
        if !isempty(relaxed.inner.opportunities)
            labels, _ex, _ = SS._run_label_setting(
                SS.JointRoutingAssignmentSearchContext(relaxed.inner);
                time_limit=120.0, reduced_cost_tol=1e-6, use_reduced_cost_pruning=false,
            )
            isempty(labels) || (relaxed_min = min(relaxed_min, minimum(l.reduced_cost for l in labels)))
        end
    end
    return exact_min, relaxed_min
end

# `lp_objective` and `stop_reason` are the DECISIVE soundness check: a certificate that
# fired early would stop CG on an incomplete pool and land on a strictly worse (higher) LP
# objective than the uncertified baseline. `relaxed_min_rc` below is NOT that check -- it is
# a standalone diagnostic of the RAW relaxation with NO cuts applied, so it is expected to
# sit far below zero even on a run that certifies; the loop certifies over routes that
# escape every cut, not over all relaxed routes.
@printf("%-6s %-10s %14s %-24s %10s %8s %9s %12s %14s %14s %s\n",
        "K", "certified", "lp_objective", "stop_reason", "wall_s", "rounds", "refuted",
        "inconclusive", "exact_min_rc", "relaxed_min_rc", "cluster sizes")
probe_ks = Any[x == "base" ? nothing : parse(Int, x)
               for x in split(get(ENV, "PROBE_K", "base,3,6,9,12,$(n_stations)"), ',')]
for K in probe_ks
    t0 = time()
    result = run_opt(problem, formulation(K), solver(K))
    wall = time() - t0
    md = result.metadata
    sizes = if haskey(result.model.obj_dict, :joint_routing_assignment_station_clustering)
        join(station_cluster_sizes(result.model[:joint_routing_assignment_station_clustering]), '|')
    else
        "-"
    end
    exact_min, relaxed_min = if isnothing(K)
        (NaN, NaN)
    else
        measure_margins(result.model, result.mapping,
                        result.model[:joint_routing_assignment_station_clustering])
    end
    # How much work the no-good loop actually did. Cuts are per certification ATTEMPT
    # (one per CG iteration per scenario) and are NOT carried across iterations -- a
    # support barren at one dual vector can hold an improving column at the next, so
    # reusing cuts would be unsound. The number that answers "how many cuts to certify"
    # is therefore the count on the rows whose outcome was :certified.
    stats = get(md, "cg_relaxed_cluster_guide_stats", Any[])
    certified_rows = [r for r in stats if get(r, :nogood_outcome, nothing) === :certified]
    refuted_rows = [r for r in stats if get(r, :nogood_outcome, nothing) === :refuted]
    if !isempty(stats)
        @printf("       cuts-to-certify per scenario: %s | rounds: %s\n",
                isempty(certified_rows) ? "none" :
                    string([r.nogood_cuts for r in certified_rows]),
                isempty(certified_rows) ? "none" :
                    string([r.nogood_rounds for r in certified_rows]))
        for r in certified_rows
            # The last entry of each row is the certifying round: `relaxed_rc` there is the
            # surviving minimum (>= -tol, which is why it certifies) and `subset_rc` is
            # "n/a" because no support was left to check. Every EARLIER round is a barren
            # support -- its `subset_rc` sits at about zero, which is what "barren" means.
            @printf("       certifying scenario %d: relaxed_rc after each cut = %s\n",
                    r.scenario, join([x == Inf ? "no-reward" : @sprintf("%+.1f", x)
                                      for x in r.nogood_rc_trace], " -> "))
            @printf("                            best REAL rc in |S|    = %s\n",
                    join([c ? (x == Inf ? "no-reward" : @sprintf("%+.1f", x)) : "n/a"
                          for (x, c) in zip(r.nogood_subset_rc_trace,
                                            r.nogood_subset_checked_trace)], " -> "))
            @printf("                            |S|                    = %s\n",
                    join([c ? string(sz) : "n/a"
                          for (sz, c) in zip(r.nogood_subset_size_trace,
                                             r.nogood_subset_checked_trace)], " -> "))
        end
        @printf("       refuted attempts: %d (median rounds %.1f, median cuts %.1f) | attempts total %d\n",
                length(refuted_rows),
                isempty(refuted_rows) ? NaN : median([r.nogood_rounds for r in refuted_rows]),
                isempty(refuted_rows) ? NaN : median([r.nogood_cuts for r in refuted_rows]),
                length(stats))
    end
    @printf("%-6s %-10s %14.4f %-24s %10.1f %8d %9d %12d %14.4f %14.4f %s\n",
            isnothing(K) ? "base" : string(K),
            string(get(md, "cg_certified_by_relaxation", false)),
            something(result.objective_value, NaN),
            string(get(md, "cg_stop_reason", "?")),
            wall,
            Int(get(md, "cg_certification_rounds", 0)),
            Int(get(md, "cg_certification_refuted_rounds", 0)),
            Int(get(md, "cg_certification_inconclusive_rounds", 0)),
            exact_min, relaxed_min, sizes)
    flush(stdout)
end
