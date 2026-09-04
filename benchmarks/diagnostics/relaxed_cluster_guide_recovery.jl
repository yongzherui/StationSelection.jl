"""Does the relaxation's winning CLUSTER set contain the exact pricer's winning STATION set?

That is the whole premise of `pricing_mode = :relaxed_cluster_guided`
(`relaxed_cluster/guide.jl`): if the cluster route minimizing the relaxed reduced cost is
the image of the real route minimizing the exact one, then the stations of the real optimum
all live inside the clusters the relaxed optimum visited, and searching only those stations
finds the same column far more cheaply.

Unlike certification, this premise does NOT need the relaxation to be tight -- it only needs
the argmin to land in the right neighbourhood. So it can succeed on exactly the instances
where certification failed 0/31, and it is worth measuring separately.

# Method

The dual trajectory has to be the REAL one, so CG is run unguided (`pricing_mode = :exact`)
and stopped after `max_iterations = t` for a ladder of `t`. That gives dual vectors from
early (far from convergence) to final. At each one, three searches on the same duals:

  full     -- the exact pricer over all n stations         -> `full_rc`, and its argmin route
  guide    -- the relaxed pricer over K clusters           -> a station subset S
  subset   -- the exact pricer over S only                 -> `subset_rc`

and two questions per scenario:

  containment  -- are the full argmin route's stations a subset of S?  (the literal premise)
  recovery     -- is `subset_rc == full_rc`?                            (what actually matters)

Recovery is the weaker and more useful test: the subset can miss the specific argmin route
and still contain an equally good one, and CG does not care which route it gets. Containment
is reported because it is the premise as stated, and because the two coming apart is itself
informative.

`|S| / n` is reported alongside, because recovery is trivial when the subset is everything.
A row with recovery = yes and `|S| = n` bought nothing.

Usage: sbatch benchmarks/diagnostics/run_guide_recovery.sh
"""

using StationSelection
using JuMP
using Printf

const SS = StationSelection

n_stations = parse(Int, get(ENV, "GUIDE_N", "15"))
n_pairs = parse(Int, get(ENV, "GUIDE_P", "16"))
n_scenarios = parse(Int, get(ENV, "GUIDE_S", "3"))
seed = parse(Int, get(ENV, "GUIDE_SEED", "42"))
cluster_counts = [parse(Int, x) for x in split(get(ENV, "GUIDE_K", "3,6,9,12"), ',')]
iteration_ladder = [parse(Int, x) for x in split(get(ENV, "GUIDE_ITERS", "1,2,4,8,16,32"), ',')]

include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))
problem, k, _meta = benchmark_problem(@__DIR__, "GUIDE", n_stations, n_pairs, n_scenarios, seed)
@printf("instance: n=%d p=%d s=%d seed=%d (k=%d)\n", n_stations, n_pairs, n_scenarios, seed, k)
@printf("cluster counts: %s | iteration checkpoints: %s\n\n",
        string(cluster_counts), string(iteration_ladder))

"""Best reduced cost and the argmin route, over a pricing graph, searched to exhaustion.
Pruning stays ON here: this is looking for the best IMPROVING route, which is exactly what
pruning preserves, and leaving it on keeps the full-graph search affordable."""
function best_route(ctx)
    labels, exhausted, _ = SS._run_label_setting(
        ctx; time_limit=120.0, reduced_cost_tol=1e-6,
    )
    isempty(labels) && return (Inf, Int[], exhausted)
    best = argmin(l -> l.reduced_cost, labels)
    return (best.reduced_cost, best.route, exhausted)
end

@printf("%-6s %-6s %-5s %6s %7s %14s %14s %-11s %-9s\n",
        "K", "iters", "scen", "|S|", "|S|/n", "full_rc", "subset_rc", "containment", "recovery")

for n_clusters in cluster_counts, max_iterations in iteration_ladder
    formulation = AggregateODRouteJointRoutingAssignmentFormulation(
        ; BENCHMARK_BASELINE..., max_stops=10, pricing_mode=:exact,
        relaxed_cluster_count=n_clusters,
    )
    solver = CGSolver(
        config=SolverOptions(silent=true, time_limit_sec=300.0, threads=1),
        max_iterations=max_iterations, reduced_cost_tol=1e-6,
        pricing_time_limit_sec=120.0, certifying_pricing_time_limit_sec=600.0,
        total_time_limit_sec=1800.0, parallel_scenario_pricing=true,
        recover_integer_solution=false,
    )
    result = run_opt(problem, formulation, solver)
    m, mapping = result.model, result.mapping
    clustering = m[:joint_routing_assignment_station_clustering]

    build_result = BuildResult(m, mapping, nothing,
        ModelCounts(Dict{String,Int}(), Dict{String,Int}(), Dict{String,Int}()), Dict{String,Any}())
    alpha, gamma_o, gamma_d = StationSelection.extract_duals(build_result, mapping, m)
    data = m[:joint_routing_assignment_data]
    all_nodes = m[:joint_routing_assignment_nodes]
    shared = (
        route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
        max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
        repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
        max_stops=Int(m[:joint_routing_assignment_max_stops]),
        compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
    )

    for s in 1:length(mapping.scenarios)
        candidates = joint_routing_assignment_pricing_candidates(
            data, mapping, alpha, gamma_o, gamma_d,
            Float64(m[:joint_routing_assignment_walk_cost_weight]),
            Float64(m[:joint_routing_assignment_detour_factor]), s,
        )
        isempty(candidates) && continue

        full = create_joint_routing_assignment_pricing_data(
            s, all_nodes, m[:joint_routing_assignment_travel_cost], candidates; shared...,
        )
        isempty(full.opportunities) && continue
        full_rc, full_route, _ = best_route(SS.JointRoutingAssignmentSearchContext(full))

        relaxed = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
            s, clustering, m[:joint_routing_assignment_travel_cost], candidates; shared...,
        )
        cluster_routes, _relaxed_exhausted = SS._relaxed_cluster_guide_routes(
            relaxed, Int(m[:joint_routing_assignment_relaxed_cluster_guide_routes]),
            Float64(m[:joint_routing_assignment_relaxed_cluster_guide_time_limit_sec]),
        )
        subset = isempty(cluster_routes) ? Int[] :
            relaxed_cluster_station_subset(clustering, cluster_routes)

        subset_rc = Inf
        if !isempty(subset)
            subset_candidates = SS._restrict_candidates_to_subset(candidates, subset)
            if !isempty(subset_candidates)
                subset_pricing = create_joint_routing_assignment_pricing_data(
                    s, subset, m[:joint_routing_assignment_travel_cost], subset_candidates; shared...,
                )
                isempty(subset_pricing.opportunities) ||
                    ((subset_rc, _r, _e) = best_route(SS.JointRoutingAssignmentSearchContext(subset_pricing)))
            end
        end

        contained = isempty(full_route) ? "n/a" :
            (issubset(Set(full_route), Set(subset)) ? "yes" : "no")
        # Recovery is judged on VALUE, not identity: a different route of equal reduced cost
        # is just as good to the master.
        recovered = if !isfinite(full_rc)
            "n/a"
        elseif isfinite(subset_rc) && subset_rc <= full_rc + 1e-6
            "yes"
        else
            "no"
        end
        @printf("%-6d %-6d %-5d %6d %7.2f %14.4f %14.4f %-11s %-9s\n",
                n_clusters, max_iterations, s, length(subset),
                length(subset) / length(all_nodes), full_rc, subset_rc, contained, recovered)
        flush(stdout)
    end
end
