"""Reproduce the cut-search / brute-force mismatch and identify its cause.

The randomized test found the cut-aware search reporting a HIGHER minimum than exhaustive
enumeration over cut-satisfying routes. Two candidate causes, with opposite implications:

  (A) **candidate generation is reward-driven.** `_joint_routing_assignment_candidate_next_nodes`
      only proposes nodes that unlock a not-yet-activated reward layer. With cuts, a route
      may need to visit a cluster PURELY to escape a cut, collecting nothing there -- and
      the search would never propose it. That is a real bug: the loop could then fail to
      find a surviving improving route and certify falsely.

  (B) **seeding is restricted to opportunity origins.** Brute force starts routes anywhere.
      If the brute-force optimum merely STARTS at a non-origin node, the search misses it
      legitimately (truncating a reward-free prefix is WLOG for the loop's purposes), and
      the test is over-strict rather than the code being wrong.

This prints the brute-force winner and, for each of its nodes, whether it is an
opportunity origin and whether arriving there activates any new reward layer -- which
distinguishes the two directly.
"""

using StationSelection
using Printf
using Random

const SS = StationSelection

Random.seed!(20260905)
tol = 1e-6

function brute_force(nodes, max_stops, node_clusters, cut_sets, rc_of)
    satisfies(route) = all(any(!in(node_clusters[v], T) for v in route) for T in cut_sets)
    best, best_route = Inf, Int[]
    frontier = [[v] for v in nodes]
    while !isempty(frontier)
        next_frontier = Vector{Int}[]
        for route in frontier
            if satisfies(route)
                rc = rc_of(route)
                rc < best && ((best, best_route) = (rc, copy(route)))
            end
            length(route) < max_stops || continue
            for v in nodes
                v == route[end] && continue
                push!(next_frontier, vcat(route, v))
            end
        end
        frontier = next_frontier
    end
    return best, best_route
end

for trial in 1:10
    n_stations = rand(6:8)
    nodes = collect(1:n_stations)
    xs, ys = rand(n_stations) .* 100, rand(n_stations) .* 100
    costs = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        costs[(i, j)] = hypot(xs[i] - xs[j], ys[i] - ys[j])
    end
    candidates = PassengerAssignmentCandidate[]
    for p in 1:rand(3:5), _ in 1:2
        j = rand(nodes); k = rand(filter(!=(j), nodes))
        push!(candidates, PassengerAssignmentCandidate(
            p, j, k, costs[(j, k)] * (1.0 + rand()), 3.0 * (5.0 + rand() * 25.0)))
    end
    max_stops = rand(3:4)
    clustering = cluster_stations_by_travel_cost(nodes, costs, rand(2:4))
    relaxed = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
        1, clustering, costs, candidates;
        route_regularization_weight=1.0, max_wait_time=maximum(values(costs)) * 0.6,
        repositioning_time=5.0, max_stops=max_stops, compensated_dominance=isodd(trial))
    isempty(relaxed.inner.opportunities) && continue
    node_clusters = SS._relaxed_cluster_node_clusters(relaxed)
    rc_of(route) = SS._joint_routing_assignment_column_from_route(route, relaxed.inner)[3]

    cut_sets = Set{Int}[]
    for round in 1:5
        labels, _ex, _ = SS._run_label_setting(
            SS.RelaxedClusterCutSearchContext(relaxed, cut_sets);
            time_limit=60.0, reduced_cost_tol=tol, use_reduced_cost_pruning=false)
        search_min = isempty(labels) ? Inf : minimum(l.reduced_cost for l in labels)
        bf_min, bf_route = brute_force(relaxed.inner.nodes, max_stops, node_clusters, cut_sets, rc_of)

        if bf_min < -tol && !(isapprox(search_min, bf_min; atol=1e-6))
            @printf("\nMISMATCH trial %d round %d: search=%.4f  bruteforce=%.4f\n",
                    trial, round, search_min, bf_min)
            @printf("  cuts: %s\n", string([sort(collect(T)) for T in cut_sets]))
            @printf("  brute-force winner route: %s (clusters %s)\n",
                    string(bf_route), string([node_clusters[v] for v in bf_route]))
            origins = Set(o.origin for o in relaxed.inner.opportunities)
            @printf("  %-6s %-9s %-14s %s\n", "node", "isorigin", "opens-new-reward", "note")
            activated = SS.RewardLayerBitset()
            elapsed = 0.0
            for (i, v) in enumerate(bf_route)
                travel = i == 1 ? 0.0 :
                    SS._joint_routing_assignment_travel(relaxed.inner, bf_route[i-1], v)
                elapsed += travel
                gained = SS.RewardLayerBitset()
                for opp in get(relaxed.inner.assignments_by_destination, v,
                               StationSelection.PassengerAssignmentOpportunity[])
                    union!(gained, opp.layer_mask)
                end
                new_layers = setdiff(gained, activated)
                union!(activated, new_layers)
                @printf("  %-6d %-9s %-14s %s\n", v, string(v in origins),
                        string(!isempty(new_layers)),
                        i == 1 ? "(start)" : @sprintf("arrive t=%.1f", elapsed))
            end
            exit(0)
        end
        bf_min < -tol || break
        improving = filter(l -> l.reduced_cost < -tol, labels)
        isempty(improving) && break
        best = argmin(l -> l.reduced_cost, improving)
        push!(cut_sets, Set{Int}(node_clusters[v] for v in best.route))
    end
end
println("no mismatch reproduced")
