"""
    joint_routing_assignment_two_stop_seed_columns(data, mapping; next_column_id=1)

Every two-stop route `[j, k]` that any demand group can use, one column per
`(scenario, j, k)`, built directly off `AggregateODRouteMap`.

# Why this exists

There is no unserved-demand slack in this master (see
`aggregate_od_route_validate_feasible_coverage`, `data/maps/aggregate_od_route_map.jl`)
-- RMP feasibility is established once, at build time, by construction: `build_model`
always calls this function before returning. Without it, CG would start from an empty
pool and its first several iterations would spend all their effort hunting for *any*
feasible column per demand group rather than improving routing cost. Two-stop routes
remove that phase entirely, because they are exactly the columns needed to cover every
demand group whose `get_valid_jk_pairs` contains a real (non-walk-only) pair.

# Coverage claim

`AggregateODRouteProblem`'s constructor enforces `detour_factor >= 1.0`, and replaying
`[j, k]` gives the pickup at `j` an age of exactly `travel(j,k)` on arrival at `k` --
i.e. exactly at its own ride limit `detour_factor * travel(j,k)` when `detour_factor ==
1.0`, and strictly under it otherwise. A finite door-to-door ratio caps that limit per
passenger, but never below `travel(j,k)`: `compute_valid_jk_pairs` kept `(j,k)` only if the
direct ride fits the same budget, and `joint_routing_assignment_ride_limit` asserts it.
So *every* `(o,d,j,k)` this function considers is
certified by its own two-stop route unconditionally; no explicit ride-limit check is
needed (unlike the discarded `MasterData`-based version, which computed one that could
never fail given that same constructor invariant).

One column per `(s, j, k)` carrying every demand group of that scenario whose `(j,k)` it
certifies, PLUS one singleton column per `(s, p, j, k)` carrying that group alone.

# Why the singletons (2026-09-28)

The master's coverage is set partitioning: each group must be covered by exactly one
selected column, and a column serves its whole assignment set at once. Bundled seeds alone
therefore need two forced groups never to share a bundle with a third group. That held
while every group had many valid pairs, and broke under `door_to_door_ratio`: at k = 20 on
sub35 (ratio 2.0), groups with a single valid pair forced two bundles that both also
carried another group, the RMP was infeasible before any pricing (IIS: two coverage rows,
one dropoff_link row, one `y <= 1` bound), and CG stopped. `ClusteringTwoStageOD`, which
assigns groups independently, was feasible on the same instance.

With a singleton for every `(s, p, j, k)`, any per-group assignment is representable, so the
restricted master is feasible whenever some station set serves every group. The bundles are
kept: they give CG multi-group routes from iteration 1. A singleton that would duplicate a
one-group bundle is not emitted. Seed count is bounded by `n_scenarios * n * (n-1)` bundles
plus one singleton per valid (group, pair).
"""
function joint_routing_assignment_two_stop_seed_columns(
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    next_column_id::Int=1,
)::Vector{JointRoutingAssignmentRouteColumn}
    by_route = Dict{Tuple{Int, Int, Int}, Vector{Tuple{Int, Int, Int}}}()
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, k = pair
                tau = get_routing_cost(data, j, k)
                isfinite(tau) || continue
                push!(get!(by_route, (s, j, k), Tuple{Int, Int, Int}[]), (p, j, k))
            end
        end
    end

    columns = JointRoutingAssignmentRouteColumn[]
    id = next_column_id
    for (s, j, k) in sort!(collect(keys(by_route)))
        assignments = sort!(by_route[(s, j, k)])
        push!(columns, _two_stop_seed_column(id, s, j, k, assignments, data, "two_stop"))
        id += 1
    end
    # Singletons, in addition to the bundles (see the docstring): one group per column.
    for (s, j, k) in sort!(collect(keys(by_route)))
        assignments = by_route[(s, j, k)]
        length(assignments) > 1 || continue      # a one-group bundle already is the singleton
        for a in sort(assignments)
            push!(columns, _two_stop_seed_column(id, s, j, k, [a], data, "two_stop_singleton"))
            id += 1
        end
    end
    return columns
end

function _two_stop_seed_column(id::Int, s::Int, j::Int, k::Int,
                               assignments::Vector{Tuple{Int, Int, Int}},
                               data::StationSelectionData, seed::String)
    return JointRoutingAssignmentRouteColumn(
        id, [j, k], assignments, get_routing_cost(data, j, k);
        metadata=Dict{String, Any}(
            "scenario" => s, "seed" => seed,
            # Same key the priced columns carry (`exact/hooks.jl`). Trivial on a
            # two-stop route -- board at stop 1, alight at stop 2, no revisits
            # possible -- but recorded so the key is not silently absent on seeds.
            "assignment_positions" =>
                Dict{Int, Tuple{Int, Int}}(p => (1, 2) for (p, _j, _k) in assignments),
        ),
    )
end
