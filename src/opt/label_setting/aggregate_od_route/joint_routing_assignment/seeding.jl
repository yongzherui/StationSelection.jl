"""
    joint_routing_assignment_two_stop_seed_columns(data, mapping; next_column_id=1)

Every two-stop route `[j, k]` that any demand group can use, one column per
`(scenario, j, k)`, built directly off `AggregateODRouteMap`.

# Why this exists

The `v[(s,p)]` slack makes the RMP feasible from an *empty* pool, but an empty pool is
not a requirement -- it was just the starting point. Starting empty means the first
several CG iterations are not improving the routing cost at all, they are hunting for
enough columns to cover every demand group, and until they succeed the LP objective is
dominated by `unserved_penalty * sum(v)`. Two-stop routes remove that phase entirely,
because they are exactly the columns the big-M was standing in for.

# Coverage claim

`AggregateODRouteProblem`'s constructor enforces `detour_factor >= 1.0`, and replaying
`[j, k]` gives the pickup at `j` an age of exactly `travel(j,k)` on arrival at `k` --
i.e. exactly at its own ride limit `detour_factor * travel(j,k)` when `detour_factor ==
1.0`, and strictly under it otherwise. So *every* `(o,d,j,k)` this function considers is
certified by its own two-stop route unconditionally; no explicit ride-limit check is
needed (unlike the discarded `MasterData`-based version, which computed one that could
never fail given that same constructor invariant).

One column per `(s, j, k)` rather than per demand group: a two-stop route carries every
demand group of that scenario whose `(j,k)` it certifies, and that is the same column,
so the seed count is bounded by `n_scenarios * n * (n-1)`.
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
                requires_no_vehicle_route(pair) && continue
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
        push!(columns, JointRoutingAssignmentRouteColumn(
            id, [j, k], sort!(by_route[(s, j, k)]), get_routing_cost(data, j, k);
            metadata=Dict{String, Any}("scenario" => s, "seed" => "two_stop"),
        ))
        id += 1
    end
    return columns
end
