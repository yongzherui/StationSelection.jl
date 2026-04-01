"""
Exhaustive DFS-based route generation for RouteVehicleCapacityModel.

Generates all demand-justified routes up to `max_route_length` stops using a
coverage/salvageability pruning rule.

Each generated route's `detour_feasible_legs` field records the (j,k) pairs whose
in-vehicle detour satisfies the detour constraints. The MILP uses this to suppress
`alpha_r_jkts` variables for infeasible legs.
"""

export generate_simple_routes


"""
    generate_simple_routes(
        valid_jk_pairs          :: Set{Tuple{Int,Int}},
        array_idx_to_station_id :: Vector{Int},
        data                    :: StationSelectionData;
        max_route_length        :: Int     = 4,
        max_detour_time         :: Float64 = Inf,
        max_detour_ratio        :: Float64 = Inf
    ) :: Vector{RouteData}

Generate all demand-justified routes via exhaustive DFS with coverage pruning.

`valid_jk_pairs` is the allowed-assignment set A_s for a single (scenario, time-bucket):
each (j_idx, k_idx) means station j_idx can be a pickup and k_idx a dropoff for some
request in this bucket.

For each recorded route, `detour_feasible_legs` contains every (j_id, k_id) leg whose
in-vehicle detour satisfies `max_detour_time` and `max_detour_ratio`; infeasible legs
are omitted.

Routes are deduplicated by station-ID sequence. The returned vector is sorted by
internal route id (creation order).

# Algorithm

1. Build `pickup_partners[j]` = list of k where (j,k) ∈ A_s.
2. Start DFS only from P_s = stations that appear as a pickup in some pair.
3. At each DFS node, prune if any station in the current route is:
   - not covered (no realized pair within the route), AND
   - not salvageable (no unused station u with (station, u) ∈ A_s).
4. Before extending to candidate u, skip u if it is neither useful_now (closes some
   existing pickup) nor useful_later (can open a new pickup for a future station).
5. When the route has length ≥ 2 and all positions are covered, record it with
   detour-filtered `detour_feasible_legs`.
"""
function generate_simple_routes(
    valid_jk_pairs          :: Set{Tuple{Int,Int}},
    array_idx_to_station_id :: Vector{Int},
    data                    :: StationSelectionData;
    max_route_length        :: Int     = 4,
    max_detour_time         :: Float64 = Inf,
    max_detour_ratio        :: Float64 = Inf,
    stop_dwell_time         :: Float64 = 0.0
) :: Vector{RouteData}

    isempty(valid_jk_pairs) && return RouteData[]

    # ── Preprocessing ────────────────────────────────────────────────────────────
    # Build V_s (all active stations) and pickup_partners (j → dropoff candidates)
    active_set     = Set{Int}()
    pickup_partners = Dict{Int, Vector{Int}}()
    for (j, k) in valid_jk_pairs
        push!(active_set, j)
        push!(active_set, k)
        push!(get!(pickup_partners, j, Int[]), k)
    end

    v_s = sort!(collect(active_set))                            # V_s sorted for determinism
    p_s = [j for j in v_s if haskey(pickup_partners, j)]       # P_s (valid start stations)

    # Dedup by station-ID sequence; id assigned in creation order
    routes_map = Dict{Vector{Int}, RouteData}()
    next_id    = Ref(0)

    # ── Phase 0: seed one direct 2-stop route for every valid (j,k) pair ─────────
    # This guarantees that every (j,k) demand bucket has at least one covering route,
    # preventing the lazy capacity constraint from forcing x=0 for uncoverable pairs.
    # A direct j→k route has zero detour, so its single leg is always detour-feasible.
    # DFS-generated 2-stop routes that duplicate a seed are skipped via the existing
    # !haskey(routes_map, sids) check.
    for (j_idx, k_idx) in valid_jk_pairs
        j_idx == k_idx && continue   # skip trivial self-assignment
        j_id = array_idx_to_station_id[j_idx]
        k_id = array_idx_to_station_id[k_idx]
        j_id == k_id && continue     # same physical station — skip
        sids = [j_id, k_id]
        haskey(routes_map, sids) && continue
        tt = get_routing_cost(data, j_id, k_id)
        isinf(tt) && continue        # stations not connected in road network
        next_id[] += 1
        routes_map[sids] = RouteData(next_id[], sids, tt, [(j_id, k_id)])
    end

    # DFS state (mutated in-place with backtracking)
    route    = Int[]       # current station-index sequence
    in_route = Set{Int}()  # O(1) membership test

    # ── DFS ──────────────────────────────────────────────────────────────────────
    function dfs!(covered :: BitSet)
        m = length(route)

        # Pruning: every uncovered position must be salvageable
        for h in 1:m
            h in covered && continue
            partners = get(pickup_partners, route[h], Int[])
            # Salvageable if any partner is not yet in the route
            if !any(u ∉ in_route for u in partners)
                return   # cannot be justified — prune
            end
        end

        # Record route when fully covered and length ≥ 2
        if m >= 2 && length(covered) == m
            sids = [array_idx_to_station_id[route[i]] for i in 1:m]
            if !haskey(routes_map, sids)
                # Precompute consecutive segment costs
                seg = Vector{Float64}(undef, m - 1)
                for i in 1:(m - 1)
                    seg[i] = get_routing_cost(data, sids[i], sids[i + 1])
                end
                n_intermediate = m - 2
                tt = sum(seg; init = 0.0) + n_intermediate * stop_dwell_time
                # Build detour_feasible_legs: include (i,j) leg only if detour is feasible
                feasible_legs = Tuple{Int,Int}[]
                for i in 1:m
                    cum = 0.0
                    for j in (i + 1):m
                        cum += seg[j - 1]
                        direct = get_routing_cost(data, sids[i], sids[j])
                        if (cum - direct <= max_detour_time) &&
                           (direct == 0.0 || cum / direct <= 1.0 + max_detour_ratio)
                            push!(feasible_legs, (sids[i], sids[j]))
                        end
                    end
                end
                next_id[] += 1
                routes_map[sids] = RouteData(next_id[], sids, tt, feasible_legs)
            end
        end

        m >= max_route_length && return

        # Extend to each candidate station
        for u in v_s
            u in in_route && continue

            # useful_now: u closes a pickup already in the route
            useful_now = false
            for h in 1:m
                if (route[h], u) in valid_jk_pairs
                    useful_now = true
                    break
                end
            end

            # useful_later: u can open a pickup for some future station
            if !useful_now
                partners_u = get(pickup_partners, u, Int[])
                if !any(v ∉ in_route && v != u for v in partners_u)
                    continue   # u is irrelevant — skip
                end
            end

            # Incremental coverage update when appending u at position m+1
            new_covered = copy(covered)
            for h in 1:m
                if (route[h], u) in valid_jk_pairs
                    push!(new_covered, h)
                    push!(new_covered, m + 1)
                end
            end

            push!(route, u)
            push!(in_route, u)
            dfs!(new_covered)
            pop!(route)
            delete!(in_route, u)
        end
    end

    # Launch DFS from each valid pickup station
    n_roots = length(p_s)
    for (j_num, j) in enumerate(p_s)
        push!(route, j)
        push!(in_route, j)
        dfs!(BitSet())
        pop!(route)
        delete!(in_route, j)
        print("\r    DFS: $j_num/$n_roots roots done, $(length(routes_map)) routes found")
        flush(stdout)
    end
    println()

    return sort!(collect(values(routes_map)), by = r -> r.id)
end
