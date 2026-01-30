using Combinatorics

"""
    find_detour_combinations(
        model::AbstractSingleDetourModel,
        data::StationSelectionData
    ) -> Vector{Tuple{Int, Int, Int}}

Find all valid detour triplets (j, k, l) based on station geometry and travel times.

For a triplet (j, k, l):
- j = source station
- k = intermediate station
- l = destination station
- j→l is the "longest edge" (direct route)
- j→k→l is the detour path

A triplet is valid if:
1. t(j→l) is the longest edge (t(j→l) > t(j→k) and t(j→l) > t(k→l))
2. Triangle inequality holds: t(j→k) + t(k→l) >= t(j→l)
3. Detour constraint: t(j→k) + t(k→l) <= t(j→l) + max_delay

This method works for TwoStageSingleDetourModel regardless of walking limit settings,
since routing_delay is independent of walking constraints.
"""
function find_detour_combinations(
        model::AbstractSingleDetourModel,
        data::StationSelectionData
    )::Vector{Tuple{Int, Int, Int}}

    max_delay = model.routing_delay

    j_k_l_combinations = Vector{Tuple{Int, Int, Int}}()

    station_ids = Vector{Int}(data.stations.id)

    j_k_l_sets = Set{Tuple{Int, Int, Int}}()

    # we want to compute based on the travel time and the routing delay concerned, which sets of edges should have the detour constraint
    # the requirements is t(A -> B) + t(B -> C) \le t(A -> C) + \Delta
    # Going over permutations is probably less efficient than combinations
    # But i am lazy to retrieve the longest edge
    # Can probably be refactored
    for (a, b, c) in collect(permutations(station_ids, 3))
        # we want to check if we already included this combination
        # we sort the tuple to ensure no repetition
        if Tuple(sort([a, b, c])) in j_k_l_sets
            continue
        end

        # first we want to check if t(A -> B) + t(B -> C) > t(A -> C)
        # and that t(A -> C) is the longest edge
        # The idea here is that if t(A -> C) is the longest edge, we save he most time
        # Thus, we are only concerned with this specific ordering
        time_from_a_to_c = get_routing_cost(data, a, c)
        time_from_a_to_b = get_routing_cost(data, a, b)
        time_from_b_to_c = get_routing_cost(data, b, c)

        # if it is not the longest edge
        if !((time_from_a_to_c > time_from_b_to_c) && (time_from_a_to_c > time_from_a_to_b))
            continue
        end

        # if it is, we should also have the other condition
        # of t(A -> B) + t(B -> C) > t(A -> C)
        # just in case
        if (time_from_a_to_b + time_from_b_to_c) < time_from_a_to_c
            @error "Travel time does not obey triangle inequality: stations $a $b $c"
            continue
        end

        # now we check if it fulfils the delay function
        if (time_from_a_to_b + time_from_b_to_c) <= time_from_a_to_c + max_delay
            # we add it to the combinations
            # if it fulfils
            push!(j_k_l_combinations, (a, b, c))

            # we add it to the set to check duplicates
            # whih should be eliminated since the lengths will not be correct
            push!(j_k_l_sets, Tuple(sort([a, b, c])))
        end

    end

    return j_k_l_combinations
end


"""
    find_same_source_detour_combinations(
        model::AbstractSingleDetourModel,
        data::StationSelectionData
    ) -> Vector{Tuple{Int, Int, Int}}

Find valid same-source detour triplets (j, k, l) for pooling.

Same-source pooling (y_{t,jl,kl}): pools trip (j→l) with trip (k→l)
- Both trips end at destination l
- Vehicle goes j→k→l, picking up at k

Returns triplets (j, k, l) where the detour j→k→l is feasible.

Used in constraints:
- x_{od,t,jk,s} >= y_{t,jl,kl,s}  (need assignment on j→k edge)
- x_{od,t,jl,s} >= y_{t,jl,kl,s}  (need assignment on j→l edge)

This method works for TwoStageSingleDetourModel regardless of walking limit settings.
"""
function find_same_source_detour_combinations(
        model::AbstractSingleDetourModel,
        data::StationSelectionData
    )::Vector{Tuple{Int, Int, Int}}

    # Same source uses the base detour combinations directly
    return find_detour_combinations(model, data)
end


"""
    find_same_dest_detour_combinations(
        model::AbstractSingleDetourModel,
        data::StationSelectionData
    ) -> Vector{Tuple{Int, Int, Int, Int}}

Find valid same-destination detour quadruplets (j, k, l, t') for pooling.

Same-dest pooling (y_{t,jl,jk}): pools trip (j→l) with trip (j→k)
- Both trips start at source j
- Vehicle picks up both passengers at j, drops first at k, continues to l

Returns quadruplets (j, k, l, t') where:
- (j, k, l) is a valid detour triplet
- t' = floor(t(j→k) / time_window) is the time delta

Used in constraints:
- x_{od,t,jl,s} >= y_{t,jl,jk,s}     (need assignment on j→l edge at time t)
- x_{od,t+t',kl,s} >= y_{t,jl,jk,s}  (need assignment on k→l edge at time t+t')

This method works for TwoStageSingleDetourModel regardless of walking limit settings.
"""
function find_same_dest_detour_combinations(
        model::AbstractSingleDetourModel,
        data::StationSelectionData
    )::Vector{Tuple{Int, Int, Int, Int}}

    base_combinations = find_detour_combinations(model, data)
    time_window = floor(Int, model.time_window)

    same_dest_combinations = Vector{Tuple{Int, Int, Int, Int}}()

    for (j, k, l) in base_combinations
        travel_time_jk = get_routing_cost(data, j, k)
        time_delta = floor(Int, travel_time_jk / time_window)
        push!(same_dest_combinations, (j, k, l, time_delta))
    end

    return same_dest_combinations
end
