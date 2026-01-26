using Combinatorics

function find_detour_combinations(
        model::TwoStageSingleDetourModel,
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

