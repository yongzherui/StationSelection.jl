"""
Detour pooling variable creation functions for station selection optimization models.

These functions add detour pooling decision variables for same-source and
same-destination pooling.

Used by: TwoStageSingleDetourModel (with or without walking limits)
"""

using JuMP

export add_detour_variables!


"""
    add_detour_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )

Add detour pooling variables for same-source and same-destination pooling.

When walking limits are enabled (has_walking_distance_limit(mapping) is true),
only creates variables for feasible detour combinations based on
mapping.feasible_same_source and mapping.feasible_same_dest.

Variables:
- u[s][t] = Vector{VariableRef} for same-source detours
- v[s][t] = Vector{VariableRef} for same-dest detours

The mapping from local variable index to Xi index is stored in:
- mapping.feasible_same_source[s][t] for same-source (when walking limits enabled)
- mapping.feasible_same_dest[s][t] for same-dest (when walking limits enabled)

Used by: TwoStageSingleDetourModel
"""
function add_detour_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    n_same_source = length(Xi_same_source)

    u = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]
    v = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]

    use_sparse = has_walking_distance_limit(mapping)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            # Same-source variables
            if use_sparse
                feasible_u_indices = get(mapping.feasible_same_source[s], time_id, Int[])
                if !isempty(feasible_u_indices) && length(od_vector) > 1
                    u[s][time_id] = @variable(m, [1:length(feasible_u_indices)], Bin)
                else
                    u[s][time_id] = VariableRef[]
                end
            else
                # No walking limits - all triplets feasible
                if length(od_vector) > 1 && n_same_source > 0
                    u[s][time_id] = @variable(m, [1:n_same_source], Bin)
                else
                    u[s][time_id] = VariableRef[]
                end
            end

            # Same-dest variables
            if use_sparse
                feasible_v_indices = get(mapping.feasible_same_dest[s], time_id, Int[])
                if !isempty(feasible_v_indices)
                    v[s][time_id] = @variable(m, [1:length(feasible_v_indices)], Bin)
                else
                    v[s][time_id] = VariableRef[]
                end
            else
                # No walking limits - check time validity only
                valid_indices = Int[]
                for (idx, (_, _, _, time_delta)) in enumerate(Xi_same_dest)
                    future_time_id = time_id + time_delta
                    if haskey(mapping.Omega_s_t[s], future_time_id)
                        push!(valid_indices, idx)
                    end
                end

                if !isempty(valid_indices)
                    v[s][time_id] = @variable(m, [1:length(valid_indices)], Bin)
                else
                    v[s][time_id] = VariableRef[]
                end
            end
        end
    end

    m[:u] = u
    m[:v] = v

    return JuMP.num_variables(m) - before
end

