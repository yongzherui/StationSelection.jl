"""
Flow activation constraint creation functions.

Links f_flow variables to assignment variables x, ensuring
f_flow[s][(j,k)] = 1 iff any OD pair uses route (j,k) in scenario s.

Both a lower bound (f_flow ≥ x for each OD pair) and an upper bound
(f_flow ≤ Σ x over all OD pairs sharing that route) are added, so that
f_flow tracks x exactly regardless of the flow_regularization_weight.

Supported mappings:
- ClusteringTwoStageODMap (ClusteringTwoStageODModel with flow_regularization_weight; sparse x only)
"""

using JuMP

export add_flow_activation_constraints!

"""
    add_flow_activation_constraints!(m, data, mapping::ClusteringTwoStageODMap; variable_reduction) -> Int

Links f_flow to x with both lower and upper bounds:
    f_flow[s][(j,k)] ≥ x[s][od_idx][idx]            (one per OD pair, sparse x only)
    f_flow[s][(j,k)] ≤ Σ_{od} x[s][od][idx]         (one per (s,j,k))

Requires variable_reduction=true (sparse x only).
Used by: ClusteringTwoStageODModel (when flow_regularization_weight is set)
"""
function add_flow_activation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        variable_reduction::Bool=true
    )::Int
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]
    f_flow = m[:f_flow]

    x_terms = Dict{Tuple{Int,Int,Int}, Vector{VariableRef}}()

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (idx, (j, k)) in enumerate(valid_pairs)
                x_var = x[s][od_idx][idx]
                @constraint(m, f_flow[s][(j, k)] >= x_var)
                push!(get!(x_terms, (s, j, k), VariableRef[]), x_var)
            end
        end
    end

    for ((s, j, k), xs) in x_terms
        @constraint(m, f_flow[s][(j, k)] <= sum(xs))
    end

    return _total_num_constraints(m) - before
end
