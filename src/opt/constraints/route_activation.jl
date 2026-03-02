"""
Route activation constraint creation functions.

Links f_flow variables to assignment variables x, ensuring
f_flow[s][(j,k)] = 1 whenever any OD pair uses route (j,k) in scenario s.

Supported mappings:
- CorridorTwoStageODMap  (XCorridorWithFlowRegularizerModel; sparse or dense x)
- ClusteringTwoStageODMap (ClusteringTwoStageODModel with flow_regularization_weight; sparse x only)
"""

using JuMP

export add_route_activation_constraints!

"""
    add_route_activation_constraints!(m, data, mapping::CorridorTwoStageODMap; variable_reduction) -> Int

Links sparse f_flow to x:
    f_flow[s][(j,k)] ≥ x[s][od_idx][idx]   (sparse x)
    f_flow[s][(j,k)] ≥ x[s][od_idx][j,k]   (dense x)
for all (o,d)∈Ω_s and all (j,k) in valid_pairs(o,d).

Used by: XCorridorWithFlowRegularizerModel
"""
function add_route_activation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap;
        variable_reduction::Bool=true
    )::Int
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]
    f_flow = m[:f_flow]
    use_sparse = variable_reduction && has_walking_distance_limit(mapping)

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (idx, (j, k)) in enumerate(valid_pairs)
                if use_sparse
                    @constraint(m, f_flow[s][(j, k)] >= x[s][od_idx][idx])
                else
                    @constraint(m, f_flow[s][(j, k)] >= x[s][od_idx][j, k])
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end

"""
    add_route_activation_constraints!(m, data, mapping::ClusteringTwoStageODMap; variable_reduction) -> Int

Links sparse f_flow to x for ClusteringTwoStageODModel:
    f_flow[s][(j,k)] ≥ x[s][od_idx][idx]   (sparse x)
for all (o,d)∈Ω_s and all (j,k) in valid_pairs(o,d).

Requires variable_reduction=true (sparse x only).
Used by: ClusteringTwoStageODModel (when flow_regularization_weight is set)
"""
function add_route_activation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        variable_reduction::Bool=true
    )::Int
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]
    f_flow = m[:f_flow]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (idx, (j, k)) in enumerate(valid_pairs)
                @constraint(m, f_flow[s][(j, k)] >= x[s][od_idx][idx])
            end
        end
    end

    return _total_num_constraints(m) - before
end
