"""
Objective function for ClusteringTwoStageODModel.

Contains objective for:
- ClusteringTwoStageODModel: walking + routing costs (no pooling)
"""

using JuMP

export set_clustering_od_objective!
export set_clustering_od_flow_regularizer_objective!


"""
    set_clustering_od_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        in_vehicle_time_weight::Float64=1.0,
    )

Set the minimization objective for ClusteringTwoStageODModel.

Objective:
    min Σ_s Σ_{(o,d)∈Ω_s} Σ_{j,k} (d^origin_{oj} + d^dest_{dk} + w_ivt·c_{jk}) · x[s][od_idx][pair_idx]

Where:
- x[s][od_idx][pair_idx] = integer assigned demand for that OD and valid station pair
- d^origin_{oj} = walking cost from origin o to pickup station j
- d^dest_{dk} = walking cost from dropoff station k to destination d
- c_{jk} = routing cost from station j to k
- w_ivt (in_vehicle_time_weight) = weight for routing costs
"""
function set_clustering_od_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        in_vehicle_time_weight::Float64=1.0
    )

    S = n_scenarios(data)
    x = m[:x]

    @objective(m, Min,
        sum(
            (
                get_walking_cost(data, o, j) +
                get_walking_cost(data, k, d) +
                in_vehicle_time_weight * get_routing_cost(data, j, k)
            ) * x[s][od_idx][idx]
            for s in 1:S
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            for (idx, (j, k)) in enumerate(get_valid_jk_pairs(mapping, o, d))
        )
    )

    return nothing
end


"""
    set_clustering_od_flow_regularizer_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        in_vehicle_time_weight::Float64=1.0,
        flow_regularization_weight::Float64=1.0,
    )

Extends the base clustering-OD objective with a route-activation penalty:

    min  Σ_s Σ_{od} Σ_{j,k} cost_{odjk} · x[s][od][pair]
       + μ · Σ_s Σ_{(j,k)} c_{jk} · f_flow[s][(j,k)]

μ = flow_regularization_weight penalises distinct (j,k) route segments per scenario,
weighted by routing time. Requires f_flow variables from add_flow_variables!.
"""
function set_clustering_od_flow_regularizer_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        in_vehicle_time_weight::Float64=1.0,
        flow_regularization_weight::Float64=1.0
    )
    set_clustering_od_objective!(m, data, mapping;
        in_vehicle_time_weight=in_vehicle_time_weight)

    S = n_scenarios(data)
    f_flow = m[:f_flow]
    route_penalty = @expression(m,
        flow_regularization_weight * sum(
            get_routing_cost(data, j, k) * v
            for s in 1:S
            for ((j, k), v) in f_flow[s]
        )
    )

    obj = objective_function(m)
    @objective(m, Min, obj + route_penalty)
    return nothing
end
