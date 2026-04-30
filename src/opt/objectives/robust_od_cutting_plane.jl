"""
Objective and cut helpers for cutting-plane robust solves.
"""

using JuMP

export set_robust_total_demand_cap_cp_objective!

function _robust_assignment_pair_cost(
        data::StationSelectionData,
        o::Int,
        d::Int,
        j::Int,
        k::Int;
        in_vehicle_time_weight::Float64
    )::Float64
    return get_walking_cost(data, o, j) +
           get_walking_cost(data, k, d) +
           in_vehicle_time_weight * get_routing_cost(data, j, k)
end


function _build_robust_assignment_cost_expression(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap,
        s::Int,
        od_idx::Int;
        in_vehicle_time_weight::Float64
    )
    x_od = get(m[:x][s], od_idx, VariableRef[])
    isempty(x_od) && return nothing

    o, d = mapping.Omega_s[s][od_idx]
    valid_pairs = get_valid_jk_pairs(mapping, o, d)
    return @expression(
        m,
        sum(
            _robust_assignment_pair_cost(
                data, o, d, j, k;
                in_vehicle_time_weight=in_vehicle_time_weight
            ) * x_od[pair_idx]
            for (pair_idx, (j, k)) in enumerate(valid_pairs)
        )
    )
end


"""
    set_robust_total_demand_cap_cp_objective!(m, data, mapping)

Set the cutting-plane master objective:

    min Σ_s η_s
"""
function set_robust_total_demand_cap_cp_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    eta = m[:eta]
    S = n_scenarios(data)
    @objective(m, Min, sum(eta[s] for s in 1:S))
    return nothing
end


function _add_robust_cutting_plane_cut!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap,
        s::Int,
        q_wc::Dict{Int, Float64};
        in_vehicle_time_weight::Float64
    )
    eta = m[:eta]
    expr = AffExpr(0.0)

    for (od_idx, q_val) in q_wc
        q_val > 0 || continue
        cost_expr = _build_robust_assignment_cost_expression(
            m, data, mapping, s, od_idx;
            in_vehicle_time_weight=in_vehicle_time_weight
        )
        isnothing(cost_expr) && continue
        add_to_expression!(expr, q_val, cost_expr)
    end

    return @constraint(m, eta[s] >= expr)
end
