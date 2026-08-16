"""
Master-problem-facing dual extraction for the joint routing+assignment CG master, keyed
by demand group `(s,p)` (and `((s,p),j)` for the linking rows) instead of a synthetic
passenger id.

`alpha_p >= 0` from the `>=` coverage rows; `gamma^O/gamma^D >= 0` as the *negated* duals
of the `<=` linking rows, so that `rc_theta = cost - sum alpha + sum gamma^O + sum gamma^D`
matches the pricer's `beta*(tau+repo) - sum rho` sign convention directly.
"""

export extract_joint_routing_assignment_duals

function extract_joint_routing_assignment_duals(m::JuMP.Model)
    alpha = Dict{Tuple{Int, Int}, Float64}()
    for (key2, con) in m[:joint_routing_assignment_coverage]
        alpha[key2] = dual(con)
    end
    gamma_o = Dict{Tuple{Tuple{Int, Int}, Int}, Float64}()
    for (key, con) in m[:joint_routing_assignment_pickup_link]
        gamma_o[key] = -dual(con)
    end
    gamma_d = Dict{Tuple{Tuple{Int, Int}, Int}, Float64}()
    for (key, con) in m[:joint_routing_assignment_dropoff_link]
        gamma_d[key] = -dual(con)
    end
    return alpha, gamma_o, gamma_d
end

# CGSolver hook real logic (dispatched from
# optimize/aggregate_od_route/column_generation/dispatch.jl, which disambiguates from
# AggregateODRouteBaseFormulation's own extract_duals by formulation type, since both
# share mapping::AggregateODRouteMap).
_aggregate_od_route_extract_duals(::AggregateODRouteJointRoutingAssignmentFormulation, build_result, mapping, m::JuMP.Model) =
    extract_joint_routing_assignment_duals(m)

"""
Cross-check that the pricer's reported reduced cost equals the one implied by the
master's own duals and the column's true objective coefficient. Catches any drift
between the two formulations (a wrong dual sign, a missing linking row, a walking-cost
weight applied on one side only) at the moment it happens instead of as a silently wrong
LP bound.
"""
function _verify_joint_routing_assignment_master_reduced_cost(
    column::JointRoutingAssignmentRouteColumn,
    m::JuMP.Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    alpha::Dict{Tuple{Int, Int}, Float64},
    gamma_o::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
    gamma_d::Dict{Tuple{Tuple{Int, Int}, Int}, Float64};
    atol::Float64=1e-5,
)
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = joint_routing_assignment_column_cost(m, data, mapping, column)
    s = Int(column.metadata["scenario"])
    for (p, j, k) in column.assignments
        key2 = (s, p)
        master_rc -= get(alpha, key2, 0.0)
        master_rc += get(gamma_o, (key2, j), 0.0)
        master_rc += get(gamma_d, (key2, k), 0.0)
    end
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
