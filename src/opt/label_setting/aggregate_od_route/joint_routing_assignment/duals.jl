"""
Master-problem-facing dual extraction for the joint routing+assignment CG master, keyed
by demand group `(s,o,d)` (and `((s,o,d),j)` for the linking rows) instead of a
synthetic passenger id.

`alpha_p >= 0` from the `>=` coverage rows; `gamma^O/gamma^D >= 0` as the *negated* duals
of the `<=` linking rows, so that `rc_theta = cost - sum alpha + sum gamma^O + sum gamma^D`
matches the pricer's `beta*(tau+repo) - sum rho` sign convention directly.
"""

export extract_joint_routing_assignment_duals

function extract_joint_routing_assignment_duals(m::JuMP.Model)
    alpha = Dict{NTuple{3, Int}, Float64}()
    for (key3, con) in m[:joint_routing_assignment_coverage]
        alpha[key3] = dual(con)
    end
    gamma_o = Dict{Tuple{NTuple{3, Int}, Int}, Float64}()
    for (key, con) in m[:joint_routing_assignment_pickup_link]
        gamma_o[key] = -dual(con)
    end
    gamma_d = Dict{Tuple{NTuple{3, Int}, Int}, Float64}()
    for (key, con) in m[:joint_routing_assignment_dropoff_link]
        gamma_d[key] = -dual(con)
    end
    return alpha, gamma_o, gamma_d
end

# CGSolver hook (opt/solvers/cg_solver.jl) -- dispatches on mapping::AggregateODRouteMap
# so it doesn't collide with the generic fallback stub's identical (BuildResult, Any,
# JuMP.Model) signature.
extract_duals(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model) =
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
    alpha::Dict{NTuple{3, Int}, Float64},
    gamma_o::Dict{Tuple{NTuple{3, Int}, Int}, Float64},
    gamma_d::Dict{Tuple{NTuple{3, Int}, Int}, Float64};
    atol::Float64=1e-5,
)
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = joint_routing_assignment_column_cost(m, data, mapping, column)
    s = Int(column.metadata["scenario"])
    omega = mapping.Omega_s[s]
    for (od_idx, j, k) in column.assignments
        o, d = omega[od_idx]
        key3 = (s, o, d)
        master_rc -= get(alpha, key3, 0.0)
        master_rc += get(gamma_o, (key3, j), 0.0)
        master_rc += get(gamma_d, (key3, k), 0.0)
    end
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
