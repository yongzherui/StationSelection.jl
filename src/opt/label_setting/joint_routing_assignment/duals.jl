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
    # The walking cap's dual is not part of the returned tuple: it enters the pricer only as
    # a surcharge on the walk weight, so it is stashed on `m` for
    # `joint_routing_assignment_pricing_walk_weight`. Refreshed here because this is the
    # one call per CG iteration that reads the master's duals, so every pricing,
    # certification and verification call of the iteration sees the same `mu`.
    if haskey(m, :joint_routing_assignment_walk_cap_row)
        mu = max(0.0, -dual(m[:joint_routing_assignment_walk_cap_row]))   # `<=` row in a Min
        m[:joint_routing_assignment_walk_cap_dual] = mu
        push!(m[:joint_routing_assignment_walk_cap_dual_history], mu)
    end
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
    joint_routing_assignment_pricing_walk_weight(m) -> Float64

The weight the pricer charges per walking second: `walk_cost_weight + mu`, `mu >= 0` the
walking cap's current dual (0 without a cap). With it every pricing path keeps the reduced
cost `rho_pjk = alpha_p - gamma^O_pj - gamma^D_pk - (walk_cost_weight + mu) * walk_pjk`
exact, and so do the relaxed-cluster bounds built from the same candidates. The column's
objective coefficient (`joint_routing_assignment_column_cost`) still uses
`walk_cost_weight` alone: `mu` is a dual price, not a cost.
"""
joint_routing_assignment_pricing_walk_weight(m::JuMP.Model) =
    Float64(m[:joint_routing_assignment_walk_cost_weight]) +
    Float64(get(m.obj_dict, :joint_routing_assignment_walk_cap_dual, 0.0))

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
    mu = Float64(get(m.obj_dict, :joint_routing_assignment_walk_cap_dual, 0.0))
    mu > 0 && (master_rc += mu * joint_routing_assignment_column_walk(data, mapping, column))
    s = Int(column.metadata["scenario"])
    for (p, j, k) in column.assignments
        key2 = (s, p)
        master_rc -= get(alpha, key2, 0.0)
        master_rc += get(gamma_o, (key2, j), 0.0)
        master_rc += get(gamma_d, (key2, k), 0.0)
    end
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
