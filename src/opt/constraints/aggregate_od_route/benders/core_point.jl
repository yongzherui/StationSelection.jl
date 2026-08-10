"""
Constraint declarations for the Benders core-point LPs (`y_mw_cut.jl`'s `_y_master_core_point`,
`yz_mw_cut.jl`'s `_yz_joint_core_point`). Extracted with textually identical algebra -- see
`variables/aggregate_od_route/benders/dual_lp.jl`'s module docstring.
"""

"""
    add_relaxed_station_budget_constraints!(m, y, l, endpoint_rows)

The permanent structural region shared by `BendersY`'s and `BendersYZ`'s core-point LPs:
`sum(y) == l` plus one `sum_{j in row} y_j >= 1` row per permanent endpoint row
(`_restricted_mw_endpoint_rows`).
"""
function add_relaxed_station_budget_constraints!(m::JuMP.Model, y, l::Int, endpoint_rows)
    @constraint(m, sum(y) == l)
    for row in endpoint_rows
        @constraint(m, sum(y[j] for j in row) >= 1.0)
    end
    return nothing
end

"""
    add_big_m_chain_variables_and_constraints!(m, y, chains) -> (z, z_slack_exprs)

`BendersYZ`-only: reproduces `_endpoint_big_m_variable!`'s exact row family for every chain
(tie-break-adjusted costs, NOT reapplied here -- `chains`' costs are already tie-broken by
`_restricted_mw_chains`), for the joint `(y,z)` core-point LP. Also returns every non-equality
row touching `z` as a `>= 0` slack expression, for the caller's affine-hull probe.
"""
function add_big_m_chain_variables_and_constraints!(m::JuMP.Model, y, chains)
    z = Dict{Any, Vector{VariableRef}}()
    z_slack_exprs = AffExpr[]
    for (key, chain) in chains
        n_chain = length(chain.stations)
        zvar = @variable(m, [1:n_chain], lower_bound = 0.0, upper_bound = 1.0)
        z[key] = zvar
        @constraint(m, sum(zvar) == 1.0)
        tb_costs = chain.costs
        max_cost = maximum(tb_costs)
        selected_cost = sum(tb_costs[idx] * zvar[idx] for idx in 1:n_chain)
        for (idx, station) in enumerate(chain.stations)
            big_m = max_cost - tb_costs[idx]
            @constraint(m, zvar[idx] <= y[station])
            @constraint(m, selected_cost <= tb_costs[idx] + big_m * (1.0 - y[station]))
            cheaper_sum = sum(y[chain.stations[p]] for p in 1:(idx - 1); init = 0.0)
            @constraint(m, zvar[idx] >= y[station] - cheaper_sum)

            push!(z_slack_exprs, 1.0 * zvar[idx])                                             # zvar >= 0
            push!(z_slack_exprs, 1.0 - zvar[idx])                                              # zvar <= 1
            push!(z_slack_exprs, y[station] - zvar[idx])                                       # zvar <= y[station]
            push!(z_slack_exprs, tb_costs[idx] + big_m * (1.0 - y[station]) - selected_cost)    # Big-M ordering
            push!(z_slack_exprs, zvar[idx] - y[station] + cheaper_sum)                          # nearest-open lower bound
        end
    end
    return z, z_slack_exprs
end

"""
    add_core_point_slack_constraints!(m, y, endpoint_rows, lb_slack_max, ub_slack_max, endpoint_slack_max, affine_hull_tol) -> delta

Section B2's shared `delta` variable and the normalized max-min-slack rows over `y`'s own
bound/endpoint rows, common to both `_y_master_core_point` (BendersY) and `_yz_joint_core_point`
(BendersYZ, which adds its own `z`-row constraints via `add_core_point_z_slack_constraints!`
afterward, before the shared `optimize!` call).
"""
function add_core_point_slack_constraints!(
    m::JuMP.Model, y, endpoint_rows, lb_slack_max, ub_slack_max, endpoint_slack_max, affine_hull_tol::Float64,
)
    delta = @variable(m, 0 <= delta <= 1)
    n = length(y)
    for j in 1:n
        lb_slack_max[j] > affine_hull_tol && @constraint(m, y[j] >= delta * lb_slack_max[j])
        ub_slack_max[j] > affine_hull_tol && @constraint(m, 1.0 - y[j] >= delta * ub_slack_max[j])
    end
    for (i, row) in enumerate(endpoint_rows)
        endpoint_slack_max[i] > affine_hull_tol || continue
        @constraint(m, sum(y[j] for j in row) - 1.0 >= delta * endpoint_slack_max[i])
    end
    return delta
end

"""
    add_core_point_z_slack_constraints!(m, delta, z_slack_exprs, z_slack_max, affine_hull_tol)

`BendersYZ`-only continuation of `add_core_point_slack_constraints!`: the normalized
max-min-slack rows for every `z`-touching row collected by `add_big_m_chain_variables_and_constraints!`.
"""
function add_core_point_z_slack_constraints!(m::JuMP.Model, delta, z_slack_exprs, z_slack_max, affine_hull_tol::Float64)
    for (i, expr) in enumerate(z_slack_exprs)
        z_slack_max[i] > affine_hull_tol || continue
        @constraint(m, expr >= delta * z_slack_max[i])
    end
    return nothing
end
