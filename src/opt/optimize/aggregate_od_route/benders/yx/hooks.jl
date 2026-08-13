"""
`BendersSolver` hooks for `AggregateODRouteBendersYXFormulation`'s master (see
`build_master.jl`) and subproblem (`subproblem.jl`). Dispatched on
`mapping::AggregateODRouteMap` -- the only Benders-decomposed formulation currently
using that map type, matching `CGSolver`'s identical convention
(`opt/solvers/cg_solver.jl`); a second `AggregateODRouteMap`-based Benders decomposition
would need a finer disambiguator than map type alone, not a concern until one exists.

Standard LP-duality Benders: the subproblem's `x`/`theta` are continuous, `y` is fixed
via an explicit equality constraint (`add_fixed_station_selection_variables!`), so
`JuMP.dual` of that constraint is exactly `d(subproblem objective)/d(y_hat[j])` by LP
sensitivity/the envelope theorem -- a valid subgradient of the (convex, in `y`)
subproblem value function, giving a provably valid optimality cut. Not yet verified
against an instance where the route-covering LP relaxation has an integrality gap
(fractional `theta` splitting coverage of several station pairs more cheaply than any
integral combination) -- untested risk, not a known issue; if ever observed (the
`test_aggregate_od_route_benders_yx.jl` cross-check against
`AggregateODRouteBaseFormulation`+`DirectMIPSolver` would surface it as a lower,
unachieved objective), the fix is to verify each subproblem's LP objective against its
own MIP objective and use the MIP value for the convergence check while keeping the LP
duals for the cut's slope, not a full redesign.
"""

function extract_incumbent(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model)
    return round.(Int, JuMP.value.(m[:y]))
end

function solve_subproblem(
    build_result::BuildResult,
    mapping::AggregateODRouteMap,
    m::JuMP.Model,
    incumbent,
    solver::BendersSolver,
)
    data = m[:aggregate_od_route_benders_yx_data]
    formulation = m[:aggregate_od_route_benders_yx_formulation]
    groups = m[:aggregate_od_route_benders_yx_groups]
    y_hat = Float64.(incumbent)
    n = data.n_stations

    results = Dict{Int, NamedTuple{(:objective, :duals), Tuple{Float64, Vector{Float64}}}}()
    for (group_id, scenarios) in groups
        total_obj = 0.0
        total_duals = zeros(Float64, n)
        for s in scenarios
            sub, _y_sub, fix_cons = _build_benders_yx_subproblem(data, mapping, formulation, y_hat, s)
            optimize!(sub)
            JuMP.termination_status(sub) == MOI.OPTIMAL || throw(ArgumentError(
                "AggregateODRouteBendersYXFormulation subproblem (scenario $(s)) failed " *
                "with status $(JuMP.termination_status(sub))"
            ))
            total_obj += JuMP.objective_value(sub)
            for j in 1:n
                total_duals[j] += JuMP.dual(fix_cons[j])
            end
        end
        results[group_id] = (objective = total_obj, duals = total_duals)
    end
    return results
end

function benders_converged(
    build_result::BuildResult,
    mapping::AggregateODRouteMap,
    m::JuMP.Model,
    subproblem_result,
    solver::BendersSolver,
)::Bool
    theta = m[:aggregate_od_route_benders_yx_theta]
    for (group_id, res) in subproblem_result
        JuMP.value(theta[group_id]) >= res.objective - solver.optimality_tol || return false
    end
    return true
end

function add_benders_cut!(
    build_result::BuildResult,
    mapping::AggregateODRouteMap,
    m::JuMP.Model,
    subproblem_result,
    solver::BendersSolver,
)
    y = m[:y]
    theta = m[:aggregate_od_route_benders_yx_theta]
    n = length(y)
    y_hat = JuMP.value.(y)
    for (group_id, res) in subproblem_result
        rhs_const = res.objective - sum(res.duals[j] * y_hat[j] for j in 1:n)
        @constraint(m, theta[group_id] >= rhs_const + sum(res.duals[j] * y[j] for j in 1:n))
    end
    return nothing
end
