"""
Fixed-decision + shared `lambda` variable builders for the Benders subproblem LPs
(`_build_nearest_open_y_subproblem_lp`/`_build_xy_route_subproblem_lp`/
`_build_yz_route_subproblem_lp`/`_build_yzh_route_subproblem_lp`,
`src/opt/optimize/aggregate_od_route/benders/{y,xy,yz,yzh}.jl`). Each `add_fixed_*_variables!`
declares a continuous relaxation of the real master decision and fixes it via an equality
constraint (not `JuMP.fix`) so its dual is a valid Benders subgradient.
"""

"""
    add_fixed_station_selection_variables!(m, data, y_hat) -> (y, fix_cons)

BendersY's subproblem `y`: a fresh `[0,1]` relaxation of station selection, fixed to `y_hat`
station-by-station.
"""
function add_fixed_station_selection_variables!(
    m::JuMP.Model,
    data::StationSelectionData,
    y_hat::Vector{Float64},
)
    y = @variable(m, 0 <= y[1:data.n_stations] <= 1)
    fix_cons = Dict(j => @constraint(m, y[j] == y_hat[j]) for j in 1:data.n_stations)
    return y, fix_cons
end

"""
    add_fixed_pair_assignment_variables!(m, requests, feasible_pairs, x_hat) -> (x, fix_cons)

BendersXY's subproblem `x`: fixed fully (unlike BendersY's `y`, which leaves `x` free) to
`x_hat`, defaulting to `0.0` for any `(request, pair)` not present in `x_hat`.
"""
function add_fixed_pair_assignment_variables!(
    m::JuMP.Model,
    requests,
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    x_hat::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
)
    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    fix_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for request in requests, pair in feasible_pairs[request]
        key = (request, pair)
        x[key] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
        fix_cons[key] = @constraint(m, x[key] == get(x_hat, key, 0.0))
    end
    return x, fix_cons
end

"""
    add_fixed_endpoint_chain_variables!(m, z_hat, needed_keys) -> (z_by_key, fix_cons)

BendersYZ's subproblem `z`: builds and fixes only the chain keys in `needed_keys` (typically
the pickup/dropoff keys this cut group's own `requests` actually resolve to via
`_sorted_endpoint_chain`), not every key in `z_hat` -- `z_hat` itself is the master's *full*
`nearest_endpoint_chain_cache` (every physical endpoint in the whole problem), and building a
`z` variable per unused key would needlessly inflate the subproblem LP. Duplicate keys in
`needed_keys` are built once. Throws if a needed key has no `z_hat` entry.

`z_hat`'s key type (`_AggregateODRouteEndpointChainKey`) is defined in
`constraints/aggregate_od_route/core.jl`, included after `variables.jl` -- left untyped here to
avoid an include-order dependency (dispatch doesn't need it: this function has a unique name).
"""
function add_fixed_endpoint_chain_variables!(
    m::JuMP.Model,
    z_hat::AbstractDict,
    needed_keys,
)
    z_by_key = Dict{keytype(z_hat), Vector{VariableRef}}()
    fix_cons = Dict{Tuple{keytype(z_hat), Int}, ConstraintRef}()
    for key in needed_keys
        haskey(z_by_key, key) && continue
        haskey(z_hat, key) || throw(ArgumentError(
            "BendersYZ subproblem: no master z_hat entry for chain key $(key)"
        ))
        values = z_hat[key]
        n = length(values)
        zvar = @variable(m, [1:n], lower_bound = 0.0, upper_bound = 1.0)
        z_by_key[key] = zvar
        for i in 1:n
            fix_cons[(key, i)] = @constraint(m, zvar[i] == values[i])
        end
    end
    return z_by_key, fix_cons
end

"""
    add_fixed_physical_pair_variables!(m, physical_pairs, feasible_pairs_by_p, h_hat) -> (h, fix_cons)

BendersYZH's subproblem `h`: fixed fully per scenario-compressed physical pair, mirroring
`add_fixed_pair_assignment_variables!`'s shape one level up (physical pair instead of a single
request). `physical_pairs`/`feasible_pairs_by_p` are already scoped to the current cut group.
"""
function add_fixed_physical_pair_variables!(
    m::JuMP.Model,
    physical_pairs::Vector{Tuple{Int, Int}},
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    h_hat::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, Float64},
)
    h = Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, VariableRef}()
    fix_cons = Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for p in physical_pairs, pair in feasible_pairs_by_p[p]
        key = (p, pair)
        h[key] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
        fix_cons[key] = @constraint(m, h[key] == get(h_hat, key, 0.0))
    end
    return h, fix_cons
end

"""
    add_benders_lambda_variables!(m, columns, n_scenarios; binary=false) -> lambda

Shared route-activation variable for all four subproblem builders: `lambda[idx, s] >= 0`
(continuous) by default, or `Bin` when `binary=true` (only `BendersY`'s subproblem exposes this,
via `BendersSubproblemModel.lambda_binary`).
"""
function add_benders_lambda_variables!(
    m::JuMP.Model,
    columns::Vector{AggregateODRouteColumn},
    n_scenarios::Int;
    binary::Bool=false,
)
    return binary ?
        @variable(m, [1:length(columns), 1:n_scenarios], Bin) :
        @variable(m, [1:length(columns), 1:n_scenarios], lower_bound = 0.0)
end
