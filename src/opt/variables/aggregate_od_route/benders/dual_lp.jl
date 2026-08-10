"""
Variable declarations for the Benders core-point / dual-completion LPs
(`y_mw_cut.jl`'s `_y_master_core_point`/`_restricted_mw_completion_lp`, `yz_mw_cut.jl`'s
`_yz_joint_core_point`/`_yz_completion_lp`). Extracted with textually identical algebra --
these LPs' correctness depends on exact row/column shape and documented past
tie-break/degeneracy bugs, so nothing here re-derives anything, it only moves the same
declarations into named functions.
"""

"""
    add_relaxed_station_selection_variables!(m, n) -> y

Continuous `[0,1]` relaxation of station selection for a core-point/dual LP -- deliberately
`NOT` `add_station_selection_variables!` (`variables/base.jl`), which declares `y::Bin`; these
LPs need a genuine relaxation, not the master's binary `y`.
"""
function add_relaxed_station_selection_variables!(m::JuMP.Model, n::Int)
    return @variable(m, 0 <= y[1:n] <= 1)
end

"""
    add_x_dual_variables!(m, requests, feasible_pairs) -> (alpha, rhoO, rhoD, sigma)

The `x`-linking dual block shared by `BendersY`'s and `BendersYZ`'s completion LPs: `alpha`
free (one per request), `rhoO`/`rhoD`/`sigma` >= 0 (one per non-walk-only feasible pair).
"""
function add_x_dual_variables!(
    m::JuMP.Model,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
)
    alpha = Dict{NTuple{3, Int}, VariableRef}(p => @variable(m) for p in requests)
    rhoO = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    rhoD = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    sigma = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        rhoO[(p, pair)] = @variable(m, lower_bound = 0.0)
        rhoD[(p, pair)] = @variable(m, lower_bound = 0.0)
        sigma[(p, pair)] = @variable(m, lower_bound = 0.0)
    end
    return alpha, rhoO, rhoD, sigma
end

"""
    add_chain_dual_variables!(m, chains) -> (lambda, mu, nu)

`BendersY`-only chain dual block (relates `z`'s Big-M chain structure back to `y`, since `z`
lives in the subproblem, not the master, for this decomposition): `lambda` free (one per
chain), `mu`/`nu` >= 0 (one per chain station).
"""
function add_chain_dual_variables!(m::JuMP.Model, chains)
    lambda = Dict{Any, VariableRef}(key => @variable(m) for key in keys(chains))
    mu = Dict{Tuple{Any, Int}, VariableRef}()
    nu = Dict{Tuple{Any, Int}, VariableRef}()
    for (key, chain) in chains, idx in eachindex(chain.stations)
        mu[(key, idx)] = @variable(m, lower_bound = 0.0)
        nu[(key, idx)] = @variable(m, lower_bound = 0.0)
    end
    return lambda, mu, nu
end
