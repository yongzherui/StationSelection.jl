"""
Constraint declarations for the Benders dual-completion LPs (`y_mw_cut.jl`'s
`_restricted_mw_completion_lp`, `yz_mw_cut.jl`'s `_yz_completion_lp`). Extracted with textually
identical algebra -- see `variables/aggregate_od_route/benders/dual_lp.jl`'s module docstring.
"""

"""
    add_x_dual_feasibility_constraints!(m, data, base, requests, feasible_pairs, pi_full, alpha, rhoO, rhoD, sigma) -> x_dual_cons

Row shared by `BendersY`'s and `BendersYZ`'s completion LPs (unaffected by which variable the
decomposition fixes): `alpha[p] - rhoO - rhoD + sigma - pi_full <= c_walk` per non-walk-only
feasible pair.
"""
function add_x_dual_feasibility_constraints!(
    m::JuMP.Model,
    data::StationSelectionData,
    base::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    pi_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
    alpha,
    rhoO,
    rhoD,
    sigma,
)
    x_dual_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        c_walk = _assignment_pair_cost(data, p, pair; weight = base.walk_cost_weight)
        pi_val = pi_full[(p, pair)]
        x_dual_cons[(p, pair)] =
            @constraint(m, alpha[p] - rhoO[(p, pair)] - rhoD[(p, pair)] + sigma[(p, pair)] - pi_val <= c_walk)
    end
    return x_dual_cons
end

"""
    add_z_dual_feasibility_constraints!(m, chains, requests, feasible_pairs, pk_key_of, dp_key_of, pk_rank_of, dp_rank_of, lambda, mu, nu, rhoO, rhoD, sigma) -> z_dual_cons

`BendersY`-only: `lambda - mu - cost*sum(nu over chain) + role-specific (rhoO/rhoD - sigma) <= 0`
per chain station, relating the chain dual block back to the `x`-linking dual block. Precomputes
`(chain key, rank) -> [(request, pair)]` reverse indices to avoid an
`O(chains * candidates * requests * pairs)` nested scan.
"""
function add_z_dual_feasibility_constraints!(
    m::JuMP.Model,
    chains,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    pk_key_of::Dict{NTuple{3, Int}, Any},
    dp_key_of::Dict{NTuple{3, Int}, Any},
    pk_rank_of::Dict{NTuple{3, Int}, Dict{Int, Int}},
    dp_rank_of::Dict{NTuple{3, Int}, Dict{Int, Int}},
    lambda,
    mu,
    nu,
    rhoO,
    rhoD,
    sigma,
)
    pickup_terms = Dict{Tuple{Any, Int}, Vector{Tuple{NTuple{3, Int}, Tuple{Int, Int}}}}()
    dropoff_terms = Dict{Tuple{Any, Int}, Vector{Tuple{NTuple{3, Int}, Tuple{Int, Int}}}}()
    for p in requests, pair in feasible_pairs[p]
        is_walk_only_pair(pair) && continue
        j, k = pair
        push!(get!(pickup_terms, (pk_key_of[p], pk_rank_of[p][j]), Tuple{NTuple{3, Int}, Tuple{Int, Int}}[]), (p, pair))
        push!(get!(dropoff_terms, (dp_key_of[p], dp_rank_of[p][k]), Tuple{NTuple{3, Int}, Tuple{Int, Int}}[]), (p, pair))
    end

    z_dual_cons = Dict{Tuple{Any, Int}, ConstraintRef}()
    for (key, chain) in chains
        nu_sum = sum(nu[(key, idx2)] for idx2 in eachindex(chain.stations))
        for (idx, _station) in enumerate(chain.stations)
            cost = chain.costs[idx]
            expr = AffExpr(0.0)
            add_to_expression!(expr, 1.0, lambda[key])
            add_to_expression!(expr, -1.0, mu[(key, idx)])
            add_to_expression!(expr, -cost, nu_sum)
            terms = chain.side == :pickup ? get(pickup_terms, (key, idx), Tuple{NTuple{3, Int}, Tuple{Int, Int}}[]) :
                    get(dropoff_terms, (key, idx), Tuple{NTuple{3, Int}, Tuple{Int, Int}}[])
            for (p, pair) in terms
                if chain.side == :pickup
                    add_to_expression!(expr, 1.0, rhoO[(p, pair)])
                else
                    add_to_expression!(expr, 1.0, rhoD[(p, pair)])
                end
                add_to_expression!(expr, -1.0, sigma[(p, pair)])
            end
            z_dual_cons[(key, idx)] = @constraint(m, expr <= 0.0)
        end
    end
    return z_dual_cons
end

"""
    add_completion_tightness_constraint!(m, phi_expr, Q_bar) -> tightness_con

Pins the completed dual to be tight at the fixed hat point: `phi_expr == Q_bar`.
"""
function add_completion_tightness_constraint!(m::JuMP.Model, phi_expr::AffExpr, Q_bar::Float64)
    return @constraint(m, phi_expr == Q_bar)
end
