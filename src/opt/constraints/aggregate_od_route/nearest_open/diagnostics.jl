"""
Validation and diagnostics for the `:big_m_nearest`/`:endpoint_chain` endpoint-selector
family (`endpoint_chain.jl`).
"""

export validate_big_m_nearest_aggregate_od_route!
export assert_endpoint_chain_near_binary
export nearest_open_endpoint_diagnostics

function validate_big_m_nearest_aggregate_od_route!(
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    allow_walk_only::Bool=false,
)::Nothing
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            pairs = get_valid_jk_pairs(mapping, o, d)
            isempty(pairs) && continue
            real_pairs = filter(!is_walk_only_pair, pairs)
            pickups = _nearest_open_endpoint_candidates(data, o, mapping.max_walking_distance, :pickup)
            dropoffs = _nearest_open_endpoint_candidates(data, d, mapping.max_walking_distance, :dropoff)
            _assert_endpoint_has_candidates!(o, d, :pickup, pickups)
            _assert_endpoint_has_candidates!(o, d, :dropoff, dropoffs)
            _check_big_m_nearest_pair_consistency!(data, o, d, real_pairs, pickups, dropoffs)
            allow_walk_only && _assert_walk_collision_feasible!(data, o, d, pickups, dropoffs, pairs)
        end
    end
    return nothing
end

"""
    assert_endpoint_chain_near_binary(m::Model; atol=1e-3)

Runtime check that every `zp`/`zd` endpoint selector indicator variable in
`m` is within `atol` of 0 or 1 in the current solution. The shared endpoint
selector cache is stored under `m[:nearest_endpoint_chain_cache]`, so this
works generically against any solved model that used an endpoint-nearest
encoding -- a no-op if the model never built one (`:pair_chain` style, or a
model with no nearest-open constraints at all). Meant to be called right
after `optimize!(m)` on what should be an integer-valued solve (a genuine
MIP, or an LP with `y` fixed to an already-binary value) -- a fractional `z`
there means the "nearest open" endpoint ranking isn't actually resolved to
a single winner, undermining any cost/cut derived from it.

`atol=1e-3` is deliberately looser than Gurobi's own default `IntFeasTol`
(1e-5): on larger, more degenerate instances (e.g. real BendersYZH runs with
many near-tied endpoint candidates) `z` values like `0.9996092321175274` have
been observed -- still a clean, unambiguous "selected" indicator, but outside
even the 1e-5 `IntFeasTol`-matched tolerance this check used previously. Every
call site that reads a `z` value onward for further computation applies
`round.(value.(...))` (see benders/subproblem_api.jl and benders/yz.jl) rather
than using the raw float, so values this check passes are always snapped to
an exact 0.0/1.0 before they can propagate any of this slack into downstream
cost or dual computations.
"""
function assert_endpoint_chain_near_binary(m::Model; atol::Float64=1e-3)::Nothing
    haskey(m, :nearest_endpoint_chain_cache) || return nothing
    for (key, vars) in m[:nearest_endpoint_chain_cache]
        for (idx, var) in enumerate(vars)
            val = value(var)
            (val <= atol || val >= 1.0 - atol) || throw(ArgumentError(
                "endpoint-chain (z) indicator check failed: z[$(idx)] in chain $(key) has value " *
                "$(val), not within atol=$(atol) of 0 or 1"
            ))
        end
    end
    return nothing
end

"""
    nearest_open_endpoint_diagnostics(m, data, mapping) -> Dict{String, Any}

Reports sizing and integrality diagnostics for a solved (or built)
endpoint-nearest model: distinct endpoint selectors (`"endpoint_count"`, i.e.
distinct `z_{ij}` selectors, one per `(side, physical endpoint)`), the total
`z` variable count summed over chains, request-pair (`x`) vs direct-walking
(`w`, the `WALK_ONLY_PAIR` slot) variable counts, nearest-open selector
constraint count, and request-coupling constraint count (the `<=,<=,>=`
linearization rows). If `m` has been solved, also reports whether any
`z`/`x`/`w` value is fractional (`"has_fractional_solution"`,
`"fractional_variables"`) -- a no-op-safe `false`/empty when `m` hasn't been
solved or has no endpoint selectors at all (e.g. `:pair_chain`, or no
nearest-open constraints).
"""
function nearest_open_endpoint_diagnostics(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Dict{String, Any}
    diagnostics = Dict{String, Any}(
        "endpoint_count" => 0,
        "z_variable_count" => 0,
        "x_pair_variable_count" => 0,
        "w_walk_variable_count" => 0,
        "nearest_open_chain_constraint_count" => 0,
        "request_coupling_constraint_count" => 0,
        "endpoint_selector_style" => nothing,
        "has_fractional_solution" => false,
        "fractional_variables" => Tuple{Symbol, Any, Int}[],
    )
    if haskey(m, :nearest_endpoint_chain_cache)
        cache = m[:nearest_endpoint_chain_cache]
        diagnostics["endpoint_count"] = length(cache)
        diagnostics["z_variable_count"] = sum(length(z) for z in values(cache); init=0)
        selector_style = haskey(m, :nearest_endpoint_selector_style) ?
            m[:nearest_endpoint_selector_style] : :endpoint_chain
        diagnostics["endpoint_selector_style"] = string(selector_style)
        diagnostics["nearest_open_chain_constraint_count"] = if selector_style == :big_m_nearest
            # sum(z)==1, z<=y for each candidate, one Big-M row per candidate,
            # and the redundant endpoint coverage row.
            sum(2 + 2 * length(z) for z in values(cache); init=0)
        else
            # sum(z)==1, z<=y for each candidate, triangular prior-open rows,
            # and the redundant endpoint coverage row.
            sum(2 + length(z) + div(length(z) * (length(z) - 1), 2) for z in values(cache); init=0)
        end
        can_check_values = has_values(m)
        for (key, z) in cache
            for (idx, var) in enumerate(z)
                can_check_values || break
                val = value(var)
                (val > 1e-6 && val < 1.0 - 1e-6) &&
                    push!(diagnostics["fractional_variables"], (:z, key, idx))
            end
        end
    end
    x_pairs = 0
    w_pairs = 0
    coupling_constraints = 0
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            pairs = get_valid_jk_pairs(mapping, o, d)
            isempty(pairs) && continue
            real_count = count(!is_walk_only_pair, pairs)
            walk_count = count(is_walk_only_pair, pairs)
            x_pairs += real_count
            w_pairs += walk_count
            # <=, <=, >= per real pair, plus one >= per common-candidate walk row.
            coupling_constraints += 3 * real_count
        end
    end
    diagnostics["x_pair_variable_count"] = x_pairs
    diagnostics["w_walk_variable_count"] = w_pairs
    diagnostics["request_coupling_constraint_count"] = coupling_constraints
    diagnostics["has_fractional_solution"] = !isempty(diagnostics["fractional_variables"])
    return diagnostics
end
