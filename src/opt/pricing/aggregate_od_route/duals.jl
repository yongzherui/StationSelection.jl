"""
Master-problem-facing dual extraction for AggregateODRouteModel column generation.
"""

export AggregateODRouteCoverageDuals
export extract_aggregate_od_route_coverage_duals
export aggregate_od_route_coverage_sigma

struct AggregateODRouteCoverageDuals
    raw_duals::Dict{Any, Float64}
    sigma::Dict{NTuple{3, Int}, Float64}
end

"""
    aggregate_od_route_coverage_sigma(raw_dual) -> Float64

Coverage rows are stored as:

    sum(theta[c,s] for c covering (j,k)) - u[j,k,s] >= 0

For a minimization RMP, a new column with coefficient +1 in this row has reduced
cost `mu * (tau + rho) - raw_dual`. Pricing therefore uses
`profit(c,s) = sum(sigma[j,k,s]) - mu * (tau[c] + rho)` with `sigma = raw_dual`.
"""
aggregate_od_route_coverage_sigma(raw_dual::Real)::Float64 = Float64(raw_dual)

function extract_aggregate_od_route_coverage_duals(m::Model)::AggregateODRouteCoverageDuals
    coverage = m[:aggregate_od_route_coverage_constraints]
    raw = Dict{Any, Float64}()
    sigma = Dict{NTuple{3, Int}, Float64}()
    for (key, con) in coverage
        raw_dual = dual(con)
        raw[key] = raw_dual
        pair_s = (Int(key[1]), Int(key[2]), Int(key[3]))
        sigma[pair_s] = get(sigma, pair_s, 0.0) + aggregate_od_route_coverage_sigma(raw_dual)
    end
    return AggregateODRouteCoverageDuals(raw, sigma)
end

extract_aggregate_od_route_coverage_duals(build_result::BuildResult)::AggregateODRouteCoverageDuals =
    extract_aggregate_od_route_coverage_duals(build_result.model)

function _scenario_pricing_duals(
    duals::AggregateODRouteCoverageDuals,
    scenario::Int,
)::AggregateODRoutePricingDuals
    sigma = Dict{Tuple{Int, Int}, Float64}()
    for ((j, k, s), value) in duals.sigma
        s == scenario || continue
        sigma[(j, k)] = value
    end
    return AggregateODRoutePricingDuals(sigma)
end

function _aggregate_od_route_dual_stats(duals::AggregateODRouteCoverageDuals)
    vals = collect(values(duals.sigma))
    isempty(vals) && return (nothing, nothing, nothing, nothing)
    mean = sum(vals) / length(vals)
    std = length(vals) > 1 ? sqrt(sum((v - mean)^2 for v in vals) / (length(vals) - 1)) : 0.0
    return (minimum(vals), maximum(vals), mean, std)
end
