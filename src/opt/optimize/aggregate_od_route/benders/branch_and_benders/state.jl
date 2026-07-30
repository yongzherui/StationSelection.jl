abstract type _BranchBendersCacheKey end

struct _BranchBendersYCacheKey <: _BranchBendersCacheKey
    open_stations::Tuple{Vararg{Int}}
end

struct _BranchBendersYZCacheKey <: _BranchBendersCacheKey
    open_stations::Tuple{Vararg{Int}}
    endpoint_assignments::Tuple
end

mutable struct _BranchBendersRuntimeState
    cache::Dict{_BranchBendersCacheKey, _BranchBendersOracleResult}
    shared_pool::Vector{AggregateODRouteColumn}
    submitted::Set{Tuple{_BranchBendersCacheKey, Int}}
    stats::_BranchBendersStats
    best_ub::Float64
    best_result::Union{Nothing, _BranchBendersOracleResult}
    callback_events::Vector{NamedTuple}
    callback_count::Int
end

function _BranchBendersRuntimeState(initial_columns)
    pool = isnothing(initial_columns) ? AggregateODRouteColumn[] : copy(initial_columns)
    return _BranchBendersRuntimeState(
        Dict{_BranchBendersCacheKey, _BranchBendersOracleResult}(),
        pool,
        Set{Tuple{_BranchBendersCacheKey, Int}}(),
        _BranchBendersStats(),
        Inf,
        nothing,
        NamedTuple[],
        0,
    )
end
