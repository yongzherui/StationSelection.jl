"""Shared, reward-model-independent mechanics for the aggregate and PFA pricers."""

function _make_sparse_station_ages(
    station_age,
    node_index::Dict{Int, Int},
)::Tuple{Vector{Int32}, Vector{Float64}, UInt64}
    n_live = length(station_age)
    age_idx = Vector{Int32}(undef, n_live)
    age_val = Vector{Float64}(undef, n_live)
    age_mask = UInt64(0)
    i = 0
    @inbounds for (station, age) in station_age
        idx = Int32(node_index[station])
        age_mask |= UInt64(1) << ((idx - 1) & 63)
        j = i
        while j >= 1 && age_idx[j] > idx
            age_idx[j + 1] = age_idx[j]
            age_val[j + 1] = age_val[j]
            j -= 1
        end
        age_idx[j + 1] = idx
        age_val[j + 1] = age
        i += 1
    end
    return age_idx, age_val, age_mask
end

@inline function _sparse_station_ages_dominate(
    a_idx::Vector{Int32}, a_val::Vector{Float64}, a_mask::UInt64,
    b_idx::Vector{Int32}, b_val::Vector{Float64}, b_mask::UInt64,
)::Bool
    _sparse_station_age_support_rejection(a_idx, a_mask, b_idx, b_mask) == 0 || return false
    return _sparse_station_age_values_dominate(a_idx, a_val, b_idx, b_val)
end

"""Return 0 when support may dominate, or 1=size and 2=mask rejection."""
@inline function _sparse_station_age_support_rejection(
    a_idx::Vector{Int32}, a_mask::UInt64,
    b_idx::Vector{Int32}, b_mask::UInt64,
)::Int
    length(a_idx) >= length(b_idx) || return 1
    b_mask & ~a_mask == 0 || return 2
    return 0
end

@inline function _sparse_station_age_values_dominate(
    a_idx::Vector{Int32}, a_val::Vector{Float64},
    b_idx::Vector{Int32}, b_val::Vector{Float64},
)::Bool
    ia = 1
    na = length(a_idx)
    @inbounds for ib in eachindex(b_idx)
        idx = b_idx[ib]
        while ia <= na && a_idx[ia] < idx
            ia += 1
        end
        ia <= na && a_idx[ia] == idx || return false
        a_val[ia] <= b_val[ib] + 1e-9 || return false
    end
    return true
end

"""Sort pricing results while constructing the expensive route string once."""
function _sort_pricing_results_by_route(scored, key)
    isempty(scored) && return scored
    return scored[sortperm([key(entry) for entry in scored])]
end
