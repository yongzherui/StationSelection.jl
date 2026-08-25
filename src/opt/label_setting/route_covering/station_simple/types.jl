"""
Label/bitsets/dominance types specific to the elementary-route pricer
(`station_simple.jl`). See `../types.jl` for the pricing graph/duals types
this pricer shares with `exact/`.
"""

export RouteCoveringStationSimpleLabel

struct RouteCoveringStationSimpleLabel
    current::Int
    route::Vector{Int}
    visited::Set{Int}
    time::Float64
    live_origin_age::Dict{Int, Float64}
    served_pairs::Set{Tuple{Int, Int}}
    tau::Float64
    reduced_cost::Float64
end

"""
`live_origin_age` is stored sparse -- `(age_idx, age_val, age_mask)`, the same
representation `_make_sparse_station_ages` (`label_setting/utils.jl`) builds for the
revisit-tolerant pricer and for the PFA station-simple pricer -- rather than a
dense `Vector` over every node, since a partial elementary route typically has
only a handful of live pickup clocks regardless of network size.
"""
struct RouteCoveringStationSimpleBitsets
    visited_bits::BitSet
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    age_mask::UInt64
end

"""
Inline scalar fields the state's label-list scan needs, read straight off the
entry rather than chasing into `label`/`bitsets` -- same rationale as
`RouteCoveringDominanceFilters` (`../exact/types.jl`). `current`/`visited_bits` are
not carried here because the state itself is exactly that key (see
`_route_covering_station_simple_state`, `labels.jl`), so every pair the
state's label-list scan tests already agrees on them.
"""
struct RouteCoveringStationSimpleDominanceFilters
    reduced_cost::Float64
    time::Float64
    route_length::Int32
end

"""Dominance-rule marker for the station-simple pricer; no switches yet, unlike
the revisit-tolerant pricer's `BoundedStops` (elementary routes have no
analogous optional cap to toggle)."""
struct RouteCoveringStationSimpleDominanceRules <: AbstractPricingDominanceRules end
