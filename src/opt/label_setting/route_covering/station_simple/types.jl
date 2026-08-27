"""
Label/bitsets/dominance types specific to the elementary-route pricer. See
`../types.jl` for the pricing graph/duals types this pricer shares with
`exact/`, and `seed.jl` / `extend.jl` / `prune.jl` / `dominate.jl` /
`context.jl` / `hooks.jl` for the label-setting functionality built on top of
the types below -- `seed.jl` is the file to start from for "is the label
search correct".

An alternative to the revisit-tolerant search in `../exact/` (types.jl/
seed.jl/extend.jl/prune.jl/dominate.jl/context.jl/hooks.jl): a route may
never revisit a station, so a certified `(j,k)` pair settles permanently the
first (and only) time `k` is visited after `j` -- there is no need for a
`station_age` `Dict` tracking every past visit, only `live_origin_age` for
stations already on the route whose destination hasn't been reached yet.

Because `visited` only grows and is part of the dominance signature (exact match,
not subset), a dominating label's `served_pairs` need not be compared separately:
identical `(current, visited)` plus reduced-cost/time/live-origin-age domination
already implies every future extension available to the dominated label is at
least as good from the dominating one, since which nodes remain reachable depends
only on `visited` and rewards not yet banked are fully described by
`live_origin_age`. Do not add a served-pairs comparison here without re-deriving
that invariant; see `_dominates_route_covering_station_simple_label`
(`dominate.jl`).

# Why this is faster

Fewer extensions, since candidate generation drops any already-visited node --
the branching factor shrinks as a route grows.

# Correctness caveat

Restricting to elementary routes restricts the column universe the master
problem prices over. Where the model's optimum genuinely wants a revisiting
route this pricer is a *heuristic* -- it can terminate CG with a weaker LP
bound or miss improving columns. It is therefore opt-in and off by default (not
currently reachable from any `pricing_round.jl`); validate the LP bound against
the revisit-tolerant pricer before relying on it.

# Reuse

Shares `RouteCoveringPricingData`/`RouteCoveringPricingDuals` and the
`_route_covering_travel`/`_direct_ride_limit` helpers from `../data.jl`, and
produces `AggregateODRouteColumn`s via the same `_aggregate_od_route_column_from_label`
convention as the revisit-tolerant pricer (a new method dispatched on this
directory's label type, in `hooks.jl`).
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
`_route_covering_station_simple_state`, `dominate.jl`), so every pair the
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
