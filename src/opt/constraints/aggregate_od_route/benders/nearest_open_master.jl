"""
BendersYZ/BendersYZH master `z`/`h` builders -- populate the shared nearest-open endpoint
selector machinery (`../nearest_open/endpoint_chain.jl`) directly on the master, without a
per-scenario `x`.
"""

"""
    _add_nearest_open_master_z!(master, data, y, requests, feasible_pairs, max_walking_distance, allow_walk_only, selector_style)

BendersYZ/BendersYZH master `z`-builder: populates/reuses
`master[:nearest_endpoint_chain_cache]` for every physical endpoint touched
by `requests`, without creating any `x`/`h`. Continuous `[0,1]` (`binary=false`
— see `_add_nearest_open_endpoint_master_x!`'s docstring for why this is
sound given `y` is `Bin`). Naturally deduplicated across scenario-repeats of
the same physical `(o,d)` via `_nearest_open_endpoint_selectors!`'s cache.
"""
function _add_nearest_open_master_z!(
    master::Model,
    data::StationSelectionData,
    y,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    max_walking_distance::Float64,
    allow_walk_only::Bool,
    selector_style::Symbol,
)::Nothing
    for request in requests
        _s, o, d = request
        _nearest_open_endpoint_selectors!(
            master, data, y, o, d, feasible_pairs[request], max_walking_distance;
            binary=false, allow_walk_only=allow_walk_only, selector_style=selector_style,
        )
    end
    return nothing
end

"""
    _add_nearest_open_master_h!(master, data, y, physical_pairs, feasible_pairs_by_p, max_walking_distance, allow_walk_only, selector_style)

BendersYZH's master `h`-builder: one continuous `[0,1]` `h[(p,pair)]` per
physical OD pair `p` (not per `(scenario,o,d)`, unlike BendersXY's `x`),
linked to `zp`/`zd` via `_add_nearest_open_endpoint_linked_x!` exactly as
BendersXY's `x` is -- `h` plays the identical collision-blocking role `x`
plays there (`sum(h over pairs)==1` with no diagonal entry unless
walk-only), so BendersYZH's master, like BendersXY's, needs no separate
feasibility-cut branch. Iterating `physical_pairs` (not the flat
per-scenario `requests`) already touches every endpoint any request would,
so this alone populates/reuses `master[:nearest_endpoint_chain_cache]` --
no separate `_add_nearest_open_master_z!` call is needed before this one.
"""
function _add_nearest_open_master_h!(
    master::Model,
    data::StationSelectionData,
    y,
    physical_pairs::Vector{Tuple{Int, Int}},
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    max_walking_distance::Float64,
    allow_walk_only::Bool,
    selector_style::Symbol,
)
    h = Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, VariableRef}()
    for p in physical_pairs
        o, d = p
        pairs = feasible_pairs_by_p[p]
        for pair in pairs
            h[(p, pair)] = @variable(master, lower_bound = 0.0, upper_bound = 1.0)
        end
        @constraint(master, sum(h[(p, pair)] for pair in pairs; init=0.0) == 1.0)
        h_by_pair = Dict(pair => h[(p, pair)] for pair in pairs)
        _add_nearest_open_endpoint_linked_x!(
            master, data, y, o, d, pairs, h_by_pair, max_walking_distance;
            binary=false, allow_walk_only=allow_walk_only, selector_style=selector_style,
        )
    end
    return h
end
