"""
Endpoint-open feasibility cuts for the Benders outer loop -- "some candidate station on
each side of every touched physical endpoint must be open." Uses the nearest-open
candidate-set machinery in `../nearest_open/endpoint_chain.jl`.

NOTE: `_add_default_endpoint_coverage_constraints!` calls `_base_aggregate_od_route_model`,
which has no definition anywhere in this codebase (pre-existing staleness carried over from
`core.jl`, not introduced by this move) -- this function is currently unreachable/broken.
"""

function _add_endpoint_open_feasibility_cut!(
    master::Model,
    y,
    candidates::Vector{Int},
)::ConstraintRef
    return @constraint(master, sum(y[j] for j in candidates) >= 1.0)
end

"""
    _aggregate_od_route_endpoint_candidate_sets(data, requests, max_walking_distance)
        -> Dict{Tuple{Int, Symbol}, Vector{Int}}

Unique physical `(endpoint, side)` -> nearest-open candidate station set
(`_nearest_open_endpoint_candidates`), deduplicated across every scenario
occurrence of that endpoint in `requests`. `compute_valid_jk_pairs` builds
every request's real `(j,k)` pairs as exactly the off-diagonal (or full, with
`allow_same_station`) Cartesian product of these same independently-computed
per-side sets, regardless of `feasibility_cut_style` -- so "some candidate on
each side must be open" is a necessary condition for any request to have a
servable real pair, whether resolution is `:pair_chain`'s joint ranking or
`:big_m_nearest`/`:endpoint_chain`'s independent per-side selection.
"""
function _aggregate_od_route_endpoint_candidate_sets(
    data::StationSelectionData,
    requests::Vector{NTuple{3, Int}},
    max_walking_distance::Float64,
)::Dict{Tuple{Int, Symbol}, Vector{Int}}
    sets = Dict{Tuple{Int, Symbol}, Vector{Int}}()
    for (_s, o, d) in requests
        for (endpoint, side) in ((o, :pickup), (d, :dropoff))
            key = (endpoint, side)
            haskey(sets, key) && continue
            sets[key] = _nearest_open_endpoint_candidates(data, endpoint, max_walking_distance, side)
        end
    end
    return sets
end

"""
    _add_default_endpoint_coverage_constraints!(master, y, data, model, requests) -> Int

Adds, by default, one `sum(y[j] for j in candidates) >= 1` constraint per
unique physical endpoint touched by `requests` (aggregated across every
scenario, since `y` is scenario-agnostic) -- the simplest necessary condition
for subproblem feasibility, ensuring every request's pickup and dropoff side
has at least one open candidate station. Combined with `allow_same_station=true`
always being in effect (`create_map`), this is also *sufficient*: every
request then always resolves to a real pair (possibly same-station), so
`_fixed_assignments_from_y` can never report a request infeasible and the
reactive feasibility-cut machinery in the outer loop becomes structurally
unreachable, not just less likely. Returns the number of constraints added.
"""
function _add_default_endpoint_coverage_constraints!(
    master::Model,
    y,
    data::StationSelectionData,
    model::AnyAggregateODRouteProblem,
    requests::Vector{NTuple{3, Int}},
)::Int
    base = _base_aggregate_od_route_model(model)
    sets = _aggregate_od_route_endpoint_candidate_sets(data, requests, base.max_walking_distance)
    for candidates in values(sets)
        _add_endpoint_open_feasibility_cut!(master, y, candidates)
    end
    return length(sets)
end
