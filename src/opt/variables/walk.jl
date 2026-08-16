"""
Walk variable creation functions for station selection optimization models.

These functions add direct-walk ("no vehicle, no station") assignment variables for
demand groups that can walk their whole trip. Uses multiple dispatch to provide
specialized implementations for different mapping types, mirroring `assignment.jl`'s own
`add_assignment_variables!` -- only `AggregateODRouteMap` needs a dedicated `x_walk`
today (`ClusteringTwoStageODMap`'s `add_assignment_variables!` already folds
`WALK_ONLY_PAIR` into its own `x[s][p][pair_idx]` vector, one entry per valid pair,
without singling it out), but the generic name leaves room for a future mapping type that
wants the same dedicated treatment.
"""

using JuMP

export add_walk_variables!

# ============================================================================
# AggregateODRouteMap (AggregateODRouteBaseFormulation, AggregateODRouteJointRoutingAssignmentFormulation)
# ============================================================================

"""
    add_walk_variables!(m, data, mapping::AggregateODRouteMap; scenarios=1:n_scenarios(data), relax_integrality=false)
        -> Dict{Tuple{Int,Int}, VariableRef}

Direct-walk `x_walk[s,p]` for every demand group `p` (position within
`mapping.Omega_s[s]`) with `s in scenarios` whose `get_valid_jk_pairs` includes
`WALK_ONLY_PAIR`. Keyed only by the demand group, not by a candidate station: there is
exactly one "walk directly" option per OD pair, not one per station, and it needs no
linking to `y`/`z` at all -- unlike `x[s,p,j,k]`
(`add_assignment_variables!(m, data, mapping::AggregateODRouteMap)`), which gets linked
to `y` and `θ` separately.

Shared verbatim by `AggregateODRouteBaseFormulation`'s `build_model`
(`optimize/aggregate_od_route/direct/build_base.jl`) and
`_build_joint_routing_assignment_model`
(`optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl`) -- the
two formulations disagree on `x`/`θ` but need the identical direct-walk option, so there
is exactly one implementation to keep in sync rather than two near-duplicates drifting
apart.

`scenarios` restricts variable creation to a subset of scenarios -- used by
`AggregateODRouteBendersYXFormulation`'s subproblem, which is solved one scenario at a
time (default keeps every existing caller's behavior unchanged). `relax_integrality`
declares `x_walk[s,p] >= 0` (no upper bound -- primal-redundant for this positive-cost
covering row, same reasoning as `add_route_variables!`'s own
`relax_integrality` kwarg) instead of `Bin`.
"""
function add_walk_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
    relax_integrality::Bool=false,
)::Dict{Tuple{Int, Int}, VariableRef}
    x_walk = Dict{Tuple{Int, Int}, VariableRef}()
    for s in scenarios
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) || continue
            x_walk[(s, p)] = relax_integrality ?
                @variable(m, lower_bound = 0.0, base_name = "x_walk[$s,$p]") :
                @variable(m, binary = true, base_name = "x_walk[$s,$p]")
        end
    end
    m[:x_walk] = x_walk
    return x_walk
end
