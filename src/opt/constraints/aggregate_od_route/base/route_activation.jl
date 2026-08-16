"""
Incremental route-column (theta) adder for `AggregateODRouteBaseFormulation`'s `CGSolver`
master. Unlike `DirectMIPSolver`'s build (`optimize/aggregate_od_route/direct/build_base.jl`,
`add_route_variables!`, `variables/routes.jl`), which knows its whole column pool up front and
builds `theta`/`route_link` in one closed-form batch, the CG master starts with an empty
pool and both its seed pass and its mid-loop pricing additions go through this single
incremental entry point -- mirroring
`add_joint_routing_assignment_column!`
(`constraints/aggregate_od_route/joint_routing_assignment/routing_and_assignment.jl`), but
simpler: a new `theta` only ever has to be patched into `route_link` (no `assignments` to
wire into coverage rows, since `x` carries assignment separately in Base).

Geometry (`AggregateODRouteColumn.od_pairs`/`tau`, scenario-agnostic) and activation
(`theta[(column_id,s)]`, scenario-specific) are two different things here. Geometry dedup
reuses `mapping`'s own bookkeeping directly
(`_register_aggregate_od_route_column_metadata!`, `data/maps/aggregate_od_route_map.jl`) --
unlike Joint, whose column type doesn't match `AggregateODRouteMap`'s built-in
`AggregateODRouteColumn`, Base's does, so there is no need for a second, parallel column
store the way Joint keeps one.
"""

export add_aggregate_od_route_base_column!

"""
    _aggregate_od_route_base_x_keys_by_pair_scenario(x) -> Dict{Tuple{Int,Int,Int}, Vector{NTuple{4,Int}}}

Groups every declared `x` key `(s,p,j,k)` by `(s,j,k)`. Built once, right after `x` and
`route_link` exist (`optimize/aggregate_od_route/column_generation/build_base.jl`), purely as
a derived index so `add_aggregate_od_route_base_column!` can find every `route_link[(s,p,j,k)]`
constraint a freshly activated `(column,scenario)` needs patched in, without rescanning all
of `x` per call.
"""
function _aggregate_od_route_base_x_keys_by_pair_scenario(
    x::Dict{NTuple{4, Int}, VariableRef},
)::Dict{Tuple{Int, Int, Int}, Vector{NTuple{4, Int}}}
    index = Dict{Tuple{Int, Int, Int}, Vector{NTuple{4, Int}}}()
    for key in keys(x)
        s, _, j, k = key
        push!(get!(() -> NTuple{4, Int}[], index, (s, j, k)), key)
    end
    return index
end

"""
    add_aggregate_od_route_base_column!(m, data, mapping, column) -> (theta, action)

`column::AggregateODRouteColumn` must carry `column.metadata["scenario"]` -- the
`(column_id, scenario)` pair is the actual master key, mirroring the pricer's own
convention (`_aggregate_od_route_column_from_label`,
`label_setting/aggregate_od_route/labels.jl`, tags every priced column with the scenario it
was priced against). `column.id` on the argument itself is never trusted: geometry identity
is decided purely by `_aggregate_od_route_column_signature(column)`
(`label_setting/aggregate_od_route/labels.jl`) against
`m[:aggregate_od_route_base_column_signatures]`.

On a genuinely new signature: mints a fresh id off `m[:aggregate_od_route_base_next_column_id]`
(a `Ref{Int}`), registers a real `AggregateODRouteColumn` into `mapping` via
`_register_aggregate_od_route_column_metadata!`, and records it in
`m[:aggregate_od_route_base_column_signatures]`/`m[:aggregate_od_route_base_columns_by_id]`.
On a signature match, the already-registered geometry's `id`/`tau` is always reused --
`mapping.columns` is add-only here, so a marginally cheaper `tau` rediscovered later for an
identical pair set is never swapped in. This is a deliberate simplification: `tau` for a
fixed served-pairs set is close to invariant across searches, and CG's other geometries still
compete on price regardless.

`action` is `:skipped` when `theta[(column_id, s)]` already exists (this exact geometry is
already activated for this exact scenario) -- a plain `haskey`, not a cost comparison, since
geometry id and `(column_id,s)` activation are separate keys. Otherwise `:added`: creates one
new `theta` variable (domain from `m[:aggregate_od_route_base_relax_integrality]`), sets its
objective coefficient via the same `aggregate_od_route_column_objective_coefficient`
(`constraints/aggregate_od_route/core.jl`) Direct's closed-form objective uses, and patches
its `-1.0` coefficient into every `route_link[(s,p,j,k)]` row the column's `(j,k)` pairs
touch for scenario `s` (`route_link[key] = @constraint(m, x[key] <= sum(theta))` normalizes
to `x - sum(theta) <= 0`, hence `-1.0`).
"""
function add_aggregate_od_route_base_column!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    column::AggregateODRouteColumn,
)
    theta = m[:aggregate_od_route_base_theta]
    signatures = m[:aggregate_od_route_base_column_signatures]
    columns_by_id = m[:aggregate_od_route_base_columns_by_id]
    route_link = m[:aggregate_od_route_base_route_link]
    keys_by_pair_scenario = m[:aggregate_od_route_base_route_link_keys_by_pair]

    s = Int(column.metadata["scenario"])
    signature = _aggregate_od_route_column_signature(column)
    column_id = get(signatures, signature, nothing)
    if isnothing(column_id)
        next_id_ref = m[:aggregate_od_route_base_next_column_id]
        column_id = next_id_ref[]
        next_id_ref[] += 1
        registered = AggregateODRouteColumn(
            column_id, column.od_pairs, column.tau; metadata=copy(column.metadata),
        )
        _register_aggregate_od_route_column_metadata!(mapping, registered)
        signatures[signature] = column_id
        columns_by_id[column_id] = registered
    end

    haskey(theta, (column_id, s)) && return theta[(column_id, s)], :skipped

    registered_column = columns_by_id[column_id]
    relax_integrality = Bool(m[:aggregate_od_route_base_relax_integrality])
    theta_var = relax_integrality ?
        @variable(m, lower_bound = 0.0, base_name = "theta[$column_id,$s]") :
        @variable(m, binary = true, base_name = "theta[$column_id,$s]")
    theta[(column_id, s)] = theta_var

    set_objective_coefficient(
        m, theta_var,
        aggregate_od_route_column_objective_coefficient(
            Float64(m[:aggregate_od_route_base_route_regularization_weight]),
            Float64(m[:aggregate_od_route_base_repositioning_time]),
            registered_column,
        ),
    )

    for (j, k) in registered_column.od_pairs
        for key in get(keys_by_pair_scenario, (s, j, k), NTuple{4, Int}[])
            set_normalized_coefficient(route_link[key], theta_var, -1.0)
        end
    end
    return theta_var, :added
end

"""
`CGSolver` hook real logic (dispatched from
`optimize/aggregate_od_route/column_generation/dispatch.jl`). Count excludes `:skipped`
columns, mirroring `_aggregate_od_route_add_columns!`'s Joint counterpart.
"""
function _aggregate_od_route_add_columns!(
    ::AggregateODRouteBaseFormulation,
    build_result::BuildResult,
    mapping::AggregateODRouteMap,
    m::JuMP.Model,
    columns,
)::Int
    data = m[:aggregate_od_route_base_data]
    added_count = 0
    for column in columns
        _theta, action = add_aggregate_od_route_base_column!(m, data, mapping, column)
        action === :skipped || (added_count += 1)
    end
    return added_count
end
