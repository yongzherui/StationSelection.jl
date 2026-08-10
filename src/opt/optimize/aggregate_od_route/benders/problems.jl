"""
Model-shaped Benders problem types for `AggregateODRouteModel`: `BendersMasterModel`
wraps the master construction each `_run_aggregate_od_route_nearest_open_benders_*`
function used to build by hand, so it can go through the standard
`build_model`/`run_opt` interface like every other `AbstractODModel`. One
`build_model` method per decomposition (`BendersY` first; `BendersXY`/`BendersYZ`/
`BendersYZH` follow in later passes), each picking only the master variables its
decomposition needs from the shared `src/opt/variables|constraints|objectives/`
helpers.
"""

export BendersMasterModel

"""
    BendersMasterModel{D<:AbstractBendersDecomposition} <: AbstractODModel

Wraps `.base::AggregateODRouteModel` and the full `BendersSolver` config -- which
master variant gets built (plain `sum(theta)` vs. `lifted_walking_objective`'s
walking-cost term vs. `direct_enumeration_guide`'s extra coverage, the
`route_regularization_weight_schedule`'s initial stage, `seed_cuts` replay) is
entirely solver-config-driven, mirroring the inline construction each
`_run_aggregate_od_route_nearest_open_benders_*` function used to do by hand
field-for-field. `cut_ids` is the caller's already-computed `_benders_cut_groups`
key set -- cut *grouping* is an outer-loop policy choice via `solver.cut_mode`, not
something this type re-derives.

The built model stashes `m[:benders_beta_schedule]`/`m[:walking_cost_expr]`/
`m[:direct_cost_expr]` (when `lifted_walking_objective`) so an outer Benders loop can
mutate this *same* persistent model's objective in place on a
`route_regularization_weight_schedule` stage advance, rather than rebuilding from
scratch and losing accumulated cuts.
"""
struct BendersMasterModel{D <: AbstractBendersDecomposition} <: AbstractODModel
    base::AggregateODRouteModel
    solver::BendersSolver
    cut_ids::Vector{Int}
    seed_cuts::Vector{<:NamedTuple}
    direct_enumeration_pool::Union{Nothing, Vector{AggregateODRouteColumn}}

    function BendersMasterModel(
        base::AggregateODRouteModel,
        solver::BendersSolver,
        cut_ids::Vector{Int};
        seed_cuts::Vector{<:NamedTuple}=NamedTuple[],
        direct_enumeration_pool::Union{Nothing, Vector{AggregateODRouteColumn}}=nothing,
    )
        new{typeof(solver.decomposition)}(base, solver, sort!(collect(cut_ids)), seed_cuts, direct_enumeration_pool)
    end
end

function build_model(
    model::BendersMasterModel{BendersY},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    base = model.base
    solver = model.solver
    cut_ids = model.cut_ids
    mapping = create_map(base, data)
    requests, _demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()

    variable_counts["station_selection"] = add_station_selection_variables!(master, data)
    y = master[:y]
    variable_counts["benders_cut_placeholder"] = add_benders_cut_placeholder_variables!(master, cut_ids)
    theta = master[:theta]
    constraint_counts["station_limit"] = add_station_limit_constraint!(master, data, base.l)
    constraint_counts["endpoint_coverage"] =
        _add_default_endpoint_coverage_constraints!(master, y, data, base, requests)
    if _is_endpoint_nearest_style(base.assignment_policy.feasibility_cut_style)
        validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=base.allow_walk_only)
    end
    isempty(model.seed_cuts) || _seed_y_cuts!(master, y, theta, model.seed_cuts)

    beta_schedule = solver.lifted_walking_objective && !isnothing(solver.route_regularization_weight_schedule) ?
        solver.route_regularization_weight_schedule : [base.route_regularization_weight]
    isapprox(beta_schedule[end], base.route_regularization_weight; atol=1e-9) || throw(ArgumentError(
        "route_regularization_weight_schedule must end at base.route_regularization_weight " *
        "($(base.route_regularization_weight)); got $(beta_schedule[end])"
    ))
    current_beta = beta_schedule[1]

    if solver.lifted_walking_objective
        walking_cost_expr, x_by_pair_full = _add_nearest_open_master_walking_cost!(
            master, data, base, y, requests, feasible_pairs,
        )
        direct_cost_expr = AffExpr(0.0)
        if !isnothing(model.direct_enumeration_pool)
            direct_cost_expr = _add_direct_enumeration_guide!(
                master, data, base, requests, feasible_pairs, x_by_pair_full, model.direct_enumeration_pool;
                relax_integrality=solver.direct_enumeration_relax_integrality,
            )
        end
        @objective(master, Min, current_beta * (sum(theta[cid] for cid in cut_ids) + direct_cost_expr) + walking_cost_expr)
        master[:walking_cost_expr] = walking_cost_expr
        master[:direct_cost_expr] = direct_cost_expr
    else
        @objective(master, Min, sum(theta[cid] for cid in cut_ids))
    end
    master[:benders_beta_schedule] = beta_schedule
    master[:benders_cut_ids] = cut_ids

    counts = ModelCounts(variable_counts, constraint_counts, Dict{String, Int}())
    metadata = Dict{String, Any}("requests" => requests, "feasible_pairs" => feasible_pairs)
    return BuildResult(master, mapping, nothing, counts, metadata)
end
