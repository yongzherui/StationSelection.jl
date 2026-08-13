"""
Model-shaped Benders problem types for `AggregateODRouteProblem`: `BendersMasterModel`
wraps the master construction each `_run_aggregate_od_route_nearest_open_benders_*`
function used to build by hand, so it can go through the standard
`build_model`/`run_opt` interface like every other `AbstractODModel`. One
`build_model` method per decomposition (`BendersY` first; `BendersXY`/`BendersYZ`/
`BendersYZH` follow in later passes), each picking only the master variables its
decomposition needs from the shared `src/opt/variables|constraints|objectives/`
helpers.
"""

export BendersMasterModel
export BendersSubproblemModel
export BendersCorePointProblem
export BendersCompletionProblem

"""
    BendersMasterModel{D<:AbstractBendersDecomposition} <: AbstractODModel

Wraps `.base::AggregateODRouteProblem` and the full `BendersSolver` config -- which
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
    base::AggregateODRouteProblem
    solver::BendersSolver
    cut_ids::Vector{Int}
    seed_cuts::Vector{<:NamedTuple}
    direct_enumeration_pool::Union{Nothing, Vector{AggregateODRouteColumn}}

    function BendersMasterModel(
        base::AggregateODRouteProblem,
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
    isempty(requests) && throw(ArgumentError("AggregateODRouteProblem nearest-open Benders requires positive demand"))

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
        set_benders_lifted_master_objective!(
            master, theta, cut_ids, walking_cost_expr, direct_cost_expr, current_beta;
            lifted_walking_objective=true,
        )
        master[:walking_cost_expr] = walking_cost_expr
        master[:direct_cost_expr] = direct_cost_expr
    else
        set_benders_lifted_master_objective!(
            master, theta, cut_ids, AffExpr(0.0), AffExpr(0.0), current_beta;
            lifted_walking_objective=false,
        )
    end
    master[:benders_beta_schedule] = beta_schedule
    master[:benders_cut_ids] = cut_ids

    counts = ModelCounts(variable_counts, constraint_counts, Dict{String, Int}())
    metadata = Dict{String, Any}("requests" => requests, "feasible_pairs" => feasible_pairs)
    return BuildResult(master, mapping, nothing, counts, metadata)
end

# ---------------------------------------------------------------------------
# BendersSubproblemModel: fixed-y (or, for other decompositions, fixed-x/z/h)
# recourse LP -- free assignment + route selection, dual off the fixing rows.
# ---------------------------------------------------------------------------

"""
    BendersSubproblemModel{D<:AbstractBendersDecomposition} <: AbstractODModel

`.base::AggregateODRouteProblem` is whatever model the caller wants priced against --
already resolved for `lifted_walking_objective` (the outer loop's unit-weighted
`_unit_weighted_routing_model` swap when that's active, or the real model otherwise);
this type is agnostic to that choice. `hat` is the fixed decision each decomposition's
subproblem builds around -- `y_hat::Vector{Float64}` for `BendersY`,
`x_hat::Dict{...,Float64}` for `BendersXY`, `z_hat`/`h_hat` for `BendersYZ`/`BendersYZH` --
deliberately untyped since its shape is decomposition-specific; each `build_model` method
knows how to interpret its own. Fixed via an equality constraint on a continuous relaxation
of the real (binary/chain) variable so its dual is a valid subgradient. `requests`/
`feasible_pairs`/`columns` are per-call inputs (one cut group's requests, the current
shared column pool), not re-derivable from `(base, data)` alone.
"""
struct BendersSubproblemModel{D <: AbstractBendersDecomposition} <: AbstractODModel
    base::AggregateODRouteProblem
    hat
    requests::Vector{NTuple{3, Int}}
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}}
    columns::Vector{AggregateODRouteColumn}
    decomposition::D
    lambda_binary::Bool

    function BendersSubproblemModel(
        base::AggregateODRouteProblem,
        hat,
        requests::Vector{NTuple{3, Int}},
        feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
        columns::Vector{AggregateODRouteColumn},
        decomposition::D;
        lambda_binary::Bool=false,
    ) where {D <: AbstractBendersDecomposition}
        new{D}(base, hat, requests, feasible_pairs, columns, decomposition, lambda_binary)
    end
end

function build_model(
    model::BendersSubproblemModel{BendersY},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    mapping = create_map(model.base, data)
    m, fix_cons, x, cover_cons = _build_nearest_open_y_subproblem_lp(
        data, model.base, mapping, model.requests, nothing, model.feasible_pairs,
        model.columns, model.hat, optimizer_env, false;
        lambda_binary=model.lambda_binary,
    )
    metadata = Dict{String, Any}("fix_cons" => fix_cons, "x" => x, "cover_cons" => cover_cons)
    return BuildResult(m, mapping, nothing, nothing, metadata)
end

"""
    run_opt(instance, problem::BendersSubproblemModel{BendersY}, solver::DirectSolver) -> OptResult

Builds and solves the subproblem LP once (no repricing loop here -- that's an outer-loop
concern: call this repeatedly with a growing `columns` pool, exactly as
`_solve_nearest_open_y_subproblem_lp_with_repricing` already does by calling
`_build_nearest_open_y_subproblem_lp` in a loop). `OptResult.objective_value` is `v_hat`;
`OptResult.duals` is `rho`, the fixing-constraint duals keyed by station id -- the
subgradient the Benders cut needs. Does not reuse `_run_opt_impl`, since that hardcodes
`m[:x]`/`m[:y]` solution extraction for the compact model's shape and this subproblem's
`x` is a local dict, not `m[:x]`.
"""
function run_opt(
    instance::StationSelectionData,
    problem::BendersSubproblemModel{BendersY},
    solver::DirectSolver,
)::OptResult
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    start = time()
    build_result = build_model(problem, instance; optimizer_env=optimizer_env)
    m = build_result.model
    cfg.silent && set_silent(m)
    optimize!(m)
    runtime_sec = time() - start
    term_status = termination_status(m)
    if primal_status(m) != MOI.FEASIBLE_POINT
        return OptResult(
            term_status, nothing, nothing, runtime_sec, m, build_result.mapping,
            nothing, nothing, nothing, build_result.metadata,
        )
    end
    fix_cons = build_result.metadata["fix_cons"]
    rho = Dict(j => dual(con) for (j, con) in fix_cons)
    return OptResult(
        term_status, objective_value(m), nothing, runtime_sec, m, build_result.mapping,
        nothing, nothing, nothing, build_result.metadata, rho,
    )
end

# ---------------------------------------------------------------------------
# BendersXY: master = y,x (full nearest-open resolution + assignment, or free
# assignment); subproblem = theta (route covering) only. Two master variants
# selected by base.assignment_policy -- no lifted_walking_objective/
# direct_enumeration_guide/route_regularization_weight_schedule support needed
# (all three are forbidden for BendersXY at BendersSolver construction), and
# cut_derivation is ignored (always the plain subgradient cut, see BendersXY's
# own docstring), so this build_model is simpler than BendersY's.
# ---------------------------------------------------------------------------

function build_model(
    model::BendersMasterModel{BendersXY},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    base = model.base
    cut_ids = model.cut_ids
    mapping = create_map(base, data)
    requests, _demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteProblem Benders requires positive demand"))

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

    x = if base.assignment_policy isa NearestOpenAggregateODAssignmentPolicy
        if _is_endpoint_nearest_style(base.assignment_policy.feasibility_cut_style)
            validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=base.allow_walk_only)
        else
            assert_no_walk_only_pairs(mapping, "AggregateODRouteProblem Benders (BendersXY, NearestOpen, :pair_chain)")
        end
        _add_nearest_open_master_x!(master, data, base, y, requests, feasible_pairs)
    else
        for request in requests
            isempty(feasible_pairs[request]) &&
                throw(ArgumentError("BendersXY master has no open feasible pair candidate for $(request)"))
        end
        _add_unrestricted_master_x!(master, y, requests, feasible_pairs)
    end
    master[:x] = x

    set_benders_xy_master_objective!(master, data, x, theta, cut_ids, requests, feasible_pairs, base.walk_cost_weight)

    counts = ModelCounts(variable_counts, constraint_counts, Dict{String, Int}())
    metadata = Dict{String, Any}("requests" => requests, "feasible_pairs" => feasible_pairs)
    return BuildResult(master, mapping, nothing, counts, metadata)
end

function build_model(
    model::BendersSubproblemModel{BendersXY},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    m, fix_cons = _build_xy_route_subproblem_lp(
        data, model.base, model.requests, model.feasible_pairs, model.columns, model.hat, optimizer_env, false,
    )
    mapping = create_map(model.base, data)
    metadata = Dict{String, Any}("fix_cons" => fix_cons)
    return BuildResult(m, mapping, nothing, nothing, metadata)
end

"""
    run_opt(instance, problem::BendersSubproblemModel{BendersXY}, solver::DirectSolver) -> OptResult

`x` is fixed fully (unlike `BendersY`, where only `y` is fixed and `x` stays free) -- no
repricing companion exists or is needed for `BendersXY`, since CG-priming against a fully
fixed assignment is always exhaustive for exactly this LP's own dual structure (see
`BendersXY`'s docstring). `OptResult.duals` is `rho`, keyed by `(request, pair)`.
"""
function run_opt(
    instance::StationSelectionData,
    problem::BendersSubproblemModel{BendersXY},
    solver::DirectSolver,
)::OptResult
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    start = time()
    build_result = build_model(problem, instance; optimizer_env=optimizer_env)
    m = build_result.model
    cfg.silent && set_silent(m)
    optimize!(m)
    runtime_sec = time() - start
    term_status = termination_status(m)
    if primal_status(m) != MOI.FEASIBLE_POINT
        return OptResult(
            term_status, nothing, nothing, runtime_sec, m, build_result.mapping,
            nothing, nothing, nothing, build_result.metadata,
        )
    end
    fix_cons = build_result.metadata["fix_cons"]
    rho = Dict(key => dual(con) for (key, con) in fix_cons)
    return OptResult(
        term_status, objective_value(m), nothing, runtime_sec, m, build_result.mapping,
        nothing, nothing, nothing, build_result.metadata, rho,
    )
end

# ---------------------------------------------------------------------------
# BendersCorePointProblem / BendersCompletionProblem: cut-derivation LPs over
# the cut's own dual algebra, not station selection -- AbstractBendersDualProblem.
# Both delegate their real work to the existing, unmodified `y_mw_cut.jl`
# functions (`_y_master_core_point`, `_restricted_mw_completion_lp`/
# `_solve_restricted_mw_completion`) rather than re-deriving that algebra --
# these are the most numerically delicate LPs in the package (tie-breaking,
# degenerate-face reasoning extensively documented there), and a from-scratch
# transcription here would risk exactly the kind of subtle sign/indexing bug
# that algebra's own docstrings describe fixing in the past.
# ---------------------------------------------------------------------------

"""
    BendersCorePointProblem{D<:AbstractBendersDecomposition} <: AbstractBendersDualProblem

A relative-interior point of the Benders master's permanent structural region
(`Y_LP = {sum(y)==l, 0<=y<=1, endpoint rows}` for `BendersY`), used as the Magnanti-Wong
target point for `cut_derivation=:restricted_mw_fixed_pi`. `_y_master_core_point`'s
affine-hull-probe-then-max-min-slack procedure is a multi-`optimize!` computation with no
clean build/solve boundary, so `build_model` here is a placeholder satisfying the
`AbstractOptimizationProblem` interface -- the actual computation happens in `run_opt`,
which delegates to `_y_master_core_point` directly.
"""
struct BendersCorePointProblem{D <: AbstractBendersDecomposition} <: AbstractBendersDualProblem
    base::AggregateODRouteProblem
    requests::Vector{NTuple{3, Int}}
    decomposition::D
    affine_hull_tol::Float64
    core_point_tol::Float64

    function BendersCorePointProblem(
        base::AggregateODRouteProblem,
        requests::Vector{NTuple{3, Int}},
        decomposition::D;
        affine_hull_tol::Float64=1e-7,
        core_point_tol::Float64=1e-7,
    ) where {D <: AbstractBendersDecomposition}
        new{D}(base, requests, decomposition, affine_hull_tol, core_point_tol)
    end
end

function build_model(
    problem::BendersCorePointProblem{BendersY},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    return BuildResult(m, EmptyStationSelectionMap(), nothing, nothing, Dict{String, Any}())
end

"""
    run_opt(instance, problem::BendersCorePointProblem{BendersY}, solver::DirectSolver) -> OptResult

Delegates to `_y_master_core_point` unmodified. `OptResult.solution` is
`(y_core, delta)`; the full `AggregateODRouteYCorePoint` (with `fixed_zero`/`fixed_one`/
`n_endpoint_rows`/`n_always_tight_endpoint_rows`) is in `metadata["core_point"]`.
"""
function run_opt(
    instance::StationSelectionData,
    problem::BendersCorePointProblem{BendersY},
    solver::DirectSolver,
)::OptResult
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    start = time()
    core_point = _y_master_core_point(
        instance, problem.base, problem.requests, optimizer_env, cfg.silent;
        affine_hull_tol=problem.affine_hull_tol, core_point_tol=problem.core_point_tol,
    )
    runtime_sec = time() - start
    return OptResult(
        MOI.OPTIMAL, core_point.delta, (core_point.y, core_point.delta), runtime_sec,
        Model(), EmptyStationSelectionMap(), nothing, nothing, nothing,
        Dict{String, Any}(
            "core_point" => core_point,
            "fixed_zero" => core_point.fixed_zero,
            "fixed_one" => core_point.fixed_one,
            "n_endpoint_rows" => core_point.n_endpoint_rows,
            "n_always_tight_endpoint_rows" => core_point.n_always_tight_endpoint_rows,
        ),
    )
end

"""
    BendersCompletionProblem{D<:AbstractBendersDecomposition} <: AbstractBendersDualProblem

Completes a fixed route-covering dual block `pi_full` (from CG certification, an
outer-loop concern -- see `_certified_qbar`/`_certified_route_covering_pi`) into a full
Benders cut, either Magnanti-Wong-maximized at `core` (`objective_mode=:maximize_core`)
or the `:zero_completion` baseline (`objective_mode=:zero`), subject to tightness at
`hat`. `hat`/`core` are deliberately untyped: `y_hat::Vector{Float64}`/
`y_core::Vector{Float64}` for `BendersY`, `z_hat::Dict{...}`/`z_core::AbstractDict` for
`BendersYZ`. `build_model` calls the decomposition's completion-LP builder (unmodified)
for inspection; `run_opt` calls the higher-level completion solver (which itself calls the
LP builder again) rather than reusing `build_model`'s result, so that the extensive
post-solve diagnostics (IIS dump on infeasibility for `BendersY`, tightness verification,
`cut_constant`/`beta` extraction) stay exactly as they are today -- a minor duplication of
LP construction traded for zero risk of diverging from that logic.
"""
struct BendersCompletionProblem{D <: AbstractBendersDecomposition} <: AbstractBendersDualProblem
    base::AggregateODRouteProblem
    requests::Vector{NTuple{3, Int}}
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}}
    hat
    core
    pi_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64}
    Q_bar::Float64
    objective_mode::Symbol
    decomposition::D

    function BendersCompletionProblem(
        base::AggregateODRouteProblem,
        requests::Vector{NTuple{3, Int}},
        feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
        hat,
        core,
        pi_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
        Q_bar::Float64,
        objective_mode::Symbol,
        decomposition::D,
    ) where {D <: AbstractBendersDecomposition}
        objective_mode in (:maximize_core, :zero) ||
            throw(ArgumentError("unsupported objective_mode $(objective_mode)"))
        new{D}(base, requests, feasible_pairs, hat, core, pi_full, Q_bar, objective_mode, decomposition)
    end
end

function build_model(
    problem::BendersCompletionProblem{BendersY},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    built = _restricted_mw_completion_lp(
        data, problem.base, problem.requests, problem.feasible_pairs, problem.hat, problem.core,
        problem.pi_full, problem.Q_bar, problem.objective_mode, optimizer_env, false,
    )
    return BuildResult(built.model, EmptyStationSelectionMap(), nothing, nothing, Dict{String, Any}("built" => built))
end

"""
    run_opt(instance, problem::BendersCompletionProblem{BendersY}, solver::DirectSolver) -> OptResult

Delegates to `_solve_restricted_mw_completion` unmodified. `OptResult.objective_value` is
`cut_constant`; `OptResult.duals` is `beta` (`Dict{Int,Float64}`, keyed by station id --
`nothing` when `status != :optimal`); `metadata` carries `phi_core`/`phi_ybar`/`status`.
"""
function run_opt(
    instance::StationSelectionData,
    problem::BendersCompletionProblem{BendersY},
    solver::DirectSolver,
)::OptResult
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    completion = _solve_restricted_mw_completion(
        instance, problem.base, problem.requests, problem.feasible_pairs, problem.hat, problem.core,
        problem.pi_full, problem.Q_bar, problem.objective_mode, optimizer_env, cfg.silent,
    )
    status = completion.status == :optimal ? MOI.OPTIMAL : MOI.INFEASIBLE
    return OptResult(
        status, completion.cut_constant, nothing, completion.runtime_sec,
        Model(), EmptyStationSelectionMap(), nothing, nothing, nothing,
        Dict{String, Any}(
            "phi_core" => completion.phi_core, "phi_ybar" => completion.phi_ybar,
            "status" => completion.status,
        ),
        completion.status == :optimal ? completion.beta : nothing,
    )
end

# ---------------------------------------------------------------------------
# BendersYZ: master = y,z; subproblem = x,theta. Needs its own core-point/
# completion LP (joint (y,z) structural region, no lambda/mu/nu chain block --
# see yz_mw_cut.jl's module docstring for why it differs from BendersY's).
# ---------------------------------------------------------------------------

"""
    run_opt(instance, problem::BendersCorePointProblem{BendersYZ}, solver::DirectSolver) -> OptResult

Delegates to `_yz_joint_core_point` unmodified. `OptResult.solution` is
`(y_core, delta)`; the full `AggregateODRouteYZCorePoint` (with `.z::Dict{Any,Vector{Float64}}`,
`fixed_zero`/`fixed_one`/row-count diagnostics) is in `metadata["core_point"]`.
"""
function run_opt(
    instance::StationSelectionData,
    problem::BendersCorePointProblem{BendersYZ},
    solver::DirectSolver,
)::OptResult
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    start = time()
    core_point = _yz_joint_core_point(
        instance, problem.base, problem.requests, optimizer_env, cfg.silent;
        affine_hull_tol=problem.affine_hull_tol, core_point_tol=problem.core_point_tol,
    )
    runtime_sec = time() - start
    return OptResult(
        MOI.OPTIMAL, core_point.delta, (core_point.y, core_point.delta), runtime_sec,
        Model(), EmptyStationSelectionMap(), nothing, nothing, nothing,
        Dict{String, Any}(
            "core_point" => core_point,
            "fixed_zero" => core_point.fixed_zero,
            "fixed_one" => core_point.fixed_one,
            "n_endpoint_rows" => core_point.n_endpoint_rows,
            "n_always_tight_endpoint_rows" => core_point.n_always_tight_endpoint_rows,
        ),
    )
end

function build_model(
    problem::BendersCompletionProblem{BendersYZ},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    built = _yz_completion_lp(
        data, problem.base, problem.requests, problem.feasible_pairs, problem.hat, problem.core,
        problem.pi_full, problem.Q_bar, problem.objective_mode, optimizer_env, false,
    )
    return BuildResult(built.model, EmptyStationSelectionMap(), nothing, nothing, Dict{String, Any}("built" => built))
end

"""
    run_opt(instance, problem::BendersCompletionProblem{BendersYZ}, solver::DirectSolver) -> OptResult

Delegates to `_solve_yz_completion` unmodified. `OptResult.objective_value` is
`cut_constant`; `OptResult.duals` is `beta` (`Dict{Tuple{Any,Int},Float64}`, keyed by
endpoint-chain `(key, index)` -- `z` is the fixed master variable directly here, unlike
`BendersY`'s `y`-keyed `beta`).
"""
function run_opt(
    instance::StationSelectionData,
    problem::BendersCompletionProblem{BendersYZ},
    solver::DirectSolver,
)::OptResult
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    completion = _solve_yz_completion(
        instance, problem.base, problem.requests, problem.feasible_pairs, problem.hat, problem.core,
        problem.pi_full, problem.Q_bar, problem.objective_mode, optimizer_env, cfg.silent,
    )
    status = completion.status == :optimal ? MOI.OPTIMAL : MOI.INFEASIBLE
    return OptResult(
        status, completion.cut_constant, nothing, completion.runtime_sec,
        Model(), EmptyStationSelectionMap(), nothing, nothing, nothing,
        Dict{String, Any}(
            "phi_core" => completion.phi_core, "phi_zhat" => completion.phi_zhat,
            "status" => completion.status,
        ),
        completion.status == :optimal ? completion.beta : nothing,
    )
end

function build_model(
    model::BendersMasterModel{BendersYZ},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    base = model.base
    solver = model.solver
    cut_ids = model.cut_ids
    _is_endpoint_nearest_style(base.assignment_policy.feasibility_cut_style) ||
        throw(ArgumentError(
            "BendersYZ requires NearestOpenAggregateODAssignmentPolicy(:big_m_nearest) or " *
            "(:endpoint_chain); got :$(base.assignment_policy.feasibility_cut_style) -- :pair_chain has no " *
            "addressable z separate from x, so there is nothing for BendersYZ's master to lift."
        ))
    mapping = create_map(base, data)
    validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=base.allow_walk_only)
    requests, _demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteProblem nearest-open Benders requires positive demand"))

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
        # _seed_yz_cuts! reads master[:nearest_endpoint_chain_cache], populated by the call
        # just above -- must run after it, not before. Matches the original
        # _run_aggregate_od_route_nearest_open_benders_yz, which likewise only replays seed
        # cuts in this branch (seed_cuts/direct_enumeration_guide are only ever used together
        # with lifted_walking_objective=true in practice).
        isempty(model.seed_cuts) || _seed_yz_cuts!(master, theta, model.seed_cuts)
        direct_cost_expr = AffExpr(0.0)
        if !isnothing(model.direct_enumeration_pool)
            direct_cost_expr = _add_direct_enumeration_guide!(
                master, data, base, requests, feasible_pairs, x_by_pair_full, model.direct_enumeration_pool;
                relax_integrality=solver.direct_enumeration_relax_integrality,
            )
        end
        set_benders_lifted_master_objective!(
            master, theta, cut_ids, walking_cost_expr, direct_cost_expr, current_beta;
            lifted_walking_objective=true,
        )
        master[:walking_cost_expr] = walking_cost_expr
        master[:direct_cost_expr] = direct_cost_expr
    else
        _add_nearest_open_master_z!(
            master, data, y, requests, feasible_pairs, base.max_walking_distance, base.allow_walk_only,
            base.assignment_policy.feasibility_cut_style,
        )
        set_benders_lifted_master_objective!(
            master, theta, cut_ids, AffExpr(0.0), AffExpr(0.0), current_beta;
            lifted_walking_objective=false,
        )
    end
    master[:benders_beta_schedule] = beta_schedule
    master[:benders_cut_ids] = cut_ids

    counts = ModelCounts(variable_counts, constraint_counts, Dict{String, Int}())
    metadata = Dict{String, Any}("requests" => requests, "feasible_pairs" => feasible_pairs)
    return BuildResult(master, mapping, nothing, counts, metadata)
end

function build_model(
    model::BendersSubproblemModel{BendersYZ},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    m, fix_cons, cover_cons = _build_yz_route_subproblem_lp(
        data, model.base, model.requests, model.feasible_pairs, model.columns, model.hat, optimizer_env, false,
    )
    mapping = create_map(model.base, data)
    metadata = Dict{String, Any}("fix_cons" => fix_cons, "cover_cons" => cover_cons)
    return BuildResult(m, mapping, nothing, nothing, metadata)
end

"""
    run_opt(instance, problem::BendersSubproblemModel{BendersYZ}, solver::DirectSolver) -> OptResult

`z` is fixed (via `hat` = `z_hat::Dict{_AggregateODRouteEndpointChainKey,Vector{Float64}}`);
`x`/`lambda` stay free, so this subproblem has the same structural completeness gap as
`BendersY`'s -- callers needing a provably optimal result should route through the
repricing path (`_benders_solve_subproblem`'s `BendersYZ` method) rather than this
single-solve `run_opt`, exactly as with `BendersY`.
"""
function run_opt(
    instance::StationSelectionData,
    problem::BendersSubproblemModel{BendersYZ},
    solver::DirectSolver,
)::OptResult
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    start = time()
    build_result = build_model(problem, instance; optimizer_env=optimizer_env)
    m = build_result.model
    cfg.silent && set_silent(m)
    optimize!(m)
    runtime_sec = time() - start
    term_status = termination_status(m)
    if primal_status(m) != MOI.FEASIBLE_POINT
        return OptResult(
            term_status, nothing, nothing, runtime_sec, m, build_result.mapping,
            nothing, nothing, nothing, build_result.metadata,
        )
    end
    fix_cons = build_result.metadata["fix_cons"]
    rho = Dict(key => dual(con) for (key, con) in fix_cons)
    return OptResult(
        term_status, objective_value(m), nothing, runtime_sec, m, build_result.mapping,
        nothing, nothing, nothing, build_result.metadata, rho,
    )
end

# ---------------------------------------------------------------------------
# BendersYZH: master = y,z,h (h scenario-compressed per physical OD pair);
# subproblem = theta only. No core-point/completion LP types needed at all --
# :zero_completion is a plain certified-dual sum (_zero_completion_yzh_rho,
# generic_runner.jl), and :restricted_mw_fixed_pi is rejected at
# BendersSolver construction (no free dual block left once h is fixed).
# ---------------------------------------------------------------------------

function build_model(
    model::BendersMasterModel{BendersYZH},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    base = model.base
    cut_ids = model.cut_ids
    _is_endpoint_nearest_style(base.assignment_policy.feasibility_cut_style) ||
        throw(ArgumentError(
            "BendersYZH requires NearestOpenAggregateODAssignmentPolicy(:big_m_nearest) or " *
            "(:endpoint_chain); got :$(base.assignment_policy.feasibility_cut_style) -- :pair_chain has no " *
            "addressable z separate from x, so there is nothing for BendersYZH's master to lift."
        ))
    mapping = create_map(base, data)
    validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=base.allow_walk_only)
    requests, _demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteProblem nearest-open Benders requires positive demand"))
    physical_pairs, occurrences, feasible_pairs_by_p = _aggregate_od_route_benders_physical_pairs(mapping)
    occurrence_count = Dict(p => length(occurrences[p]) for p in physical_pairs)

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
    h = _add_nearest_open_master_h!(
        master, data, y, physical_pairs, feasible_pairs_by_p, base.max_walking_distance, base.allow_walk_only,
        base.assignment_policy.feasibility_cut_style,
    )
    master[:h] = h

    walking_expr = physical_pair_walking_cost_expr(
        data, physical_pairs, feasible_pairs_by_p, occurrence_count, h; weight=base.walk_cost_weight,
    )
    set_benders_yzh_master_objective!(master, walking_expr, theta, cut_ids)

    counts = ModelCounts(variable_counts, constraint_counts, Dict{String, Int}())
    metadata = Dict{String, Any}("requests" => requests, "feasible_pairs" => feasible_pairs)
    return BuildResult(master, mapping, nothing, counts, metadata)
end

function build_model(
    model::BendersSubproblemModel{BendersYZH},
    data::StationSelectionData;
    optimizer_env=nothing,
)::BuildResult
    isnothing(optimizer_env) && (optimizer_env = Gurobi.Env())
    feasible_pairs_by_p = _yzh_feasible_pairs_by_p(model.feasible_pairs, model.requests)
    m, fix_cons, cover_cons = _build_yzh_route_subproblem_lp(
        data, model.base, model.requests, feasible_pairs_by_p, model.columns, model.hat, optimizer_env, false,
    )
    mapping = create_map(model.base, data)
    metadata = Dict{String, Any}("fix_cons" => fix_cons, "cover_cons" => cover_cons)
    return BuildResult(m, mapping, nothing, nothing, metadata)
end

"""
    run_opt(instance, problem::BendersSubproblemModel{BendersYZH}, solver::DirectSolver) -> OptResult

`h` is fixed fully (like `BendersXY`'s `x`), so CG-priming is structurally exhaustive for
this LP -- callers wanting the extra empirical soundness check against dual-basis degeneracy
should still route through `_benders_solve_subproblem`'s repricing branch
(`solver.reprice_subproblem=true`), exactly as with `BendersY`/`BendersYZ`.
"""
function run_opt(
    instance::StationSelectionData,
    problem::BendersSubproblemModel{BendersYZH},
    solver::DirectSolver,
)::OptResult
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    start = time()
    build_result = build_model(problem, instance; optimizer_env=optimizer_env)
    m = build_result.model
    cfg.silent && set_silent(m)
    optimize!(m)
    runtime_sec = time() - start
    term_status = termination_status(m)
    if primal_status(m) != MOI.FEASIBLE_POINT
        return OptResult(
            term_status, nothing, nothing, runtime_sec, m, build_result.mapping,
            nothing, nothing, nothing, build_result.metadata,
        )
    end
    fix_cons = build_result.metadata["fix_cons"]
    rho = Dict(key => dual(con) for (key, con) in fix_cons)
    return OptResult(
        term_status, objective_value(m), nothing, runtime_sec, m, build_result.mapping,
        nothing, nothing, nothing, build_result.metadata, rho,
    )
end
