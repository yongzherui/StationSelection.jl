"""
Generic Benders decomposition outer loop, shared by every `AbstractBendersDecomposition`
over `AggregateODRouteModel`. The loop shape (solve master, derive priming assignments,
solve each cut group's subproblem, certify/derive/attach a cut, check convergence) is
identical across `BendersY`/`BendersXY`/`BendersYZ`/`BendersYZH`; what differs is (1) what
gets fixed in the subproblem and read off the master, (2) how to derive CG-priming
assignments from that fixed point, (3) how to build the subproblem, and (4) how to derive
and attach the optimality cut. Those four steps are factored into small functions dispatched
on the decomposition type (`_benders_hat_point`, `_benders_priming_assignments`,
`_benders_solve_subproblem`, `_benders_add_optimality_cut!`, below) -- `BendersY` has real
methods; `BendersXY`/`BendersYZ`/`BendersYZH` (Phase 2+) plug in by adding their own methods
for these four functions, not by copying `_run_benders_decomposition` itself.

`_benders_solve_subproblem` deliberately keeps `solver.reprice_subproblem=true` routed
through the existing, unmodified `_solve_nearest_open_y_subproblem_lp_with_repricing` (a
multi-solve certification loop with no clean single build/solve shape) rather than
reimplementing repricing against `BendersSubproblemModel` -- the non-repricing path (the
default, and the only one required for correctness under `cut_derivation ∈
(:zero_completion, :restricted_mw_fixed_pi)`, see `BendersY`/`BendersYZ`'s own docstrings)
goes through the new `run_opt`-based `BendersSubproblemModel`.
"""

# ---------------------------------------------------------------------------
# Decomposition-dispatched hooks. BendersY/BendersXY methods; BendersYZ/YZH add
# their own methods here in later passes.
# ---------------------------------------------------------------------------

"""
    _benders_uses_certified_cut_derivation(decomposition, solver) -> Bool

Whether this decomposition's optimality cut ever needs `_certified_qbar`-based `v_hat`
tightening. Default (`BendersY`/`BendersYZ`): whenever `solver.cut_derivation != :standard`.
`BendersXY` overrides to always `false` -- its `cut_derivation` field is documented as
ignored, always using the plain subgradient cut.
"""
_benders_uses_certified_cut_derivation(::D, solver::BendersSolver) where {D <: AbstractBendersDecomposition} =
    solver.cut_derivation != :standard
_benders_uses_certified_cut_derivation(::BendersXY, ::BendersSolver) = false

"""
    _benders_needs_core_point(decomposition, solver) -> Bool

Whether this decomposition's cut derivation needs a Magnanti-Wong-style core point
(`BendersCorePointProblem`). Default mirrors `_benders_uses_certified_cut_derivation`
(true for `BendersY`/`BendersYZ` whenever `cut_derivation != :standard`); overridden
separately for decompositions (`BendersXY` here, `BendersYZH` later) that either never
certify or certify without ever needing a core point (`BendersYZH` has no free dual block
left once `h` is fixed, so its `:zero_completion` mode needs no core point at all).
"""
_benders_needs_core_point(::D, solver::BendersSolver) where {D <: AbstractBendersDecomposition} =
    solver.cut_derivation != :standard
_benders_needs_core_point(::BendersXY, ::BendersSolver) = false

"""
    _benders_residual_lower_bound_value(master, decomposition, cut_id) -> Float64

Extra, already-live lower-bound term folded into this cut group's `theta`/`eta` at the
master, used only when deciding whether a cut is needed
(`theta_hat[cut_id] + this < v_hat - tol`). Default `0.0` for decompositions without such a
term. `BendersYZ` overrides to read `value(master[:route_lb_exprs][cut_id])` when
`lifted_routing_lower_bound`/`common_od_mcf_lower_bound` built one (see
`BendersMasterModel{BendersYZ}`'s `build_model`, which always stashes
`master[:route_lb_exprs]`, `nothing` when neither feature is active).
"""
_benders_residual_lower_bound_value(::Model, ::D, ::Int) where {D <: AbstractBendersDecomposition} = 0.0

"""
    _benders_tighten_subproblem_value(data, subproblem_model, decomposition, solver, cg_result,
        group_requests, feasible_pairs, assignments, v_hat) -> (v_hat, certification_kwargs::NamedTuple)

Only called when `_benders_uses_certified_cut_derivation` is true. Tightens `v_hat` using
CG's own already-certified duals from `cg_result` (this iteration's priming solve, already
proven `cg_stop_reason==:optimality_proven`) rather than trusting the subproblem LP's own
pool completeness, and returns a `NamedTuple` of keyword arguments to splat into
`_benders_add_optimality_cut!` so the (expensive) certification isn't recomputed there --
each decomposition's cut-attach method accepts whatever shape its own certification takes
(`BendersY`/`BendersYZ`: `certified`/`Q_bar`, matching `_certified_qbar`'s return; `BendersYZH`:
a single `zc_result`, since `_zero_completion_yzh_rho` bundles a differently-adjusted `Q_bar`
or which `_certified_qbar` alone would double-count `h`'s walking cost -- see that function's
docstring). Default (`BendersY`/`BendersYZ`) uses `_certified_qbar` directly.
"""
function _benders_tighten_subproblem_value(
    data::StationSelectionData,
    subproblem_model::AggregateODRouteModel,
    ::D,
    solver::BendersSolver,
    cg_result,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    v_hat::Float64,
) where {D <: AbstractBendersDecomposition}
    certified, qbar = _certified_qbar(data, subproblem_model, cg_result, group_requests, assignments)
    return min(v_hat, qbar), (certified=certified, Q_bar=qbar)
end

"""
    _benders_hat_point(master::Model, decomposition) -> NamedTuple

Rounded fixed-decision point read off a solved master -- `(y_hat=...,)` for `BendersY`,
`(y_hat=..., z_hat=...)` for `BendersYZ`, etc. Every decomposition's master has `y`, so
`hat.y_hat` always exists (used generically by `_benders_y_hat_bookkeeping`).
"""
function _benders_hat_point(master::Model, ::BendersY)
    n = length(master[:y])
    return (y_hat=[round(value(master[:y][j])) for j in 1:n],)
end

"""
    _benders_priming_assignments(data, model, decomposition, requests, feasible_pairs, hat)
        -> (assignments, open_stations)

Derives the fixed-assignment map CG-priming needs from the master's solved `hat` point.
`BendersY`/`BendersYZ` resolve it via nearest-open station selection from `y_hat`
(`_fixed_assignments_from_y`); `BendersXY`/`BendersYZH` (Phase 2+) will read it directly off
their own `x_hat`/`h_hat` instead.
"""
function _benders_priming_assignments(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    ::BendersY,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    hat,
)
    assignments, infeasible = _fixed_assignments_from_y(
        data, requests, feasible_pairs, hat.y_hat;
        style=model.assignment_policy.feasibility_cut_style,
        max_walking_distance=model.max_walking_distance,
        allow_walk_only=model.allow_walk_only, allow_same_station=true,
    )
    # The master's own eager `_add_default_endpoint_coverage_constraints!` makes every
    # request resolve to a real pair by construction -- see y.jl's original outer loop for
    # the identical reasoning; this is a correctness assertion, not reactive cut derivation.
    isempty(infeasible) || throw(ArgumentError(
        "BendersY: y_hat=$(hat.y_hat) left requests infeasible ($(infeasible)); this should be " *
        "structurally impossible given the master's eager endpoint-coverage constraints"
    ))
    return assignments, _open_station_values(hat.y_hat)
end

"""
    _benders_solve_subproblem(data, subproblem_model, mapping, decomposition, hat,
        group_requests, feasible_pairs, columns, solver, optimizer_env, silent)
        -> (v_hat, duals, pool, n_new_columns, exhausted)

One cut group's subproblem solve. `pool`/`n_new_columns` only change under
`solver.reprice_subproblem=true` (the repricing branch grows the shared column pool);
otherwise `pool` is returned unchanged and `n_new_columns=0`.
"""
function _benders_solve_subproblem(
    data::StationSelectionData,
    subproblem_model::AggregateODRouteModel,
    mapping::AggregateODRouteMap,
    decomposition::BendersY,
    hat,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    solver::BendersSolver,
    optimizer_env,
    silent::Bool,
)
    if solver.reprice_subproblem
        v_hat, rho, pool, n_new, _rounds, exhausted, _delta = _solve_nearest_open_y_subproblem_lp_with_repricing(
            data, subproblem_model, mapping, group_requests, nothing, feasible_pairs, columns, hat.y_hat,
            optimizer_env, silent; max_reprice_rounds=solver.max_reprice_rounds,
        )
        return v_hat, rho, pool, n_new, exhausted
    end
    sub_problem = BendersSubproblemModel(subproblem_model, hat.y_hat, group_requests, feasible_pairs, columns, decomposition)
    sub_result = run_opt(data, sub_problem, DirectSolver(config=SolverConfig(optimizer_env=optimizer_env, silent=silent)))
    sub_result.termination_status == MOI.OPTIMAL ||
        throw(ArgumentError("BendersY subproblem failed with status $(sub_result.termination_status)"))
    return sub_result.objective_value, sub_result.duals, columns, 0, true
end

"""
    _benders_add_optimality_cut!(master, decomposition, cut_id, data, base, solver,
        group_requests, feasible_pairs, hat, assignments, open_stations, core_point,
        optimizer_env, v_hat, sub_duals; certified, Q_bar, certification_already_failed)

Derives and attaches one optimality cut. Delegates to the existing, unmodified
`_add_aggregate_od_route_benders_y_optimality_cut!` for `BendersY` -- dispatches internally
on `solver.cut_derivation` (`:standard`/`:zero_completion`/`:restricted_mw_fixed_pi`), calling
into `BendersCompletionProblem`'s underlying `_solve_restricted_mw_completion` for the two
restricted-completion modes.
"""
function _benders_add_optimality_cut!(
    master::Model,
    decomposition::BendersY,
    cut_id::Int,
    data::StationSelectionData,
    base::AggregateODRouteModel,
    solver::BendersSolver,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    hat,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    core_point::Union{Nothing, AggregateODRouteYCorePoint},
    optimizer_env,
    v_hat::Float64,
    sub_duals::Dict,
    cg_result;
    certified=nothing,
    Q_bar=nothing,
    certification_already_failed::Bool=false,
)
    y = master[:y]
    theta = master[:theta]
    return _add_aggregate_od_route_benders_y_optimality_cut!(
        master, y, theta, cut_id, data, base, solver, group_requests, feasible_pairs,
        hat.y_hat, assignments, open_stations, core_point, optimizer_env, v_hat, sub_duals;
        certified=certified, Q_bar=Q_bar, certification_already_failed=certification_already_failed,
    )
end

# ---------------------------------------------------------------------------
# BendersXY hooks: master fixes y AND x jointly, so the subproblem has no free
# assignment variable at all (only route selection) -- no feasibility-cut
# machinery, no repricing, no core-point/completion (cut_derivation ignored).
# ---------------------------------------------------------------------------

function _benders_hat_point(master::Model, ::BendersXY)
    n = length(master[:y])
    y_hat = [round(value(master[:y][j])) for j in 1:n]
    x_hat = Dict(key => round(value(var)) for (key, var) in master[:x])
    return (y_hat=y_hat, x_hat=x_hat)
end

function _benders_priming_assignments(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    ::BendersXY,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    hat,
)
    assignments = _selected_assignments_from_x(requests, feasible_pairs, hat.x_hat)
    return assignments, _open_station_values(hat.y_hat)
end

"""
    _benders_solve_subproblem(..., decomposition::BendersXY, ...) -> (v_hat, duals, pool, n_new, exhausted)

`x` is fixed fully in the subproblem, so CG-priming is always exhaustive for this LP's own
dual structure (see `BendersXY`'s docstring) -- no repricing companion exists, unlike
`BendersY`. `pool`/`n_new` are always unchanged/`0`.
"""
function _benders_solve_subproblem(
    data::StationSelectionData,
    subproblem_model::AggregateODRouteModel,
    mapping::AggregateODRouteMap,
    decomposition::BendersXY,
    hat,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    solver::BendersSolver,
    optimizer_env,
    silent::Bool,
)
    sub_problem = BendersSubproblemModel(subproblem_model, hat.x_hat, group_requests, feasible_pairs, columns, decomposition)
    sub_result = run_opt(data, sub_problem, DirectSolver(config=SolverConfig(optimizer_env=optimizer_env, silent=silent)))
    sub_result.termination_status == MOI.OPTIMAL ||
        throw(ArgumentError("BendersXY subproblem failed with status $(sub_result.termination_status)"))
    return sub_result.objective_value, sub_result.duals, columns, 0, true
end

"""
    _benders_add_optimality_cut!(..., decomposition::BendersXY, ...)

Always the plain subgradient cut (`cut_derivation` is ignored for `BendersXY`, see its own
docstring) -- byte-identical formula to the original inline
`_run_aggregate_od_route_nearest_open_benders_xy`/`_run_aggregate_od_route_free_benders_xy`.
"""
function _benders_add_optimality_cut!(
    master::Model,
    decomposition::BendersXY,
    cut_id::Int,
    data::StationSelectionData,
    base::AggregateODRouteModel,
    solver::BendersSolver,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    hat,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    core_point,
    optimizer_env,
    v_hat::Float64,
    sub_duals::Dict,
    cg_result;
    certified=nothing,
    Q_bar=nothing,
    certification_already_failed::Bool=false,
)
    x = master[:x]
    x_hat = hat.x_hat
    @constraint(master, master[:theta][cut_id] >= v_hat + sum(sub_duals[key] * (x[key] - get(x_hat, key, 0.0)) for key in keys(sub_duals)))
    cut_constant = v_hat - sum(sub_duals[key] * get(x_hat, key, 0.0) for key in keys(sub_duals); init=0.0)
    return (cut_constant=cut_constant, coeffs=sub_duals)
end

# ---------------------------------------------------------------------------
# BendersYZ hooks: master fixes y AND the nearest-open endpoint selectors z
# (via `_add_nearest_open_master_z!`/`_add_nearest_open_master_walking_cost!`,
# both already populate `master[:nearest_endpoint_chain_cache]`), leaving x/θ
# to the subproblem -- structurally a hybrid of BendersY's feasibility-cut
# reasoning (y_hat alone can still admit an infeasible collision) and
# BendersXY's per-cut-group loop shape. Needs its own core-point/completion
# LP (yz_mw_cut.jl) and, uniquely among the four decompositions, an optional
# routing lower-bound term folded into theta/eta via
# _benders_residual_lower_bound_value.
# ---------------------------------------------------------------------------

function _benders_hat_point(master::Model, ::BendersYZ)
    n = length(master[:y])
    y_hat = [round(value(master[:y][j])) for j in 1:n]
    z_hat = Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}}(
        key => round.(value.(vars)) for (key, vars) in master[:nearest_endpoint_chain_cache]
    )
    return (y_hat=y_hat, z_hat=z_hat)
end

"""
    _benders_priming_assignments(..., decomposition::BendersYZ, ...)

Same derivation as `BendersY`'s: safe to derive CG-priming `assignments` from `y_hat` alone
via `_fixed_assignments_from_y`, ignoring `z_hat`, since the chain constraints make that a
deterministic bijection whenever the master is feasible (see `BendersYZ`'s own docstring).
"""
function _benders_priming_assignments(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    ::BendersYZ,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    hat,
)
    assignments, infeasible = _fixed_assignments_from_y(
        data, requests, feasible_pairs, hat.y_hat;
        style=model.assignment_policy.feasibility_cut_style,
        max_walking_distance=model.max_walking_distance,
        allow_walk_only=model.allow_walk_only, allow_same_station=true,
    )
    isempty(infeasible) || throw(ArgumentError(
        "BendersYZ: y_hat=$(hat.y_hat) left requests infeasible ($(infeasible)); this should be " *
        "structurally impossible given the master's eager endpoint-coverage constraints"
    ))
    return assignments, _open_station_values(hat.y_hat)
end

function _benders_solve_subproblem(
    data::StationSelectionData,
    subproblem_model::AggregateODRouteModel,
    mapping::AggregateODRouteMap,
    decomposition::BendersYZ,
    hat,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    solver::BendersSolver,
    optimizer_env,
    silent::Bool,
)
    if solver.reprice_subproblem
        v_hat, rho, pool, n_new, _rounds, exhausted, _delta = _solve_yz_route_subproblem_lp_with_repricing(
            data, subproblem_model, mapping, group_requests, feasible_pairs, columns, hat.z_hat,
            optimizer_env, silent; max_reprice_rounds=solver.max_reprice_rounds,
        )
        return v_hat, rho, pool, n_new, exhausted
    end
    sub_problem = BendersSubproblemModel(subproblem_model, hat.z_hat, group_requests, feasible_pairs, columns, decomposition)
    sub_result = run_opt(data, sub_problem, DirectSolver(config=SolverConfig(optimizer_env=optimizer_env, silent=silent)))
    sub_result.termination_status == MOI.OPTIMAL ||
        throw(ArgumentError("BendersYZ subproblem failed with status $(sub_result.termination_status)"))
    return sub_result.objective_value, sub_result.duals, columns, 0, true
end

function _benders_residual_lower_bound_value(master::Model, ::BendersYZ, cut_id::Int)
    route_lb_exprs = haskey(master, :route_lb_exprs) ? master[:route_lb_exprs] : nothing
    isnothing(route_lb_exprs) && return 0.0
    return value(route_lb_exprs[cut_id])
end

"""
    _benders_add_optimality_cut!(..., decomposition::BendersYZ, ...)

Delegates to the existing, unmodified `_add_aggregate_od_route_benders_yz_optimality_cut!`,
which already handles subtracting the live `route_lb_expr` from the cut RHS when
`master[:route_lb_exprs]` is present (`nothing` otherwise, a no-op).
"""
function _benders_add_optimality_cut!(
    master::Model,
    decomposition::BendersYZ,
    cut_id::Int,
    data::StationSelectionData,
    base::AggregateODRouteModel,
    solver::BendersSolver,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    hat,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    core_point::Union{Nothing, AggregateODRouteYZCorePoint},
    optimizer_env,
    v_hat::Float64,
    sub_duals::AbstractDict,
    cg_result;
    certified=nothing,
    Q_bar=nothing,
    certification_already_failed::Bool=false,
)
    theta = master[:theta]
    z_core = isnothing(core_point) ? nothing : core_point.z
    route_lb_exprs = haskey(master, :route_lb_exprs) ? master[:route_lb_exprs] : nothing
    route_lb_expr = isnothing(route_lb_exprs) ? nothing : route_lb_exprs[cut_id]
    return _add_aggregate_od_route_benders_yz_optimality_cut!(
        master, theta, cut_id, data, base, solver, group_requests, feasible_pairs,
        hat.z_hat, assignments, open_stations, z_core, optimizer_env, v_hat, sub_duals;
        route_lb_expr=route_lb_expr, certified=certified, Q_bar=Q_bar,
        certification_already_failed=certification_already_failed,
    )
end

# ---------------------------------------------------------------------------
# BendersYZH hooks: master fixes y, z (via h's zp/zd linking), and h itself
# (scenario-compressed per physical OD pair), leaving only theta to the
# subproblem. No lifted_walking_objective variant exists (h already carries
# walking cost directly, unlike x/z), no core point (no free dual block left
# once h is fixed, so :restricted_mw_fixed_pi is rejected at construction),
# and :zero_completion is a plain certified-dual sum
# (_zero_completion_yzh_rho), not a completion LP -- so, uniquely among the
# four decompositions, BendersYZH needs neither a BendersCorePointProblem nor
# a BendersCompletionProblem method at all.
# ---------------------------------------------------------------------------

_benders_needs_core_point(::BendersYZH, ::BendersSolver) = false

"""
    _yzh_feasible_pairs_by_p(feasible_pairs, requests) -> Dict{Tuple{Int,Int},Vector{Tuple{Int,Int}}}

`_build_yzh_route_subproblem_lp`/`_zero_completion_yzh_rho`/the master `h`-builder all key
feasible pairs by *physical* OD pair `(o,d)`, not by the flat `(s,o,d)` request `feasible_pairs`
uses -- safe to derive one from the other since feasible pairs never depend on scenario, only
on the physical endpoints (mirrors `_zero_completion_yzh_rho`'s own identical derivation the
other way).
"""
function _yzh_feasible_pairs_by_p(
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    requests,
)::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
    return Dict((request[2], request[3]) => feasible_pairs[request] for request in requests)
end

function _benders_hat_point(master::Model, ::BendersYZH)
    n = length(master[:y])
    y_hat = [round(value(master[:y][j])) for j in 1:n]
    h_hat = Dict(key => round(value(var)) for (key, var) in master[:h])
    return (y_hat=y_hat, h_hat=h_hat)
end

function _benders_priming_assignments(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    ::BendersYZH,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    hat,
)
    mapping = create_map(model, data)
    physical_pairs, occurrences, feasible_pairs_by_p = _aggregate_od_route_benders_physical_pairs(mapping)
    assignments = _selected_assignments_from_h(physical_pairs, occurrences, feasible_pairs_by_p, hat.h_hat)
    return assignments, _open_station_values(hat.y_hat)
end

"""
    _benders_solve_subproblem(..., decomposition::BendersYZH, ...)

`h` is fixed fully in the subproblem (like `BendersXY`'s `x`), so CG-priming is structurally
exhaustive for exactly this LP -- but `solver.reprice_subproblem` is still fully wired here
(unlike `BendersXY`, which has no repricing companion at all) as an empirical soundness check
against the route-covering LP's own potential dual degeneracy; see `BendersYZH`'s module
docstring for the 2026-07-21 correction explaining why repricing is not actually a structural
non-issue here despite `h` being fully fixed.
"""
function _benders_solve_subproblem(
    data::StationSelectionData,
    subproblem_model::AggregateODRouteModel,
    mapping::AggregateODRouteMap,
    decomposition::BendersYZH,
    hat,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    solver::BendersSolver,
    optimizer_env,
    silent::Bool,
)
    feasible_pairs_by_p = _yzh_feasible_pairs_by_p(feasible_pairs, group_requests)
    if solver.reprice_subproblem
        v_hat, rho, pool, n_new, _rounds, exhausted, _delta = _solve_yzh_route_subproblem_lp_with_repricing(
            data, subproblem_model, mapping, group_requests, feasible_pairs_by_p, columns, hat.h_hat,
            optimizer_env, silent; max_reprice_rounds=solver.max_reprice_rounds,
        )
        return v_hat, rho, pool, n_new, exhausted
    end
    sub_problem = BendersSubproblemModel(subproblem_model, hat.h_hat, group_requests, feasible_pairs, columns, decomposition)
    sub_result = run_opt(data, sub_problem, DirectSolver(config=SolverConfig(optimizer_env=optimizer_env, silent=silent)))
    sub_result.termination_status == MOI.OPTIMAL ||
        throw(ArgumentError("BendersYZH subproblem failed with status $(sub_result.termination_status)"))
    return sub_result.objective_value, sub_result.duals, columns, 0, true
end

"""
    _benders_tighten_subproblem_value(..., decomposition::BendersYZH, ...)

Uses `_zero_completion_yzh_rho` (a certified-dual sum, not `_certified_qbar` directly) --
its `Q_bar` is already adjusted to remove the walking-cost double-count `_certified_qbar`
alone would introduce (`h`'s walking cost is priced once, in the master; see
`_zero_completion_yzh_rho`'s docstring). Payload is a single `zc_result` (a
`(Q_bar, rho, certified)` triple), not `certified`/`Q_bar` separately, matching
`_add_aggregate_od_route_benders_yzh_optimality_cut!`'s own kwarg.
"""
function _benders_tighten_subproblem_value(
    data::StationSelectionData,
    subproblem_model::AggregateODRouteModel,
    ::BendersYZH,
    solver::BendersSolver,
    cg_result,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    v_hat::Float64,
)
    feasible_pairs_by_p = _yzh_feasible_pairs_by_p(feasible_pairs, group_requests)
    zc_result = _zero_completion_yzh_rho(data, subproblem_model, cg_result, group_requests, feasible_pairs_by_p, assignments)
    return min(v_hat, zc_result[1]), (zc_result=zc_result,)
end

"""
    _benders_add_optimality_cut!(..., decomposition::BendersYZH, ...)

Delegates to the existing, unmodified `_add_aggregate_od_route_benders_yzh_optimality_cut!`.
Its own `cg_result` parameter is only used internally when `zc_result` is `nothing` (i.e.
`solver.cut_derivation == :standard`, where `_benders_tighten_subproblem_value` never runs and
`zc_result` is never pre-supplied) -- passed through here rather than re-derived.
"""
function _benders_add_optimality_cut!(
    master::Model,
    decomposition::BendersYZH,
    cut_id::Int,
    data::StationSelectionData,
    base::AggregateODRouteModel,
    solver::BendersSolver,
    group_requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    hat,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Vector{Int},
    core_point,
    optimizer_env,
    v_hat::Float64,
    sub_duals::AbstractDict,
    cg_result;
    zc_result=nothing,
    certification_already_failed::Bool=false,
)
    feasible_pairs_by_p = _yzh_feasible_pairs_by_p(feasible_pairs, group_requests)
    h = master[:h]
    theta = master[:theta]
    return _add_aggregate_od_route_benders_yzh_optimality_cut!(
        master, h, theta, cut_id, data, base, solver, group_requests, feasible_pairs_by_p,
        hat.h_hat, assignments, cg_result, v_hat, sub_duals;
        zc_result=zc_result, certification_already_failed=certification_already_failed,
    )
end

# ---------------------------------------------------------------------------
# The generic outer loop.
# ---------------------------------------------------------------------------

"""
    _run_benders_decomposition(data, model, solver; direct_enumeration_pool, seed_cuts, harvested_cuts)
        -> OptResult

Generic Benders decomposition outer loop -- see this file's module docstring. Reuses the
already-decomposition-generic bookkeeping helpers (`_benders_solve_master!`,
`_benders_y_hat_bookkeeping`, `_finalize_benders_result`, `_benders_not_converged!`,
`_flush_benders_iteration_log!`) unchanged, and the `BendersMasterModel`/
`BendersCorePointProblem` `run_opt`-based types for the master and core-point steps.
`solver.check_lp_ip_gap` is not yet supported here (untested/unused in the current test
suite) and raises rather than silently ignoring the request.
"""
function _run_benders_decomposition(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver;
    direct_enumeration_pool::Union{Nothing, Vector{AggregateODRouteColumn}}=nothing,
    seed_cuts::Vector{<:NamedTuple}=NamedTuple[],
    harvested_cuts::Union{Nothing, Vector{<:NamedTuple}}=nothing,
)::OptResult
    solver.check_lp_ip_gap && throw(ArgumentError(
        "BendersSolver(check_lp_ip_gap=true) is not yet supported by the generic Benders runner"
    ))
    decomposition = solver.decomposition
    decomposition_name = string(nameof(typeof(decomposition)))
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    direct_solver = DirectSolver(config=SolverConfig(optimizer_env=optimizer_env, silent=cfg.silent))

    mapping = create_map(model, data)
    # FreeAggregateODAssignmentPolicy (BendersXY's free-assignment path) has no
    # feasibility_cut_style field at all -- this check only applies to the NearestOpen family.
    if model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy &&
       model.assignment_policy.feasibility_cut_style == :pair_chain
        assert_no_walk_only_pairs(mapping, "AggregateODRouteModel Benders ($(decomposition_name), NearestOpen, :pair_chain)")
    end
    requests, _demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))
    _check_aggregate_od_route_endpoint_feasibility!(data, model, requests, optimizer_env, cfg.silent)
    cut_groups = _benders_cut_groups(requests, solver.cut_mode)
    cut_ids = sort!(collect(keys(cut_groups)))

    if _benders_uses_certified_cut_derivation(decomposition, solver)
        (model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy &&
            model.assignment_policy.feasibility_cut_style == :big_m_nearest) ||
            throw(ArgumentError(
                "BendersSolver(cut_derivation=$(solver.cut_derivation)) requires " *
                "NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)"
            ))
        model.allow_walk_only && throw(ArgumentError(
            "BendersSolver(cut_derivation=$(solver.cut_derivation)) does not support allow_walk_only=true"
        ))
    end
    if model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy &&
       _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    end

    # Under lifted_walking_objective, every subproblem/pricing/cut-derivation call below uses
    # `subproblem_model` (walk_cost_weight=0, route_regularization_weight=1) instead of `model`
    # -- see benders/lifted_walking.jl. `model` is kept for everything master-side.
    subproblem_model = solver.lifted_walking_objective ? _unit_weighted_routing_model(model) : model
    core_point = if _benders_needs_core_point(decomposition, solver)
        cp_result = run_opt(data, BendersCorePointProblem(subproblem_model, requests, decomposition), direct_solver)
        cp_result.metadata["core_point"]
    else
        nothing
    end

    # See BendersSolver's `route_regularization_weight_schedule` docstring: a single implicit
    # stage at model.route_regularization_weight reproduces today's behavior exactly.
    beta_schedule = solver.lifted_walking_objective && !isnothing(solver.route_regularization_weight_schedule) ?
        solver.route_regularization_weight_schedule : [model.route_regularization_weight]
    stage_idx = 1

    master_problem = BendersMasterModel(
        model, solver, cut_ids; seed_cuts=seed_cuts, direct_enumeration_pool=direct_enumeration_pool,
    )
    master_build = build_model(master_problem, data; optimizer_env=optimizer_env)
    master = master_build.model
    cfg.silent && set_silent(master)
    theta = master[:theta]

    best_result = nothing
    best_open_stations = nothing
    best_ub = Inf
    optimality_cuts = 0
    inner_cg_iters = 0
    benders_rows = NamedTuple[]
    stage_log = NamedTuple[]
    shared_pool = isnothing(model.initial_columns) ? AggregateODRouteColumn[] : copy(model.initial_columns)
    total_reprice_columns_found = 0
    total_reprice_rounds = 0
    previous_hat_signature = nothing
    hat_repeat_streak = 0

    for iteration in 1:solver.max_iterations
        master_termination_status, lower_bound, master_solve_seconds = _benders_solve_master!(master, decomposition_name)
        hat = _benders_hat_point(master, decomposition)
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)

        hat_signature, hat_changed, hat_repeat_streak = _benders_y_hat_bookkeeping(
            mapping, hat.y_hat, iteration, decomposition_name, lower_bound,
            previous_hat_signature, hat_repeat_streak,
        )
        previous_hat_signature = hat_signature

        assignments, open_stations = _benders_priming_assignments(data, model, decomposition, requests, feasible_pairs, hat)

        cg_start = time()
        cg_result = _solve_fixed_route_covering_by_cg(
            data, subproblem_model, assignments, solver, iteration, open_stations; seed_columns=shared_pool,
        )
        priming_cg_seconds = time() - cg_start
        inner_cg_iters += cg_result.n_cg_iters
        final_result = cg_result.final_result
        walking_cost_hat = solver.lifted_walking_objective ? _lifted_walking_cost(data, model, assignments) : 0.0
        current_beta = beta_schedule[stage_idx]
        incumbent_objective_value = if isnothing(final_result.objective_value)
            nothing
        elseif solver.lifted_walking_objective
            walking_cost_hat + current_beta * final_result.objective_value
        else
            final_result.objective_value
        end
        if !isnothing(incumbent_objective_value) && incumbent_objective_value < best_ub
            best_ub = incumbent_objective_value
            best_result = solver.lifted_walking_objective ?
                _with_objective_value(final_result, incumbent_objective_value) : final_result
            best_open_stations = open_stations
            println(
                "  [$decomposition_name iteration $iteration] new best incumbent: obj=$(round(best_ub, digits=2))  ",
                "stations=$(sort([mapping.array_idx_to_station_id[i] for i in best_open_stations]))",
            )
            flush(stdout)
        end
        shared_pool = _deduplicate_aggregate_od_route_columns(vcat(shared_pool, final_result.mapping.columns))

        iteration_lp_value = 0.0
        cuts_added_this_iteration = 0
        subproblem_lp_seconds = 0.0
        reprice_columns_found = 0
        reprice_rounds_total = 0
        for cut_id in cut_ids
            group_requests = cut_groups[cut_id]
            lp_start = time()
            v_hat, sub_duals, repriced_pool, n_new, exhausted = _benders_solve_subproblem(
                data, subproblem_model, mapping, decomposition, hat, group_requests, feasible_pairs,
                shared_pool, solver, optimizer_env, cfg.silent,
            )
            if n_new > 0
                shared_pool = _deduplicate_aggregate_od_route_columns(vcat(shared_pool, repriced_pool))
                reprice_columns_found += n_new
            end
            exhausted ||
                @warn "$decomposition_name subproblem repricing hit max_reprice_rounds without pricing exhaustion" iteration cut_id
            subproblem_lp_seconds += time() - lp_start

            certification_kwargs = NamedTuple()
            if _benders_uses_certified_cut_derivation(decomposition, solver)
                v_hat, certification_kwargs = _benders_tighten_subproblem_value(
                    data, subproblem_model, decomposition, solver, cg_result, group_requests, feasible_pairs,
                    assignments, v_hat,
                )
            end
            iteration_lp_value += v_hat

            current_full_lb = theta_hat[cut_id] + _benders_residual_lower_bound_value(master, decomposition, cut_id)
            if current_full_lb < v_hat - solver.optimality_tol
                cut_diag = _benders_add_optimality_cut!(
                    master, decomposition, cut_id, data, subproblem_model, solver, group_requests, feasible_pairs,
                    hat, assignments, open_stations, core_point, optimizer_env, v_hat, sub_duals, cg_result;
                    certification_kwargs...,
                )
                optimality_cuts += 1
                cuts_added_this_iteration += 1
                isnothing(harvested_cuts) || push!(
                    harvested_cuts, (cut_id=cut_id, cut_constant=cut_diag.cut_constant, coeffs=cut_diag.coeffs),
                )
            end
        end
        total_reprice_columns_found += reprice_columns_found
        total_reprice_rounds += reprice_rounds_total

        push!(benders_rows, (
            iteration=iteration,
            master_status=string(master_termination_status),
            lower_bound=lower_bound,
            incumbent_objective=isfinite(best_ub) ? best_ub : nothing,
            outer_gap=_outer_gap(lower_bound, best_ub),
            outer_gap_absolute=_outer_gap_absolute(lower_bound, best_ub),
            outer_gap_relative=_outer_gap_relative(lower_bound, best_ub),
            master_solve_seconds=master_solve_seconds,
            priming_cg_seconds=priming_cg_seconds,
            subproblem_lp_seconds=subproblem_lp_seconds,
            cuts_added=cuts_added_this_iteration,
            feasibility_cuts_added=0,
            optimality_cuts_added=optimality_cuts,
            selected_assignment_count=length(assignments),
            generated_column_pool_size=length(shared_pool),
            inner_cg_iterations=inner_cg_iters,
            subproblem_ip_seconds=0.0,
            lp_ip_gap=nothing,
            reprice_objective_delta=0.0,
            reprice_columns_found=reprice_columns_found,
            reprice_rounds=reprice_rounds_total,
            cut_derivation=string(solver.cut_derivation),
            mw_fallback_count=0,
            mw_completion_seconds=0.0,
            mw_phi_core=nothing,
            route_regularization_weight=current_beta,
            y_hat_signature=hat_signature,
            y_hat_changed=hat_changed,
            y_hat_repeat_streak=hat_repeat_streak,
        ))
        _flush_benders_iteration_log!(
            solver, benders_rows;
            extra_headers=[
                :subproblem_ip_seconds, :lp_ip_gap, :reprice_objective_delta, :reprice_columns_found, :reprice_rounds,
                :cut_derivation, :mw_fallback_count, :mw_completion_seconds, :mw_phi_core, :route_regularization_weight,
            ],
        )

        if cuts_added_this_iteration == 0 && stage_idx < length(beta_schedule)
            stage_idx += 1
            next_beta = beta_schedule[stage_idx]
            if solver.lifted_walking_objective
                walking_cost_expr = master[:walking_cost_expr]
                direct_cost_expr = master[:direct_cost_expr]
                @objective(master, Min, next_beta * (sum(theta[cid] for cid in cut_ids) + direct_cost_expr) + walking_cost_expr)
            end
            # An incumbent optimal for the previous stage's beta is not comparable once beta
            # changes (same y_hat, different weighted total) -- only the master (with its
            # accumulated cuts) and shared_pool carry forward into the next stage.
            best_ub, best_result, best_open_stations = Inf, nothing, nothing
            push!(stage_log, (stage=stage_idx, route_regularization_weight=next_beta, iterations_to_reach=iteration))
            println(
                "  [$decomposition_name] route_regularization_weight_schedule: advancing to stage ",
                "$stage_idx/$(length(beta_schedule)) (β=$next_beta) at iteration $iteration",
            )
            flush(stdout)
            continue
        end

        if cuts_added_this_iteration == 0
            return _finalize_benders_result(best_result, Dict{String, Any}(
                "route_regularization_weight_schedule" => beta_schedule,
                "route_regularization_weight_stage_log" => stage_log,
                "solve_method" => "benders",
                "benders_decomposition" => decomposition_name,
                "benders_open_stations" => best_open_stations,
                "benders_cut_mode" => _benders_cut_mode_name(solver),
                "benders_iterations" => iteration,
                "benders_lower_bound" => lower_bound,
                "benders_incumbent_objective" => best_ub,
                "benders_outer_gap" => _outer_gap(lower_bound, best_ub),
                "benders_outer_gap_absolute" => _outer_gap_absolute(lower_bound, best_ub),
                "benders_outer_gap_relative" => _outer_gap_relative(lower_bound, best_ub),
                "benders_master_solve_time_sec" => master_solve_seconds,
                "benders_priming_cg_time_sec" => priming_cg_seconds,
                "benders_subproblem_lp_time_sec" => subproblem_lp_seconds,
                "benders_subproblem_ip_time_sec" => 0.0,
                "benders_subproblem_lp_ip_gap" => nothing,
                "reprice_columns_found" => reprice_columns_found,
                "reprice_rounds" => reprice_rounds_total,
                "total_reprice_columns_found" => total_reprice_columns_found,
                "total_reprice_rounds" => total_reprice_rounds,
                "feasibility_cuts_added" => 0,
                "optimality_cuts_added" => optimality_cuts,
                "inner_cg_iterations" => inner_cg_iters,
                "benders_lp_value" => iteration_lp_value,
                "best_upper_bound" => best_ub,
                "selected_assignment_count" => length(assignments),
                "generated_column_pool_size" => length(shared_pool),
                "feasibility_cut_style" => model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy ?
                    string(model.assignment_policy.feasibility_cut_style) : "free",
                "cut_derivation" => string(solver.cut_derivation),
            ), solver; phase1_guided=!isnothing(direct_enumeration_pool))
        end
    end
    _benders_not_converged!(decomposition_name, solver, best_result)
end
