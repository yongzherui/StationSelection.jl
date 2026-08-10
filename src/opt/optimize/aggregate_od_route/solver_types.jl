export AbstractBendersDecomposition
export BendersY
export BendersXY
export BendersYZ
export BendersYZH
export AbstractBendersCutMode
export SingleCut
export MultiCut
export BendersSolver
export BranchAndBendersSolver
export BranchBendersCut
export HeuristicEnumerationSolver
export AggregateODRouteCG
export PassengerFreeAssignmentCG

abstract type AbstractBendersDecomposition end

"""
    BendersY

Benders decomposition whose master/cuts are expressed over first-stage design
variables only.
"""
struct BendersY <: AbstractBendersDecomposition end

"""
    BendersXY

Benders decomposition whose master/cuts include first-stage design variables and
linking or assignment variables.
"""
struct BendersXY <: AbstractBendersDecomposition end

"""
    BendersYZ

Benders decomposition (`AggregateODRouteModel`, `NearestOpenAggregateODAssignmentPolicy`
with `feasibility_cut_style in (:big_m_nearest, :endpoint_chain)` only) whose master
includes the first-stage design variables `y` and the nearest-open endpoint selectors `z`;
the assignment variables `x` and route-covering `θ` are left to the subproblem. Unlike
`BendersXY`, `y_hat` alone does not guarantee a feasible nearest-open resolution here (`z`'s
two sides can independently resolve to a colliding station), so this decomposition also
uses `BendersY`-style feasibility cuts.

The subproblem fixes only `z`, leaving `x` free -- the same structural gap `BendersY`'s
subproblem has (see `_solve_nearest_open_y_subproblem_lp_with_repricing`'s docstring), which
lets a column pool that's exhaustive for one nearest-open assignment be incomplete for the
LP's own dual structure. **`BendersSolver(reprice_subproblem=true)` is required for a
provably optimal result under `cut_derivation=:standard`**, exactly as with `BendersY`;
without it, BendersYZ can converge to a genuinely suboptimal-but-correctly-costed `y`
(confirmed empirically on the real-data alignment fixture).

Repricing is no longer the *only* route to a provably-optimal result: `cut_derivation ∈
(:zero_completion, :restricted_mw_fixed_pi)` (see `BendersSolver`'s docstring) certifies the
route-covering dual directly via a from-scratch column-generation solve on the fixed-assignment
problem, then completes the remaining `x`-linking duals with a small LP -- valid by LP duality
regardless of column-pool completeness, so `reprice_subproblem=false` is sound under those
modes. See `benders/yz_mw_cut.jl` and `notes/2026-07-17_restricted_mw_cut_benders_y.md` (the
`BendersY` derivation this mirrors, over a simpler primal since `z` has no chain structure of
its own inside `BendersYZ`'s subproblem).
"""
struct BendersYZ <: AbstractBendersDecomposition end

"""
    BendersYZH

Benders decomposition (`AggregateODRouteModel`, `NearestOpenAggregateODAssignmentPolicy`
with `feasibility_cut_style in (:big_m_nearest, :endpoint_chain)` only) whose master
includes `y`, `z`, and a scenario-compressed assignment variable `h` -- one `h` per
*physical* OD pair `(o,d)`, shared across every scenario in which that pair appears
(weighted by its raw scenario-occurrence count), rather than `BendersXY`'s per-`(scenario,
o, d)` `x`. Only route-covering `θ` is left to the subproblem.

**Correction (2026-07-21, see notes/2026-07-21_bendersyz_yzh_verification_gaps.md and
notes/2026-07-21_benders_final_result_vs_best_result_bug.md):** earlier text here and in that
note claimed `h` being fixed fully makes CG-priming provably exhaustive for the subproblem LP's
own dual structure, needing no repricing. That reasoning is incomplete: `h` being fixed removes
degeneracy *in the master's choice of assignment*, but the theta-only subproblem's route-covering
LP (fixed `h`, free continuous route-selection `lambda`) is a set-cover-style LP, which commonly
has a *degenerate* dual-optimal face. CG's own pricing only certifies exhaustiveness against
*the one dual vertex CG's solver happened to return* -- `_build_yzh_route_subproblem_lp` builds
and solves a *separately formulated* LP for the cut, with no guarantee Gurobi returns that same
vertex rather than a different, equally-optimal one the pool isn't proven exhaustive against.
Empirically (`reprice_subproblem=true`), repricing does find real columns beyond the seeded pool,
growing with instance size (negligible at n=15, 12-18x subproblem-time overhead at n=20) -- so
this is not a hypothetical concern. It has not yet been observed to change the final objective on
any tested fixture, but nothing rules that out at larger scale; treat "exact without repricing" as
unproven, not disproven, absent one of: (a) `reprice_subproblem=true`, or (b) reusing CG's own
already-certified dual directly instead of re-solving (a "zero completion" analogous to
`BendersY`'s `cut_derivation=:zero_completion`, see notes/2026-07-17_restricted_mw_cut_benders_y.md).

**(b) is now implemented**: `cut_derivation=:zero_completion` reuses CG's own certified,
zero-extended route-covering dual directly as the cut's `h`-coefficients (`benders/yzh.jl`'s
`_zero_completion_yzh_rho`) -- no completion LP at all, unlike `BendersY`/`BendersYZ`, since `h`
has no other free dual block to complete once it's fixed. `reprice_subproblem=false` is sound
under this mode. `cut_derivation=:restricted_mw_fixed_pi` is rejected at `BendersSolver`
construction for this decomposition: with no free dual block left, there is no distinct
Magnanti-Wong-style variant to optimize over -- it would coincide exactly with `:zero_completion`.
"""
struct BendersYZH <: AbstractBendersDecomposition end

abstract type AbstractBendersCutMode end

"""
    SingleCut

Aggregate all scenario subproblem values into one Benders theta/cut.
"""
struct SingleCut <: AbstractBendersCutMode end

"""
    MultiCut(:scenario)

Generate separate Benders theta variables and cuts by scenario.
"""
struct MultiCut <: AbstractBendersCutMode
    dimension::Symbol

    function MultiCut(dimension::Symbol=:scenario)
        dimension == :scenario ||
            throw(ArgumentError("only MultiCut(:scenario) is currently supported"))
        new(dimension)
    end
end

"""
    _validate_cut_derivation_compatibility(decomposition, cut_derivation; allowed_modes, forbidden)

Shared cross-field validation between `BendersSolver`'s and `BranchAndBendersSolver`'s
constructors: `cut_derivation` must be a member of `allowed_modes` (each caller has its own,
different set -- `BendersSolver` allows `:zero_completion` too, `BranchAndBendersSolver` doesn't),
then each `(predicate, message)` pair in `forbidden` is checked in order and throws `message` if
`predicate(decomposition, cut_derivation)` holds (each caller's decomposition-specific exclusion
rule is different in substance -- `BendersYZH` vs. `:restricted_mw_fixed_pi` for `BendersSolver`,
`BendersY` vs. anything non-`:standard` for `BranchAndBendersSolver` -- so those stay
caller-supplied rather than hardcoded here).
"""
function _validate_cut_derivation_compatibility(
    decomposition::AbstractBendersDecomposition,
    cut_derivation::Symbol;
    allowed_modes::Tuple{Vararg{Symbol}},
    forbidden::Vector{<:Tuple{<:Function, <:AbstractString}}=Tuple{Function, String}[],
)
    cut_derivation in allowed_modes || throw(ArgumentError(
        "cut_derivation must be one of $(allowed_modes); got $(cut_derivation)"
    ))
    for (predicate, message) in forbidden
        predicate(decomposition, cut_derivation) && throw(ArgumentError(message))
    end
    return nothing
end

"""
    BendersSolver

# `cut_derivation`

Controls how `BendersY`'s, `BendersYZ`'s, and `BendersYZH`'s optimality cuts are derived
(`BendersXY` always uses the standard subgradient cut; this field is ignored there). One of:

- `:standard`: the pre-existing subgradient cut from the fixed-`y`/`z`/`h` subproblem
  LP's duals off the fixing constraints. Byte-identical to behavior before this field existed.
- `:zero_completion` (default): a restricted dual-completion cut with a zero completion objective, i.e. any
  dual-feasible completion tight at `y_hat`/`z_hat`/`h_hat` — a baseline for comparison, not a
  stronger cut. For `BendersYZH` this needs no completion LP at all (see `BendersYZH`'s docstring).
- `:restricted_mw_fixed_pi`: a restricted, fixed-pricing-dual Magnanti-Wong-style cut. Fixes the
  route-covering dual block at the vector certified by exact column-generation pricing on the
  fixed-assignment route-covering problem, then completes the remaining duals by maximizing the
  completed cut at a relative-interior core point of the master's structural region for the
  decomposition's own fixed variable. This is *not* a full Magnanti-Wong procedure over the
  entire subproblem dual optimal face and is not claimed to be globally Pareto-optimal.
  **Not supported for `BendersYZH`** (constructor throws `ArgumentError`): once `h` is fixed
  fully there is no remaining free dual block to optimize over, so this mode would coincide
  exactly with `:zero_completion`.

For all three decompositions that honor this field, the non-`:standard` modes are only supported
for `NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)` with `allow_walk_only=false`
and `inner_solver isa ColumnGenerationSolver`. Any
completion/certification failure is fatal; the solver never substitutes a standard cut. See
`notes/2026-07-17_restricted_mw_cut_benders_y.md` (the `BendersY` derivation) and
`benders/yz_mw_cut.jl`/`benders/yzh.jl` (the `BendersYZ`/`BendersYZH` analogues).

`cut_derivation=:standard` with `reprice_subproblem=false` is retained for diagnostics but emits
a warning because its restricted-pool cuts are not correctness-certified. A completed solve also
warns when its recorded relative outer gap exceeds `outer_gap_warning_tol` (default `0.03`). The
incumbent is still returned, and the threshold/check result are recorded in result metadata.

# `lifted_walking_objective`

Only supported for `decomposition isa Union{BendersY, BendersYZ}` (checked here) with
`AggregateODRouteModel`'s `NearestOpenAggregateODAssignmentPolicy` and
`feasibility_cut_style in (:big_m_nearest, :endpoint_chain)` (checked in
`_run_aggregate_od_route_nearest_open_benders_y`/`_yz`, since the policy/style live on the model,
not the solver). When `true`, walking cost is moved entirely out of the Benders subproblem/cuts
and into the master (computed exactly via the same nearest-open chain/linking machinery, now built
once in the master instead of fresh per iteration in the subproblem), and
`route_regularization_weight` (this package's `β`) is applied once, in the master, as the
coefficient of `theta`, instead of being baked into route-column costs inside the subproblem and
pricing. The subproblem, its duals, and its stored cuts are all in unweighted routing-cost units
regardless of `route_regularization_weight`'s value -- see `benders/lifted_walking.jl`. Defaults
to `false`, reproducing prior behavior exactly (walking cost and `route_regularization_weight`
both inside the subproblem, as today).

# `route_regularization_weight_schedule`

Only supported when `lifted_walking_objective=true`: since the subproblem's cuts are then stored
in unweighted-routing units regardless of `route_regularization_weight` (see above), the *same*
accumulated cuts, shared route-column pool, and master remain valid across any `β =
route_regularization_weight` -- only the master's coefficient on `theta` needs to change. When
set to a `Vector{Float64}` `[β_1 < β_2 < ... < β_T]` (must end at the model's own
`route_regularization_weight`), the outer loop solves stage `β_1` to full Benders convergence,
then reuses the same master/cuts/pool and bumps `theta`'s coefficient to `β_2`, and so on, in one
continuous run bounded by the single `max_iterations` budget -- not `T` independent solves.
Motivation: when walking cost dominates the objective, low-`β` stages converge in very few
iterations and the cuts they find remain useful once `β` is ramped up, so this can reach the same
`β_T` result with less total work than solving at `β_T` directly. Defaults to `nothing`
(single implicit stage at `model.route_regularization_weight`, i.e. today's behavior).

# `direct_enumeration_guide`

Only supported for `decomposition isa Union{BendersY, BendersYZ}` with
`lifted_walking_objective=true` (checked here). When `true`, `run_opt` performs two
phases in one call instead of a single Benders solve:

- **Phase 1**: the usual `lifted_walking_objective=true` master (`y`, `theta`, and the
  exact nearest-open `x`/walking-cost structure) is additionally augmented, once, with
  the *complete* enumerated route universe (`enumerate_aggregate_od_route_columns`, the
  same exhaustive DFS `DirectSolver` uses) as a second, exact routing-cost term
  (`theta_direct` binaries plus route-covering constraints against the master's own
  `x`) added to the objective alongside `theta`. This makes the master's own `y_hat`
  choices exact-cost-guided rather than only cut-bounded, so the standard outer Benders
  loop run against this augmented master should need fewer iterations to converge, and
  the cuts it derives along the way are recorded (harvested) rather than only being
  used to prove convergence.
- **Phase 2**: a fresh master, structurally identical to today's plain
  `lifted_walking_objective=true` master (no `theta_direct`, no enumerated pool), is
  built and seeded with every cut harvested in phase 1 before its own outer loop runs.
  This master is cheap to re-solve (no enumerated-route binaries), and because the
  seeded cuts already characterize the optimum found in phase 1, it should converge in
  very few (ideally 0-1) additional iterations.

`run_opt`'s returned `OptResult` is phase 2's (the certified, non-heuristic-guided
result); phase 1's objective/iteration/cut-count are recorded under `phase1_*` keys in
its metadata purely as diagnostics -- `phase1_objective` is a heuristic-guide artifact
(theta and the exact `theta_direct` term are both costed simultaneously in phase 1, so
it double-counts routing cost) and is never the certified answer. A mismatch between
phase 1's and phase 2's final objective triggers a `@warn`, not a fatal error, mirroring
`outer_gap_warning_tol`'s cross-check pattern elsewhere in this solver.

`direct_enumeration_max_routes`/`direct_enumeration_time_limit_sec` bound the phase-1
enumeration (`enumerate_aggregate_od_route_columns`'s own `max_routes`/`time_limit_sec`);
unlike `DirectSolver`'s enumeration (implicitly scoped by whichever single `y_hat` it is
solving for), this enumerates over the *entire* station set regardless of `l`, so these
limits typically need to be set tighter than `DirectSolver`'s defaults for the same
instance. Defaults to `false` (today's single-phase behavior).

`direct_enumeration_max_stops` decouples the enumerated pool's own `max_stops` cap from
`model.max_stops`: the underlying model's `max_stops` still governs label-setting/CG
pricing and the Benders subproblem (in both phases, and for any non-guided solve of the
same model) unchanged, but the *enumeration* that builds phase 1's `theta_direct` pool
uses `direct_enumeration_max_stops` instead when set, letting a much cheaper (fewer
stops, far fewer routes) pool guide the master without also restricting what the real
subproblem/pricing is allowed to search over. Defaults to `nothing` (use `model.max_stops`
for enumeration too, i.e. today's behavior of a single shared cap).

`direct_enumeration_relax_integrality`, when `true`, declares phase 1's `theta_direct`
route-selection variables as continuous (`0<=theta_direct<=1`) instead of binary. Phase
1's own result is never the certified answer (only phase 2's, built without
`theta_direct` at all) -- `theta_direct`'s only job is to keep the master's `y_hat`
choices exact-cost-guided, so an LP-relaxed route selection is a legitimate, much cheaper
substitute whenever the underlying route-covering LP/IP gap is small. Defaults to `false`
(today's binary behavior).
"""

struct BendersSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    decomposition::AbstractBendersDecomposition
    cut_mode::AbstractBendersCutMode
    inner_solver::Union{ColumnGenerationSolver, DirectSolver}
    max_iterations::Int
    optimality_tol::Float64
    log_dir::Union{String, Nothing}
    log_subiteration_details::Bool
    check_lp_ip_gap::Bool
    reprice_subproblem::Bool
    max_reprice_rounds::Int
    cut_derivation::Symbol
    outer_gap_warning_tol::Float64
    lifted_walking_objective::Bool
    route_regularization_weight_schedule::Union{Nothing, Vector{Float64}}
    direct_enumeration_guide::Bool
    direct_enumeration_max_routes::Int
    direct_enumeration_time_limit_sec::Float64
    direct_enumeration_max_stops::Union{Nothing, Int}
    direct_enumeration_relax_integrality::Bool

    function BendersSolver(;
        config::SolverConfig=SolverConfig(),
        decomposition::AbstractBendersDecomposition=BendersY(),
        cut_mode::AbstractBendersCutMode=MultiCut(),
        inner_solver::Union{ColumnGenerationSolver, DirectSolver, Nothing}=nothing,
        max_iterations::Int=2_000,
        optimality_tol::Union{Number, Nothing}=nothing,
        reduced_cost_tol::Union{Number, Nothing}=nothing,
        max_columns_per_iteration::Int=20,
        n_candidates::Int=max(100, max_columns_per_iteration),
        pricing_time_limit_sec::Number=30.0,
        final_ip_time_limit_sec::Number=3600.0,
        log_dir::Union{AbstractString, Nothing}=nothing,
        log_subiteration_details::Bool=true,
        check_lp_ip_gap::Bool=false,
        reprice_subproblem::Bool=false,
        max_reprice_rounds::Int=10_000,
        cut_derivation::Symbol=:zero_completion,
        outer_gap_warning_tol::Number=0.03,
        lifted_walking_objective::Union{Bool, Nothing}=nothing,
        route_regularization_weight_schedule::Union{AbstractVector{<:Number}, Nothing}=nothing,
        direct_enumeration_guide::Bool=false,
        direct_enumeration_max_routes::Int=10_000,
        direct_enumeration_time_limit_sec::Number=30.0,
        direct_enumeration_max_stops::Union{Int, Nothing}=nothing,
        direct_enumeration_relax_integrality::Bool=false,
    )
        max_reprice_rounds > 0 || throw(ArgumentError("max_reprice_rounds must be positive"))
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        _validate_cut_derivation_compatibility(
            decomposition, cut_derivation;
            allowed_modes=(:standard, :zero_completion, :restricted_mw_fixed_pi),
            forbidden=[
                ((d, c) -> d isa BendersYZH && c == :restricted_mw_fixed_pi,
                 "BendersYZH has no free dual block left to optimize once h is fixed fully -- " *
                 "cut_derivation=:restricted_mw_fixed_pi would coincide exactly with :zero_completion; use that instead"),
            ],
        )
        resolved_lifted_walking = isnothing(lifted_walking_objective) ?
            decomposition isa Union{BendersY, BendersYZ} : lifted_walking_objective
        resolved_lifted_walking && !(decomposition isa Union{BendersY, BendersYZ}) && throw(ArgumentError(
            "lifted_walking_objective is only supported for decomposition isa Union{BendersY, BendersYZ}; " *
            "got $(typeof(decomposition))"
        ))
        direct_enumeration_guide && !(decomposition isa Union{BendersY, BendersYZ}) && throw(ArgumentError(
            "direct_enumeration_guide is only supported for decomposition isa Union{BendersY, BendersYZ}; " *
            "got $(typeof(decomposition))"
        ))
        direct_enumeration_guide && !resolved_lifted_walking && throw(ArgumentError(
            "direct_enumeration_guide requires lifted_walking_objective=true -- the exact enumerated " *
            "routing-cost term only makes sense against the same unit-weighted, x-linked master " *
            "structure lifted_walking_objective builds"
        ))
        direct_enumeration_max_routes > 0 ||
            throw(ArgumentError("direct_enumeration_max_routes must be positive"))
        direct_enumeration_time_limit_sec > 0 ||
            throw(ArgumentError("direct_enumeration_time_limit_sec must be positive"))
        isnothing(direct_enumeration_max_stops) || direct_enumeration_max_stops > 0 ||
            throw(ArgumentError("direct_enumeration_max_stops must be positive"))
        direct_enumeration_relax_integrality && !direct_enumeration_guide && throw(ArgumentError(
            "direct_enumeration_relax_integrality requires direct_enumeration_guide=true"
        ))
        resolved_schedule = isnothing(route_regularization_weight_schedule) ?
            nothing : Float64.(route_regularization_weight_schedule)
        if !isnothing(resolved_schedule)
            resolved_lifted_walking || throw(ArgumentError(
                "route_regularization_weight_schedule requires lifted_walking_objective=true -- " *
                "cuts are only guaranteed valid across route_regularization_weight values in that mode"
            ))
            !isempty(resolved_schedule) || throw(ArgumentError(
                "route_regularization_weight_schedule must not be empty"
            ))
            all(resolved_schedule .> 0) || throw(ArgumentError(
                "route_regularization_weight_schedule entries must all be positive"
            ))
            all(resolved_schedule[i] < resolved_schedule[i + 1] for i in 1:(length(resolved_schedule) - 1)) ||
                throw(ArgumentError("route_regularization_weight_schedule must be strictly increasing"))
        end
        resolved_tol = isnothing(optimality_tol) ?
            (isnothing(reduced_cost_tol) ? 1e-6 : Float64(reduced_cost_tol)) :
            Float64(optimality_tol)
        resolved_tol >= 0 || throw(ArgumentError("optimality_tol must be non-negative"))
        resolved_outer_gap_warning_tol = Float64(outer_gap_warning_tol)
        resolved_outer_gap_warning_tol >= 0 ||
            throw(ArgumentError("outer_gap_warning_tol must be non-negative"))
        resolved_inner = isnothing(inner_solver) ?
            ColumnGenerationSolver(
                config=config,
                max_columns_per_iteration=max_columns_per_iteration,
                n_candidates=n_candidates,
                reduced_cost_tol=isnothing(reduced_cost_tol) ? resolved_tol : Float64(reduced_cost_tol),
                pricing_time_limit_sec=pricing_time_limit_sec,
                final_ip_time_limit_sec=final_ip_time_limit_sec,
                log_dir=log_dir,
            ) :
            inner_solver
        new(
            config,
            decomposition,
            cut_mode,
            resolved_inner,
            max_iterations,
            resolved_tol,
            isnothing(log_dir) ? nothing : String(log_dir),
            log_subiteration_details,
            check_lp_ip_gap,
            reprice_subproblem,
            max_reprice_rounds,
            cut_derivation,
            resolved_outer_gap_warning_tol,
            resolved_lifted_walking,
            resolved_schedule,
            direct_enumeration_guide,
            direct_enumeration_max_routes,
            Float64(direct_enumeration_time_limit_sec),
            direct_enumeration_max_stops,
            direct_enumeration_relax_integrality,
        )
    end
end

"""A globally valid full-recourse cut used by `BranchAndBendersSolver`.

`beta` is keyed by station index for `BendersY`, and by
`(endpoint_chain_key, rank)` for `BendersYZ`.
"""
struct BranchBendersCut
    decomposition::Symbol
    block_id::Int
    alpha::Float64
    beta::Dict{Any, Float64}
    recourse_value::Float64
end

function BranchBendersCut(
    decomposition::Symbol,
    block_id::Int,
    alpha::Real,
    beta::AbstractDict,
    recourse_value::Real,
)
    decomposition in (:y, :yz) || throw(ArgumentError("cut decomposition must be :y or :yz"))
    block_id >= 0 || throw(ArgumentError("cut block_id must be non-negative"))
    return BranchBendersCut(
        decomposition, block_id, Float64(alpha),
        Dict{Any, Float64}(key => Float64(value) for (key, value) in beta),
        Float64(recourse_value),
    )
end

"""Single-tree branch-and-Benders solver."""
struct BranchAndBendersSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    decomposition::Union{BendersY, BendersYZ}
    cut_derivation::Symbol
    inner_solver::ColumnGenerationSolver
    initial_cuts::Vector{BranchBendersCut}
    initial_benders_cut_rounds::Int
    integrality_tolerance::Float64
    lazy_cut_tolerance::Float64
    cut_tightness_tolerance::Float64
    dual_feasibility_tolerance::Float64
    pricing_tolerance::Float64
    max_reprice_rounds::Int
    log_dir::Union{Nothing, String}

    function BranchAndBendersSolver(;
        config::SolverConfig=SolverConfig(),
        decomposition::Union{BendersY, BendersYZ}=BendersYZ(),
        cut_derivation::Symbol=:standard,
        inner_solver::Union{Nothing, ColumnGenerationSolver}=nothing,
        initial_cuts::Vector{BranchBendersCut}=BranchBendersCut[],
        initial_benders_cut_rounds::Int=0,
        integrality_tolerance::Number=1e-6,
        lazy_cut_tolerance::Number=1e-6,
        cut_tightness_tolerance::Number=1e-5,
        dual_feasibility_tolerance::Number=1e-7,
        pricing_tolerance::Number=1e-7,
        max_reprice_rounds::Int=10_000,
        log_dir::Union{Nothing, AbstractString}=nothing,
    )
        initial_benders_cut_rounds >= 0 || throw(ArgumentError("initial_benders_cut_rounds must be non-negative"))
        max_reprice_rounds > 0 || throw(ArgumentError("max_reprice_rounds must be positive"))
        _validate_cut_derivation_compatibility(
            decomposition, cut_derivation;
            allowed_modes=(:standard, :restricted_mw_fixed_pi),
            forbidden=[
                ((d, c) -> d isa BendersY && c != :standard,
                 "BranchAndBendersSolver restricted MW integration currently supports BendersYZ only"),
            ],
        )
        tolerances = (
            integrality_tolerance, lazy_cut_tolerance, cut_tightness_tolerance,
            dual_feasibility_tolerance, pricing_tolerance,
        )
        all(t -> t >= 0, tolerances) || throw(ArgumentError("branch-and-Benders tolerances must be non-negative"))
        resolved_inner = isnothing(inner_solver) ? ColumnGenerationSolver(
            config=config,
            reduced_cost_tol=pricing_tolerance,
        ) : inner_solver
        return new(
            config, decomposition, cut_derivation, resolved_inner, copy(initial_cuts), initial_benders_cut_rounds,
            Float64(integrality_tolerance), Float64(lazy_cut_tolerance),
            Float64(cut_tightness_tolerance), Float64(dual_feasibility_tolerance),
            Float64(pricing_tolerance), max_reprice_rounds,
            isnothing(log_dir) ? nothing : String(log_dir),
        )
    end
end

"""
    HeuristicEnumerationSolver

Solve `AggregateODRouteModel` by trying a caller-supplied list of candidate open-station
sets (fixed `y`). For each candidate, the nearest-open assignment is derived and the
resulting fixed-station, fixed-assignment routing sub-problem (`RouteCoveringProblem`) is
solved to proven optimality via column generation. The best-scoring feasible candidate is
then used to warm-start a direct solve of the full `AggregateODRouteModel` (with the
winning routes folded into its column pool).

Candidates are not generated internally — supply them via `candidate_open_stations`
(e.g. station sets read from a prior run).
"""
struct HeuristicEnumerationSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    candidate_open_stations::Vector{Vector{Int}}
    cg_solver::ColumnGenerationSolver

    function HeuristicEnumerationSolver(;
        config::SolverConfig=SolverConfig(),
        candidate_open_stations::Vector{Vector{Int}},
        cg_solver::ColumnGenerationSolver=ColumnGenerationSolver(config=config),
    )
        !isempty(candidate_open_stations) ||
            throw(ArgumentError("candidate_open_stations must not be empty"))
        for candidate in candidate_open_stations
            length(candidate) == length(unique(candidate)) ||
                throw(ArgumentError("candidate_open_stations entries must not contain duplicate station ids"))
        end
        new(config, candidate_open_stations, cg_solver)
    end
end

"""
    AggregateODRouteCG

Column-generation algorithm over the aggregate station-pair-per-request formulation (one
assignment variable per `(scenario, origin, destination)` request) -- `run_aggregate_od_route_column_generation`'s
algorithm. The knobs shared with [`PassengerFreeAssignmentCG`](@ref) under matching semantics
(`n_candidates`, `reduced_cost_tol`, `pricing_time_limit_sec`, `max_columns_per_iteration`,
`final_ip_time_limit_sec`, `max_iterations`) live on `ColumnGenerationSolver`, exactly as before.

The remaining fields exist only because `run_aggregate_od_route_column_generation` (the public,
still-independently-callable function this algorithm's hooks were extracted from -- see
`pricing/generic_runner.jl`) has its own kwarg surface, used by direct callers
(`_solve_fixed_route_covering_by_cg`, tests, scripts) that predate `ColumnGenerationSolver`
entirely: per-call log file paths, and pricing knobs with dynamic (`model`/solver-dependent)
defaults that can't be resolved until a hook actually runs against a concrete `model`/`solver`
pair, hence the `Union{Nothing, _}` sentinels (resolved in `_cg_build_master`). Every field
defaults to a placeholder here since a default-constructed `AggregateODRouteCG()` is also what
`pricing/dispatch.jl`'s `_default_cg_algorithm` returns purely for its `isa` mismatch check --
`run_aggregate_od_route_column_generation`'s own wrapper always constructs its own instance with
real values from its own kwargs.
"""
struct AggregateODRouteCG <: AbstractColumnGenerationAlgorithm
    pricing_initial_sec::Union{Nothing, Float64}
    pricing_ramp_factor::Float64
    use_station_simple::Union{Nothing, Bool}
    profile_pricing::Bool
    verbose::Bool
    cg_log_path::Union{Nothing, String}
    column_log_path::Union{Nothing, String}
    dual_log_path::Union{Nothing, String}

    function AggregateODRouteCG(;
        pricing_initial_sec::Union{Number, Nothing}=nothing,
        pricing_ramp_factor::Number=1.0,
        use_station_simple::Union{Bool, Nothing}=nothing,
        profile_pricing::Bool=false,
        verbose::Bool=true,
        cg_log_path::Union{AbstractString, Nothing}=nothing,
        column_log_path::Union{AbstractString, Nothing}=nothing,
        dual_log_path::Union{AbstractString, Nothing}=nothing,
    )
        isnothing(pricing_initial_sec) || pricing_initial_sec > 0 ||
            throw(ArgumentError("pricing_initial_sec must be positive"))
        pricing_ramp_factor > 0 || throw(ArgumentError("pricing_ramp_factor must be positive"))
        new(
            isnothing(pricing_initial_sec) ? nothing : Float64(pricing_initial_sec),
            Float64(pricing_ramp_factor),
            use_station_simple,
            profile_pricing,
            verbose,
            isnothing(cg_log_path) ? nothing : String(cg_log_path),
            isnothing(column_log_path) ? nothing : String(column_log_path),
            isnothing(dual_log_path) ? nothing : String(dual_log_path),
        )
    end
end

"""
    PassengerFreeAssignmentCG

Column-generation algorithm over the passenger-level free-assignment formulation (one
assignment variable per passenger, station budget enforced separately) with a DSSR label-setting
pricer -- `run_passenger_free_assignment_column_generation`'s algorithm. Only supported for
`AggregateODRouteModel`'s `FreeAggregateODAssignmentPolicy`.

Carries every knob that is meaningful only to this algorithm (mirroring the existing
`MultiCut(dimension)` precedent for algorithm-specific config living on the dispatch singleton
rather than on the shared solver struct). Knobs shared with `AggregateODRouteCG` under matching
semantics -- including `max_new_columns` (as `max_columns_per_iteration`) -- stay on
`ColumnGenerationSolver`: `n_candidates`, `reduced_cost_tol`, `pricing_time_limit_sec`,
`max_columns_per_iteration`, `final_ip_time_limit_sec`, `max_iterations`.
"""
struct PassengerFreeAssignmentCG <: AbstractColumnGenerationAlgorithm
    exhaustive_pricing_each_iteration::Bool
    theta_rho_core_size::Int
    theta_rho_n_outsiders::Int
    certification_time_limit_sec::Float64
    total_time_limit_sec::Float64
    seed_two_stop_routes::Bool
    parallel_scenarios::Bool
    compensated_dominance::Bool
    use_station_simple::Bool
    station_simple_warm_start::Bool
    reward_coarsening_levels::Int
    use_adaptive_cluster_certification::Bool
    cluster_initial_num_clusters::Union{Nothing, Int}
    cluster_max_num_clusters::Union{Nothing, Int}
    cluster_max_size::Union{Nothing, Int}
    cluster_time_limit_sec::Float64
    unserved_penalty::Union{Nothing, Float64}
    verify_reduced_costs::Bool
    verbose::Bool
    iteration_log_path::Union{Nothing, String}
    column_log_path::Union{Nothing, String}

    function PassengerFreeAssignmentCG(;
        exhaustive_pricing_each_iteration::Bool=false,
        theta_rho_core_size::Int=0,
        theta_rho_n_outsiders::Int=1,
        certification_time_limit_sec::Number=600.0,
        total_time_limit_sec::Number=Inf,
        seed_two_stop_routes::Bool=true,
        parallel_scenarios::Bool=true,
        compensated_dominance::Bool=true,
        use_station_simple::Bool=false,
        station_simple_warm_start::Bool=true,
        reward_coarsening_levels::Int=0,
        use_adaptive_cluster_certification::Bool=false,
        cluster_initial_num_clusters::Union{Int, Nothing}=nothing,
        cluster_max_num_clusters::Union{Int, Nothing}=nothing,
        cluster_max_size::Union{Int, Nothing}=nothing,
        cluster_time_limit_sec::Number=60.0,
        unserved_penalty::Union{Number, Nothing}=nothing,
        verify_reduced_costs::Bool=true,
        verbose::Bool=true,
        iteration_log_path::Union{AbstractString, Nothing}=nothing,
        column_log_path::Union{AbstractString, Nothing}=nothing,
    )
        certification_time_limit_sec > 0 ||
            throw(ArgumentError("certification_time_limit_sec must be positive"))
        total_time_limit_sec > 0 || throw(ArgumentError("total_time_limit_sec must be positive"))
        reward_coarsening_levels >= 0 ||
            throw(ArgumentError("reward_coarsening_levels must be nonnegative"))
        reward_coarsening_levels > 0 && (use_station_simple || station_simple_warm_start) &&
            throw(ArgumentError(
                "reward-coarsened and station-simple harvesting are alternative modes; " *
                "enable only one per run",
            ))
        theta_rho_core_size >= 0 || throw(ArgumentError("theta_rho_core_size must be nonnegative"))
        theta_rho_n_outsiders >= 0 ||
            throw(ArgumentError("theta_rho_n_outsiders must be nonnegative"))
        cluster_time_limit_sec > 0 || throw(ArgumentError("cluster_time_limit_sec must be positive"))
        new(
            exhaustive_pricing_each_iteration, theta_rho_core_size,
            theta_rho_n_outsiders, Float64(certification_time_limit_sec),
            Float64(total_time_limit_sec), seed_two_stop_routes, parallel_scenarios,
            compensated_dominance, use_station_simple,
            station_simple_warm_start, reward_coarsening_levels,
            use_adaptive_cluster_certification, cluster_initial_num_clusters,
            cluster_max_num_clusters, cluster_max_size, Float64(cluster_time_limit_sec),
            isnothing(unserved_penalty) ? nothing : Float64(unserved_penalty),
            verify_reduced_costs,
            verbose,
            isnothing(iteration_log_path) ? nothing : String(iteration_log_path),
            isnothing(column_log_path) ? nothing : String(column_log_path),
        )
    end
end
