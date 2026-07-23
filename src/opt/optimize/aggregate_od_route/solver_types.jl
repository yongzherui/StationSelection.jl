export AbstractBendersDecomposition
export BendersY
export BendersXY
export BendersYZ
export BendersYZH
export AbstractBendersCutMode
export SingleCut
export MultiCut
export BendersSolver
export HeuristicEnumerationSolver

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
    BendersSolver

# `cut_derivation`

Controls how `BendersY`'s, `BendersYZ`'s, and `BendersYZH`'s optimality cuts are derived
(`BendersXY` always uses the standard subgradient cut; this field is ignored there). One of:

- `:standard` (default): the pre-existing subgradient cut from the fixed-`y`/`z`/`h` subproblem
  LP's duals off the fixing constraints. Byte-identical to behavior before this field existed.
- `:zero_completion`: a restricted dual-completion cut with a zero completion objective, i.e. any
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
for `NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)` with `allow_walk_only=false`,
`unmet_demand_penalty === nothing`, and `inner_solver isa ColumnGenerationSolver`; each falls back
to `:standard` for any (iteration, cut group) where the completion/certification fails. See
`notes/2026-07-17_restricted_mw_cut_benders_y.md` (the `BendersY` derivation) and
`benders/yz_mw_cut.jl`/`benders/yzh.jl` (the `BendersYZ`/`BendersYZH` analogues).
"""
struct BendersSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    decomposition::AbstractBendersDecomposition
    cut_mode::AbstractBendersCutMode
    inner_solver::Union{ColumnGenerationSolver, DirectSolver}
    max_iterations::Int
    optimality_tol::Float64
    log_dir::Union{String, Nothing}
    check_lp_ip_gap::Bool
    reprice_subproblem::Bool
    max_reprice_rounds::Int
    cut_derivation::Symbol

    function BendersSolver(;
        config::SolverConfig=SolverConfig(),
        decomposition::AbstractBendersDecomposition=BendersY(),
        cut_mode::AbstractBendersCutMode=MultiCut(),
        inner_solver::Union{ColumnGenerationSolver, DirectSolver, Nothing}=nothing,
        max_iterations::Int=10_000,
        optimality_tol::Union{Number, Nothing}=nothing,
        reduced_cost_tol::Union{Number, Nothing}=nothing,
        max_columns_per_iteration::Int=20,
        n_candidates::Int=max_columns_per_iteration,
        pricing_time_limit_sec::Number=30.0,
        final_ip_time_limit_sec::Number=3600.0,
        log_dir::Union{AbstractString, Nothing}=nothing,
        check_lp_ip_gap::Bool=false,
        reprice_subproblem::Bool=false,
        max_reprice_rounds::Int=20,
        cut_derivation::Symbol=:standard,
    )
        max_reprice_rounds > 0 || throw(ArgumentError("max_reprice_rounds must be positive"))
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        cut_derivation in (:standard, :zero_completion, :restricted_mw_fixed_pi) ||
            throw(ArgumentError("cut_derivation must be :standard, :zero_completion, or :restricted_mw_fixed_pi"))
        decomposition isa BendersYZH && cut_derivation == :restricted_mw_fixed_pi && throw(ArgumentError(
            "BendersYZH has no free dual block left to optimize once h is fixed fully -- " *
            "cut_derivation=:restricted_mw_fixed_pi would coincide exactly with :zero_completion; use that instead"
        ))
        resolved_tol = isnothing(optimality_tol) ?
            (isnothing(reduced_cost_tol) ? 1e-6 : Float64(reduced_cost_tol)) :
            Float64(optimality_tol)
        resolved_tol >= 0 || throw(ArgumentError("optimality_tol must be non-negative"))
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
            check_lp_ip_gap,
            reprice_subproblem,
            max_reprice_rounds,
            cut_derivation,
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
