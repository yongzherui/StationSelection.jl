export CGPricingConfig

"""
    CGPricingConfig(; mode, warm_start_mode, compensated_dominance,
                      relaxed_cluster_count, ...)

Everything about **how column generation searches for columns**, in one object hung off
`CGSolver.pricing`.

These settings used to live on `AggregateODRouteJointRoutingAssignmentFormulation`, which
put them on the wrong side of the Problem/Formulation/Solver split: a pricer is a search
algorithm, not part of the mathematical encoding. Two runs differing only in `mode` solve
the *identical* model and must reach the identical optimum (that is the exhaustive-
equivalence invariant `darp_modified/types.jl` and `darp/types.jl` state) -- so the mode
cannot be a property of the thing being solved. The practical consequence is that a mode
sweep now varies one solver and reuses one formulation object, instead of rebuilding a
"different" formulation per arm and having to argue that the arms are comparable.

`AbstractFormulation` still owns everything that changes the model: the cost weights,
`max_stops`, `detour_factor`, `max_wait_time`. `compensated_dominance` lives here because
it changes only label-setting search, never the encoded model or route universe.

## Fields

`mode` (default `nothing`) -- which pricer runs. `nothing` means "the formulation's own
default", which is what makes a bare `CGSolver()` work against any formulation; the build
resolves it and stashes the resolved symbol on the model, so
`metadata["cg_final_pricing_mode"]` always reports a concrete pricer. For
`AggregateODRouteJointRoutingAssignmentFormulation` the default resolves to `:exact` and
the full set is `:exact`, `:station_simple`, `:darp_modified`, `:darp`, and
`:relaxed_cluster` -- see that formulation's docstring for what each searches
and which of them can certify. A formulation with no selectable pricer
(`AggregateODRouteBaseFormulation`) rejects any non-`nothing` mode at build time rather
than silently ignoring it.

`warm_start_mode` (default `nothing`) -- when set, CG prices in *that* mode until its
universe exhausts, then hands off to `mode` for the rest of the solve. Both phases share
one master and one column pool, so every column phase 1 found stays; what the handoff
changes is only what pricing searches. Phase 2 is the one that certifies, which is the
point: `:station_simple` harvests cheaply in the elementary universe and `mode` then proves
optimality over the full one. A warm start that would be a no-op (the same pricer both
phases) is rejected by `CGSolver`'s loop, which compares against the *resolved* mode.
`:cluster_guide` is available only here: it uses relaxed cluster routes to choose a real
station subset, exact-prices that subset for real columns, then hands off to the final
pricer when the heuristic round exhausts. It requires `relaxed_cluster_count`.

`compensated_dominance` (default `true`) -- whether compatible label-setting pricers use
the compensated reward-difference dominance rule or the older plain subset rule. It
applies to the base route-covering pricer and to joint-routing `:exact`,
`:station_simple`, and `:darp_modified`; `:darp` has no compensated variant.

`relaxed_cluster_count` = K -- required by `:relaxed_cluster`. It sizes the k-medoids
station partition the mode runs on,
built **once at build time** and stashed on the model: the cells have to be identical across every CG iteration of a run for K to be a
meaningful swept parameter, which is why this is a build-time input and not something the
loop re-derives.

`relaxed_cluster_max_count` (default `nothing`) turns that partition from a fixed input
into a *starting point*: witness-guided refinement
(`relaxed_cluster/utils/refinement/refine.jl`) may split cells up to this ceiling, per
scenario. It needs a starting partition, must exceed it (equal means no refinement is
possible, which `nothing` already expresses), and is opt-in precisely because it costs the
cross-round comparability the fixed partition buys. Every qualifying barren witness now
splits its highest-reward implicated cell immediately; there is no recurrence threshold.

`relaxed_cluster_guide_routes` (default 5) is how many improving relaxed routes contribute
their clusters to each exact station-subset search in `:relaxed_cluster`. The relaxed
search receives half of the pricing round's remaining time; there is no independent guide
time limit.

Two further experimental switches used to live here -- a barren-support cache and active-cut
subsumption pruning. Both were removed: the measured cut load is far too small for either
to pay for itself (0.5-0.75 cuts per scenario attempt at n=30/40, 11 inner rounds at
worst). They are written up as possible future work in
`label_setting/joint_routing_assignment/relaxed_cluster/README.md`.
"""
struct CGPricingConfig
    mode::Union{Nothing, Symbol}
    warm_start_mode::Union{Nothing, Symbol}
    compensated_dominance::Bool
    relaxed_cluster_count::Union{Nothing, Int}
    relaxed_cluster_max_count::Union{Nothing, Int}
    relaxed_cluster_guide_routes::Int

    function CGPricingConfig(;
            mode::Union{Nothing, Symbol}=nothing,
            warm_start_mode::Union{Nothing, Symbol}=nothing,
            compensated_dominance::Bool=true,
            relaxed_cluster_count::Union{Nothing, Int}=nothing,
            relaxed_cluster_max_count::Union{Nothing, Int}=nothing,
            relaxed_cluster_guide_routes::Int=5,
        )
        _cg_validate_pricing_mode(mode, "mode")
        _cg_validate_pricing_mode(warm_start_mode, "warm_start_mode")
        # `:relaxed_cluster` never reports its own exhaustion -- the only thing it can
        # exhaust is the relaxation, and that is a full-universe certificate that ends the
        # solve outright. A warm start in it would therefore either finish the solve in
        # phase 1 or never reach phase 2 at all.
        warm_start_mode === :relaxed_cluster && throw(ArgumentError(
            "warm_start_mode=:relaxed_cluster is not a warm start: the mode never hands " *
            "off (it either certifies, which ends the solve, or keeps harvesting), so " *
            "phase 2 would be unreachable. Put :relaxed_cluster in `mode` and warm-start " *
            "from a cheaper pricer such as :station_simple",
        ))

        isnothing(relaxed_cluster_count) || relaxed_cluster_count >= 1 || throw(ArgumentError(
            "relaxed_cluster_count must be >= 1 (or nothing), got $(relaxed_cluster_count)",
        ))
        # Only this direction is an error. A relaxed-cluster mode without a partition has
        # nothing to run on; the reverse is deliberately allowed, because a partition with
        # no mode reading it is what the guide/recovery diagnostics want -- build the cells,
        # run CG on real duals with an ordinary pricer, then measure offline what the guide
        # would have done with them (`benchmarks/diagnostics/relaxed_cluster_guide_recovery.jl`).
        # It costs one k-medoids pass at build time and nothing else.
        (mode === :relaxed_cluster ||
         warm_start_mode in (:relaxed_cluster, :cluster_guide)) &&
            isnothing(relaxed_cluster_count) && throw(ArgumentError(
                "a relaxed-cluster pricing mode needs a station partition -- set " *
                "relaxed_cluster_count = K",
            ))

        relaxed_cluster_guide_routes >= 1 || throw(ArgumentError(
            "relaxed_cluster_guide_routes must be >= 1, got $(relaxed_cluster_guide_routes)",
        ))
        if !isnothing(relaxed_cluster_max_count)
            isnothing(relaxed_cluster_count) && throw(ArgumentError(
                "relaxed_cluster_max_count needs a starting partition -- set " *
                "relaxed_cluster_count = K as well",
            ))
            relaxed_cluster_max_count > relaxed_cluster_count || throw(ArgumentError(
                "relaxed_cluster_max_count ($(relaxed_cluster_max_count)) must exceed " *
                "relaxed_cluster_count ($(relaxed_cluster_count)); equal means no " *
                "refinement is possible, which is what `nothing` already expresses",
            ))
        end
        new(
            mode, warm_start_mode, compensated_dominance,
            relaxed_cluster_count, relaxed_cluster_max_count,
            relaxed_cluster_guide_routes,
        )
    end
end

"""
The set of pricer names any formulation offers today, checked here so a typo is caught when
the solver is constructed rather than on the first pricing call of a long run. Whether the
*paired* formulation actually offers the named pricer is a build-time question, answered by
that formulation's `build_model` (`AggregateODRouteBaseFormulation` offers none at all).
"""
const CG_PRICING_MODES = (:exact, :station_simple, :darp_modified, :darp, :relaxed_cluster)
const CG_WARM_START_PRICING_MODES = (CG_PRICING_MODES..., :cluster_guide)

function _cg_validate_pricing_mode(mode::Union{Nothing, Symbol}, field::AbstractString)
    isnothing(mode) && return nothing
    # `:relaxed_cluster_nogood` named the cut loop back when a cut-free relaxed round also
    # existed, selected through a since-removed `CGSolver.certification_pricing_mode`. The
    # cut-free one went too -- it is the loop's round 1, which certified 0 times in ~1130
    # measured attempts -- so both names collapsed onto `:relaxed_cluster`. Named here only
    # to say that, rather than report it as an unknown symbol.
    mode === :relaxed_cluster_nogood && throw(ArgumentError(
        ":relaxed_cluster_nogood was merged into :relaxed_cluster -- the relaxed-cluster " *
        "round is always the no-good-cut loop now, since the cut-free round is its round " *
        "1 and never certifies. Use :relaxed_cluster",
    ))
    allowed = field == "warm_start_mode" ? CG_WARM_START_PRICING_MODES : CG_PRICING_MODES
    mode in allowed || throw(ArgumentError(
        "$field must be one of $(join(map(repr, allowed), ", ")) (or nothing), " *
        "got $(repr(mode))",
    ))
    return nothing
end
