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
the full set is `:exact`, `:station_simple`, `:darp_modified`, `:darp`,
`:relaxed_cluster` and `:relaxed_cluster_two_tier` -- see that formulation's docstring for what each searches
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

`relaxed_cluster_macro_count` = K1 -- required by `:relaxed_cluster_two_tier` and rejected
without it. The coarse layer of the nested pair, built at build time by clustering the meso
medoids and lifting, so every macro cell is a union of meso cells
(`relaxed_cluster/utils/certification/two_tier/`). MEASURED at n=40 with K2=24: K1=14-16
is the optimum, turning a 22-150 s meso sweep into 0.2-1.4 s for the pair with the same
column priced; K1 <= 8 is nearly worthless (the macro support keeps 79-83% of the meso
graph) and K1 >= 18 starts paying real time in the macro sweep itself. Cannot be combined
with `relaxed_cluster_max_count`.

`relaxed_cluster_aligned_subset_max` (default 15) -- the station budget for a macro-ALIGNED
subset search. A barren support only licenses a macro cut when the station set priced is a
union of whole macro cells, so the loop rounds the meso support up to macro boundaries
before pricing it -- but only while the result fits this cap. Over the cap it prices the
support unaligned and takes the meso cut alone, which is still sound and simply forgoes the
macro cut -- and the iterative meso cuts still float up to a macro cut on their own when the
restricted meso sweep exhausts, so alignment is a shortcut rather than the only route.

Lowered from 20 to 15 after the n=40 stall: station-search cost is super-linear (7-8
stations exhaust in ~0.1 s, 11-13 need over a second), and MEASURED there, 7 of 10 seeds
burned every 300 s round while the 3 that certified had subset medians of 8-10 stations.

REVISED 2026-09-09, and the old one-line justification here ("a cheap search that earns one
cut beats an expensive one that earns two") was wrong about the mechanism: the caps do not
earn different numbers of cuts. Seed 43 at K1=16/g=3, PER ROUND rather than summed over
attempts, cap 13/16/20 give 1.253/1.287/1.287 macro cuts per macro round -- 16 and 20
identical, 13 within 3% -- and 44% zero-cut attempts in all three. What the cap actually
trades is alignment refusals (0.205 -> 0.027 -> 0.000 per station search, falling with the
cap) against unexhausted searches (0.061 -> 0.090 -> 0.142, rising with it), and what
decides the run is COST CONCENTRATION: all three produce exactly two blocking rounds
(unexhausted AND nothing improving), but they cost 2001 s / 53 s / 278 s respectively.
cap=16 wins by making rounds cheap (13.8 s per macro round against cap=13's 24.9), so more
fit in the budget. 16 is the measured optimum at n=40; this default stays 15 only because
nothing has re-measured the smaller instances. See
`notes/2026-09-09_n40_certification_frontier_5_of_10.md`.

Two further experimental switches used to live here -- a barren-support cache and active-cut
cut management. Both were removed: the measured cut load is far too small for either
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
    relaxed_cluster_macro_count::Union{Nothing, Int}
    relaxed_cluster_aligned_subset_max::Int

    function CGPricingConfig(;
            mode::Union{Nothing, Symbol}=nothing,
            warm_start_mode::Union{Nothing, Symbol}=nothing,
            compensated_dominance::Bool=true,
            relaxed_cluster_count::Union{Nothing, Int}=nothing,
            relaxed_cluster_max_count::Union{Nothing, Int}=nothing,
            relaxed_cluster_guide_routes::Int=5,
            relaxed_cluster_macro_count::Union{Nothing, Int}=nothing,
            relaxed_cluster_aligned_subset_max::Int=15,
        )
        _cg_validate_pricing_mode(mode, "mode")
        _cg_validate_pricing_mode(warm_start_mode, "warm_start_mode")
        # `:relaxed_cluster` never reports its own exhaustion -- the only thing it can
        # exhaust is the relaxation, and that is a full-universe certificate that ends the
        # solve outright. A warm start in it would therefore either finish the solve in
        # phase 1 or never reach phase 2 at all.
        warm_start_mode === :relaxed_cluster_two_tier && throw(ArgumentError(
            "warm_start_mode=:relaxed_cluster_two_tier is not a warm start, for the same " *
            "reason :relaxed_cluster is not: the mode either certifies (ending the solve) " *
            "or keeps harvesting, so it never hands off and phase 2 would be unreachable",
        ))
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
        (mode in (:relaxed_cluster, :relaxed_cluster_two_tier) ||
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
        # K1 is required by the two-tier mode and inert without it, mirroring how
        # `relaxed_cluster_count` relates to the one-tier mode -- except that a macro layer
        # costs a second k-medoids pass and means nothing to any other pricer, so unlike a
        # bare partition there is no diagnostic that wants it built and unread.
        if mode === :relaxed_cluster_two_tier
            isnothing(relaxed_cluster_macro_count) && throw(ArgumentError(
                ":relaxed_cluster_two_tier needs a macro layer -- set " *
                "relaxed_cluster_macro_count = K1 below relaxed_cluster_count = K2",
            ))
            relaxed_cluster_macro_count >= 1 || throw(ArgumentError(
                "relaxed_cluster_macro_count must be >= 1, got " *
                "$(relaxed_cluster_macro_count)",
            ))
            relaxed_cluster_macro_count < relaxed_cluster_count || throw(ArgumentError(
                "relaxed_cluster_macro_count ($(relaxed_cluster_macro_count)) must be " *
                "below relaxed_cluster_count ($(relaxed_cluster_count)); equal means no " *
                "coarsening at all, which :relaxed_cluster already expresses. MEASURED " *
                "optimum is K1 around 0.6 x K2",
            ))
            # Refinement rewrites cut sets onto a split partition; the macro layer's parent
            # map is built against the UNsplit one, and a parent map that no longer matches
            # its partition mis-translates cuts -- which fails as a false certificate rather
            # than a crash. Rejected rather than silently ignored.
            isnothing(relaxed_cluster_max_count) || throw(ArgumentError(
                ":relaxed_cluster_two_tier cannot be combined with " *
                "relaxed_cluster_max_count: refinement re-partitions the meso layer, " *
                "which invalidates the macro parent map the two tiers are nested by",
            ))
        elseif !isnothing(relaxed_cluster_macro_count)
            throw(ArgumentError(
                "relaxed_cluster_macro_count is only read by " *
                "mode=:relaxed_cluster_two_tier, got mode=$(repr(mode))",
            ))
        end
        relaxed_cluster_aligned_subset_max >= 1 || throw(ArgumentError(
            "relaxed_cluster_aligned_subset_max must be >= 1, got " *
            "$(relaxed_cluster_aligned_subset_max)",
        ))
        new(
            mode, warm_start_mode, compensated_dominance,
            relaxed_cluster_count, relaxed_cluster_max_count,
            relaxed_cluster_guide_routes, relaxed_cluster_macro_count,
            relaxed_cluster_aligned_subset_max,
        )
    end
end

"""
The set of pricer names any formulation offers today, checked here so a typo is caught when
the solver is constructed rather than on the first pricing call of a long run. Whether the
*paired* formulation actually offers the named pricer is a build-time question, answered by
that formulation's `build_model` (`AggregateODRouteBaseFormulation` offers none at all).
"""
const CG_PRICING_MODES = (
    :exact, :station_simple, :darp_modified, :darp, :relaxed_cluster,
    :relaxed_cluster_two_tier,
)
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
