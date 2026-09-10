"""
`BendersSubproblemConfig` -- how `BendersSolver` obtains and solves the second stage.

Parallel to `CGPricingConfig` (`opt/solvers/cg/pricing_config.jl`) and for the same
reason: "where do the route columns come from" is a *search algorithm*, not part of the
model, so two runs differing only in it solve the identical model. Keeping it on the
solver rather than the formulation is what lets an oracle sweep vary one solver and reuse
one formulation.
"""

export BendersSubproblemConfig

"""
    BendersSubproblemConfig(; oracle=:direct_enumeration, max_stops=4,
                              max_routes=200_000, enumeration_time_limit_sec=180.0,
                              time_limit_sec=nothing)

# `oracle`

Where the subproblem's route columns come from.

`:direct_enumeration` enumerates the whole column universe once at build time
(`enumerate_joint_routing_assignment_columns`) and solves every subproblem LP against the
complete pool. Simple and exactly right, but enumerating up front is the cost Benders
exists to avoid, and it forces a small `max_stops`.

`:column_generation` prices each subproblem's columns by label setting instead, so the pool
starts at the two-stop seed and grows on demand, and the formulation's own `max_stops`
applies with no cap.

# What makes a CG subproblem's cut valid -- and it is NOT pool fixity

The requirement is that the extracted `(alpha, gamma)` be feasible for the dual of the
**full-universe** problem. Enumeration gets that by containing the universe. CG gets it by
proving nothing improving remains: at CG convergence every column in the universe -- priced
or not -- satisfies `f_c - sum alpha + sum gamma >= 0`, which IS its dual constraint. So:

- a cut taken from a **converged** subproblem CG is globally valid, and stays valid as the
  pool grows later (the full dual's feasible set does not depend on the pool);
- a cut taken from a **budget-stopped** CG is NOT valid -- those duals are feasible only for
  the restricted dual, so the cut can over-estimate the true second-stage cost and prune the
  optimum. `solve_subproblem` therefore refuses to derive a cut unless pricing exhausted.

A non-converged CG still yields a valid UPPER bound (its restricted optimum is achievable by
pooled columns), just no cut.

Two consequences for the implementation. The pool is deliberately **accumulated across
Benders iterations** per scenario -- pure warm start, and sound by the above. And pricing must
search the **full station set**, not only the stations built at `yhat`: a column through an
unbuilt station still has a dual constraint that has to be satisfied, and the seemingly
useless columns it yields are what let that station's linking dual rise to the value that
makes the cut valid at other `y`.

# `max_stops` -- the enumeration cap, and a real restriction

`enumerate_joint_routing_assignment_columns` is exponential in `max_stops` on two axes at
once (the physical-route DFS, then the per-route cartesian product over multi-certified
passengers), and throws rather than truncating when it blows past `max_routes`. `4` is
the largest value measured tractable on the reference instances (16,320 columns at a
10-station / 8-pair instance), so it is the default here rather than the formulation's own
value.

**When it is below the formulation's own `max_stops`, the run's optimality claim narrows
with it** -- the answer is optimal over routes of at most this many stops, and a longer
route can beat it. The build records that as
`metadata["benders_optimality_scope"] = "max_stops_restricted"` (versus
`"full_route_universe"` when no narrowing happened), alongside
`benders_subproblem_max_stops` and `benders_formulation_max_stops`, in exactly the spirit
of `CGSolver`'s `cg_optimality_scope`. Read it before pooling a `BendersSolver` objective
with a `DirectMIPSolver` one. `nothing` disables the cap and enumerates the formulation's
own universe.

# The rest

`max_routes`/`enumeration_time_limit_sec` are the enumerator's own guard rails, passed
straight through. `time_limit_sec` bounds each individual subproblem LP solve
(`nothing` = no limit); a subproblem that hits it returns a non-optimal status, which
`solve_subproblem` treats as a hard error rather than a weak cut, since a truncated LP's
duals are not a valid underestimator.

`:column_generation` adds `pricing` (a `CGPricingConfig` -- the pricer, defaulting to the
formulation's own, i.e. `:exact`), `reduced_cost_tol`, `max_cg_iterations`, and
`cg_pricing_time_limit_sec` (per pricing round, per scenario).

Only exhaustive-equivalent pricers are accepted (`:exact`, `:darp`, `:darp_modified`),
because full-universe exhaustion is what licenses a cut. `:station_simple` is rejected as
UNSOUND for this role -- elementary-only exhaustion yields restricted-only duals -- and the
relaxed-cluster modes are rejected only for practical reasons, being the intended way to
extend this oracle later. The constructor's error message spells out both.
"""
struct BendersSubproblemConfig
    oracle::Symbol
    max_stops::Union{Nothing, Int}
    max_routes::Int
    enumeration_time_limit_sec::Float64
    time_limit_sec::Union{Nothing, Float64}
    pricing::CGPricingConfig
    reduced_cost_tol::Float64
    max_cg_iterations::Int
    cg_pricing_time_limit_sec::Float64

    function BendersSubproblemConfig(;
            oracle::Symbol=:direct_enumeration,
            max_stops::Union{Nothing, Int, Missing}=missing,
            max_routes::Int=200_000,
            enumeration_time_limit_sec::Number=180.0,
            time_limit_sec::Union{Nothing, Number}=nothing,
            pricing::CGPricingConfig=CGPricingConfig(),
            reduced_cost_tol::Number=1e-6,
            max_cg_iterations::Int=500,
            cg_pricing_time_limit_sec::Number=300.0,
        )
        oracle in (:direct_enumeration, :column_generation) || throw(ArgumentError(
            "unsupported Benders subproblem oracle $(repr(oracle)); expected " *
            ":direct_enumeration or :column_generation",
        ))
        # `max_stops` defaults PER ORACLE, which is why the keyword takes `missing` rather
        # than a plain value: enumeration needs the cap (its pool is exponential in it), and
        # column generation does not (the pricer respects the formulation's own `max_stops`).
        # Defaulting to 4 for both would silently narrow every CG run's route universe --
        # exactly the trap this type's `max_stops` section warns about, applied by accident.
        resolved_max_stops = if max_stops === missing
            oracle === :direct_enumeration ? 4 : nothing
        else
            max_stops
        end
        isnothing(resolved_max_stops) || resolved_max_stops >= 2 ||
            throw(ArgumentError("max_stops must be at least 2"))
        max_routes > 0 || throw(ArgumentError("max_routes must be positive"))
        enumeration_time_limit_sec > 0 ||
            throw(ArgumentError("enumeration_time_limit_sec must be positive"))
        isnothing(time_limit_sec) || time_limit_sec > 0 ||
            throw(ArgumentError("time_limit_sec must be positive"))
        reduced_cost_tol >= 0 ||
            throw(ArgumentError("reduced_cost_tol must be non-negative"))
        max_cg_iterations > 0 ||
            throw(ArgumentError("max_cg_iterations must be positive"))
        cg_pricing_time_limit_sec > 0 ||
            throw(ArgumentError("cg_pricing_time_limit_sec must be positive"))
        # ONLY EXHAUSTIVE-EQUIVALENT PRICERS. A Benders cut is licensed by full-universe
        # exhaustion, so a pricer that exhausts a strict SUBSET of the universe cannot
        # license one -- see the rejection message for why `:station_simple` specifically is
        # unsound here despite being a perfectly good pricer for the CG master.
        pricing.mode in (nothing, :exact, :darp, :darp_modified) || throw(ArgumentError(
            "Benders subproblem pricing mode $(repr(pricing.mode)) cannot license a " *
            "Benders cut; use :exact (default), :darp or :darp_modified, which all search " *
            "the full revisit-tolerant route universe and are exhaustive-equivalent.\n" *
            "  :station_simple searches ELEMENTARY routes only, so its exhaustion proves " *
            "no elementary column improves -- not that no column improves. Its duals are " *
            "then feasible for the elementary-restricted dual only, and a cut built from " *
            "them can over-estimate the true second-stage cost and prune the optimum. " *
            "(`_cg_pricing_universe_is_restricted(:station_simple)` is already `true` and " *
            "its scope label is \"elementary_routes_only\"; " *
            "notes/2026-07-27 measured 70-76% objective error from exactly this class of " *
            "invalid cut.)\n" *
            "  :relaxed_cluster / :relaxed_cluster_two_tier are NOT rejected for being " *
            "unsound -- they harvest real columns, and a `converged_by_certification` " *
            "outcome IS a valid full-universe exhaustion proof, which makes them the " *
            "natural way to extend this oracle past the sizes where `:exact` still " *
            "exhausts (the measured CG frontier is n<=20 all scenarios, n=25 to <=5, n=30 " *
            "only s=1). They are held back for two practical reasons: they need a " *
            "build-time k-medoids station partition that " *
            "`_stash_joint_routing_assignment_subproblem_pricing!` does not create (so " *
            "`cg_certification_supported` would refuse them anyway), and their " *
            "certification rate is the known weak point (0/31 one-shot, 4/10 at n=40 with " *
            "no-good cut rounds) -- multiplied by one certification loop per Benders " *
            "iteration per scenario.",
        ))
        new(
            oracle, resolved_max_stops, max_routes, Float64(enumeration_time_limit_sec),
            isnothing(time_limit_sec) ? nothing : Float64(time_limit_sec),
            pricing, Float64(reduced_cost_tol), max_cg_iterations,
            Float64(cg_pricing_time_limit_sec),
        )
    end
end
