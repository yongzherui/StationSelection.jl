using Printf

"""
The `:column_generation` subproblem oracle: solve one scenario's second stage by pricing
columns on demand instead of over an enumerated pool.

# Why this is sound, stated once and precisely

A Benders cut is valid iff its duals are feasible for the dual of the **full-universe**
second-stage problem (`benders/subproblem.jl` derives that). Enumeration achieves this by
containing the universe. Column generation achieves it by *proving nothing improving
remains*: at CG convergence, every column `c` in the universe -- priced or not -- satisfies

    rc_c = f_c - sum_{p in c} alpha_p + sum gamma  >=  -reduced_cost_tol

and that inequality IS `c`'s dual constraint. So a converged round's `(alpha, gamma)` is
feasible for the full dual, the restricted LP's optimum equals the full LP's optimum
(standard CG argument), and the cut derived from it is globally valid.

Everything below follows from that one fact:

- **Convergence is not optional.** A budget-stopped or iteration-capped CG leaves duals that
  are feasible only for the *restricted* dual, and a cut from those can over-estimate the
  true `Q_s` and prune the optimum. This function therefore reports `converged` as a hard
  bit, and `solve_subproblem` refuses to build a cut without it.
- **The pool is accumulated across Benders iterations, deliberately.** Validity comes from
  exhaustion at the moment of extraction, not from the pool being frozen, so keeping columns
  is a pure warm start: later Benders iterations should exhaust in one or two rounds.
- **Pricing searches the FULL station set, not just the stations built at `yhat`.** This
  looks wasteful and is not. A column through an unbuilt station `j` still carries a dual
  constraint that must hold, so restricting the search would never verify it. And the
  apparent livelock -- `j` unused means its linking row `sum(theta) <= 0` is slack, so
  `gamma_j = 0`, so a candidate through `j` prices negative, enters, and is forced to zero --
  resolves itself: once such a column is in the pool that linking row has a term, the LP can
  raise `gamma_j` above zero, and the reduced cost goes non-negative. Those columns are the
  certificate that the cut holds at other `y`, not dead weight.

# What is reused

The pricer is the ordinary joint routing+assignment label-setting engine: `_run_pricing_round`
(`label_setting/round.jl`) with its `only_scenarios` restriction, `extract_joint_routing_assignment_duals`
for the duals, and `add_joint_routing_assignment_column!` to install columns. Nothing about
pricing is Benders-specific -- the subproblem is the joint master's own shape with `y` fixed,
so the same machinery applies unchanged. `_run_pricing_round` reads only `reduced_cost_tol`
and `parallel_scenario_pricing` off its `CGSolver` argument, which is why a lightweight
settings carrier suffices rather than a real CG solve.
"""

"""
    _benders_subproblem_cg_settings(config) -> CGSolver

A `CGSolver` used purely as a settings carrier for `_run_pricing_round`, which reads exactly
two fields off it (`reduced_cost_tol`, `parallel_scenario_pricing`). Constructing one is
cheaper and less brittle than threading those two values through the pricing API, and it
keeps the pricer's signature untouched.

`parallel_scenario_pricing` is deliberately `false`: each subproblem model holds ONE
scenario, so there is nothing to parallelise inside a round. Scenario-level parallelism, if
ever wanted, belongs in the Benders loop across subproblems -- not here.
"""
_benders_subproblem_cg_settings(config::BendersSubproblemConfig) = CGSolver(
    config = SolverOptions(silent = true),
    pricing = config.pricing,
    reduced_cost_tol = config.reduced_cost_tol,
    pricing_time_limit_sec = config.cg_pricing_time_limit_sec,
    parallel_scenario_pricing = false,
)

"""
    BendersSubproblemCGResult

Outcome of one scenario's inner CG solve. `converged` is the bit cut validity hangs on: it
means pricing EXHAUSTED (no column in the universe prices below `-reduced_cost_tol`), not
merely that the loop stopped.

`stop_reason` distinguishes the ways it can fail to converge, because they call for different
responses: `pricing_inconclusive` (budget) wants more time, `iteration_limit` wants a higher
cap, and `dedup_stall` is the known stale-`tau` livelock -- the pricer reports an improving
column whose `(scenario, signature)` is already pooled at no greater `tau`, so
`add_joint_routing_assignment_column!` skips it and the LP cannot change. That last one is a
real defect elsewhere in the stack, not a budget problem, and is reported as its own reason so
it is not misread as one.

Under a relaxed-cluster mode two more reasons appear: `converged_by_certification` (the
relaxation exhausted -- a full-universe proof, so the cut is licensed) and
`certification_inconclusive` (the attempt ran out of budget or hit the cut cap, proving
nothing, so no cut). `certifications` counts attempts made, which is the number to watch:
refuted attempts are productive (they harvest columns), so a high count with eventual
certification is healthy, while a high count ending inconclusive is the known weak point.
"""
struct BendersSubproblemCGResult
    scenario::Int
    converged::Bool
    stop_reason::String
    cg_iterations::Int
    columns_added::Int
    pricing_sec::Float64
    # The total splits by WHICH universe the round searched, which is the question the
    # activated family exists to answer: is a cheap oracle's win a shorter full-universe
    # search, or merely a smaller column pool feeding one that is just as long?
    #   restricted -- the dual completion was installed before pricing, so unbuilt stations
    #     were driven out of the pricer's filter and the search was effectively built-only.
    #   full -- no completion installed; the search covered every station.
    # plain CG is all `full`; activated and activated_lpo are all `restricted`; only
    # `:column_generation_activated_warm_start` has both, one per phase.
    restricted_pricing_sec::Float64
    full_pricing_sec::Float64
    lp_sec::Float64
    certifications::Int
end

"""
    _solve_joint_routing_assignment_subproblem_by_cg!(build, config) -> BendersSubproblemCGResult

Run CG on one already-`y`-fixed subproblem model until pricing exhausts.

Assumes `y` has already been fixed by the caller (`subproblem.jl` does it before dispatching
on the oracle), so the LP solved here is `Q_s` at the master's incumbent over the current
pool. `incumbent` is still needed explicitly: under
`oracle = :column_generation_activated` the dual completion has to know which stations are
built, and reading that back off the fixed variables would be a needless round trip through
the solver.

Under the activated oracle the exhaustion this loop certifies is over the ACTIVATED
candidate set rather than the whole universe -- which is still a full-universe certificate
once the completion is applied, because every column outside that set has its dual
constraint satisfied by construction. That argument is in `BendersSubproblemConfig`.
"""
function _solve_joint_routing_assignment_subproblem_by_cg!(
        build::BuildResult,
        config::BendersSubproblemConfig,
        incumbent::Vector{Float64},
    )::BendersSubproblemCGResult
    sm = build.model
    mapping = build.mapping
    scenario = Int(sm[:benders_subproblem_scenario])
    data = sm[:joint_routing_assignment_data]
    pricing_formulation = sm[:joint_routing_assignment_pricing_formulation]
    mode = sm[:joint_routing_assignment_pricing_mode]::Symbol
    # BOTH activated oracles restrict pricing the same way -- the closed-form completion is
    # what makes the search cheap. They differ only in what the CUT is built from, which
    # happens after this loop returns (see subproblem.jl).
    activated = config.oracle in (:column_generation_activated,
                                  :column_generation_activated_lpo,
                                  :column_generation_activated_warm_start)
    # `:column_generation_activated_warm_start` is a WARM START, not a restriction: phase 1
    # prices built-only (cheap, and it already reaches the exact `Q_s(yhat)` -- a column
    # touching an unbuilt station is pinned to `theta = 0` by its own `theta - y_j <= 0`
    # row, so it can never improve the objective at a fixed `yhat`). Phase 1's exhaustion is
    # therefore a VALUE certificate only; the duals it leaves are feasible for the
    # completed-dual point, not for the raw one a cut would be read off here. So phase 2
    # drops the restriction and prices the full universe to exhaustion, and the cut is taken
    # from THOSE duals by the ordinary argument -- no completion involved.
    #
    # What this isolates: whether the full-station grind (columns that enter, get pinned to
    # zero, and exist only to raise `gamma` into dual feasibility) is warm-start sensitive.
    # It is the null hypothesis the completion approach is trying to route around.
    warm_start = config.oracle === :column_generation_activated_warm_start
    phase = warm_start ? 1 : 2
    settings = _benders_subproblem_cg_settings(config)

    cg_iterations = 0
    columns_added = 0
    pricing_sec = 0.0
    restricted_pricing_sec = 0.0
    full_pricing_sec = 0.0
    lp_sec = 0.0
    certifications = 0

    for iteration in 1:config.max_cg_iterations
        cg_iterations = iteration

        t_lp = time()
        optimize!(sm)
        lp_sec += time() - t_lp
        status = JuMP.termination_status(sm)
        status == MOI.OPTIMAL || return BendersSubproblemCGResult(
            scenario, false, "lp_$(status)", cg_iterations, columns_added,
            pricing_sec, restricted_pricing_sec, full_pricing_sec, lp_sec, certifications,
        )

        duals = extract_joint_routing_assignment_duals(sm)
        # Whether THIS round searches the restricted or the full universe -- the completion
        # is what restricts it, so installing it and being restricted are the same fact.
        restricted_round = activated && !(warm_start && phase == 2)
        if restricted_round
            # BEFORE pricing, deliberately. The completion raises `gamma` to `alpha_p` on
            # unbuilt stations, which drives every candidate through such a station to
            # `rho <= 0` -- so the pricer's own filter restricts the search to the activated
            # stations. The restriction and the dual repair are the same operation; see
            # `_benders_activated_complete_duals!`.
            _benders_activated_complete_duals!(
                duals..., incumbent, data, mapping, scenario,
                Float64(sm[:joint_routing_assignment_walk_cost_weight]),
            )
        end

        # Two ways to establish the exhaustion this loop needs, and they differ in HOW they
        # prove it rather than in what they prove:
        #
        #   search-based (:exact / :darp / :darp_modified) -- exhaust the route universe.
        #     `_run_pricing_round` returns the improving columns and records exhaustion on
        #     the model.
        #   certificate-based (:relaxed_cluster / :relaxed_cluster_two_tier) -- exhaust a
        #     RELAXATION that lower-bounds every real route's reduced cost. `certified` then
        #     proves no real improving column exists WITHOUT having searched for one, and it
        #     covers the full universe, so it licenses a cut exactly as a search would. A
        #     refuted attempt is not wasted: it harvests the real columns its exhaustive
        #     subset searches found, so it doubles as this round's pricing.
        #
        # This is the branch that lets the oracle work past the sizes where the exact search
        # stops exhausting (measured CG frontier: n<=20 all scenarios, n=25 to <=5, n=30 s=1).
        certifying = mode in (:relaxed_cluster, :relaxed_cluster_two_tier)
        t_price = time()
        certified_now = false
        columns = if certifying
            cert = cg_certification_round(
                build, mapping, sm, duals, settings;
                time_limit_sec = config.cg_pricing_time_limit_sec,
                iteration = iteration, only_scenarios = [scenario],
            )
            certified_now = cert.certified
            certifications += 1
            cert.certified ? Any[] : _cg_materialize_certification_columns(
                build, mapping, sm, duals, cert.candidates,
            )
        else
            _run_pricing_round(
                pricing_formulation, mapping, sm, duals, settings;
                only_scenarios = [scenario],
                time_limit = config.cg_pricing_time_limit_sec,
            )
        end
        round_sec = time() - t_price
        pricing_sec += round_sec
        if restricted_round
            restricted_pricing_sec += round_sec
        else
            full_pricing_sec += round_sec
        end

        if certifying && certified_now && warm_start && phase == 1
            # Phase 1 certified. That is a VALUE certificate, not the end of the solve: the
            # duals it certified against are the closed-form-completed ones, so returning
            # here would hand back exactly the plain activated oracle's weak cut while still
            # calling itself a warm start. Switch phases and re-price the full universe at
            # the RAW duals, which is the whole point of this oracle.
            #
            # This branch has to exist separately because it sits BEFORE the `isempty(columns)`
            # exit that handles the search-based pricers -- a certifying round returns an
            # empty column vector when it certifies, and it never sets the model's exhausted
            # flag, so the phase switch down there is unreachable under :relaxed_cluster.
            phase = 2
            if config.verbose
                println("      [cg s=$scenario it=$iteration] phase 1 (built-only) " *
                        "CERTIFIED in $(round(time() - t_price; digits=1))s; pool " *
                        "$(length(sm[:joint_routing_assignment_columns])), entering " *
                        "phase 2 (full universe, raw duals)")
                flush(stdout)
            end
            continue
        end

        if certifying && certified_now
            # The certificate IS the exhaustion proof; nothing further to search.
            config.verbose && (@printf("      [cg s=%d it=%d] CERTIFIED in %.1fs (cum %.1fs)\n",
                                       scenario, iteration, time() - t_price, pricing_sec);
                               flush(stdout))
            return BendersSubproblemCGResult(
                scenario, true, "converged_by_certification", cg_iterations, columns_added,
                pricing_sec, restricted_pricing_sec, full_pricing_sec, lp_sec, certifications,
            )
        end

        if certifying && isempty(columns)
            # Not certified and nothing harvested: the attempt was inconclusive (budget or
            # cut cap). No certificate means no cut -- exactly the same refusal as a
            # non-exhausted search, for the same reason.
            # WHICH exit fired matters more than the elapsed time, and the elapsed
            # time alone actively misleads: an n=40 attempt reported "inconclusive after
            # 31.8s" against a nominal 1800s budget, which reads as a budget shortfall and
            # is not one.
            #
            # The scenario gets the FULL nominal budget here, not a slice of it. The round
            # does divide its deadline by the scenario count, but only over the scenarios it
            # was asked to price (`certification/round.jl`), and a Benders subproblem model
            # holds exactly ONE scenario -- `only_scenarios = [scenario]` filters the list to
            # length 1, so the divisor is 1. (Getting this backwards is what sent an earlier
            # reading of the 31.8s exit chasing a budget shortfall that did not exist. The
            # relaxed guide search does take half the remaining slice, but half of 1800s is
            # still not 31.8s.) So the reason symbol, not the clock, is what says whether to
            # spend more time or change the partition.
            if config.verbose
                reasons = isempty(cert.inconclusive_reasons) ? "unreported" :
                    join(("s$sc=$rz" for (sc, rz) in
                          zip(cert.inconclusive_scenarios, cert.inconclusive_reasons)), ", ")
                @printf("      [cg s=%d it=%d] certification INCONCLUSIVE after %.1fs (nominal budget %.0fs, per-scenario slice ~%.0fs) | K=%d | relaxed bound %.4f | limited by: %s\n",
                        scenario, iteration, time() - t_price,
                        config.cg_pricing_time_limit_sec,
                        config.cg_pricing_time_limit_sec / max(1, cert.n_scenarios),
                        cert.n_clusters, cert.relaxed_rc_bound, reasons)
                # Is "cut the barren support again" viable here? These are the numbers that
                # answer it: how many cuts stayed ACTIVE after subsumption pruning, how the
                # relaxed sweep's cost moved as they accumulated, and -- decisively --
                # whether it was still EXHAUSTING at the end. An unexhausted sweep cannot
                # certify no matter how many more rounds it gets, so if that flips to false
                # the answer is a coarser cut or a tighter relaxation, not more rounds.
                tr = cert.trace
                if !isempty(tr) && haskey(first(tr), :n_active_cuts)
                    cuts_seq = [r.n_active_cuts for r in tr]
                    secs = [r.relaxed_sec for r in tr]
                    n_unexh = count(r -> !r.relaxed_exhausted, tr)
                    @printf("        cut cost: %d rounds | active cuts %d -> %d (max %d) | relaxed sweep %.3fs -> %.3fs (total %.1fs) | rounds whose sweep did NOT exhaust: %d\n",
                            length(tr), first(cuts_seq), last(cuts_seq), maximum(cuts_seq),
                            first(secs), last(secs), sum(secs), n_unexh)
                    flush(stdout)
                end
            end
            return BendersSubproblemCGResult(
                scenario, false, "certification_inconclusive", cg_iterations, columns_added,
                pricing_sec, restricted_pricing_sec, full_pricing_sec, lp_sec, certifications,
            )
        end

        if config.verbose
            # How many columns the round PRICED (search productivity), and the status that
            # licenses a cut -- which differs by branch:
            #   search branch: `exhausted`, set by _run_pricing_round on the model.
            #   certification branch: `refuted` (harvested columns, no proof yet). NOTHING
            #     sets the exhausted flag there, and `_cg_pricing_exhausted` defaults to
            #     `true` for a model that never set it -- so printing it in that branch
            #     claimed "exhausted true" for rounds that were actually refuted. Reporting
            #     the branch's own status avoids inventing a proof that was not made.
            @printf("      [cg s=%d it=%d] priced %d | %s | price %.1fs cum %.1fs\n",
                    scenario, iteration, length(columns),
                    certifying ? "refuted" : "exhausted $(_cg_pricing_exhausted(sm))",
                    time() - t_price, pricing_sec)
            flush(stdout)
        end

        if isempty(columns) && warm_start && phase == 1 && _cg_pricing_exhausted(sm)
            # Phase 1 done: the built-only universe is exhausted, so the pool now attains
            # `Q_s(yhat)` exactly. Hand the SAME master and pool to phase 2 rather than
            # returning -- the value is right, only the duals are not yet licensed.
            phase = 2
            # `println`, not `@printf`: a `*`-concatenated format string is not a literal,
            # and `@printf` rejects it at MACRO EXPANSION -- i.e. the package fails to load,
            # not at the call.
            if config.verbose
                println("      [cg s=$scenario it=$iteration] phase 1 (built-only) " *
                        "exhausted; pool " *
                        "$(length(sm[:joint_routing_assignment_columns])), entering " *
                        "phase 2 (full universe)")
                flush(stdout)
            end
            continue
        end

        if isempty(columns)
            # Empty AND exhausted is the certificate: no column in the universe prices
            # below the tolerance, so these duals are full-universe dual feasible and a cut
            # may be taken. Empty but NOT exhausted only means the search ran out of budget,
            # which proves nothing.
            exhausted = _cg_pricing_exhausted(sm)
            return BendersSubproblemCGResult(
                scenario, exhausted,
                exhausted ? "converged" : "pricing_inconclusive",
                cg_iterations, columns_added, pricing_sec, restricted_pricing_sec, full_pricing_sec, lp_sec, certifications,
            )
        end

        n_added = 0
        for column in columns
            _theta, action = add_joint_routing_assignment_column!(sm, data, mapping, column)
            action === :added && (n_added += 1)
        end
        columns_added += n_added
        config.verbose && (@printf("      [cg s=%d it=%d] added %d of %d | pool %d\n",
                                   scenario, iteration, n_added, length(columns),
                                   length(sm[:joint_routing_assignment_columns])); flush(stdout))
        if n_added == 0
            # Improving columns were priced but every one was skipped as an already-pooled
            # `(scenario, signature)` at no greater `tau`. The LP is unchanged, so the next
            # iteration would price the same thing forever. See the result type's docstring:
            # this is the stale-`tau` livelock, and it is NOT convergence -- returning
            # `converged` here would hand `solve_subproblem` duals that are only
            # restricted-feasible and produce an invalid cut.
            return BendersSubproblemCGResult(
                scenario, false, "dedup_stall", cg_iterations, columns_added,
                pricing_sec, restricted_pricing_sec, full_pricing_sec, lp_sec, certifications,
            )
        end
    end

    return BendersSubproblemCGResult(
        scenario, false, "iteration_limit", cg_iterations, columns_added,
        pricing_sec, restricted_pricing_sec, full_pricing_sec, lp_sec, certifications,
    )
end
