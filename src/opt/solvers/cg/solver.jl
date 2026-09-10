export CGSolver

"""
    CGSolver <: AbstractSolver

Column-generation outer loop: repeatedly solve the restricted master (`build_result`'s
model), price new columns against its duals, and add any improving ones back into the
master, until pricing finds nothing improving or `max_iterations` is reached.

## Which pricer (`pricing::CGPricingConfig`)

*Which* search finds those columns -- the pricer, its warm-start phasing, and every
relaxed-cluster setting -- lives on `pricing`, a `CGPricingConfig`
(`opt/solvers/cg/pricing_config.jl`), which documents each field. It is a solver concern
rather than a formulation one because a pricer is a search algorithm: two runs differing
only in `pricing.mode` solve the identical model. The fields below are about *budgets* --
how long each search may run -- and are orthogonal to it.

`pricing_time_limit_sec` is the wall-clock budget for **one whole pricing round**, across
every scenario. Under serial pricing `_run_pricing_round` divides it equally, so each
scenario's label search gets `pricing_time_limit_sec / n_scenarios`; under
`parallel_scenario_pricing` the searches overlap, so each scenario gets the *full* budget
and the round still fits the same wall. An empty result from a search that hits this
limit does not set `cg_converged=true`; only exhausted pricing can certify that the
restricted master needs no further improving columns.

The budget is per round rather than per scenario so that a round costs what it says it
costs -- no `n_scenarios` multiplier -- which is what makes `total_time_limit_sec` below
enforceable, and so that no scenario can starve another (a shared deadline spent in
scenario order would always favour the same scenarios, improving one scenario's coverage
round after round while the rest never advance).

## Two-tier pricing: regular vs. certifying rounds

Pricing runs at two budgets. Ordinary iterations use `pricing_time_limit_sec` (default
300 s), which is sized to *find* improving columns cheaply, not to prove none exist.
When a regular round comes back empty **without** having exhausted its frontier, the
empty result is inconclusive -- it may just have run out of time -- so the loop
immediately re-prices the same duals at `certifying_pricing_time_limit_sec` (default
3600 s). That second, longer pass is the *certifying* round: only it can turn an empty
result into `cg_converged=true`.

A regular round that comes back empty **and** exhausted needs no escalation -- it has
already proved no negative-reduced-cost column exists, and the loop stops certified.
Escalation is therefore paid at most once per iteration and only when it might change
the answer. `metadata["cg_certifying_rounds"]` counts how often it fired, and each
iteration log row carries `certifying_pricing` plus the `pricing_limit_sec` actually
used.

## The `:relaxed_cluster` pricing mode

`pricing.mode = :relaxed_cluster` (with `pricing.relaxed_cluster_count = K`, which it
requires) is a pricer in its own right. It is *both* halves of an iteration at once: it
finds columns, and it is the only search that can prove there are none left. Available
only on `AggregateODRouteJointRoutingAssignmentFormulation`, which is what implements the
hooks below.

Each iteration it runs the relaxed-cluster no-good-cut loop instead of an ordinary pricing
round, under the same `pricing_time_limit_sec`. The loop prices a **relaxation** of the
pricing problem whose minimum reduced cost lower-bounds the real one, on `K` cluster nodes
rather than `n` stations. Three things can come back:

  *certified* -- the relaxation exhausted with nothing below `-reduced_cost_tol`, so no
    real improving column exists either. The loop stops with `cg_converged=true` and
    `cg_stop_reason="converged_by_certification"`.

  *negative_rc_column_found* -- the relaxation named an improving cluster route, so the
    loop searched that route's cluster support exhaustively with the real exact pricer, and
    that search produced a genuinely improving column. Those columns ARE this iteration's
    pricing result: the attempt IS the pricing round rather than an overhead on top of one.
    **This is the mode working, not failing.** The mode prices first and certifies second --
    MEASURED, 96% of attempts end here
    (`notes/2026-09-06_relaxed_cluster_harvesting_refinement_and_cuts.md`), which is the
    expected shape of a solve, not a 96% failure rate. Along the way a support that comes
    back barren becomes a no-good cut ("every route must visit at least one cluster outside
    T") and the relaxation is asked again; those cuts are the evidence a later attempt
    certifies from.

  *inconclusive* -- the loop ran out of its budget (or of cut rounds) without settling
    either way, and harvested nothing. This is the only case that escalates: the same round
    is re-run at `certifying_pricing_time_limit_sec`, exactly as an inconclusive ordinary
    pricing round escalates to the certifying round above. A second inconclusive result
    ends the loop with `cg_stop_reason="pricing_inconclusive"` -- nothing was proved.

**The cuts are the mechanism, not a refinement of it.** A cut-free round -- which is
exactly this loop's round 1 -- certified 0 times across ~1130 measured attempts at every
size and every K, because a converged master's exact minimum reduced cost is 0 while the
relaxation's slack is 10^2-10^3. A cut-free mode used to exist alongside the loop for
exactly that comparison, under the name `:relaxed_cluster_nogood`; it was removed once the
answer was in, and that name is now rejected with a message pointing here.

**The two-tier certifying re-price is unreachable in this mode**, and deliberately so.
When a round comes back empty, escalating the CERTIFIER is the cheaper buy: the relaxed
search runs on `K` cluster nodes, the exact pricer's cost is super-linear in node count,
and a success is a full-route-universe certificate rather than a pricer-scoped one.
`cg_certifying_rounds` therefore stays 0 for a `:relaxed_cluster` run; its escalated
attempts appear as extra `cg_certification_rounds` with `*_escalated` outcomes instead.

The trade is measured, not free: 4 of 45 arm runs (serial) and 1 of 35
(parallel) reached OPTIMAL *only* because the two-tier round certified where the relaxation
did not. All but one were `K/n = 0.4`, where the coarse partition kept pricing columns
instead of certifying on every other ground.
At `K/n` in [0.6, 0.8] the fallback was load-bearing for exactly one run in two full
sweeps.

A certificate from here covers the **full route universe**, because the relaxation bounds
every real route rather than only the ones the active pricer searches. So a run that
certified reports `cg_optimality_scope="full_route_universe"` even when a
`pricing.warm_start_mode=:station_simple` phase found most of its columns, and
`cg_certified_by_relaxation=true` records that the certificate came from the relaxation
rather than from exhausted pricing.
`metadata["cg_certification_rounds"]`/`["cg_certification_sec"]` cost it, and
`["cg_certification_negative_rc_column_rounds"]`/`["cg_certification_inconclusive_rounds"]`
split the non-certifying rounds into two kinds that are NOT two failures.
*negative_rc_column_found* means an exhaustive real search over some cluster support found
a genuinely improving column: the round priced, the columns are where CG's progress comes
from, and a high count is the normal shape of a solve rather than a problem to fix.
*Inconclusive* is the budget one, and the only one
`pricing_time_limit_sec`/`certifying_pricing_time_limit_sec` can move. Each iteration log
row carries `certification_sec`, `certification_certified` and `certification_outcome`.

Requires a formulation implementing `cg_certification_supported`/`cg_certification_round`;
a `:relaxed_cluster` pricing mode on a formulation that supports neither is rejected up
front, never silently ignored.

## Total budget (`total_time_limit_sec`)

A strict wall-clock cap on the CG loop (default `Inf`). It is checked before every
iteration and additionally *clamps* each pricing round's budget to the remaining budget.
Because that budget is per round (see above), the clamp is exact: a round cannot spend
more than it was granted, so the loop cannot overrun the cap by more than one label
search's clock-check granularity. When the budget runs out the
loop stops and the run reports `cg_converged=false` / `cg_pricing_exhausted=false` with
`cg_stop_reason="total_budget"` and `cg_total_budget_exhausted=true`: the incumbent is
feasible but its optimality is **not** certified, and the LP value is *not* a valid
bound on the unrestricted optimum. The point is that a budget-bound run still returns a
usable result instead of being killed by the scheduler with nothing written.

`parallel_scenario_pricing` (default `false`) prices scenarios concurrently with
`Threads.@threads` when more than one thread is available. Both settings obey the same
round wall budget, so the comparison is like for like on time; what differs is how much
search fits inside it. Serial splits the round `n_scenarios` ways, parallel gives every
scenario the whole round, so parallel performs up to `n_scenarios` x more label search per
round and can therefore certify instances a serial run cannot. The round's wall bound holds
only while `Threads.nthreads() >= n_scenarios`; with fewer threads the searches run in
waves and a round can take up to `ceil(n_scenarios / nthreads)` x the budget.

Note the cap bounds the **loop**. When `recover_integer_solution=true` the recovery MIP
runs afterwards under its own `config.time_limit_sec`, so the whole solve can exceed
`total_time_limit_sec` by at most that one solve -- size the SLURM walltime with room
for both (e.g. a 4 h budget and a 300 s recovery limit fit comfortably in a 6 h job).

Relies on three formulation-specific hooks -- implemented per `AbstractFormulation`
(or per `AbstractProblem`), not here:

    extract_duals(build_result, mapping, m) -> duals
    price_columns(build_result, mapping, m, duals, solver::CGSolver) -> Union{Nothing, AbstractVector}
    add_columns!(build_result, mapping, m, columns) -> Int

`mapping` (`build_result.mapping`) is passed as its own positional argument, not just
read off `build_result`, so that formulation-specific methods can dispatch on its
concrete type (e.g. `mapping::AggregateODRouteMap`). Dispatching on `build_result::BuildResult`
alone can't distinguish formulations -- every formulation's hook would share the exact
same `(BuildResult, JuMP.Model, ...)` signature as this file's generic fallback, which
Julia treats as a redefinition (not a new method) and module precompilation then rejects
outright as illegal method overwriting.

`price_columns` returns `nothing` (or an empty collection) when no improving column
exists -- that's the convergence signal this loop watches for. `add_columns!` mutates
the restricted master in place and returns how many columns it added.

## Integer recovery (`recover_integer_solution`)

The loop above solves an LP relaxation throughout -- the master's first-stage/column
variables (e.g. `y`, `θ`) are continuous so their duals are valid for pricing. That LP
optimum is generally fractional. When `recover_integer_solution=true`, once the loop
above exits (converged or `max_iterations` reached) with an `OPTIMAL` LP on hand, a
fourth hook

    integer_recovery_build(build_result, mapping, m) -> BuildResult

is called to *rebuild* the master from scratch in its true (binary/integer) domain, over
the exact column pool CG has generated so far -- no further pricing happens. This is a
real `build_model`-shaped rebuild, not an in-place mutation of `m`: see
`_build_joint_routing_assignment_model`/`integer_recovery_build`
(`optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl`) for
why sharing the actual construction code with `build_model` (parameterized by
`relax_integrality`/seed columns) is safer than duplicating it as a set of post-hoc
`set_binary` calls. The returned `BuildResult` replaces this call's `build_result`/`m`,
which is then re-optimized once as a genuine MIP. This is the standard "restricted master
heuristic": the resulting integer solution is feasible for the real problem and its
objective is a valid upper bound, but -- because pricing only ever ran against LP duals
-- it is not guaranteed globally optimal for the original (unrestricted) column set. The
pre-recovery LP objective is preserved in `OptResult.metadata` under
`"cg_lp_objective_value"` as a lower bound for judging that gap; `"cg_converged"` records
whether pricing actually exhausted (vs. hit `max_iterations`), since only the converged
case makes that LP value a valid bound on the true (unrestricted) optimum.

## Per-iteration log (`metadata["cg_iteration_log"]`)

One `NamedTuple` per CG iteration, in order:

| field | meaning |
| --- | --- |
| `iteration` | 1-based iteration index |
| `master_sec` | wall time in this iteration's master `optimize!` |
| `pricing_sec` | wall time in `price_columns` |
| `add_columns_sec` | wall time in `add_columns!` |
| `columns_added` | how many columns pricing returned this iteration |
| `columns_accepted` | how many of those actually entered the master (`add_columns!`'s return; the rest were de-duplicated away) |
| `cumulative_columns_added` | running total of `columns_accepted`, excluding seed columns |
| `master_objective` | master LP objective, or `missing` if not `OPTIMAL` |
| `master_status` | this iteration's master termination status |
| `pricing_limit_sec` | the time limit this iteration's pricing actually ran under (regular, certifying, or whatever the total budget clamped it to) |
| `certifying_pricing` | `true` if this iteration escalated to a certifying round |
| `certification_sec` | wall time in this iteration's relaxation certification attempt (`0.0` when the feature is off) |
| `certification_certified` | `true` on the single iteration whose relaxation certified, ending the loop |
| `certification_outcome` | `"certified"` / `"negative_rc_column_found"` (the attempt priced a real improving column, which is the ordinary outcome -- this iteration made progress, it did not fail) / `"inconclusive"` (the attempt ran out of budget and learned nothing) / `"none"` (no attempt this iteration) |
| `relaxed_rc_bound` | a valid LOWER bound on the minimum reduced cost over the whole real route universe, or `NaN` when this iteration established none (no attempt, or an inconclusive one -- see `RelaxedClusterCertificationResult.relaxed_rc_bound`). The master objective is an *upper* bound on `z_LP` that descends as columns arrive; this is the only quantity in the loop that bounds from below, and it is what makes a round that priced instead of certifying a measurement rather than a failed test |

The final iteration is always logged, including the one that breaks the loop (on
convergence, on a non-`OPTIMAL` master, or on the last `max_iterations` pass), so
`length(log) == metadata["cg_iterations"]`.

Splitting the wall time three ways is what makes a master-bound run distinguishable
from a pricing-bound one without re-running under a profiler. Note this is separate
from `"cg_pricing_stats"`, which is a flat per-(iteration x scenario) list of *label
search* counters carrying no iteration index, and whose own `t_*_sec` timers are only
populated when the label-setting round is called with `profile=true`.

`"cg_lp_loop_sec"` is the CG loop alone and `"cg_integer_recovery_sec"` the recovery
solve (`0.0` when recovery is off), so the two can be reported separately even though
`OptResult.runtime_sec` covers both.
"""
struct CGSolver <: AbstractSolver
    config::SolverOptions
    pricing::CGPricingConfig
    max_iterations::Int
    reduced_cost_tol::Float64
    pricing_time_limit_sec::Float64
    certifying_pricing_time_limit_sec::Float64
    total_time_limit_sec::Float64
    parallel_scenario_pricing::Bool
    initial_columns::Union{Nothing, AbstractVector}
    recover_integer_solution::Bool
    iteration_callback::Union{Nothing, Function}
    dual_callback::Union{Nothing, Function}

    function CGSolver(;
            config::SolverOptions=SolverOptions(),
            pricing::CGPricingConfig=CGPricingConfig(),
            max_iterations::Int=1_000,
            reduced_cost_tol::Number=1e-6,
            pricing_time_limit_sec::Number=300.0,
            certifying_pricing_time_limit_sec::Number=3600.0,
            total_time_limit_sec::Number=Inf,
            parallel_scenario_pricing::Bool=false,
            initial_columns::Union{Nothing, AbstractVector}=nothing,
            recover_integer_solution::Bool=false,
            iteration_callback::Union{Nothing, Function}=nothing,
            dual_callback::Union{Nothing, Function}=nothing,
        )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        reduced_cost_tol >= 0 || throw(ArgumentError("reduced_cost_tol must be non-negative"))
        pricing_time_limit_sec > 0 || throw(ArgumentError("pricing_time_limit_sec must be positive"))
        certifying_pricing_time_limit_sec > 0 ||
            throw(ArgumentError("certifying_pricing_time_limit_sec must be positive"))
        certifying_pricing_time_limit_sec >= pricing_time_limit_sec || throw(ArgumentError(
            "certifying_pricing_time_limit_sec ($certifying_pricing_time_limit_sec) must be >= " *
            "pricing_time_limit_sec ($pricing_time_limit_sec): the certifying round exists to give " *
            "an inconclusive regular round MORE time, never less",
        ))
        total_time_limit_sec > 0 || throw(ArgumentError("total_time_limit_sec must be positive"))
        new(
            config, pricing, max_iterations, Float64(reduced_cost_tol),
            Float64(pricing_time_limit_sec), Float64(certifying_pricing_time_limit_sec),
            Float64(total_time_limit_sec), parallel_scenario_pricing,
            initial_columns, recover_integer_solution, iteration_callback,
            dual_callback,
        )
    end
end

