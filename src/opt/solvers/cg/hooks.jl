"""
The formulation-specific hooks `CGSolver`'s loop calls, and the small generic helpers that
sit beside them.

Every hook here is a fallback that throws or answers "unsupported". The real implementations
are methods on `mapping`'s concrete type -- `AggregateODRouteMap`'s live in
`../../optimize/aggregate_od_route/column_generation/dispatch.jl`. `mapping` is passed as
its own positional argument precisely so those methods can dispatch on it; see the
`CGSolver` docstring in `solver.jl` for why dispatching on `BuildResult` alone cannot work.
"""


function extract_duals(build_result::BuildResult, mapping, m::JuMP.Model)
    throw(MethodError(extract_duals, (build_result, mapping, m)))
end

"""
    cg_pricing_mode(build_result, mapping, m) -> Union{Nothing, Symbol}
    set_cg_pricing_mode!(build_result, mapping, m, mode::Symbol)

Optional hook pair supporting `CGSolver.pricing.warm_start_mode`. A formulation whose
pricer is selectable implements both: the getter returns the mode currently in force, the
setter switches it mid-solve. The defaults make the feature inert -- `cg_pricing_mode`
returns `nothing`, meaning "this formulation has no selectable pricer", and the loop then
refuses a `warm_start_mode` rather than silently ignoring it.

The switch has to reach the *model*, not the formulation object: hooks only ever receive
`build_result`/`mapping`/`m`, and the pricer is chosen per pricing call from state stashed
on `m` at build time.
"""
cg_pricing_mode(build_result::BuildResult, mapping, m::JuMP.Model) = nothing

function set_cg_pricing_mode!(build_result::BuildResult, mapping, m::JuMP.Model, mode::Symbol)
    throw(MethodError(set_cg_pricing_mode!, (build_result, mapping, m, mode)))
end

"""
    _cg_pricing_universe_is_restricted(mode) -> Bool
    _cg_optimality_scope(mode) -> String

Whether pricing in `mode` searches a strict subset of the formulation's route universe,
and a label for the scope of any optimality claim made in it.

A run finishing in a restricted mode still reports `SOLVE_OPTIMAL` when its pricing
exhausted -- the status keeps its usual meaning, "no improving column remains in the
universe that was searched". What changes is the *scope* of that statement:
`:station_simple` exhausts elementary routes only, so its optimum can be beaten by a
column that revisits a station (`o->d->o` serving both `o->d` and `d->o` is the minimal
case). These two functions exist so that scope travels with every result instead of being
something the reader has to infer from the formulation, and `cg_optimality_scope` is the
key to grep for when auditing whether a certified number is a full-universe optimum.
"""
# `:relaxed_cluster` is deliberately absent: it never reports its own exhaustion, so the
# only way it ends a solve is with a certificate that bounds every real route.
_cg_pricing_universe_is_restricted(mode::Union{Nothing, Symbol}) =
    mode === :station_simple || mode === :cluster_guide

function _cg_optimality_scope(mode::Union{Nothing, Symbol})
    mode === :station_simple && return "elementary_routes_only"
    mode === :cluster_guide && return "cluster_guided_station_subset_only"
    return "full_route_universe"
end

function price_columns(build_result::BuildResult, mapping, m::JuMP.Model, duals, solver::CGSolver;
        time_limit_sec::Real=solver.pricing_time_limit_sec)
    throw(MethodError(price_columns, (build_result, mapping, m, duals, solver)))
end

"""
    cg_certification_supported(build_result, mapping, m) -> Bool
    cg_certification_round(build_result, mapping, m, duals, solver; time_limit_sec)
        -> (; certified::Bool, improving_found::Bool, candidates, ...)

Optional hook pair backing the `:relaxed_cluster` pricing mode: a *relaxation* round that
answers "can an improving column still exist for these duals?", and harvests the real
columns its refutation searches turn up on the way.

This is a wider contract than `price_columns`, not a narrower one. The round searches a
relaxed problem whose solutions need not correspond to real columns, so on its own it
returns only a bit -- but the direction of the relaxation is what makes that bit worth
having: it must lower-bound the real pricing problem's minimum reduced cost, so

    no relaxed solution below -reduced_cost_tol  =>  no real column below it either

and the loop can stop *certified* without ever running the expensive exhaustive search
that would otherwise be needed to prove the same thing. `certified=false` proves nothing
(the relaxation may simply be loose); what it does carry is `candidates`, the improving
columns the exhaustive sub-searches behind a refutation found, which the loop materializes
as that iteration's pricing result.

The result must have `certified::Bool`, `improving_found::Bool` and `candidates`; anything
else on it is the pricer's own diagnostics. The defaults make the mode unavailable:
`cg_certification_supported` returns `false`, so the loop refuses
`pricing.mode=:relaxed_cluster` rather than silently ignoring it. Only
`AggregateODRouteJointRoutingAssignmentFormulation` implements the pair today
(`label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/certify.jl`),
and only when the solver's `CGPricingConfig` carried a `relaxed_cluster_count`.
"""
cg_certification_supported(build_result::BuildResult, mapping, m::JuMP.Model) = false

function cg_certification_round(build_result::BuildResult, mapping, m::JuMP.Model, duals,
        solver::CGSolver; time_limit_sec::Real, iteration::Int=0,
        only_scenarios::Union{Nothing, AbstractVector{Int}}=nothing)
    throw(MethodError(cg_certification_round, (build_result, mapping, m, duals, solver)))
end

"""
    _cg_materialize_certification_columns(build_result, mapping, m, duals, candidates)

Turn the candidates a failed certification attempt harvested into real columns, via the
same path a pricing round uses (`_materialize_pricing_columns` -- same id allocation, same
`_pricing_verify_column` cross-check against the master's own duals), so a harvested column
is indistinguishable from a priced one.

Empty in, empty out: a successful attempt drops its harvest (CG is about to stop), so this
is a no-op unless the attempt actually refuted something. Generic over formulations rather
than dispatched, because the candidates already
carry the search context that knows how to build their columns.
"""
function _cg_materialize_certification_columns(
    build_result::BuildResult, mapping, m::JuMP.Model, duals, candidates,
)
    isempty(candidates) && return Any[]
    return _materialize_pricing_columns(
        m[:aggregate_od_route_formulation], mapping, m, duals, candidates,
    )
end

"""
    _cg_pricing_exhausted(m) -> Bool

Did the last pricing round prove no negative-reduced-cost column remains? Formulations
record this on the model as `:label_setting_pricing_exhausted`; a model that never sets
it is treated as exhausted, preserving the pre-two-tier behaviour for pricers with no
notion of a time limit.
"""
function _cg_pricing_exhausted(m::JuMP.Model)
    key = :label_setting_pricing_exhausted
    return !haskey(JuMP.object_dictionary(m), key) || Bool(m[key])
end

function add_columns!(build_result::BuildResult, mapping, m::JuMP.Model, columns)::Int
    throw(MethodError(add_columns!, (build_result, mapping, m, columns)))
end

function integer_recovery_build(build_result::BuildResult, mapping, m::JuMP.Model)::BuildResult
    throw(MethodError(integer_recovery_build, (build_result, mapping, m)))
end
