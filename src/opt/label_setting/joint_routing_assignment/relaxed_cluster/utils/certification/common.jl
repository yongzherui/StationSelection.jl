"""
The three pieces of plumbing both certification loops need, factored out of the point
where they were copies of each other.

`certify.jl`'s one-tier loop and `two_tier/loop.jl`'s two-tier loop are different SEARCH
strategies over the same machinery: both read the same five encoding parameters off the
model, both dedupe their harvest against the same scenario column pool, and both end every
round in the same "exact-price this station subset -- refute it, or prove it barren"
search. Those three had drifted into near-copies, which is the dangerous kind of
duplication here: the station search in particular is where soundness lives (only an
EXHAUSTED search may be cut on), so two versions of it are two places to get that wrong.
"""

"""
    _certification_shared_params(m) -> NamedTuple

The route-encoding parameters every pricing-data constructor in this subtree takes,
splatted by the caller as `shared...`.

Read off the model rather than off the formulation because that is where `build_model`
stashes them, and read once per scenario attempt rather than per round because they are
build-time constants -- the same reason `relaxed_cluster_count` is a build-time input.
"""
_certification_shared_params(m::JuMP.Model) = (
    route_regularization_weight =
        Float64(m[:joint_routing_assignment_route_regularization_weight]),
    max_wait_time = Float64(m[:joint_routing_assignment_max_wait_time]),
    repositioning_time = Float64(m[:joint_routing_assignment_repositioning_time]),
    max_stops = Int(m[:joint_routing_assignment_max_stops]),
    compensated_dominance = Bool(m[:joint_routing_assignment_compensated_dominance]),
)

"""
    _certification_existing_columns(m, s) -> Vector{JointRoutingAssignmentRouteColumn}

The master's current columns for scenario `s`, which are what a harvest is deduped
against: a candidate only counts as novel if it beats the best `tau` already in the pool
for its signature, exactly as `_prepare_pricing_scenario` requires of a pricing round.

Materialized once per scenario attempt and reused across its rounds. Each round builds its
own search context (a different station subset means different signatures), so the
signature map cannot be shared, but the column list can.
"""
_certification_existing_columns(m::JuMP.Model, s::Int) =
    JointRoutingAssignmentRouteColumn[
        c for c in values(m[:joint_routing_assignment_columns])
        if Int(get(c.metadata, "scenario", 0)) == s
    ]

"""
    _certification_station_search(s, station_set, travel_cost, candidates, shared,
        existing_columns, harvested, solver; time_limit, reduced_cost_tol)
            -> (; outcome, rc, exhausted, elapsed_sec)

Exact-price the real route universe over `station_set`, harvesting into `harvested` as it
goes. This is step 4 of the one-tier loop and the station tier of the two-tier loop, and it
is the only place either of them touches the real pricer.

It has two jobs at once, which is why it is shaped the way it is:

  * **refute** -- return `rc < -reduced_cost_tol`, meaning the relaxation pointed at a
    support that really does hold an improving column, and keep that column. The labels
    are scored through `_pricing_accept_closure` (`../../../../round.jl`), so a survivor is
    deduped against the pool and against earlier rounds exactly as a pricing round's
    phase 2 would do it, and is indistinguishable from a priced column downstream.
  * **prove barren** -- return `rc >= -reduced_cost_tol` with `exhausted`, which is what
    licenses a cut.

`n_candidates` is deliberately unbounded (`typemax(Int) ÷ 2`): an accept closure that
stopped the search early would truncate it, and **a truncated search proves nothing**. Only
`exhausted == true` may be cut on -- at either layer, in either mode. That rule is the
difference between a certificate and a false certificate, and keeping one implementation
of this search is how both modes stay on the right side of it.

`outcome` distinguishes the three ways this can return without having searched:

  * `:searched` -- a real search ran; `exhausted` says whether it finished its frontier.
  * `:vacuous` -- no reward-carrying candidate, or no opportunity, exists over these
    stations at all. Barren, and provably so, without a search: `rc = Inf`,
    `exhausted = true`.
  * `:no_time` -- `time_limit` was non-positive, so nothing ran and nothing is known. The
    caller decides what that means for its loop; it must NOT be read as barren.
"""
function _certification_station_search(
    s::Int, station_set::Vector{Int}, travel_cost, candidates, shared::NamedTuple,
    existing_columns, harvested::Dict{Any, Any}, solver::CGSolver;
    time_limit::Float64, reduced_cost_tol::Float64,
)
    subset_candidates = _restrict_candidates_to_subset(candidates, station_set)
    isempty(subset_candidates) &&
        return (outcome=:vacuous, rc=Inf, exhausted=true, elapsed_sec=0.0)
    pricing_data = create_joint_routing_assignment_pricing_data(
        s, station_set, travel_cost, subset_candidates; shared...,
    )
    isempty(pricing_data.opportunities) &&
        return (outcome=:vacuous, rc=Inf, exhausted=true, elapsed_sec=0.0)
    time_limit > 0 ||
        return (outcome=:no_time, rc=Inf, exhausted=false, elapsed_sec=0.0)

    ctx = JointRoutingAssignmentSearchContext(pricing_data)
    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        sig = _pricing_pool_signature(ctx, column)
        best_pool_tau[sig] = min(get(best_pool_tau, sig, Inf), column.tau)
    end
    accept! = _pricing_accept_closure(
        ctx, s, best_pool_tau, harvested, solver, typemax(Int) ÷ 2,
    )
    t0 = time()
    labels, exhausted, _ = _run_label_setting(
        ctx; time_limit=time_limit, reduced_cost_tol=reduced_cost_tol, stop_if=accept!,
    )
    return (
        outcome = :searched,
        rc = isempty(labels) ? Inf : minimum(l.reduced_cost for l in labels),
        exhausted = exhausted,
        elapsed_sec = time() - t0,
    )
end

"""
    _certification_top_guides(improving, n_guides) -> Vector

The best `n_guides` improving relaxed routes, lowest reduced cost first, ties broken by
route length.

Both loops select guides identically -- lowest reduced cost, shortest route among equals --
and both then union the supports of the prefix. What they do with that support differs (the
two-tier loop keeps the supports PER GUIDE so alignment can shrink the prefix), so only the
selection itself is shared.

`sort!` mutates `improving` in place, which is what both callers want: they hold the only
reference and read `first(improving)` afterwards as the round's best relaxed reduced cost.
"""
function _certification_top_guides(improving::AbstractVector, n_guides::Int)
    sort!(improving; by = l -> (l.reduced_cost, length(l.route)))
    return improving[1:min(n_guides, length(improving))]
end
