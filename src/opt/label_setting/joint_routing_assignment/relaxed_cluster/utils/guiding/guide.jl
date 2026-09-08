"""
Using the relaxation as a **guide** rather than a certificate: price the cluster
graph, read the winning clusters back as a *station subset*, and run the real
exact pricer restricted to that subset.

This inverts the role the relaxation plays in `../certification/certify.jl`, and it is the role
that survives the measurement in
`benchmarks/diagnostics/relaxed_cluster_certification_probe.jl` (0/31
certifications at every K < n, because a converged master's minimum reduced
cost is exactly 0 and any relaxation slack overshoots it).

# The idea

If the cluster route minimizing the relaxed reduced cost is the image of the
real route minimizing the exact one -- which is what the relaxation is *for* --
then the stations of the real optimum all live inside the clusters the relaxed
optimum visited. So:

    relaxed search over K cluster nodes        (cheap)
      -> winning cluster route(s)
      -> S = union of those clusters' members  (a station subset)
      -> EXACT search over S only              (cheap, and produces real routes)

# Why this is sound where certification was not

Everything the second stage returns is a genuine route over genuine stations,
priced against the genuine reward structure, so its reduced cost is real and
the master can accept it unchanged -- `_pricing_verify_column` still
cross-checks it against the master's own duals like any other column. Nothing
here depends on the relaxation being tight; a bad guide costs column *quality*,
never correctness.

The price is that restricting to `S` is a **restriction of the route universe**,
exactly like `station_simple`'s elementary-route restriction: exhausting the
subset proves nothing about the stations left out, so a run finishing in this
mode cannot certify. It is handled the same way -- the run still reports
`SOLVE_OPTIMAL` when pricing exhausts, with `cg_optimality_scope` recording the
narrower claim, and `CGSolver.pricing.warm_start_mode` is the way to get a real
certificate (harvest here cheaply, hand off to the full pricer, which certifies).

# The one case where it certifies for free

If the relaxed search **exhausts** without finding any cluster route below
`-tol`, then by the bound (`../../relaxation.jl`) no real route is below `-tol` either --
so this scenario genuinely has nothing to price, and returning `nothing` is
correct rather than merely convenient. That is the certification path from
`../certification/certify.jl`, obtained here as a by-product. It just never fires in practice
(see the probe).

# What actually has to be true for this to pay

Two things, and both are measurable rather than assumed:

  1. **The guide has to be right** -- the exact optimum's stations must lie in
     the extracted subset often enough. If they don't, the subset search returns
     a worse column than the full search would, and CG needs more iterations.
  2. **The subset has to be small** -- `|S|` is bounded by
     `max_stops x (largest cell size)`, so on an instance where that product
     exceeds `n` there is no restriction at all and no saving. This wants MANY
     small clusters, i.e. large `K` -- the opposite of what certification wanted.

`m[:relaxed_cluster_guide_stats]` records `|S|` per scenario per round so both
can be read off a real run.
"""

export relaxed_cluster_station_subset

"""
    _relaxed_cluster_guide_routes(pricing_data, n_routes, time_limit; reduced_cost_tol)
        -> (routes, exhausted)

Run the relaxed search and return the `n_routes` best improving cluster routes,
cheapest reduced cost first, plus whether the search exhausted.

`reduced_cost_tol` defaults to `CGSolver`'s own default rather than being
threaded through: this stage only has to *rank* cluster routes to pick a
subset, and the real accept/reject test happens downstream in
`_pricing_accept_closure` against the solver's actual tolerance. A slightly
different threshold here changes which subset is guessed, never which columns
are admitted.
"""
function _relaxed_cluster_guide_routes(
    pricing_data::RelaxedClusterPricingData, n_routes::Int, time_limit::Float64;
    reduced_cost_tol::Float64=1e-6,
)
    labels, exhausted, _stats = _run_label_setting(
        JointRoutingAssignmentSearchContext(pricing_data.inner);
        time_limit=time_limit, reduced_cost_tol=reduced_cost_tol,
    )
    improving = filter(l -> l.reduced_cost < -reduced_cost_tol, labels)
    sort!(improving; by=l -> (l.reduced_cost, length(l.route)))
    kept = improving[1:min(n_routes, length(improving))]
    # A relaxed route can step through intra-cluster SERVICE nodes, which are not clusters.
    # Map them back to the cluster they serve, and drop the consecutive duplicate that
    # `C -> C'` then becomes, so the caller sees a plain cluster sequence.
    node_clusters = _relaxed_cluster_node_clusters(pricing_data)
    routes = Vector{Int}[]
    for label in kept
        clusters = Int[]
        for node in label.route
            cluster = node_clusters[node]
            (isempty(clusters) || clusters[end] != cluster) && push!(clusters, cluster)
        end
        push!(routes, clusters)
    end
    return routes, exhausted
end

"""
    relaxed_cluster_station_subset(clustering, cluster_routes) -> Vector{Int}

Every station belonging to any cluster visited by any of `cluster_routes`,
sorted ascending. `cluster_routes` must already be in *cluster* indices --
`_relaxed_cluster_guide_routes` maps the relaxed graph's service nodes back to
the clusters they serve before returning, since a raw relaxed route can step
through them.

The union over several routes rather than just the best one is deliberate: the
relaxed optimum is a guess, and one extra cluster route typically adds only a
cell or two while giving the exact search a materially better chance of
containing the real optimum. How many routes contribute is
`CGSolver.pricing.relaxed_cluster_guide_routes`.
"""
function relaxed_cluster_station_subset(
    clustering::StationClustering, cluster_routes::AbstractVector{<:AbstractVector{Int}},
)::Vector{Int}
    stations = Set{Int}()
    for route in cluster_routes, cluster in route
        (1 <= cluster <= clustering.n_clusters) || throw(ArgumentError(
            "cluster index $cluster is outside the partition's 1:$(clustering.n_clusters)",
        ))
        union!(stations, clustering.members[cluster])
    end
    return sort!(collect(stations))
end

"""
    _restrict_candidates_to_subset(candidates, subset) -> Vector{PassengerAssignmentCandidate}

The candidates whose pickup AND dropoff both lie in `subset` -- the only ones a
route confined to `subset` could ever certify.

Rewards are carried through untouched, which is what keeps the second stage
*exact*: a route over `subset` gets precisely the reduced cost it would get in
the unrestricted pricing problem, so the column needs no re-pricing and
`_pricing_verify_column` agrees with the master.
"""
function _restrict_candidates_to_subset(
    candidates::AbstractVector{PassengerAssignmentCandidate}, subset::AbstractVector{Int},
)::Vector{PassengerAssignmentCandidate}
    allowed = Set(subset)
    return filter(c -> c.origin in allowed && c.destination in allowed, candidates)
end

"""
Build the warm-start-only `:cluster_guide` context. The relaxed guide receives half of the
scenario's pricing slice; the returned elapsed time is deducted from the exact subset
search, so both stages share the ordinary pricing-round budget.
"""
function _build_cluster_guide_context(
    m::JuMP.Model, s::Int, candidates::AbstractVector{PassengerAssignmentCandidate},
    clustering::StationClustering, time_limit::Float64,
)
    shared = (
        route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
        max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
        repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
        max_stops=Int(m[:joint_routing_assignment_max_stops]),
        compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
    )
    relaxed = create_joint_routing_assignment_relaxed_cluster_pricing_data(
        s, clustering, m[:joint_routing_assignment_travel_cost], candidates; shared...,
    )
    isempty(relaxed.inner.opportunities) && return nothing

    started = time()
    cluster_routes, relaxed_exhausted = _relaxed_cluster_guide_routes(
        relaxed, Int(m[:joint_routing_assignment_relaxed_cluster_guide_routes]),
        0.5 * time_limit,
    )
    guide_elapsed = time() - started
    all_nodes = m[:joint_routing_assignment_nodes]
    if isempty(cluster_routes)
        _record_relaxed_cluster_stat!(m, (
            scenario=s, guide_routes=0,
            subset_size=relaxed_exhausted ? 0 : length(all_nodes),
            n_stations=length(all_nodes), relaxed_exhausted=relaxed_exhausted,
            warm_start=true, thread_id=Threads.threadid(),
        ))
        relaxed_exhausted && return nothing
        # Preserve the timeout signal without spending the subset stage on an unrestricted
        # exact fallback. A zero remaining budget makes the ordinary engine report
        # non-exhaustion, so the warm-start phase retries rather than handing off falsely.
        full = create_joint_routing_assignment_pricing_data(
            s, all_nodes, m[:joint_routing_assignment_travel_cost], candidates; shared...,
        )
        return JointRoutingAssignmentSearchContext(full), time_limit
    end

    subset = relaxed_cluster_station_subset(clustering, cluster_routes)
    subset_candidates = _restrict_candidates_to_subset(candidates, subset)
    _record_relaxed_cluster_stat!(m, (
        scenario=s, guide_routes=length(cluster_routes), subset_size=length(subset),
        n_stations=length(all_nodes), relaxed_exhausted=relaxed_exhausted,
        warm_start=true, thread_id=Threads.threadid(),
    ))
    isempty(subset_candidates) && return nothing
    pricing = create_joint_routing_assignment_pricing_data(
        s, subset, m[:joint_routing_assignment_travel_cost], subset_candidates; shared...,
    )
    isempty(pricing.opportunities) && return nothing
    return JointRoutingAssignmentSearchContext(pricing), guide_elapsed
end

"""
    _record_relaxed_cluster_stat!(m, stat)

Append one `(scenario, ...)` row to `m[:relaxed_cluster_guide_stats]`, which
surfaces on the result as `metadata["cg_relaxed_cluster_guide_stats"]`.

Both the `:cluster_guide` warm start and the no-good pricing loop write rows here.
`warm_start=true` identifies the former; `nogood_outcome` identifies the latter.

The key keeps the `guide` name it was introduced with because it is part of
`OptResult.metadata` and therefore of every study's `metrics.json`; renaming it
would silently break analysis scripts reading historical runs.

Certification is threaded over scenarios, so this takes the model's lock: `push!` onto a
shared `Vector` from several threads is a data race.
"""
function _record_relaxed_cluster_stat!(m::JuMP.Model, stat)
    haskey(m.obj_dict, :relaxed_cluster_guide_stats) || return nothing
    lock(m[:relaxed_cluster_guide_lock]) do
        push!(m[:relaxed_cluster_guide_stats], stat)
    end
    return nothing
end
