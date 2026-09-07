"""
Witness-guided cluster refinement: using a spurious relaxed route to decide which cell of
the partition manufactured it, and splitting exactly that cell.

This is counterexample-guided abstraction refinement. A **barren** round of the no-good
loop (`../certification/certify.jl`) hands over a spurious counterexample: a cluster
route that priced below `-tol` while an exhaustive exact search over `stations(T)` found
nothing improving. The relaxation lied, and this file works out where.

# Why refinement, rather than a different K

`../../clustering.jl` records that tightness is **not** monotone in `n_clusters`: two independent
k-medoids runs at different K need not be nested, so a larger K can give a looser bound.
It *is* monotone under **refinement** -- if every cell of `P'` sits inside a cell of `P`,
then `P'` takes its minima over smaller sets and its maxima over smaller sets, so every
quantity the relaxation computes moves toward reality. Splitting one cell therefore gives
the monotone ladder a K sweep structurally cannot, which is the whole reason to refine
rather than re-cluster.

There is no soundness exposure either way: the bound of `../../relaxation.jl` holds for **any**
partition, so refinement can only change tightness, never validity. And cuts are discarded
between CG iterations (they are valid only at the duals they were derived under), so
refining between attempts cannot invalidate a live cut.

# The witness

`rho_bar(p, C, D) = max{rho(p, j, k)}` lets **every passenger independently pick its own
best station** inside a cell. That was MEASURED to be the dominant slack source, ahead of
travel optimism. So the fiction is concrete and nameable: when two passengers credited at
one cluster visit require *different* stations of that cluster -- or one passenger requires
different stations at pickup and dropoff -- the cell is standing in for two places at once,
and no real route could have done what its image did.

`../../data.jl` records which real `(j, k)` achieved each `rho_bar` maximum
(`RelaxedClusterPricingData.reward_witness`), keyed by the routed node ids. Replaying the
spurious route through `../../../exact/accept.jl`'s replay -- which works unchanged, since the
relaxed `inner` is an ordinary `JointRoutingAssignmentPricingData` -- says which `(p, C, D)`
was credited. Composing the two gives, per cluster, the set of stations the relaxation
pretended it was.

# When NOT to split, which matters as much

If every cluster on the route shows exactly **one** witness station, the slack did not come
from `rho_bar` aggregation at all -- it came from travel optimism, or from the ride-limit
maximum being taken independently of the reward one. Splitting then cannot tighten what
went wrong, and `relaxed_cluster_split_candidates` returns empty rather than splitting
something arbitrary. Callers should count that outcome: if it is common, the witness
premise is wrong for this instance family and refinement is the wrong lever, which is a
finding rather than a failure.
"""

export relaxed_cluster_split_candidates, refine_station_clustering

"""
    rewrite_cut_sets_for_split(cluster_sets, split_cluster, new_cluster)

Carry no-good cuts across a refinement instead of discarding them.

A cut is a set of cluster INDICES, and splitting cell `c` into `c` and `c_new` renumbers
what those indices denote -- so a cut mentioning `c` must gain `c_new` to keep meaning the
same set of STATIONS. With that rewrite `stations(T)` is unchanged by the split, and since
barrenness is a statement about `stations(T)` at the current duals, a `T` already proven
barren stays proven. Cuts on cells the split did not touch are unaffected.

This matters more than it looks. Clearing the cuts on every split (the first
implementation) threw away all progress toward the certificate each time the partition
improved, so refinement tightened the bound and reset the search in the same move --
plausibly why splitting was measured to fire repeatedly without helping convergence.
"""
function rewrite_cut_sets_for_split(
    cluster_sets::Vector{Set{Int}}, split_cluster::Int, new_cluster::Int,
)
    for cut in cluster_sets
        split_cluster in cut && push!(cut, new_cluster)
    end
    return cluster_sets
end

"""
    RelaxedClusterSplitCandidate(cluster, witness_stations, reward_mass)

One cell the relaxation was caught impersonating several stations at once.

- `cluster` -- the offending cell, as a cluster index into the partition;
- `witness_stations` -- the distinct real stations `rho_bar` credited there on this route.
  Always at least two, or this would not be a candidate;
- `reward_mass` -- the reward credited through that cell, which is what ranks candidates.
  A cell manufacturing one large fictional reward matters more than one manufacturing
  three trivial ones, so this ranks better than the raw witness count.
"""
struct RelaxedClusterSplitCandidate
    cluster::Int
    witness_stations::Vector{Int}
    reward_mass::Float64
end

"""
    relaxed_cluster_split_candidates(data, route) -> Vector{RelaxedClusterSplitCandidate}

Census of the cells a spurious relaxed `route` implicates, worst first.

Empty means **do not split**: either the route certified nothing, or every cell it used
agreed on a single station, in which case the slack is not `rho_bar`'s doing (see the
module docstring).

`reward_mass` counts a passenger's reward against BOTH endpoint cells, since the credit
required the fiction at whichever end disagreed and there is no principled way to split
the blame between them. An INTRA-cluster credit therefore counts twice against its single
cell -- deliberate, since a cell impersonating two stations for one passenger's pickup and
dropoff is the strongest evidence of aggregation there is. That biases toward cells appearing at both ends of a route, which
is the right bias -- those are the cells doing the most aggregating.
"""
function relaxed_cluster_split_candidates(
    data::RelaxedClusterPricingData, route::Vector{Int},
)::Vector{RelaxedClusterSplitCandidate}
    isempty(route) && return RelaxedClusterSplitCandidate[]
    replay = _replay_joint_routing_assignment_route(route, data.inner)
    isempty(replay) && return RelaxedClusterSplitCandidate[]

    node_clusters = _relaxed_cluster_node_clusters(data)
    stations = Dict{Int, Set{Int}}()
    mass = Dict{Int, Float64}()
    for (p, entry) in replay
        origin_node, dest_node, reward = entry[1], entry[2], entry[3]
        witness = get(data.reward_witness, (p, origin_node, dest_node), nothing)
        isnothing(witness) && continue
        for (node, station) in ((origin_node, witness[1]), (dest_node, witness[2]))
            cluster = node_clusters[node]
            push!(get!(() -> Set{Int}(), stations, cluster), station)
            mass[cluster] = get(mass, cluster, 0.0) + reward
        end
    end

    out = RelaxedClusterSplitCandidate[]
    for (cluster, witnessed) in stations
        length(witnessed) >= 2 || continue      # agreed on one station: nothing to split
        push!(out, RelaxedClusterSplitCandidate(
            cluster, sort!(collect(witnessed)), get(mass, cluster, 0.0),
        ))
    end
    # Ties broken on the cluster index so the choice is reproducible across runs.
    sort!(out; by = c -> (-c.reward_mass, c.cluster))
    return out
end

"""
    refine_station_clustering(clustering, candidate, travel_cost) -> StationClustering

Split `candidate.cluster` in two, seeded on the two witness stations that are farthest
apart, with the cell's remaining members going to whichever seed is nearer.

The result is a genuine **refinement**: every new cell sits inside an old one and no other
cell moves, which is what makes the relaxation's bound monotonically tighter
(`../../relaxation.jl`). The split cell keeps its index and the new half is appended, so existing
cluster indices stay valid -- callers holding cut sets built on the old partition can
rewrite them by replacing the split index with itself plus the new one.

Distances are the same symmetrized travel costs `../../clustering.jl` clusters on, so a refined
partition is the same kind of object k-medoids would have produced. A cell that cannot be
split -- fewer than two members, or witnesses that are not actually in it -- is returned
unchanged rather than raising, since a stale candidate is a normal race with an
already-refined partition, not a bug.
"""
function refine_station_clustering(
    clustering::StationClustering,
    candidate::RelaxedClusterSplitCandidate,
    travel_cost::Dict{Tuple{Int, Int}, Float64},
)::StationClustering
    c = candidate.cluster
    (1 <= c <= clustering.n_clusters) || return clustering
    members = clustering.members[c]
    length(members) >= 2 || return clustering
    seeds = [s for s in candidate.witness_stations if s in members]
    length(seeds) >= 2 || return clustering

    _d(a, b) = begin
        f = get(travel_cost, (a, b), Inf)
        r = get(travel_cost, (b, a), Inf)
        isfinite(f) && isfinite(r) ? 0.5 * (f + r) : (isfinite(f) ? f : r)
    end
    # The two witnesses farthest apart: splitting between the extremes separates as much of
    # the disagreement as one cut can.
    a, b, best = seeds[1], seeds[2], -Inf
    for i in eachindex(seeds), j in (i + 1):length(seeds)
        d = _d(seeds[i], seeds[j])
        if isfinite(d) && d > best
            a, b, best = seeds[i], seeds[j], d
        end
    end

    left, right = Int[], Int[]
    for station in members
        station == a && (push!(left, station); continue)
        station == b && (push!(right, station); continue)
        da, db = _d(station, a), _d(station, b)
        # A non-finite `da` short-circuits the whole condition, so such members fall to
        # `right` (the second seed). Which side they land on does not matter; what matters
        # is that every member lands on exactly one, so none is ever dropped.
        (isfinite(da) && (!isfinite(db) || da <= db)) ? push!(left, station) : push!(right, station)
    end
    (isempty(left) || isempty(right)) && return clustering

    members_new = copy(clustering.members)
    members_new[c] = sort!(left)
    push!(members_new, sort!(right))
    medoids_new = copy(clustering.medoids)
    medoids_new[c] = a
    push!(medoids_new, b)
    cluster_of = copy(clustering.cluster_of)
    for station in members_new[c]
        cluster_of[station] = c
    end
    for station in members_new[end]
        cluster_of[station] = length(members_new)
    end
    return StationClustering(
        length(members_new), clustering.nodes, cluster_of, members_new, medoids_new,
    )
end

"""
    _relaxed_cluster_scenario_clustering(m, s) -> StationClustering

The partition scenario `s` is currently pricing against: its own refined one when
refinement is enabled, otherwise the single build-time partition every scenario shares.
"""
function _relaxed_cluster_scenario_clustering(m::JuMP.Model, s::Int)::StationClustering
    key = :joint_routing_assignment_scenario_clusterings
    haskey(m.obj_dict, key) || return _joint_routing_assignment_station_clustering(m)
    return m[key][s]
end

"""
    _relaxed_cluster_refine!(m, s, data, route, travel_cost)
        -> Union{Nothing, Tuple{StationClustering, Int}}

Fold one barren round's witness census into scenario `s`'s refinement state, and split if
the trigger fires. Returns `(refined partition, index of the cell that was split)`, or
`nothing` when nothing was split. The caller needs the split index to rewrite its cut sets
(`rewrite_cut_sets_for_split`) rather than discard them.

The trigger is a conjunction, and each clause exists for a different reason:

  1. **the route implicates a cell at all** -- `relaxed_cluster_split_candidates` empty
     means the slack was not `rho_bar`'s doing, so splitting cannot tighten it;
  2. **that cell has been implicated `recurrence` times for this scenario** -- a cut
     already neutralises the specific barren support, so splitting on first sight chases a
     symptom rather than the cause;
  3. **the scenario is below `relaxed_cluster_max_count`** -- refinement makes the relaxed
     graph bigger, and the exact pricer's cost is super-linear in node count, so an
     unbounded ladder ends up costing what it was meant to save.

Safe to call from the threaded certification round: every read and write is confined to
scenario `s`'s own index (`_pricing_scenarios` yields `1:n_scenarios`, so the scenario id
is the index), so concurrent scenarios never touch the same slot.
"""
function _relaxed_cluster_refine!(
    m::JuMP.Model, s::Int, data::RelaxedClusterPricingData, route::Vector{Int},
    travel_cost::Dict{Tuple{Int, Int}, Float64},
)
    key = :joint_routing_assignment_scenario_clusterings
    haskey(m.obj_dict, key) || return nothing
    max_count = m[:joint_routing_assignment_relaxed_cluster_max_count]
    isnothing(max_count) && return nothing

    # Why a barren round did or did not split. This is the measurement that decides the
    # recurrence threshold: `census_empty` counts rounds where refinement provably CANNOT
    # help (no cell disagreed, so the slack was travel optimism or the ride-limit max, not
    # `rho_bar`), while `blocked_recurrence` counts rounds a threshold of 1 would have
    # split. If `census_empty` dominates, the witness premise is wrong for this instance
    # family and no threshold saves it.
    stats = m[:joint_routing_assignment_scenario_refine_stats][s]
    _bump!(k) = (stats[k] = get(stats, k, 0) + 1)
    _bump!(:barren_rounds)

    clustering = m[key][s]
    if clustering.n_clusters >= max_count
        _bump!(:blocked_ceiling)
        return nothing
    end

    candidates = relaxed_cluster_split_candidates(data, route)
    if isempty(candidates)
        _bump!(:census_empty)
        return nothing
    end
    _bump!(:census_nonempty)

    counts = m[:joint_routing_assignment_scenario_disagreements][s]
    recurrence = m[:joint_routing_assignment_relaxed_cluster_refine_recurrence]
    for candidate in candidates
        counts[candidate.cluster] = get(counts, candidate.cluster, 0) + 1
    end
    # Candidates arrive worst-first by reward mass, so the first one that has recurred
    # enough is also the heaviest such -- no second sort needed.
    idx = findfirst(c -> counts[c.cluster] >= recurrence, candidates)
    if isnothing(idx)
        _bump!(:blocked_recurrence)
        return nothing
    end
    chosen = candidates[idx]

    refined = refine_station_clustering(clustering, chosen, travel_cost)
    if refined === clustering
        _bump!(:split_stale)                                  # stale candidate: no-op
        return nothing
    end
    m[key][s] = refined
    m[:joint_routing_assignment_scenario_splits][s] += 1
    _bump!(:split)
    # The split cell keeps its index and the new half is appended, so counts keyed on other
    # cells stay meaningful; only the split cell's own tally is retired.
    delete!(counts, chosen.cluster)
    return refined, chosen.cluster
end
