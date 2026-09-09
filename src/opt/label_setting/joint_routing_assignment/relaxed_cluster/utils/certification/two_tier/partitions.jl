"""
The nesting machinery: how the macro layer is built out of the meso one, how a region of
the meso layer is cut down to a standalone graph, how cut sets move between the three index
spaces, and the alignment policy that decides which station set a round actually prices.

Everything here is pure -- no model, no solver, no clock. `loop.jl` is where they are
sequenced and where the budget lives.
"""


"""
    _nested_macro_clustering(meso, k1, travel_cost) -> (macro_clustering, parent)

A `k1`-cell coarsening of `meso` in which every macro cell is a union of meso cells, plus
`parent[c]` giving the macro cell of meso cell `c`.

Built by clustering `meso.medoids` and lifting, so nesting is structural rather than
verified. The returned clustering spans the same station set as `meso`, which is what lets
the ordinary relaxed-graph constructor consume it unchanged.
"""
function _nested_macro_clustering(
    meso::StationClustering, k1::Int, travel_cost::Dict{Tuple{Int, Int}, Float64},
)
    1 <= k1 || throw(ArgumentError("macro count must be >= 1, got $k1"))
    k1 < meso.n_clusters || throw(ArgumentError(
        "macro count ($k1) must be below the meso count ($(meso.n_clusters)); equal " *
        "means no coarsening, which the one-tier mode already expresses",
    ))
    med = cluster_stations_by_travel_cost(meso.medoids, travel_cost, k1)
    parent = [med.cluster_of[meso.medoids[c]] for c in 1:meso.n_clusters]
    members = [Int[] for _ in 1:med.n_clusters]
    for c in 1:meso.n_clusters
        append!(members[parent[c]], meso.members[c])
    end
    foreach(sort!, members)
    all(!isempty, members) || error(
        "internal: a macro cell came out empty, which cannot happen when every macro " *
        "cell owns at least the meso cell of its own medoid",
    )
    cluster_of = Dict{Int, Int}()
    for (g, ms) in enumerate(members), st in ms
        cluster_of[st] = g
    end
    return StationClustering(
        med.n_clusters, copy(meso.nodes), cluster_of, members, copy(med.medoids),
    ), parent
end

"""
    _two_tier_restrict(meso, cells) -> StationClustering

`meso` restricted to `cells` (global meso indices), renumbered `1:length(cells)` over just
those cells' stations. `cells` is the caller's ordering and IS the local-to-global map.
"""
function _two_tier_restrict(meso::StationClustering, cells::Vector{Int})
    isempty(cells) && throw(ArgumentError("cannot restrict to an empty cell set"))
    members = [copy(meso.members[c]) for c in cells]
    nodes = sort!(unique(reduce(vcat, members; init=Int[])))
    cluster_of = Dict{Int, Int}()
    for (i, ms) in enumerate(members), st in ms
        cluster_of[st] = i
    end
    return StationClustering(
        length(cells), nodes, cluster_of, members, [meso.medoids[c] for c in cells],
    )
end

"""
    _two_tier_local_cut_sets(global_sets, cells) -> Vector{Set{Int}}

Translate cut sets from global meso indices into the local numbering of the subgraph over
`cells`. A cut says "visit at least one cell OUTSIDE this set", so the translation keeps
only the members that exist locally: local cell `i` (global `cells[i]`) is inside the cut
iff its global index is.

A cut that swallows every local cell translates to the full local set, which no local route
can escape -- correctly, because every route in this subgraph then lies inside a set already
proved barren.
"""
function _two_tier_local_cut_sets(
    global_sets::AbstractVector{Set{Int}}, cells::Vector{Int},
)::Vector{Set{Int}}
    local_of = Dict(g => i for (i, g) in enumerate(cells))
    out = Vector{Set{Int}}()
    for gs in global_sets
        push!(out, Set{Int}(local_of[g] for g in gs if haskey(local_of, g)))
    end
    return out
end

"""
    _two_tier_aligned_support(guide_supports, parent, meso, max_stations)
        -> (cells, stations, macro_cells_or_nothing, guides_used)

Round a meso support up to whole macro cells when the resulting station set fits inside
`max_stations`, SHRINKING the guide set until it does.

`guide_supports[i]` is the meso-cell support (global indices) of the `i`-th guide route, in
rank order (best reduced cost first). The union of a *prefix* is tried, longest first: all
`g` guides, then `g-1`, down to the single best one. The first prefix whose aligned station
set fits is priced aligned, which licenses BOTH a meso cut and a macro cut.

This is the fix for alignment starvation. Alignment is what carries a barrenness proof up to
the macro layer, and the macro layer is what certifies (only an exhausted macro sweep ends
the attempt with a certificate). MEASURED at n=40, K2=32/K1=16: alignment was refused on 53%
of station searches for seed 43, 54% for seed 50 and 82% for seed 45 -- none of which
certify -- against 0%, 0% and 7% for seeds 44, 48 and 49, all of which do. Refusing alignment
was previously terminal for the round's macro cut; unioning five guide supports routinely
spans more than the ~6 macro cells that 15 stations allows, so the cap was refusing
alignment because of `guide_routes`, not because of anything about the instance.

Shrinking is sound: soundness never depended on which guides produced the support, only on
cutting exclusively on a support whose station search actually EXHAUSTED. A prefix is simply
a smaller support, and `stations(pi^-1(pi(T))) == stations(pi(T))` holds for it identically.

`nothing` in the third slot means even the single best guide's aligned set overran the cap,
so the full support was priced unaligned: still a valid meso cut, no macro cut. `guides_used`
is the prefix length actually priced (`0` in the unaligned case), for instrumentation.
"""
function _two_tier_aligned_support(
    guide_supports::AbstractVector{Set{Int}}, parent::Vector{Int},
    meso::StationClustering, max_stations::Int,
)
    isempty(guide_supports) && throw(ArgumentError("need at least one guide support"))
    for n in length(guide_supports):-1:1
        support = Set{Int}()
        for i in 1:n
            union!(support, guide_supports[i])
        end
        macro_cells = Set{Int}(parent[c] for c in support)
        aligned = [c for c in 1:meso.n_clusters if parent[c] in macro_cells]
        aligned_stations = relaxed_cluster_station_subset(meso, [aligned])
        if length(aligned_stations) <= max_stations
            return aligned, aligned_stations, macro_cells, n
        end
    end
    full = Set{Int}()
    for gs in guide_supports
        union!(full, gs)
    end
    plain = sort!(collect(full))
    return plain, relaxed_cluster_station_subset(meso, [plain]), nothing, 0
end
