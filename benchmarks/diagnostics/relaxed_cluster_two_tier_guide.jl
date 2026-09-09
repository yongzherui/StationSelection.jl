"""Does a TELESCOPING two-tier guide find better columns than the one-tier guide?

The one-tier guide (`relaxed_cluster/utils/guiding/guide.jl`) searches one K-cell relaxed
graph, takes the best few cluster routes, unions their cells into a station subset `S`, and
exact-prices `S`. At n=40 that K-cell search does not always exhaust inside its slice, so
the guides are the best of a PARTIAL sweep.

The two-tier idea: nest a coarse macro layer `K1 < K2` under the meso layer, search the
cheap macro graph first, keep only the meso cells its improving routes touch, and search
that RESTRICTED meso graph. Both searches are then small enough to exhaust, so the guides
come from complete sweeps.

# Scope: COLUMN DISCOVERY ONLY

The restriction here is a heuristic and is measured as one. It is NOT valid for
certification: a no-good cut may only be added after an EXHAUSTED search, and restricting
the meso graph to the top few macro routes' support leaves the rest of the meso graph
unexamined -- cutting on that yields a false certificate (same failure as the invalid
`|route ∩ T| <= |T|-1` cut). Making it certification-safe needs the macro search to exhaust
AND the restriction to use the union over EVERY improving macro route; `cov_5k` below is
the measurement that says whether that stronger restriction would prune anything at all.

# Reported per scenario

  one-tier   guide_s g_exh |S| rc          -- the baseline, same duals
  two-tier   macro_s m_exh n_imp cov_5k cov_top |S| rc

`cov_top` is the meso-cell share the top-`guide_routes` macro support keeps (what discovery
would actually use); `cov_5k` is the share the union of up to 5000 improving macro routes
keeps -- a LOWER bound on the all-improving union, and therefore a lower bound on what a
certification-safe restriction would have to keep. A coverage near 1.0 means the macro
layer prunes nothing and telescoping cannot pay.

Usage: sbatch benchmarks/diagnostics/run_two_tier_guide.sh
Env: TT_N TT_P TT_S TT_SEED TT_K2 TT_K1 (comma list) TT_ITERS TT_GUIDE_LIMIT
     TT_SUBSET_LIMIT TT_CG_PRICING_LIMIT
"""

using StationSelection
using JuMP
using Printf

const SS = StationSelection

n_stations = parse(Int, get(ENV, "TT_N", "40"))
n_pairs = parse(Int, get(ENV, "TT_P", "16"))
n_scenarios = parse(Int, get(ENV, "TT_S", "3"))
seed = parse(Int, get(ENV, "TT_SEED", "48"))
k2 = parse(Int, get(ENV, "TT_K2", "24"))
k1_list = [parse(Int, x) for x in split(get(ENV, "TT_K1", "6,8,12"), ',')]
max_iterations = parse(Int, get(ENV, "TT_ITERS", "1"))
guide_limit = parse(Float64, get(ENV, "TT_GUIDE_LIMIT", "150.0"))
subset_limit = parse(Float64, get(ENV, "TT_SUBSET_LIMIT", "30.0"))
cg_pricing_limit = parse(Float64, get(ENV, "TT_CG_PRICING_LIMIT", "60.0"))
const COV_CAP = 5000   # bounds the cost of the all-improving union; see `cov_5k` above
# How many macro guide routes to union into the support. The whole ladder is measured from
# ONE macro sweep -- the improving routes are already sorted by reduced cost, so cov_top(g)
# is just a prefix of that list and costs nothing extra. Only the restricted meso sweep is
# re-run per g, and it is cheap exactly when coverage is low, which is the case of interest.
guide_ladder = [parse(Int, x) for x in split(get(ENV, "TT_GUIDE_LADDER", "1,2,3,5,8"), ',')]

include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))
problem, k, _meta = benchmark_problem(@__DIR__, "TT", n_stations, n_pairs, n_scenarios, seed)
@printf("instance: n=%d p=%d s=%d seed=%d (k=%d)\n", n_stations, n_pairs, n_scenarios, seed, k)
@printf("K2=%d | K1 ladder=%s | CG iterations=%d\n", k2, string(k1_list), max_iterations)
@printf("guide slice=%.0f s | subset limit=%.0f s | CG pricing limit=%.0f s\n\n",
        guide_limit, subset_limit, cg_pricing_limit)

"""Best reduced cost over a pricing graph, with its wall time and whether it exhausted."""
function best_rc(ctx, limit::Float64)
    t0 = time()
    labels, exhausted, _ = SS._run_label_setting(ctx; time_limit=limit, reduced_cost_tol=1e-6)
    secs = time() - t0
    isempty(labels) && return (Inf, secs, exhausted)
    return (minimum(l.reduced_cost for l in labels), secs, exhausted)
end

"""A NESTED coarsening of `meso` into `k1` macro cells.

Built by clustering the meso MEDOIDS and lifting: every meso cell joins the macro cell of
its own medoid, so each macro cell is exactly a union of meso cells and nesting holds by
construction. Medoid quality does not matter to the relaxation -- cluster travel costs are
minima over member pairs, not medoid-to-medoid distances -- so the medoid clustering is
only a device for choosing WHICH meso cells group together."""
function macro_layer(meso::StationClustering, k1::Int,
                     travel_cost::Dict{Tuple{Int, Int}, Float64})
    med = cluster_stations_by_travel_cost(meso.medoids, travel_cost, k1)
    macro_of_meso = [med.cluster_of[meso.medoids[c]] for c in 1:meso.n_clusters]
    members = [Int[] for _ in 1:med.n_clusters]
    for c in 1:meso.n_clusters
        append!(members[macro_of_meso[c]], meso.members[c])
    end
    foreach(sort!, members)
    cluster_of = Dict{Int, Int}()
    for (g, ms) in enumerate(members), st in ms
        cluster_of[st] = g
    end
    return StationClustering(med.n_clusters, copy(meso.nodes), cluster_of, members,
                             copy(med.medoids)), macro_of_meso
end

"""`meso` restricted to `cells`, renumbered 1:length(cells) over just their stations."""
function restrict_layer(meso::StationClustering, cells::Vector{Int})
    members = [copy(meso.members[c]) for c in cells]
    nodes = sort!(unique(reduce(vcat, members; init=Int[])))
    cluster_of = Dict{Int, Int}()
    for (i, ms) in enumerate(members), st in ms
        cluster_of[st] = i
    end
    return StationClustering(length(cells), nodes, cluster_of, members,
                             [meso.medoids[c] for c in cells])
end

cells_touched(routes) = (u = Set{Int}(); for r in routes, c in r; push!(u, c); end; u)

formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=10,
)
# The partition is stashed at build time but no mode reads it: CG prices with the ordinary
# exact pricer, so the duals below are the REAL ones, identical for every K1 in the ladder.
solver = CGSolver(
    config=SolverOptions(silent=true, time_limit_sec=300.0, threads=1),
    pricing=CGPricingConfig(relaxed_cluster_count=k2),
    max_iterations=max_iterations, reduced_cost_tol=1e-6,
    pricing_time_limit_sec=cg_pricing_limit, certifying_pricing_time_limit_sec=600.0,
    total_time_limit_sec=1800.0, parallel_scenario_pricing=true,
    recover_integer_solution=false,
)
result = run_opt(problem, formulation, solver)
m, mapping = result.model, result.mapping
meso = m[:joint_routing_assignment_station_clustering]
travel_cost = m[:joint_routing_assignment_travel_cost]
n_guides = Int(m[:joint_routing_assignment_relaxed_cluster_guide_routes])

build_result = BuildResult(m, mapping, nothing,
    ModelCounts(Dict{String,Int}(), Dict{String,Int}(), Dict{String,Int}()), Dict{String,Any}())
alpha, gamma_o, gamma_d = SS.extract_duals(build_result, mapping, m)
data = m[:joint_routing_assignment_data]
shared = (
    route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
    max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
    repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
    max_stops=Int(m[:joint_routing_assignment_max_stops]),
    compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
)
@printf("meso cell sizes (K2=%d): %s\n\n", meso.n_clusters,
        string(station_cluster_sizes(meso)))
@printf("%-9s %-7s %-3s %-4s %8s %-8s %7s %7s %7s %5s %8s %14s\n",
        "tier", "K1/K2", "sc", "g", "macro_s", "m_exh", "n_imp", "cov_5k", "cov_top", "|S|",
        "meso_s", "subset_rc")

for s in 1:length(mapping.scenarios)
    candidates = joint_routing_assignment_pricing_candidates(
        data, mapping, alpha, gamma_o, gamma_d,
        Float64(m[:joint_routing_assignment_walk_cost_weight]),
        Float64(m[:joint_routing_assignment_detour_factor]), s,
    )
    isempty(candidates) && continue

    # ---- baseline: one tier, the whole meso graph, exactly as production does it
    meso_relaxed = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
        s, meso, travel_cost, candidates; shared...,
    )
    t0 = time()
    base_routes, base_exh = SS._relaxed_cluster_guide_routes(meso_relaxed, n_guides, guide_limit)
    base_sec = time() - t0
    base_subset = isempty(base_routes) ? Int[] :
        relaxed_cluster_station_subset(meso, base_routes)
    base_rc = Inf
    if !isempty(base_subset)
        cands = SS._restrict_candidates_to_subset(candidates, base_subset)
        if !isempty(cands)
            pd = create_joint_routing_assignment_pricing_data(
                s, base_subset, travel_cost, cands; shared...,
            )
            isempty(pd.opportunities) ||
                ((base_rc, _bs, _be) = best_rc(SS.JointRoutingAssignmentSearchContext(pd), subset_limit))
        end
    end
    @printf("%-9s %-7s %-3d %-4s %8.1f %-8s %7s %7s %7s %5d %8s %14.4f\n",
            "one-tier", string(k2), s, string(n_guides), base_sec,
            base_exh ? "yes" : "TIMEOUT", "-", "-", "-", length(base_subset), "-", base_rc)
    flush(stdout)

    # ---- two tier: macro sweep, then the meso graph restricted to what it touched
    for k1 in k1_list
        macro_cl, macro_of_meso = macro_layer(meso, k1, travel_cost)
        macro_relaxed = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
            s, macro_cl, travel_cost, candidates; shared...,
        )
        t0 = time()
        macro_all, macro_exh = SS._relaxed_cluster_guide_routes(
            macro_relaxed, COV_CAP, 0.5 * guide_limit,
        )
        macro_sec = time() - t0
        cells_all = cells_touched(macro_all)
        cov_5k = count(c -> macro_of_meso[c] in cells_all, 1:meso.n_clusters) / meso.n_clusters
        for g in guide_ladder
        macro_top = macro_all[1:min(g, length(macro_all))]
        cells_top = cells_touched(macro_top)
        keep = [c for c in 1:meso.n_clusters if macro_of_meso[c] in cells_top]
        cov_top = length(keep) / meso.n_clusters

        subset2, rc2, meso_sec, meso_exh = Int[], Inf, 0.0, true
        if !isempty(keep)
            restricted = restrict_layer(meso, keep)
            cands_r = SS._restrict_candidates_to_subset(candidates, restricted.nodes)
            if !isempty(cands_r)
                r_relaxed = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
                    s, restricted, travel_cost, cands_r; shared...,
                )
                t1 = time()
                r_routes, meso_exh = SS._relaxed_cluster_guide_routes(
                    r_relaxed, n_guides, max(1.0, guide_limit - macro_sec),
                )
                meso_sec = time() - t1
                subset2 = isempty(r_routes) ? Int[] :
                    relaxed_cluster_station_subset(restricted, r_routes)
                if !isempty(subset2)
                    cands2 = SS._restrict_candidates_to_subset(candidates, subset2)
                    if !isempty(cands2)
                        pd2 = create_joint_routing_assignment_pricing_data(
                            s, subset2, travel_cost, cands2; shared...,
                        )
                        isempty(pd2.opportunities) ||
                            ((rc2, _s2, _e2) = best_rc(
                                SS.JointRoutingAssignmentSearchContext(pd2), subset_limit))
                    end
                end
            end
        end
        @printf("%-9s %-7s %-3d %-4d %8.1f %-8s %7d %7.2f %7.2f %5d %8.1f %14.4f%s\n",
                "two-tier", "$(k1)/$(k2)", s, g, macro_sec, macro_exh ? "yes" : "TIMEOUT",
                length(macro_all), cov_5k, cov_top, length(subset2), meso_sec, rc2,
                meso_exh ? "" : "  (meso TIMEOUT)")
        flush(stdout)
        end
    end
end
