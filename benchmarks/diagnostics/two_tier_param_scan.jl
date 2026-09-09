"""Which (K2, K1, g) makes two-tier certification cheap? One short job, one ranked table.

The 6-hour CG runs are the wrong instrument for parameter selection, and the n=40 evidence
says why: round cost is almost BINARY. Seeds whose certification rounds ran 2-6 s certified
in about two minutes; seeds whose rounds hit the 300 s cap never certified at all. So
predicting the cost of ONE round predicts the outcome, and one round can be measured in
seconds instead of hours.

# What a round costs

    macro sweep   over K1 cluster nodes                     -> a macro support U
    meso sweep    over the K2 cells inside U (a `cov_top` fraction of them)
    station search over the stations of the meso support    -> a cut, or a real column

This scans the grid on ONE dual vector -- CG is run once with the ordinary exact pricer, so
the duals are real and IDENTICAL for every configuration, which is what makes the
comparison fair. K2 is swept WITHOUT rebuilding the model: the clustering is derived from
the travel matrix, and the duals do not depend on it.

`cov_top(g)` is free for the whole g ladder: the improving macro routes come back sorted, so
each g is a prefix of one sweep. Only the meso sweep and station search are re-run per g.

# Reading the output

`round_s` = macro_s + meso_s + station_s is the predictor; rank by it. Any `TIMEOUT` means
that component did not exhaust inside its (deliberately tight) cap, which disqualifies the
configuration rather than merely slowing it -- an unexhausted search proves nothing and
cannot cut. `sub_rc` below zero says the support held a real improving route; `Inf` says it
was barren, which is fine (that is a cut) but `Inf` at every g means the guides are aiming
at nothing.

Env: PS_N PS_P PS_S PS_SEED PS_ITERS PS_K2 (list) PS_K1 (list) PS_G (list)
     PS_MESO_LIMIT PS_STATION_LIMIT PS_MACRO_LIMIT PS_SCENARIOS
"""

using StationSelection
using JuMP
using Printf
using Serialization

const SS = StationSelection

n_stations = parse(Int, get(ENV, "PS_N", "40"))
n_pairs    = parse(Int, get(ENV, "PS_P", "16"))
n_scen     = parse(Int, get(ENV, "PS_S", "3"))
seed       = parse(Int, get(ENV, "PS_SEED", "48"))
iters      = parse(Int, get(ENV, "PS_ITERS", "4"))
k2_list    = [parse(Int, x) for x in split(get(ENV, "PS_K2", "24,32"), ',')]
k1_list    = [parse(Int, x) for x in split(get(ENV, "PS_K1", "10,12,14,16,20"), ',')]
g_list     = [parse(Int, x) for x in split(get(ENV, "PS_G", "1,2,3,5,8"), ',')]
macro_lim  = parse(Float64, get(ENV, "PS_MACRO_LIMIT", "20.0"))
meso_lim   = parse(Float64, get(ENV, "PS_MESO_LIMIT", "20.0"))
stat_lim   = parse(Float64, get(ENV, "PS_STATION_LIMIT", "10.0"))
max_scen   = parse(Int, get(ENV, "PS_SCENARIOS", "1"))

include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))
problem, k, _ = benchmark_problem(@__DIR__, "PS", n_stations, n_pairs, n_scen, seed)
@printf("instance n=%d p=%d s=%d seed=%d | duals from %d CG iterations (exact pricer)\n",
        n_stations, n_pairs, n_scen, seed, iters)
@printf("grid: K2=%s x K1=%s x g=%s | caps macro=%.0fs meso=%.0fs station=%.0fs | scenarios=%d\n\n",
        string(k2_list), string(k1_list), string(g_list), macro_lim, meso_lim, stat_lim, max_scen)

formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=10)
# PS_DUALS replays a snapshot captured from a real long run (STUDY9_EXPORT_DUALS=1). The
# model is only BUILT -- never solved -- because everything the scan needs from it (travel
# matrix, node list, weights, mapping) is deterministic in the instance, while the duals are
# whatever the snapshot says. That is the whole point: late-stage duals without paying for a
# late-stage solve, so a hyperparameter sweep costs seconds instead of hours.
dual_file = get(ENV, "PS_DUALS", "")
solver = CGSolver(
    config=SolverOptions(silent=true, time_limit_sec=300.0, threads=1),
    max_iterations=iters, reduced_cost_tol=1e-6,
    pricing_time_limit_sec=60.0, certifying_pricing_time_limit_sec=600.0,
    total_time_limit_sec=1800.0, parallel_scenario_pricing=true,
    recover_integer_solution=false)
snapshot = nothing
if isempty(dual_file)
    result = run_opt(problem, formulation, solver)
    m, mapping = result.model, result.mapping
else
    snapshot = Serialization.deserialize(dual_file)
    (snapshot.n_stations == n_stations && snapshot.seed == seed &&
     snapshot.n_pairs == n_pairs && snapshot.n_scenarios == n_scen) || error(
        "dual snapshot is for n=$(snapshot.n_stations) p=$(snapshot.n_pairs) " *
        "s=$(snapshot.n_scenarios) seed=$(snapshot.seed), but this scan is configured for " *
        "n=$n_stations p=$n_pairs s=$n_scen seed=$seed -- duals are only meaningful " *
        "against the instance they came from")
    br = build_model(problem, formulation, solver)
    m, mapping = br.model, br.mapping
    @printf("REPLAYING duals from %s (captured at CG iteration %d)\n",
            basename(dual_file), snapshot.iteration)
end
travel_cost = m[:joint_routing_assignment_travel_cost]
nodes = m[:joint_routing_assignment_nodes]
build_result = BuildResult(m, mapping, nothing,
    ModelCounts(Dict{String,Int}(), Dict{String,Int}(), Dict{String,Int}()), Dict{String,Any}())
alpha, gamma_o, gamma_d = isnothing(snapshot) ?
    SS.extract_duals(build_result, mapping, m) :
    (snapshot.alpha, snapshot.gamma_o, snapshot.gamma_d)
data = m[:joint_routing_assignment_data]
shared = (
    route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
    max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
    repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
    max_stops=Int(m[:joint_routing_assignment_max_stops]),
    compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]))

timed(ctx, limit) = begin
    t0 = time()
    labels, exhausted, _ = SS._run_label_setting(ctx; time_limit=limit, reduced_cost_tol=1e-6)
    (labels, exhausted, time() - t0)
end
cells_of(routes, node_clusters) =
    (u = Set{Int}(); for r in routes, v in r; push!(u, node_clusters[v]); end; u)

@printf("%-4s %-4s %-3s %-3s %8s %-8s %7s %6s %8s %-8s %5s %8s %-8s %9s %12s\n",
        "K2","K1","g","sc","macro_s","m_exh","n_imp","cov","meso_s","s_exh","|S|",
        "stat_s","st_exh","round_s","sub_rc")
best = Tuple[]
for k2 in k2_list
    meso = cluster_stations_by_travel_cost(nodes, travel_cost, k2)
    for k1 in k1_list
        k1 < k2 || continue
        macro_cl, parent = SS._nested_macro_clustering(meso, k1, travel_cost)
        for s in 1:min(max_scen, length(mapping.scenarios))
            cands = joint_routing_assignment_pricing_candidates(
                data, mapping, alpha, gamma_o, gamma_d,
                Float64(m[:joint_routing_assignment_walk_cost_weight]),
                Float64(m[:joint_routing_assignment_detour_factor]), s)
            isempty(cands) && continue
            mac = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
                s, macro_cl, travel_cost, cands; shared...)
            isempty(mac.inner.opportunities) && continue
            mnc = SS._relaxed_cluster_node_clusters(mac)
            mlabels, m_exh, macro_s = timed(SS.RelaxedClusterCutSearchContext(mac, Set{Int}[]), macro_lim)
            imp = sort!(filter(l -> l.reduced_cost < -1e-6, mlabels);
                        by=l -> (l.reduced_cost, length(l.route)))
            isempty(imp) && continue
            for g in g_list
                U = cells_of([l.route for l in imp[1:min(g, length(imp))]], mnc)
                keep = [c for c in 1:meso.n_clusters if parent[c] in U]
                cov = length(keep) / meso.n_clusters
                rest = SS._two_tier_restrict(meso, keep)
                rc_ = SS._restrict_candidates_to_subset(cands, rest.nodes)
                meso_s = 0.0; s_exh = true; nS = 0; stat_s = 0.0; st_exh = true; sub_rc = Inf
                if !isempty(rc_)
                    rd = SS.create_joint_routing_assignment_relaxed_cluster_pricing_data(
                        s, rest, travel_cost, rc_; shared...)
                    if !isempty(rd.inner.opportunities)
                        rlabels, s_exh, meso_s = timed(
                            SS.RelaxedClusterCutSearchContext(rd, Set{Int}[]), meso_lim)
                        rimp = sort!(filter(l -> l.reduced_cost < -1e-6, rlabels);
                                     by=l -> (l.reduced_cost, length(l.route)))
                        if !isempty(rimp)
                            rnc = SS._relaxed_cluster_node_clusters(rd)
                            sup = Set{Int}(keep[rnc[v]] for l in rimp[1:min(g,length(rimp))] for v in l.route)
                            _, stations, _ = SS._two_tier_aligned_support(sup, parent, meso, 15)
                            nS = length(stations)
                            sc = SS._restrict_candidates_to_subset(cands, stations)
                            if !isempty(sc)
                                pd = create_joint_routing_assignment_pricing_data(
                                    s, stations, travel_cost, sc; shared...)
                                if !isempty(pd.opportunities)
                                    slab, st_exh, stat_s = timed(
                                        SS.JointRoutingAssignmentSearchContext(pd), stat_lim)
                                    sub_rc = isempty(slab) ? Inf : minimum(l.reduced_cost for l in slab)
                                end
                            end
                        end
                    end
                end
                round_s = macro_s + meso_s + stat_s
                ok = m_exh && s_exh && st_exh
                push!(best, (round_s, k2, k1, g, cov, ok, sub_rc))
                @printf("%-4d %-4d %-3d %-3d %8.2f %-8s %7d %6.2f %8.2f %-8s %5d %8.2f %-8s %9.2f %12s\n",
                        k2, k1, g, s, macro_s, m_exh ? "yes" : "TIMEOUT", length(imp), cov,
                        meso_s, s_exh ? "yes" : "TIMEOUT", nS, stat_s,
                        st_exh ? "yes" : "TIMEOUT", round_s,
                        isfinite(sub_rc) ? @sprintf("%.1f", sub_rc) : "Inf(barren)")
                flush(stdout)
            end
        end
    end
end

println("\n==== RANKED: cheapest predicted round first (only fully-exhausted configs qualify)")
qual = sort!([b for b in best if b[6]]; by=first)
if isempty(qual)
    println("NONE fully exhausted -- every configuration had a component hit its cap.")
    println("Loosen PS_*_LIMIT, or the grid is entirely outside the workable regime.")
else
    @printf("%-9s %-4s %-4s %-3s %6s %12s\n", "round_s","K2","K1","g","cov","sub_rc")
    for b in qual[1:min(12, length(qual))]
        @printf("%-9.2f %-4d %-4d %-3d %6.2f %12s\n", b[1], b[2], b[3], b[4], b[5],
                isfinite(b[7]) ? @sprintf("%.1f", b[7]) : "Inf(barren)")
    end
    r = first(qual)
    @printf("\nRECOMMENDATION for n=%d: K2=%d K1=%d guide_routes=%d  (round ~%.2fs, cov %.2f)\n",
            n_stations, r[2], r[3], r[4], r[1], r[5])
end
