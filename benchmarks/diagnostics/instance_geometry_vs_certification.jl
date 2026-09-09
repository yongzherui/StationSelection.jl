"""What distinguishes an instance the relaxed-cluster loop CERTIFIES from one it cannot?

MEASURED at n=40 (study 9 + the 2026-09-08 anytime run): single-tier `relaxed_k60` and
two-tier `twotier_k60m14` certify the SAME four seeds, {44, 46, 48, 49}, and fail on the
same five, {42, 45, 47, 50, 51}. Two-tier is 20-64x faster inside that certifiable set and
2.1x on its hard member (seed 46), but it moves no instance across the line. So the line is
a property of the INSTANCE, not of the search, and this script asks what that property is.

# The hypothesis being tested

The relaxation's slack has a known source: the inter-cluster travel charge is a MINIMUM over
member pairs, so every passenger may independently pick its own best station inside a cell
without paying for the disagreement (see `../../src/opt/label_setting/joint_routing_assignment/
relaxed_cluster/clustering.jl`). A cell holding one station has no such freedom and
contributes nothing to the gap. It follows that looseness should scale with the INTRA-CELL
DISPERSION of the cells demand actually touches -- not with raw instance size, not with the
spread of the stations overall, and not with how far apart the OD endpoints are.

That predicts something falsifiable: certifying seeds should have demand concentrated in
singleton or geographically tight cells, and refuting seeds should have demand landing in
fat, spread-out cells. If instead the certifying seeds merely have shorter OD trips or fewer
distinct endpoints, the story is ordinary instance easiness and the clustering is incidental.

Both are reported so they can be told apart.

# Columns

  cells_hit    distinct K2 cells the scenario's OD endpoints land in (demand footprint)
  multi_hit    how many of those hold >1 station -- the cells that CAN be over-credited
  disp_hit     mean intra-cell travel dispersion over the hit cells, in cost units;
               a singleton cell contributes 0. THIS is the hypothesis's predictor.
  disp_all     the same over every cell, as the control -- if disp_hit tracks certification
               but disp_all does not, the demand footprint is what matters, not the partition
  od_mean      mean origin->destination travel cost, the ordinary-easiness control
  od_spread    distinct endpoint stations / (2 * n_pairs); 1.0 = every endpoint distinct

Usage: sbatch benchmarks/diagnostics/run_instance_geometry.sh
Env: IG_N IG_P IG_S IG_K2 IG_SEEDS (comma list)
"""

using StationSelection
using Statistics
using Printf
using DataFrames

const SS = StationSelection
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

n_stations = parse(Int, get(ENV, "IG_N", "40"))
n_pairs    = parse(Int, get(ENV, "IG_P", "16"))
n_scen     = parse(Int, get(ENV, "IG_S", "3"))
k2         = parse(Int, get(ENV, "IG_K2", "24"))
seeds      = parse.(Int, split(get(ENV, "IG_SEEDS", "42,43,44,45,46,47,48,49,50,51"), ","))

# Ground truth from the n=40 runs, so the table can be read without cross-referencing.
const CERTIFIED = Set([44, 46, 48, 49])

"""Symmetrized travel cost between two COMPACT station indices, the same distance the
partition is built on -- so dispersion is measured in the units the relaxation charges.

Compact indices, not station ids: `build_joint_routing_assignment.jl` clusters
`m[:joint_routing_assignment_nodes] = 1:n` over a `get_routing_cost(data, i, j)` table, so
`cluster_of`/`members` are in that space. Mixing the two spaces here would silently cluster
the wrong points and produce a table that looks fine.

`isfinite` mirrors the real build, which omits non-finite pairs from the dict entirely."""
function travel(costs, i, j)
    i == j && return 0.0
    a = get(costs, (i, j), nothing); b = get(costs, (j, i), nothing)
    isnothing(a) && isnothing(b) && return NaN
    isnothing(a) && return b
    isnothing(b) && return a
    return 0.5 * (a + b)
end

"""The pricing travel table, built exactly as `build_joint_routing_assignment.jl` builds
it, so the partition below is the one a real run would use."""
function pricing_travel_cost(data, n)
    tc = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:n, j in 1:n
        i == j && continue
        c = SS.get_routing_cost(data, i, j)
        isfinite(c) && (tc[(i, j)] = c)
    end
    return tc
end

"""Mean pairwise travel cost inside one cell. A singleton contributes 0: it grants a
passenger no choice, so it cannot be a source of relaxation slack."""
function cell_dispersion(members, costs)
    length(members) < 2 && return 0.0
    ds = Float64[]
    for a in eachindex(members), b in eachindex(members)
        a < b || continue
        d = travel(costs, members[a], members[b])
        isnan(d) || push!(ds, d)
    end
    return isempty(ds) ? 0.0 : mean(ds)
end

println("n=$n_stations p=$n_pairs scenarios=$n_scen K2=$k2\n")
@printf("%5s %5s  %9s %9s %9s %9s  %9s %9s\n",
        "seed", "cert", "cells_hit", "multi_hit", "disp_hit", "disp_all", "od_mean", "od_spread")

rows = NamedTuple[]
for seed in seeds
    problem, _k, _meta = benchmark_problem(@__DIR__, "IG", n_stations, n_pairs, n_scen, seed)
    data = problem.data
    n = nrow(data.stations)
    costs = pricing_travel_cost(data, n)
    clustering = cluster_stations_by_travel_cost(collect(1:n), costs, k2)
    disp_all = mean(cell_dispersion(ms, costs) for ms in clustering.members)

    # Pool the scenarios: the pricer searches every scenario each round, so the demand
    # footprint that matters is the union, not any one scenario's.
    hit = Set{Int}(); endpoints = Set{Int}(); od = Float64[]
    for sc in data.scenarios
        r = sc.requests
        for row in eachrow(r)
            # `origin_idx`/`dest_idx` are already the compact indices the partition uses.
            o = Int(row.origin_idx); d = Int(row.dest_idx)
            push!(endpoints, o); push!(endpoints, d)
            haskey(clustering.cluster_of, o) && push!(hit, clustering.cluster_of[o])
            haskey(clustering.cluster_of, d) && push!(hit, clustering.cluster_of[d])
            t = travel(costs, o, d); isnan(t) || push!(od, t)
        end
    end
    hit_cells = sort(collect(hit))
    multi = count(c -> length(clustering.members[c]) > 1, hit_cells)
    disp_hit = isempty(hit_cells) ? NaN :
        mean(cell_dispersion(clustering.members[c], costs) for c in hit_cells)
    spread = length(endpoints) / (2 * n_pairs * n_scen)

    @printf("%5d %5s  %9d %9d %9.1f %9.1f  %9.1f %9.2f\n",
            seed, seed in CERTIFIED ? "YES" : "no", length(hit_cells), multi,
            disp_hit, disp_all, isempty(od) ? NaN : mean(od), spread)
    push!(rows, (seed=seed, cert=seed in CERTIFIED, cells_hit=length(hit_cells),
                 multi_hit=multi, disp_hit=disp_hit, disp_all=disp_all,
                 od_mean=isempty(od) ? NaN : mean(od), od_spread=spread))
end

# ── does any column separate the two groups? ────────────────────────────────────
df = DataFrame(rows)
yes = df[df.cert, :]; no = df[.!df.cert, :]
println()
@printf("%-12s %12s %12s %12s\n", "column", "certified", "refuted", "separation")
for c in (:cells_hit, :multi_hit, :disp_hit, :disp_all, :od_mean, :od_spread)
    a = mean(skipmissing(getproperty(yes, c))); b = mean(skipmissing(getproperty(no, c)))
    # Overlap matters more than the gap in means at this sample size: a column only
    # SEPARATES if every certified value sits on one side of every refuted one.
    va = collect(getproperty(yes, c)); vb = collect(getproperty(no, c))
    clean = maximum(va) < minimum(vb) || minimum(va) > maximum(vb)
    @printf("%-12s %12.2f %12.2f %12s\n", c, a, b, clean ? "CLEAN" : "overlaps")
end
println("\nA column marked CLEAN separates the two groups with no overlap at this sample.")
