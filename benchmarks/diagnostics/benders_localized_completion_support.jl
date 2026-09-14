"""How many unbuilt stations can the closed-form completion actually charge? The count that
decides whether a LOCALIZED pricing set can replace full-station pricing.

The activated oracle prices only over the built stations `S` and completes the duals on
everything else, and the completion's charge on an unbuilt station is fiction -- measured at
1.80x the true dual's coefficient mass, and the reason its cuts do not converge past n=10
(`notes/2026-09-10_activated_dual_completion_verified_and_why_weak.md`).

The idea this script measures: price over `S+ = S union T` for a SMALL strategically chosen
`T` instead of over all `n` stations. The shortcut argument that licenses the closed-form
completion goes through verbatim with `S+` in place of `S`, so any `T` is sound; what `T`
buys is that the completion's fiction is confined to stations outside `S+`, and what it
costs is a pricing set of `|S+|` stations instead of `k`.

# The number that decides it

The completion charges `gammaO_pj = max(0, alpha_p - w * min_k walk(o,d,(j,k)))`, so it
charges NOTHING on a station `j` unless some positive-demand group `p` has
`alpha_p > w * walk_min(p,j)`. A station it cannot charge needs no `T` membership: its
columns are discharged at `gamma = 0`, for free, whatever the pricing set was. So the
fiction lives on

    F = { j : exists p with alpha_p > w * walk_min(p,j) }

and the strongest form of the idea is `T = F \\ S`, which makes the cut EXACTLY as strong as
full-station pricing while pricing only `|S| + |F \\ S|` stations. Whether that is a saving
is entirely a question of how big `F` is, and `F` is bounded by geometry rather than by `n`:
a group only has valid station pairs within `max_walking_distance` of its endpoints.

# Why this needs no solve

`alpha_p` would need an exhausted subproblem, which at n=40 is the very thing that is hard.
Two supersets of `F` are computable from the mapping alone:

  A = { j : j appears in some valid (j,k) pair of some positive-demand group }
      `walk_min(p,j)` is undefined otherwise, so no linking row mentions `(p,j)` and the
      completion never touches it. A rigorous superset, pure geometry.

  B = A refined by the OTHER dual family. A group with a direct-walk option has
      `alpha_p <= c_walk[s,p] = w * demand_p * walk(o,d,WALK_ONLY_PAIR)`, so
      `demand_p * walk_direct_p <= walk_min(p,j)` rules `(p,j)` out with no solve at all.
      A group with NO walk fallback has no such bound, so every station valid for it stays
      in -- reported separately, since that is what limits the refinement.

`|F| <= |B| <= |A|`, so `k + |B \\ S|` is an upper bound on the localized pricing set and the
comparison against `n` is the whole answer. `S` is unknown before the solve, so this reports
the bound against a plausible greedy `S` (the `k` stations appearing in the most valid pairs)
as well as the incumbent-free worst case `min(n, k + |B|)`.

Usage: sbatch benchmarks/diagnostics/run_benders_localized_completion_support.sh
Env: LOC_CELLS (semicolon-separated n,p,s,seed), LOC_MAX_STOPS
"""

using StationSelection
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const CELLS = get(ENV, "LOC_CELLS",
    "10,8,1,42;20,8,1,42;30,8,1,42;40,8,1,42;40,8,3,42;40,16,1,42;50,8,1,45")
const MAX_STOPS = parse(Int, get(ENV, "LOC_MAX_STOPS", "10"))
const W = 0.1   # walk_cost_weight; BENCHMARK_BASELINE's value. Cancels in every ratio below.

@printf("%-22s %5s %5s %6s %6s %6s %7s %8s %9s %9s\n",
        "cell", "n", "k", "|A|", "|B|", "noWF", "|B\\Sg|", "k+|B\\Sg|", "vs n", "ceiling")
println("-"^104)

for cell in split(CELLS, ';')
    n, p, s, seed = parse.(Int, split(cell, ','))
    problem, k, _meta = benchmark_problem(@__DIR__, "LOC", n, p, s, seed)
    data = problem.data
    formulation = AggregateODRouteJointRoutingAssignmentFormulation(
        ; BENCHMARK_BASELINE..., max_stops = MAX_STOPS)
    mapping = create_aggregate_od_route_map(problem, formulation, data)

    A = Set{Int}()
    B = Set{Int}()
    pair_count = Dict{Int, Int}()      # how many valid pairs each station appears in
    n_no_walk_fallback = 0
    n_groups = 0

    for sc in 1:n_scenarios(data)
        for (pi, (o, d)) in enumerate(mapping.Omega_s[sc])
            demand = mapping.Q_s[sc][pi]
            demand > 0 || continue
            n_groups += 1
            pairs = get_valid_jk_pairs(mapping, o, d)
            has_walk = any(is_walk_only_pair, pairs)
            has_walk || (n_no_walk_fallback += 1)
            # alpha_p's only a-priori upper bound, when the group has a walk fallback at all.
            alpha_ub = has_walk ?
                W * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR) : Inf

            walk_min_o = Dict{Int, Float64}()
            walk_min_d = Dict{Int, Float64}()
            for pair in pairs
                is_walk_only_pair(pair) && continue
                j, kk = pair
                cost = od_pair_walking_cost(data, o, d, pair)
                walk_min_o[j] = min(get(walk_min_o, j, Inf), cost)
                walk_min_d[kk] = min(get(walk_min_d, kk, Inf), cost)
                pair_count[j] = get(pair_count, j, 0) + 1
                pair_count[kk] = get(pair_count, kk, 0) + 1
                push!(A, j); push!(A, kk)
            end
            # The completion charges (p,j) only if the reward can exceed the walk credit.
            for (j, wmin) in walk_min_o
                alpha_ub > W * wmin && push!(B, j)
            end
            for (j, wmin) in walk_min_d
                alpha_ub > W * wmin && push!(B, j)
            end
        end
    end

    # A plausible `S`: the k stations appearing in the most valid pairs. Not the optimum, but
    # the right shape -- a station in many pairs is one the master has reason to build, so
    # this is a fair stand-in for "how much of B does S already cover".
    greedy_S = Set(first.(sort!(collect(pair_count); by = x -> -x[2])[1:min(k, end)]))
    b_outside = length(setdiff(B, greedy_S))

    @printf("%-22s %5d %5d %6d %6d %6d %7d %8d %9s %9d\n",
            "n$(n)_p$(p)_s$(s)_seed$(seed)", n, k, length(A), length(B),
            n_no_walk_fallback, b_outside, k + b_outside,
            k + b_outside < n ? @sprintf("%.0f%% of n", 100 * (k + b_outside) / n) : "no saving",
            min(n, k + length(B)))
    flush(stdout)
end

println("-"^104)
println("""
|A|  stations the completion could touch at all (geometry only; rigorous superset of F)
|B|  refined by alpha_p <= c_walk where the group has a direct-walk fallback
noWF groups with NO walk fallback -- these admit no alpha bound, so they keep every valid
     station in B and are what limits the refinement
|B\\Sg|, k+|B\\Sg|  the localized pricing set against a plausible greedy S of size k
ceiling  min(n, k+|B|) -- the incumbent-free worst case

Read `k+|B\\Sg|` against `n`. Below n, a localized completion prices a smaller station set
than full pricing while giving up NOTHING in cut strength (F is fully covered). At or above
n, the fiction set is not local and the idea reduces to full-station pricing.""")
