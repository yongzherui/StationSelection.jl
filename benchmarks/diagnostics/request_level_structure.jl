"""What do the 16 trip requests actually LOOK like, easy seed vs hard seed?

Everything measured so far about the demand draw was an aggregate -- spread about a
centroid, mean trip length, count of clusters touched -- pooled over all three scenarios.
None of it separated easy seeds from hard ones, and pooling was itself wrong: each scenario
is drawn from its own RNG (`seed + (s-1)*1000`) and priced as a SEPARATE problem, so the
structure the sweep sees is one scenario's 16 requests, not the 48-request union.

This dumps the requests and describes them as what they are: a directed multigraph on
stations, 16 edges per scenario. The relaxed sweep's cost should track how much that graph
lets routes combine, and the graph-level quantities that decide it are visible without any
solve:

  stations    distinct stations the 16 requests touch (of 40)
  max_out     largest number of requests leaving one station -- a pickup hub
  max_in      largest number arriving at one station -- a dropoff hub
  hub_share   fraction of the 16 requests incident to the single busiest station
  chains      ordered request pairs where one's destination IS another's origin; a route
              can serve such a pair with no detour at all, so these are the cheapest
              possible combinations and the most likely to proliferate
  recip       unordered station pairs served in BOTH directions
  dup_ends    requests sharing an (origin) or (destination) with another request

Sampling is WITHOUT replacement from the valid-pair pool, so the 16 pairs are always
distinct; repetition can only appear at the endpoint level, which is what these count.

Usage: sbatch benchmarks/diagnostics/run_request_level.sh
Env: RL_N RL_P RL_S RL_SEEDS RL_DUMP (comma list of seeds to print raw)
"""

using StationSelection
using Statistics
using Printf
using DataFrames

const SS = StationSelection
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

n_stations = parse(Int, get(ENV, "RL_N", "40"))
n_pairs    = parse(Int, get(ENV, "RL_P", "16"))
n_scen     = parse(Int, get(ENV, "RL_S", "3"))
seeds      = parse.(Int, split(get(ENV, "RL_SEEDS", "42,44,45,46,47,48,49,50,51"), ","))
dump_seeds = Set(parse.(Int, split(get(ENV, "RL_DUMP", "48,49,45,47"), ",")))

# Seconds per certification round, single-tier relaxed_k60, n=40 -- the difficulty scale.
const ROUND_COST = Dict(48=>174.9, 44=>236.0, 49=>256.2, 46=>330.7, 42=>432.0,
                        50=>622.5, 51=>675.0, 47=>744.8, 45=>1136.8)
const CERTIFIED = Set([44, 46, 48, 49])

function scenario_stats(pairs)
    outdeg = Dict{Int,Int}(); indeg = Dict{Int,Int}()
    for (o, d) in pairs
        outdeg[o] = get(outdeg, o, 0) + 1
        indeg[d]  = get(indeg, d, 0) + 1
    end
    stations = union(keys(outdeg), keys(indeg))
    # Busiest station by total incidence, and how much of the demand runs through it.
    inc = Dict(s => get(outdeg, s, 0) + get(indeg, s, 0) for s in stations)
    busiest = argmax(inc)
    origins = Set(o for (o, _) in pairs); dests = Set(d for (_, d) in pairs)
    chains = count(((o2, _),) -> o2 in dests, pairs)
    pairset = Set(pairs)
    recip = count(((o, d),) -> o < d && (d, o) in pairset, pairs)
    dup = count(((o, d),) -> get(outdeg, o, 0) > 1 || get(indeg, d, 0) > 1, pairs)
    return (stations=length(stations), max_out=maximum(values(outdeg)),
            max_in=maximum(values(indeg)), hub_share=inc[busiest] / length(pairs),
            chains=chains, recip=recip, dup_ends=dup)
end

rows = NamedTuple[]
for seed in seeds
    problem, _k, _m = benchmark_problem(@__DIR__, "RL", n_stations, n_pairs, n_scen, seed)
    per = NamedTuple[]
    for (si, sc) in enumerate(problem.data.scenarios)
        pairs = [(Int(r.origin_idx), Int(r.dest_idx)) for r in eachrow(sc.requests)]
        st = scenario_stats(pairs)
        push!(per, st)
        if seed in dump_seeds && si == 1
            tag = seed in CERTIFIED ? "EASY" : "hard"
            @printf("\n--- seed %d scenario 1 (%s, %.1f s/round) ---\n", seed, tag,
                    get(ROUND_COST, seed, NaN))
            println("  requests (origin -> dest, compact station index):")
            for (i, (o, d)) in enumerate(sort(pairs))
                @printf("    %2d.  %3d -> %3d\n", i, o, d)
            end
            @printf("  stations=%d max_out=%d max_in=%d hub_share=%.2f chains=%d recip=%d dup_ends=%d\n",
                    st.stations, st.max_out, st.max_in, st.hub_share, st.chains, st.recip, st.dup_ends)
        end
    end
    m(f) = mean(getproperty(p, f) for p in per)
    push!(rows, (seed=seed, cost=get(ROUND_COST, seed, NaN), cert=seed in CERTIFIED,
        stations=m(:stations), max_out=m(:max_out), max_in=m(:max_in),
        hub_share=m(:hub_share), chains=m(:chains), recip=m(:recip), dup_ends=m(:dup_ends)))
end

function spearman(x, y)
    n = length(x); n < 3 && return NaN
    rk(v) = (p = sortperm(v); r = zeros(Float64, n); for (i, j) in enumerate(p); r[j] = i; end; r)
    rx, ry = rk(collect(float.(x))), rk(collect(float.(y)))
    mx, my = mean(rx), mean(ry)
    d = sqrt(sum((rx .- mx).^2) * sum((ry .- my).^2))
    return d == 0 ? NaN : sum((rx .- mx) .* (ry .- my)) / d
end

df = sort(DataFrame(rows), :cost)
println("\n\n=== per-scenario means, sorted EASIEST to HARDEST ===\n")
@printf("%5s %5s %9s  %9s %8s %8s %10s %8s %7s %9s\n", "seed", "cert", "s/round",
        "stations", "max_out", "max_in", "hub_share", "chains", "recip", "dup_ends")
for r in eachrow(df)
    @printf("%5d %5s %9.1f  %9.1f %8.1f %8.1f %10.2f %8.1f %7.1f %9.1f\n",
            r.seed, r.cert ? "YES" : "no", r.cost, r.stations, r.max_out, r.max_in,
            r.hub_share, r.chains, r.recip, r.dup_ends)
end

println("\nSpearman vs seconds-per-round (n=$(nrow(df))); |rho|>=0.68 is p<0.05 here:")
for c in (:stations, :max_out, :max_in, :hub_share, :chains, :recip, :dup_ends)
    rho = spearman(getproperty(df, c), df.cost)
    flag = abs(rho) >= 0.80 ? "  <== STRONG" : (abs(rho) >= 0.68 ? "  <== crosses p<0.05" : "")
    @printf("  %-10s rho = %+.3f%s\n", c, rho, flag)
end
println("\nNOTE: 7 measures tested here and 11 before. One crossing p<0.05 is what chance")
println("gives at this sample -- only a STRONG, mechanism-consistent result should be believed.")
