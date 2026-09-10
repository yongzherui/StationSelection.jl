"""Does the SPATIAL SPREAD of the sampled OD pairs predict relaxed-round cost?

Two facts set this up. First, a seed sweep on this generator varies ONLY the OD-pair draw --
stations, costs and the k-medoids partition are byte-identical across seeds (see
`instance_geometry_vs_certification.jl`), so the demand footprint is the only thing that can
explain a difficulty difference. Second, that difficulty is concentrated in ONE quantity:
across n=40 study-9 runs, easy and hard seeds produce columns at the same rate per
iteration (140 vs 148), use the same subset sizes (8.2 vs 9.9) and cut at the same rate
(0.8 vs 0.7) -- but a certification ROUND costs 2.9x more on the hard ones (249 s vs 722 s),
with no overlap. The expense is the relaxed cluster-graph sweep itself.

So the question is which geometric property of the 16-pair draw makes that sweep expensive.
`cert_s/round` is used as the difficulty scale rather than the certified/uncertified label,
because it is continuous, it separated cleanly, and a TREND is the claim worth testing at
n=9 -- a binary split throws away the ordering that carries most of the signal.

Reported as Spearman rank correlation, not Pearson: with 9 points a single outlier seed
dominates a linear fit, and only the monotone ordering is being claimed.

Distances are haversine kilometres on the station coordinates -- genuinely geographic, and
independent of the routing-cost table the partition is built from, so a correlation here is
not an artifact of the clustering.

Usage: sbatch benchmarks/diagnostics/run_od_spread.sh
Env: OS_N OS_P OS_S OS_SEEDS
"""

using StationSelection
using Statistics
using Printf
using DataFrames

const SS = StationSelection
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

n_stations = parse(Int, get(ENV, "OS_N", "40"))
n_pairs    = parse(Int, get(ENV, "OS_P", "16"))
n_scen     = parse(Int, get(ENV, "OS_S", "3"))
seeds      = parse.(Int, split(get(ENV, "OS_SEEDS", "42,44,45,46,47,48,49,50,51"), ","))

# MEASURED: seconds per certification round, single-tier `relaxed_k60`, n=40, study 9.
# Seed 43 is absent -- that arm never ran -- so it is excluded rather than imputed.
const ROUND_COST = Dict(48=>174.9, 44=>236.0, 49=>256.2, 46=>330.7, 42=>432.0,
                        50=>622.5, 51=>675.0, 47=>744.8, 45=>1136.8)
const CERTIFIED = Set([44, 46, 48, 49])

haversine_km(lo1, la1, lo2, la2) = begin
    R = 6371.0; φ1, φ2 = deg2rad(la1), deg2rad(la2)
    dφ, dλ = φ2 - φ1, deg2rad(lo2 - lo1)
    a = sin(dφ/2)^2 + cos(φ1)*cos(φ2)*sin(dλ/2)^2
    2R * asin(min(1.0, sqrt(a)))
end

"""Mean distance from each point to the set's centroid -- the dispersion of a cloud,
insensitive to how many points it holds (unlike a diameter, which only sees the extremes)."""
function centroid_spread(pts)
    length(pts) < 2 && return 0.0
    clo = mean(p[1] for p in pts); cla = mean(p[2] for p in pts)
    return mean(haversine_km(p[1], p[2], clo, cla) for p in pts)
end

function spearman(x, y)
    n = length(x); n < 3 && return NaN
    rank(v) = (p = sortperm(v); r = zeros(Float64, n); for (i, j) in enumerate(p); r[j] = i; end; r)
    rx, ry = rank(collect(float.(x))), rank(collect(float.(y)))
    mx, my = mean(rx), mean(ry)
    num = sum((rx .- mx) .* (ry .- my))
    den = sqrt(sum((rx .- mx).^2) * sum((ry .- my).^2))
    return den == 0 ? NaN : num / den
end

rows = NamedTuple[]
for seed in seeds
    problem, _k, _m = benchmark_problem(@__DIR__, "OS", n_stations, n_pairs, n_scen, seed)
    data = problem.data
    lon = Float64.(data.stations.lon); lat = Float64.(data.stations.lat)
    pt(i) = (lon[i], lat[i])

    origins = Tuple{Float64,Float64}[]; dests = Tuple{Float64,Float64}[]
    mids = Tuple{Float64,Float64}[]; trips = Float64[]
    for sc in data.scenarios, row in eachrow(sc.requests)
        o = Int(row.origin_idx); d = Int(row.dest_idx)
        push!(origins, pt(o)); push!(dests, pt(d))
        push!(mids, ((lon[o]+lon[d])/2, (lat[o]+lat[d])/2))
        push!(trips, haversine_km(lon[o], lat[o], lon[d], lat[d]))
    end
    allpts = vcat(origins, dests)
    diam = maximum(haversine_km(a[1],a[2],b[1],b[2]) for a in allpts, b in allpts)

    push!(rows, (seed=seed, cost=get(ROUND_COST, seed, NaN), cert=seed in CERTIFIED,
        orig_spread=centroid_spread(origins), dest_spread=centroid_spread(dests),
        all_spread=centroid_spread(allpts), mid_spread=centroid_spread(mids),
        diam_km=diam, trip_mean=mean(trips), trip_sd=std(trips)))
end

df = sort(DataFrame(rows), :cost)
println("n=$n_stations p=$n_pairs scenarios=$n_scen -- sorted EASIEST to HARDEST\n")
@printf("%5s %5s %9s  %10s %10s %10s %10s %9s %9s %8s\n", "seed", "cert", "s/round",
        "orig_sprd", "dest_sprd", "all_sprd", "mid_sprd", "diam_km", "trip_mn", "trip_sd")
for r in eachrow(df)
    @printf("%5d %5s %9.1f  %10.2f %10.2f %10.2f %10.2f %9.2f %9.2f %8.2f\n",
            r.seed, r.cert ? "YES" : "no", r.cost, r.orig_spread, r.dest_spread,
            r.all_spread, r.mid_spread, r.diam_km, r.trip_mean, r.trip_sd)
end

println("\nSpearman rank correlation with seconds-per-round (+1 = spread rises with cost):")
for c in (:orig_spread, :dest_spread, :all_spread, :mid_spread, :diam_km, :trip_mean, :trip_sd)
    rho = spearman(getproperty(df, c), df.cost)
    flag = abs(rho) >= 0.80 ? "  <== STRONG" : (abs(rho) >= 0.60 ? "  <- moderate" : "")
    @printf("  %-12s rho = %+.3f%s\n", c, rho, flag)
end
println("\nn=9, so rho is indicative only: |rho|>=0.68 is p<0.05 two-sided at this size.")
