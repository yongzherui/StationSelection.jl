"""Write Study 11's job tables. Run from this directory: `julia --startup-file=no generate_jobs.jl`."""

const HEADER = [
    "job_id", "phase", "arm", "n_stations", "n_pairs", "n_scenarios", "seed", "max_stops",
    "n_threads", "cluster_count", "guide_routes", "pricing_limit_sec",
    "certifying_limit_sec", "total_limit_sec", "ip_every", "ip_time_limit_sec",
]

"""
    job(id, phase, arm; kwargs...) -> String

One tab-separated row. `cluster_count` follows Study 9's `K = 0.6n` rule for the
`relaxed_k60` arm and is 0 for `exact`, which `run_benchmark.jl` enforces.
"""
function job(id, phase, arm; n, p, s, seed, max_stops=10, threads=3, k0,
             guides=5, pricing=300.0, certifying=3600.0, total, ip_every, ip_limit=120.0)
    return join(string.([id, phase, arm, n, p, s, seed, max_stops, threads, k0, guides,
                         pricing, certifying, total, ip_every, ip_limit]), '\t')
end

write_table(name, rows) = open(joinpath(@__DIR__, "config", name), "w") do io
    println(io, join(HEADER, '\t'))
    for r in rows
        println(io, r)
    end
    println("wrote config/$name  ($(length(rows)) jobs)")
end

mkpath(joinpath(@__DIR__, "config"))

# Smoke: small enough to certify in seconds, exercising the snapshot callback and the
# per-iteration MIP rebuild on both arms before anything long is submitted.
write_table("smoke.tsv", [
    job(1, "smoke", "relaxed_k60"; n=8, p=4, s=3, seed=42, max_stops=6, k0=5,
        pricing=30.0, certifying=120.0, total=600.0, ip_every=1, ip_limit=30.0),
    job(2, "smoke", "exact"; n=8, p=4, s=3, seed=42, max_stops=6, k0=0,
        pricing=30.0, certifying=120.0, total=600.0, ip_every=1, ip_limit=30.0),
])

# The study proper. n=30 at K=18 is the cell Study 9 certified on 10/10 seeds in 9-25 CG
# iterations (worst wall 5195 s), which is what makes a snapshot at EVERY iteration
# affordable and makes "the intermediate iterations" a bounded, fully observable set rather
# than a prefix of a censored run.
write_table("n30.tsv", [
    job(i, "main", "relaxed_k60"; n=30, p=16, s=3, seed=seed, k0=18,
        total=14400.0, ip_every=1)
    for (i, seed) in enumerate(42:51)
])

# Control. The master LP is IDENTICAL across pricers -- only the column pool differs -- so
# an `exact` arm on the same instances separates "the LP relaxation of this formulation is
# (not) tight" from "the relaxed-cluster pricer feeds it a pool that happens to be
# (non-)integral". n=30 is beyond `exact`'s certification frontier (it certifies only at
# s=1), so these runs are expected to stop on budget; that costs nothing here, because the
# question is about the iterations they DO reach, not about their final objective.
write_table("n30_exact_control.tsv", [
    job(i, "control", "exact"; n=30, p=16, s=3, seed=seed, k0=0,
        total=14400.0, ip_every=1)
    for (i, seed) in enumerate(42:46)
])

# max_stops probe. Two questions, one table: does the BENCHMARK_BASELINE cap of 10 bind at
# all (answered directly by the new col_n_at_cap / pool_max_stops columns), and does
# changing it move the intermediate fractionality? Seeds 45 and 48 are the two that went
# most fractional mid-run (max intermediate gaps 3.98% and 7.67%); 42 stayed integral at
# every one of its 23 iterations and is the negative control. If the cap is slack, the
# max_stops=14 arm reproduces the max_stops=10 objective exactly.
write_table("n30_maxstops.tsv", [
    job(i, "maxstops", "relaxed_k60"; n=30, p=16, s=3, seed=seed, k0=18,
        max_stops=ms, total=14400.0, ip_every=1)
    for (i, (seed, ms)) in enumerate([(s, m) for s in (45, 48, 42) for m in (10, 14)])
])
