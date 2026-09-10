"""Study 10 job tables: does the certification frontier scale in `n`, or in `n x s`?

Study 9 measured everything at s=3 -- ALL 490 job rows it ever wrote. That leaves the
scenario count completely unmeasured as a difficulty axis, while the round-level rule
"certify only when EVERY scenario certifies in the SAME iteration" makes s a plausible
first-order driver. This study varies s and holds everything else fixed.

Two readings of the n=40/n=50 wall, and they disagree:

- CONJUNCTION. Three scenarios must certify simultaneously, so the round's success
  probability is roughly the product of three per-scenario ones. Dropping to s=1 removes
  the conjunction entirely and should move the frontier a long way.
- PER-SCENARIO. The blocking is inside individual scenarios, so removing the conjunction
  changes nothing. EVIDENCE FOR THIS, from reach5's per-scenario dumps at n=40: the five
  failing seeds had scenarios that never certified even ONCE -- seed 42 recorded zero
  certifications across all three scenarios in 59 attempts, and 8 of the 10
  scenario-slots in the failing seeds were C=0.

Note s=1 is NOT the same problem made smaller: `y` (station build) is shared across
scenarios, so with one scenario the station selection is driven by that scenario's demand
alone. Different master, ~1/3 the columns, different optimum. This measures where the
frontier sits as a function of s, not how much easier one instance got.

Also note the per-round cost argument does NOT predict a 3x speedup: scenarios already
price CONCURRENTLY (`Threads.@threads` in `certify.jl`, gated on `length(scenarios) > 1`),
so a round's wall is the MAX over scenarios, not the sum. s=1 additionally takes the
serial branch, which Study 9 never exercised.

Tables:
  smoke_s1.tsv    2 jobs,  n=20 s=1, both arms, 900 s   -- validates the untested s=1 path
  s1_frontier.tsv 40 jobs, n=40/50 x {relaxed,twotier} x 10 seeds, s=1, 7200 s
  s3_control.tsv  20 jobs, n=50 x {relaxed,twotier} x 10 seeds, s=3, 7200 s

`s3_control.tsv` exists because the s=3 side is only partly covered by Study 9: n=40 is
well measured (relaxed_k60 4/10 at 21600 s; twotier c16g3 5/10 at 7200 s) but n=50 s=3 has
only 4 seeds of single-tier at 21600 s (0/4) and no two-tier at all. Run it if the s=1
result at n=50 turns out interesting enough to need a matched control.

K2 follows Study 9's 0.6n rule (n=40 -> 24, n=50 -> 30). K1=16 is an ABSOLUTE node count
from the measured 12-16 band and is valid against both (see
`notes/2026-09-09_n40_certification_frontier_5_of_10.md`); cap=16 and g=3 are that note's
recommendation. `aligned_subset_max` is inert for the single-tier arm and is left at its
default 15 there rather than being made to look like a swept parameter.
"""

const HEADER = ("job_id", "phase", "arm", "n_stations", "n_pairs", "n_scenarios",
    "seed", "max_stops", "n_threads", "cluster_count", "cluster_max_count",
    "guide_routes", "barren_cache", "cut_management", "pricing_limit_sec",
    "certifying_limit_sec", "total_limit_sec", "macro_count", "aligned_subset_max")

const SEEDS      = 42:51
const N_PAIRS    = 16
const MAX_STOPS  = 10
const N_THREADS  = 3
const PRICING    = 300.0
const CERTIFYING = 3600.0
const TOTAL      = 7200.0

k2_for(n) = round(Int, 0.6 * n)

"""One row. `macro_count=0` marks a single-tier arm -- `run_benchmark.jl` rejects a
non-zero one there rather than silently ignoring it."""
row(id, phase, arm, n, s, seed; k2, k1, guides, cap, total=TOTAL) = (
    id, phase, arm, n, N_PAIRS, s, seed, MAX_STOPS, N_THREADS, k2, 0,
    guides, false, false, PRICING, CERTIFYING, total, k1, cap)

function write_table(path, rows)
    open(path, "w") do io
        println(io, join(HEADER, '\t'))
        for r in rows
            println(io, join(r, '\t'))
        end
    end
    println("wrote $(length(rows)) jobs -> $path")
end

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)

const SINGLE = ("relaxed_k60", 0, 5, 15)          # arm, K1 (0 = single-tier), guides, cap
const TWOTIER = ("twotier_k60m16c16g3", 16, 3, 16)

# ── smoke: the s=1 path has never run, and it is the serial branch
smoke = Any[]
for (i, (arm, k1, g, cap)) in enumerate((SINGLE, TWOTIER))
    push!(smoke, row(i, "smoke", arm, 20, 1, 42;
                     k2=k2_for(20), k1=(k1 == 0 ? 0 : 8), guides=g, cap=cap, total=900.0))
end
write_table(joinpath(config_dir, "smoke_s1.tsv"), smoke)

# ── the study proper
s1 = Any[]; id = Ref(1)
for n in (40, 50), (arm, k1, g, cap) in (SINGLE, TWOTIER), seed in SEEDS
    push!(s1, row(id[], "frontier", arm, n, 1, seed; k2=k2_for(n), k1=k1, guides=g, cap=cap))
    id[] += 1
end
write_table(joinpath(config_dir, "s1_frontier.tsv"), s1)

# ── the matched s=3 control at n=50, the one size Study 9 left thin
s3 = Any[]; cid = Ref(1)
for (arm, k1, g, cap) in (SINGLE, TWOTIER), seed in SEEDS
    push!(s3, row(cid[], "control", arm, 50, 3, seed; k2=k2_for(50), k1=k1, guides=g, cap=cap))
    cid[] += 1
end
write_table(joinpath(config_dir, "s3_control.tsv"), s3)
