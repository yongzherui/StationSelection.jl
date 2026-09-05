"""Generate Study 10's jobs: does **no-good-cut** relaxed-cluster certification still fire
at n=20 and n=30, where CG's own certification is known to break down?

# The question, and why it is not Study 9's

Study 9 asked whether the relaxed-cluster relaxation certifies at all, at n=15, using the
*one-shot* mode (`certification_pricing_mode = :relaxed_cluster`). The answer was a clean
no: 0/31, at every K < n. The reason is structural, not a tuning failure -- at a converged
master the exact minimum reduced cost is exactly 0 (complementary slackness on `theta >=
0`), so certifying one-shot needs the relaxation tight to within `reduced_cost_tol`, and
its slack is 10^2-10^3.

`:relaxed_cluster_nogood` closes exactly that gap. When the relaxation names an improving
*cluster* route, the loop does not give up: it takes that route's cluster support `T`,
searches `stations(T)` exhaustively with the real pricer, and -- if that finds nothing --
adds the cut *"every route must visit a cluster outside T"* and asks again. Cuts are only
ever placed on supports an exhaustive search proved barren, so no real improving route's
image is ever removed and the certificate still covers the **full** revisit-tolerant route
universe. It certifies at n=15 (K=9 and K=12, 5/4/1 and 10/6/1 cuts).

So the open question is scaling, and it is the one that matters: the measured CG
certification frontier on this instance family (p=16) certifies through n<=20 for every
scenario count, n=25 only to s<=5, and n=30 only at s=1. **n=30 at s=3 is past the
frontier** -- baseline CG is not expected to certify there at all. That is the cell this
study exists for.

# The two regimes, and what a win looks like in each

| size | baseline expectation | what an arm certifying means |
| --- | --- | --- |
| n=20, s=3 | certifies (inside the frontier) | no-good certification is *cheaper* than the two-tier certifying round it replaces -- read the paired speedup |
| n=30, s=3 | does NOT certify (past the frontier) | no-good certification reaches an optimality proof **nothing else here can** -- read the certification rate, not the speedup |

Both are reported, and `analyze.jl` deliberately does not require the baseline to have
certified before it reports an arm: at n=30 that requirement would discard the entire
finding. The "arm certified, baseline did not" count is its own headline table.

# The grid

n in {20, 25, 30} x K/n in {0.4, 0.6, 0.8} + baseline, p=16, s=3, ms=10, seeds 42-46.
5 seeds x 4 arms x 3 sizes = **60 jobs** (20 per size: 15 arm runs + 5 baselines).
n=25 was added after the first run -- see the SIZES table below for why, and why it is
appended rather than inserted.

`K` is swept as a *fraction of n* rather than an absolute count so the arms mean the same
thing at every size and the sizes can be read against each other. n=15's working
points (K=9, K=12) are 0.6 and 0.8 of n, so the fractions bracket what already worked and
add a coarser 0.4 to see whether scale buys tolerance for coarser partitions.

There is no `K = n` ceiling control and no one-shot arm, both of which Study 9 needed:

- the **one-shot mode is subsumed**. The no-good loop's round 1 *is* a one-shot attempt,
  so a run whose trace certifies at round 1 is exactly a run the one-shot mode would have
  certified. The `nogood_rounds` trace answers Study 9's question for free.
- the **K = n control is unaffordable and no longer load-bearing**. It makes the relaxed
  search cost exactly what the real one costs, which at n=30 is the thing being avoided.
  Study 9 needed it to disambiguate "partition too coarse" from "budget too small"; the
  no-good loop reports that split directly, per round, as
  refuted/inconclusive plus the per-round cut trace.

Instance family, `max_stops`, scenario count and pair count all match Studies 7/8 at n=20
so the walls are comparable to measurements already in hand.

# Budgets

Per-round pricing budgets are Studies 5/7's (300 s regular, 3600 s certifying); the total
is 4 h at n=20 (Study 7's) and 6 h at n=30 (Study 5's scaling ceiling).

`certification_time_limit_sec` is the one genuinely new knob, and it is deliberately much
larger than Study 9's 60 s: a no-good round pays one *exhaustive* subset search per cut it
places, so 60 s cannot walk more than a barren support or two at these sizes. The cost
profile makes a large budget affordable -- an attempt in an early CG iteration is refuted
by the first subset search it runs (there really are improving columns then) and exits
immediately, so the budget is only ever spent near convergence, which is the round it is
trying to replace. `non_certifying_certification_sec` measures where that time went -- but
note it is NOT overhead once harvesting is on, since a refuted attempt returns columns and
displaces a pricing round; pair it with `certification_harvested_columns`.
"""

const SEEDS = 42:46
const N_PAIRS = 16
const N_SCENARIOS = 3
const MAX_STOPS = 10
# K as a share of n, so an arm means the same thing at both sizes. n=15's measured working
# points (K=9, K=12) sit at 0.6 and 0.8; 0.4 probes whether more stations buy tolerance for
# a coarser partition.
const K_FRACTIONS = (0.4, 0.6, 0.8)
const PRICING_TIME_LIMIT_SEC = 300.0
const CERTIFYING_TIME_LIMIT_SEC = 3600.0
# Cap on cuts per scenario per attempt, raised from CGSolver's default of 32 to the 64 the
# UInt64 satisfied-mask allows. MEASURED binding at n=30 in the 2026-09-05 run: K=18 hit
# exactly 32 and K=24 hit 31, so some attempts were cut off mid-loop and reported
# `inconclusive` when more rounds might have certified. Leaving it at 32 would measure n=30
# through a known artefact. NOTE this changes alongside the parallel round, so a comparison
# against that run confounds the two; `nogood_max_rounds` says whether the cap still binds.
const CERTIFICATION_MAX_ROUNDS = 64

# (total budget, per-attempt certification budget) per size. n=20 inherits Study 7's 4 h
# total; n=30 inherits Study 5's 6 h scaling ceiling. The certification budget scales with
# it because a no-good attempt's expensive half is an exhaustive subset search, which is
# what grows with n.
#
# **n=25 is APPENDED, not inserted, and that ordering is load-bearing.** Job ids are
# assigned by position here and are the array indices the submit script maps through
# `sed -n "$((TASK + 1))p"`. n=25 was added while the n=20/n=30 arrays were already live on
# a preemptable partition, where a requeued task re-reads this file at restart -- so
# inserting n=25 in size order would have silently handed running n=30 tasks n=25 rows.
# Appending keeps rows 1-40 byte-identical:
#     1-20  n=20    21-40  n=30    41-60  n=25
#
# n=25 exists because the n=20 half could not test the feature's actual claim: baseline's
# `certifying_rounds` was 0 on every n=20 cell, so there was no expensive two-tier
# escalation for certification to replace and every arm was pure overhead by construction.
# n=25 at s=3 sits inside the previously measured frontier (which certified n=25 up to
# s<=5), so baseline should still certify -- but it is the size where it plausibly starts
# needing that escalation, which is the only regime where "cheaper than the certifying
# round" is a measurable question.
const SIZES = (
    (n_stations = 20, total_time_limit_sec = 14400.0, certification_time_limit_sec = 600.0),
    (n_stations = 30, total_time_limit_sec = 21600.0, certification_time_limit_sec = 1200.0),
    (n_stations = 25, total_time_limit_sec = 18000.0, certification_time_limit_sec = 900.0),
)

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
header = ("job_id", "cell_id", "arm", "n_clusters", "k_fraction", "n_stations", "n_pairs",
    "n_scenarios", "seed", "max_stops", "n_threads", "time_limit_sec",
    "certifying_time_limit_sec", "total_time_limit_sec", "certification_time_limit_sec",
    "certification_max_rounds")

outpath = joinpath(config_dir, "jobs.tsv")
job_id = Ref(0)
open(outpath, "w") do io
    println(io, join(header, '\t'))
    # Size varies slowest, then arm, then seed: each size is one contiguous 20-job --array
    # range and each arm within it a contiguous 5-job range, so the cheap n=20 half (or a
    # single arm of it) can be submitted and read before committing the n=30 half.
    for size in SIZES
        n = size.n_stations
        # `n_clusters = 0` encodes the baseline arm (no partition built, feature off), so
        # every row keeps one schema and the arm is just another value of the parameter.
        cluster_counts = (0, (round(Int, f * n) for f in K_FRACTIONS)...)
        for n_clusters in cluster_counts, seed in SEEDS
            job_id[] += 1
            arm = n_clusters == 0 ? "baseline" : "nogood_k$(n_clusters)"
            k_fraction = n_clusters == 0 ? 0.0 : round(n_clusters / n; digits = 2)
            cell_id = "n$(n)_p$(N_PAIRS)_s$(N_SCENARIOS)_ms$(MAX_STOPS)_seed$(seed)"
            println(io, join((job_id[], cell_id, arm, n_clusters, k_fraction, n, N_PAIRS,
                N_SCENARIOS, seed, MAX_STOPS, N_SCENARIOS, PRICING_TIME_LIMIT_SEC,
                CERTIFYING_TIME_LIMIT_SEC, size.total_time_limit_sec,
                size.certification_time_limit_sec, CERTIFICATION_MAX_ROUNDS), '\t'))
        end
    end
end
println("Wrote $(job_id[]) jobs to $outpath " *
        "($(length(SIZES)) sizes x $(length(K_FRACTIONS) + 1) arms x $(length(SEEDS)) seeds)")
# Ranges are printed from SIZES rather than hardcoded, so appending another size cannot
# leave this hint stale (and the array ranges it prints are the ones to submit).
let start = 1
    for size in SIZES
        stop = start + (length(K_FRACTIONS) + 1) * length(SEEDS) - 1
        println("  n=$(size.n_stations): --array=$start-$stop  " *
                "(total $(round(Int, size.total_time_limit_sec / 3600)) h, " *
                "certification $(round(Int, size.certification_time_limit_sec)) s/attempt)")
        start = stop + 1
    end
end
