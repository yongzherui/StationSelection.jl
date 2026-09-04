"""Generate Study 9's jobs: does the relaxed-cluster relaxation ever certify, and at
which cluster count?

# The question

CG's expensive step is not finding columns, it is *proving there are no more*. Study 5's
iteration logs put 99.86% of in-loop time in pricing, of which the two-tier escalation to
the certifying round is a measured 8.2% pure re-pricing tax, and the certifying share
reaches 77.7% at n=30. `CGSolver(certification_pricing_mode=:relaxed_cluster)` attacks
exactly that: before each real pricing round it asks a *relaxation* of the pricing problem
whether an improving column can still exist. A `no` is a full-route-universe optimality
certificate obtained without any exhaustive search; a `yes` proves nothing and costs only
the early-exit it took to find one improving relaxed route.

So there are two things to measure, and one arm measures both:

1. **Does it ever certify at all?** (`certified_by_relaxation`.) A relaxation that never
   fires is dead weight; this is the go/no-go.
2. **At which `K`?** The cluster count is the entire tightness knob. Small `K` makes the
   relaxed search trivially cheap and hopelessly loose; `K = n` makes it exactly as
   expensive as the real search and exactly as tight. The useful `K`, if one exists, is in
   between, and its location is what this sweep is for.

# The grid

One cell size, one difficulty, five seeds: **n=15, p=16, s=3, seeds 42-46**. This is a
probe, not a measurement study -- the question is whether the relaxation ever fires, and
30 jobs answer that as well as 180 would while finishing in minutes rather than hours.

Budgets are sized off Study 3's measured walls on this exact instance size (n=15, p=16,
ms=10, s=1): 7-154 s, median ~26 s, every cell certified. Tripling the scenarios does not
triple the wall under `parallel_scenario_pricing` (the round's wall is the max over
scenarios, not the sum), but it does add CG iterations, so the 1800 s total budget leaves
roughly an order of magnitude of headroom over the measured median and still caps a job at
30 minutes. If cells start coming back `budget_exhausted`, that budget -- not the grid --
is the thing to raise.

`n_scenarios = 3` is deliberately NOT reduced to 1 even though it is the largest remaining
cost multiplier: a certification round has to certify *every* scenario before CG may stop,
so a single-scenario study would measure a degenerate case of the very thing being tested.

# Why K = n is an arm and not redundant with the baseline

`K = 0` is the baseline (feature off: CG stops via exhausted pricing or the two-tier
certifying round). `K = 15 = n` is the *identity* partition, where the relaxation coincides
with the exact pricing graph arc for arc -- so it is "run the exact pricer as a cheap
early-exit probe, then run it again for real", which is a different run from the baseline
and strictly slower than it.

It earns its 5 jobs as the **ceiling control**, because it separates the two ways a K can
fail. If K=15 certifies and K=6 does not, K=6's partition was too coarse and some finer
partition might do. If even K=15 does not certify, `certification_time_limit_sec` was the
binding constraint and no partition would have worked at that budget. Without that control
a sweep where nothing certifies cannot be read at all.
`cg_certification_refuted_rounds` vs `cg_certification_inconclusive_rounds` records which
of the two happened per run.

Note the arms are **not** a monotone ladder. Tightness improves under partition
*refinement*, but two independent k-medoids runs at different K need not be nested, so a
larger K is not guaranteed to give a tighter bound. The only ordering that always holds is
that K = n is refined by nothing and is therefore tightest -- which is exactly why it, and
not "the largest K that certified", is the control.
"""

const SEEDS = 42:46
const N_STATIONS = 15
const N_SCENARIOS = 3
const N_PAIRS = 16
const MAX_STOPS = 10
# Sized off Study 3's n=15/p=16/ms=10 walls (7-154 s at s=1, all certified), not inherited
# from Study 7's n=20 grid: the point of this study is a fast answer, and a 4 h cap on a
# 30 s instance only buys queue time.
const PRICING_TIME_LIMIT_SEC = 120.0
const CERTIFYING_TIME_LIMIT_SEC = 600.0
const TOTAL_TIME_LIMIT_SEC = 1800.0
# Budget for one relaxed certification attempt. Deliberately much smaller than the
# certifying round it is trying to replace: the point is a cheap try. A K that cannot
# settle in 60 s is not an operating point -- and if even K = n cannot, that is what
# `cg_certification_inconclusive_rounds` will say.
const CERTIFICATION_TIME_LIMIT_SEC = 60.0
# `n_clusters = 0` encodes the baseline arm (no certification pricer, no clustering built),
# so every row keeps one schema and the arm is just another value of the swept parameter.
# `15` is the identity partition -- the ceiling control, see the module docstring.
const CLUSTER_COUNTS = (0, 3, 6, 9, 12, 15)

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
header = ("job_id", "cell_id", "arm", "n_clusters", "n_stations", "n_pairs", "n_scenarios",
    "seed", "max_stops", "n_threads", "time_limit_sec", "certifying_time_limit_sec",
    "total_time_limit_sec", "certification_time_limit_sec")

outpath = joinpath(config_dir, "jobs.tsv")
job_id = Ref(0)
open(outpath, "w") do io
    println(io, join(header, '\t'))
    # Cluster count varies slowest, so each arm is one contiguous 5-job --array range and
    # can be submitted (or re-submitted) alone -- e.g. the baseline and the K = n ceiling
    # control first, to bracket what the middle arms can possibly achieve.
    for n_clusters in CLUSTER_COUNTS, seed in SEEDS
        job_id[] += 1
        arm = n_clusters == 0 ? "baseline" : "relaxed_k$(n_clusters)"
        cell_id = "n$(N_STATIONS)_p$(N_PAIRS)_s$(N_SCENARIOS)_ms$(MAX_STOPS)_seed$(seed)"
        println(io, join((job_id[], cell_id, arm, n_clusters, N_STATIONS, N_PAIRS,
            N_SCENARIOS, seed, MAX_STOPS, N_SCENARIOS, PRICING_TIME_LIMIT_SEC,
            CERTIFYING_TIME_LIMIT_SEC, TOTAL_TIME_LIMIT_SEC,
            CERTIFICATION_TIME_LIMIT_SEC), '\t'))
    end
end
println("Wrote $(job_id[]) jobs to $outpath ($(length(CLUSTER_COUNTS)) arms x " *
        "$(length(SEEDS)) seeds at n=$(N_STATIONS), p=$(N_PAIRS), s=$(N_SCENARIOS))")
