"""Why is `:station_simple` pricing slower than `:exact` on the Study 8 cells?

Prices ONE identical round both ways -- same master, same duals, same pool -- and reports
(a) the label-search counters each pricer records and (b) a flat self-time profile.

Design notes that matter for trusting the output:

- **Same duals, not just same instance.** The two pricers are run back to back against one
  frozen master state, so the comparison cannot be confounded by the arms having diverged
  to different pools/duals by the time they are measured -- which is exactly what a
  whole-run comparison like Study 8 cannot control for.
- **Single-threaded on purpose.** `parallel_scenario_pricing=false` and 1 Julia thread, so
  the profile contains no idle `poptask` samples. With N threads those idle samples are a
  fixed fraction of every capture and silently divide every reported share; sidestepping
  them beats filtering them.
- **Warm up before profiling.** Each pricer is called once untimed so JIT compilation is
  not attributed to the search.
- Counters come from `m[:label_setting_pricing_stats]`, which the round records per
  (iteration x scenario) search; they are exact, not sampled, so they are the primary
  evidence and the profile is the explanation for them.

Usage:
    julia --project=. benchmarks/diagnostics/profile_station_simple_vs_exact.jl [n_pairs] [seed] [warmup_iters]
Defaults: n_pairs=16, seed=51, warmup_iters=6  (Study 8's slowest p=16 cell, 0.78x).
"""

using StationSelection
using JuMP
using Printf
using Profile
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N_PAIRS = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 16
const SEED = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 51
const WARMUP_ITERS = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 6
const N_STATIONS, N_SCENARIOS, MAX_STOPS = 20, 3, 10

Threads.nthreads() == 1 || @warn "expected 1 Julia thread; idle samples may skew shares" nthreads=Threads.nthreads()

problem, k, _ = benchmark_problem(@__DIR__, "PROFILE", N_STATIONS, N_PAIRS, N_SCENARIOS, SEED)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=MAX_STOPS, pricing_mode=:exact,
)
solver = StationSelection.CGSolver(
    config=StationSelection.SolverOptions(silent=true),
    pricing_time_limit_sec=900.0, certifying_pricing_time_limit_sec=900.0,
    parallel_scenario_pricing=false,
)
build_result = StationSelection.build_model(problem, formulation, solver)
m = build_result.model
mapping = build_result.mapping
StationSelection._apply_solver_config!(m, solver.config)

@printf("instance n=%d p=%d s=%d seed=%d k=%d  max_stops=%d\n",
        N_STATIONS, N_PAIRS, N_SCENARIOS, SEED, k, MAX_STOPS)
flush(stdout)

# ── warm the master to a realistic dual point ────────────────────────────────
# A round priced against the seed pool's duals is not representative: early duals make
# almost everything attractive, so both pricers look fast and similar. Warming up puts the
# comparison at duals CG actually spends its time on.
for it in 1:WARMUP_ITERS
    optimize!(m)
    JuMP.termination_status(m) == MOI.OPTIMAL || (println("master not optimal at $it"); break)
    duals = StationSelection.extract_duals(build_result, mapping, m)
    cols = StationSelection.price_columns(build_result, mapping, m, duals, solver;
                                          time_limit_sec=900.0)
    (isnothing(cols) || isempty(cols)) && (println("converged during warmup at $it"); break)
    StationSelection.add_columns!(build_result, mapping, m, cols)
end
optimize!(m)
@printf("after %d warmup iterations: pool=%d  master obj=%.4f\n\n",
        WARMUP_ITERS, length(m[:joint_routing_assignment_columns]), JuMP.objective_value(m))
flush(stdout)

# One dual vector, reused by both pricers.
duals = StationSelection.extract_duals(build_result, mapping, m)

function price_once(mode::Symbol)
    m[:joint_routing_assignment_pricing_mode] = mode
    empty!(get!(JuMP.object_dictionary(m), :label_setting_pricing_stats, Any[]))
    t = time()
    cols = StationSelection.price_columns(build_result, mapping, m, duals, solver;
                                          time_limit_sec=900.0)
    wall = time() - t
    stats = get(JuMP.object_dictionary(m), :label_setting_pricing_stats, Any[])
    agg(f, init) = sum((f(s) for s in stats); init=init)
    return (
        wall = wall,
        n_columns = isnothing(cols) ? 0 : length(cols),
        best_rc = (isnothing(cols) || isempty(cols)) ? NaN :
                  minimum(Float64(get(c.metadata, "reduced_cost", NaN)) for c in cols),
        searches = length(stats),
        generated = agg(s -> s.labels_generated, 0),
        rejected = agg(s -> s.labels_rejected_by_dominance, 0),
        removed = agg(s -> s.labels_removed_by_dominance, 0),
        max_frontier = maximum((s.max_frontier_size for s in stats); init=0),
        max_live = maximum((s.max_live_labels for s in stats); init=0),
        search_sec = agg(s -> Float64(s.search_sec), 0.0),
    )
end

# JIT warmup for BOTH pricers before any measurement.
price_once(:station_simple); price_once(:exact)

results = Dict{Symbol, Any}()
for mode in (:exact, :station_simple)
    results[mode] = price_once(mode)
end

println("Counters for ONE pricing round at identical duals")
println("-"^100)
@printf("%-16s %9s %9s %12s %12s %12s %11s %11s\n",
        "mode", "wall_s", "columns", "generated", "rej_by_dom", "removed", "max_front", "max_live")
for mode in (:exact, :station_simple)
    r = results[mode]
    @printf("%-16s %9.2f %9d %12d %12d %12d %11d %11d\n",
            mode, r.wall, r.n_columns, r.generated, r.rejected, r.removed, r.max_frontier, r.max_live)
end
e, s = results[:exact], results[:station_simple]
println()
@printf("station_simple / exact:  wall %.2fx   labels generated %.2fx   frontier %.2fx   dominance rejections %.2fx\n",
        s.wall / max(e.wall, 1e-9), s.generated / max(e.generated, 1),
        s.max_frontier / max(e.max_frontier, 1), s.rejected / max(e.rejected, 1))
@printf("best reduced cost:  exact %.4f   station_simple %.4f\n\n", e.best_rc, s.best_rc)
flush(stdout)

# ── flat self-time profile per mode ──────────────────────────────────────────
function profile_mode(mode::Symbol, label::AbstractString)
    Profile.clear()
    Profile.init(n = 10_000_000, delay = 0.001)
    @profile price_once(mode)
    data, lidict = Profile.retrieve()
    counts = Dict{String, Int}()
    total = 0
    # `data` is a flat array of instruction pointers with backtraces separated by 0, each
    # ordered LEAF-FIRST. Self time therefore means: for every backtrace, credit only its
    # first non-C frame. Iterating `data` without honouring the separators credits every
    # frame on every stack instead, which reports *inclusive* time -- the whole call chain
    # then shows an identical share, which is how the earlier version of this script was
    # wrong.
    at_leaf = true
    for ip in data
        if ip == 0
            at_leaf = true   # separator: next ip starts a new backtrace
            continue
        end
        at_leaf || continue
        frames = get(lidict, ip, nothing)
        frames === nothing && continue
        for fr in frames
            fr.from_c && continue
            key = "$(fr.func) @ $(basename(String(fr.file))):$(fr.line)"
            counts[key] = get(counts, key, 0) + 1
            total += 1
            at_leaf = false
            break
        end
    end
    println("Top self-time frames — $label (total leaf samples: $total)")
    println("-"^100)
    for (k, v) in first(sort(collect(counts), by = last, rev = true), 18)
        @printf("  %6.2f%%  %6d  %s\n", 100 * v / max(total, 1), v, k)
    end
    println()
    flush(stdout)
end

profile_mode(:exact, "pricing_mode=:exact")
profile_mode(:station_simple, "pricing_mode=:station_simple")

# ── dominance rejection census ───────────────────────────────────────────────
# Which condition was FIRST to reject each tested pair. This is the measurement the
# "cheapest-and-likeliest-to-reject first" ordering claim in both dominance docstrings
# has to be justified against. Instrumentation is a type-parameter specialization, so
# this run compiles a *different* predicate than production: use it for the rejection
# distribution, never for wall-clock comparison against the numbers above.
function census(mode::Symbol, label::AbstractString)
    scenarios = StationSelection._pricing_scenarios(
        m[:aggregate_od_route_formulation], mapping, m)
    s = first(scenarios)
    built = StationSelection._pricing_build_scenario_context(
        m[:aggregate_od_route_formulation], mapping, s, m, duals)
    built === nothing && (println("no context for scenario $s"); return)
    base_ctx, existing = built
    ctx = mode === :station_simple ?
        StationSelection.JointRoutingAssignmentStationSimpleSearchContext(
            base_ctx.pricing_data; dominance_census=true) :
        StationSelection.JointRoutingAssignmentSearchContext(
            base_ctx.pricing_data; dominance_census=true)
    StationSelection.joint_routing_assignment_dominance_rejections(; reset=true)
    StationSelection._run_label_setting(ctx, existing; time_limit=900.0,
        max_new_columns=typemax(Int), n_candidates=typemax(Int), next_column_id=1)
    counts = StationSelection.joint_routing_assignment_dominance_rejections()
    tested = sum(last, counts)
    println("Dominance rejection census — $label (scenario $s, tested pairs: $tested)")
    println("-"^100)
    for (cond, n) in counts
        @printf("  %-20s %12d  %6.2f%%\n", cond, n, 100 * n / max(tested, 1))
    end
    println()
    flush(stdout)
end

try
    census(:exact, "pricing_mode=:exact")
    census(:station_simple, "pricing_mode=:station_simple")
catch err
    println("census skipped: ", sprint(showerror, err))
end
