using StationSelection
using CSV
using DataFrames
using JuMP
using Printf
using Statistics
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

length(ARGS) == 1 || error("usage: run_benchmark.jl '<tab-separated jobs.tsv row>'")
f = split(strip(ARGS[1]), '\t')
length(f) == 16 || error("expected 16 fields, got $(length(f))")
job_id = parse(Int, f[1])
phase, arm = f[2], f[3]
n, p, s, seed, max_stops, n_threads, k0, guide_routes = parse.(Int, f[4:11])
pricing_limit, certifying_limit, total_limit = parse.(Float64, f[12:14])
# `ip_every` = re-solve the CURRENT restricted pool as a MIP every this-many iterations
# (1 = every iteration, 0 = never). See the header comment on `_ip_snapshot` for why that
# solve is safe to run mid-loop, and the README for the wall-clock caveat it carries.
ip_every = parse(Int, f[15])
ip_time_limit = parse(Float64, f[16])

arm in ("relaxed_k60", "relaxed_k80", "exact") || error("unknown arm $arm")
Threads.nthreads() == n_threads || error(
    "job requests $n_threads Julia threads, process has $(Threads.nthreads())")
(arm == "exact") == (k0 == 0) || error("exact iff cluster_count=0")
ip_every >= 0 || error("ip_every must be non-negative")

# Two tolerances on purpose. 1e-6 is "the solver calls this integral"; 1e-3 is the band a
# human reading a solution would call integral. A variable at 0.999 is fractional at 1e-6
# and is not a fractional solution in any practical sense, so reporting only the tight
# tolerance would make a near-integral LP look shattered.
const TOL_TIGHT = 1e-6
const TOL_LOOSE = 1e-3

# Distance from each value to the nearer of {0,1}. A value above 1 -- which `theta` permits,
# since `add_joint_routing_assignment_column!` creates it with `lower_bound = 0.0` and NO
# upper bound, the coverage rows `sum(theta) + x_walk >= 1` being what hold it near 1 --
# contributes `v - 1` here and is additionally counted by `_binary_stats`'s `n_above_one`.
_dists(values) = Float64[min(abs(v), abs(v - 1.0)) for v in values]

"""
    _binary_stats(values, prefix) -> NamedTuple

Distance-to-{0,1} summary of one master variable family, as `prefix`-named columns.

`frac_share`'s denominator is the SUPPORT, not the variable count: a pool of 5000 columns
of which 4990 sit at exactly 0 would otherwise report ~100% integral no matter how
shattered the active ones are.
"""
function _binary_stats(values::Vector{Float64}, prefix::Symbol)
    n = length(values)
    n == 0 && return (;
        Symbol(prefix, :_n) => 0, Symbol(prefix, :_n_support) => 0,
        Symbol(prefix, :_n_frac) => 0, Symbol(prefix, :_n_frac_loose) => 0,
        Symbol(prefix, :_frac_share) => NaN, Symbol(prefix, :_sum_dist) => 0.0,
        Symbol(prefix, :_max_dist) => 0.0, Symbol(prefix, :_sum) => 0.0,
        Symbol(prefix, :_n_above_one) => 0)
    dist = _dists(values)
    n_support = count(v -> v > TOL_TIGHT, values)
    return (;
        Symbol(prefix, :_n) => n,
        Symbol(prefix, :_n_support) => n_support,
        Symbol(prefix, :_n_frac) => count(d -> d > TOL_TIGHT, dist),
        Symbol(prefix, :_n_frac_loose) => count(d -> d > TOL_LOOSE, dist),
        Symbol(prefix, :_frac_share) =>
            n_support == 0 ? NaN : count(d -> d > TOL_TIGHT, dist) / n_support,
        Symbol(prefix, :_sum_dist) => sum(dist),
        Symbol(prefix, :_max_dist) => maximum(dist),
        Symbol(prefix, :_sum) => sum(values),
        Symbol(prefix, :_n_above_one) => count(v -> v > 1.0 + TOL_TIGHT, values))
end

problem, selection_k, instance_meta = benchmark_problem(@__DIR__, "STUDY11", n, p, s, seed)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops)
pricing = arm == "exact" ? CGPricingConfig(mode=:exact) :
    CGPricingConfig(mode=:relaxed_cluster, relaxed_cluster_count=k0,
                    relaxed_cluster_guide_routes=guide_routes)

outdir = benchmark_output_dir(@__DIR__, "STUDY11", "study11_master_lp_integrality")
# `max_stops` is IN THE STEM because it is a swept parameter: without it the ms=10 and
# ms=14 rows of n30_maxstops.tsv are the same seed and the same arm, so they write the
# same filename and whichever finishes last silently overwrites the other. MEASURED
# 2026-09-09: 4 completed tasks of array 22417454 collapsed into 2 result files exactly
# this way. Any parameter two rows of a table differ in has to reach the filename.
stem = "n$(n)_seed$(seed)_$(arm)_ms$(max_stops)"
iter_dir = joinpath(outdir, "iterations")
yval_dir = joinpath(outdir, "y_values")
mkpath(iter_dir)
mkpath(yval_dir)
iter_path = joinpath(iter_dir, "$(stem).csv")
yval_path = joinpath(yval_dir, "$(stem).csv")
result_path = joinpath(outdir, "$(stem).csv")
# `.progress.csv` deliberately does NOT match the `n<N>_seed<S>_<arm>.csv` pattern
# `analyze.jl` globs, so an in-flight run is visible to a human without being swept into an
# analysis as if it were a finished result.
progress_path = joinpath(outdir, "$(stem).progress.csv")
for path in (iter_path, yval_path, result_path, progress_path)
    isfile(path) && rm(path)
end

const IDENTITY = (job_id=job_id, phase=phase, arm=arm, n_stations=n, n_pairs=p,
                  n_scenarios=s, seed=seed, cluster_count=k0, selection_k=selection_k,
                  max_stops=max_stops)

# The snapshot callback needs the model the loop is solving, but that model only exists
# after `build_model`, which needs the solver the callback is attached to. One `Ref` filled
# in right after the build breaks the cycle; the alternative -- build with one CGSolver and
# optimize with a second differing only in the callback -- is two objects that have to be
# kept identical by hand.
const BR = Ref{Any}(nothing)
const SNAPSHOTS = NamedTuple[]
const STARTED = Ref(0.0)

"""
    _ip_snapshot() -> (objective, status, seconds)

The CURRENT restricted column pool re-solved as a MIP, i.e. the integrality gap of the
restricted master at this iteration.

`integer_recovery_build` builds a FRESH model (`y`/`theta`/`x_walk` all binary) seeded with
exactly the pool the live master holds, and reads no primal values off it, so calling it
mid-loop cannot disturb the CG run. It is the same rebuild `CGSolver` performs once at the
end under `recover_integer_solution`, which is what makes the per-iteration numbers
directly comparable to the run's own final `lp_objective` vs `objective_value`.
"""
function _ip_snapshot()
    t0 = time()
    br = BR[]
    ip_br = StationSelection.integer_recovery_build(br, br.mapping, br.model)
    ip_m = ip_br.model
    set_silent(ip_m)
    set_optimizer_attribute(ip_m, "TimeLimit", ip_time_limit)
    set_optimizer_attribute(ip_m, "Threads", 1)
    optimize!(ip_m)
    st = termination_status(ip_m)
    obj = result_count(ip_m) > 0 ? objective_value(ip_m) : NaN
    # Model and Gurobi environment are per-call; drop them promptly rather than letting one
    # accumulate per snapshot across a multi-hour run.
    ip_m = nothing
    ip_br = nothing
    GC.gc()
    return obj, string(st), time() - t0
end

"""
    on_master_solved(iteration, duals)

`CGSolver.dual_callback`: fires immediately after this iteration's master `optimize!` and
before its pricing round. That is the only point in the loop where the LP solution is on
the model and pricing has not yet added variables on top of it -- reading the same values
from `iteration_callback` would read a solution `add_columns!` has since invalidated.
"""
function on_master_solved(iteration::Int, _duals)
    br = BR[]
    m = br.model
    theta_dict = m[:joint_routing_assignment_theta]
    columns_dict = m[:joint_routing_assignment_columns]
    y_vars = m[:y]
    x_walk = m[:x_walk]

    lp_obj = objective_value(m)
    yv = Float64[value(y_vars[j]) for j in eachindex(y_vars)]
    wv = Float64[value(v) for v in values(x_walk)]
    tv = Float64[value(v) for v in values(theta_dict)]
    worst = maximum(vcat(_dists(yv), _dists(wv), _dists(tv), [0.0]))

    # Per-scenario theta mass. Every demand group carries `sum(theta) + x_walk >= 1`, so a
    # scenario whose active-column count runs well past its demand-group count is buying
    # coverage fractionally from several routes at once -- the shape a shattered LP takes
    # in this formulation.
    scen_support = zeros(Int, s)
    scen_mass = zeros(Float64, s)
    for (cid, var) in theta_dict
        v = value(var)
        v > TOL_TIGHT || continue
        sc = Int(columns_dict[cid].metadata["scenario"])
        scen_support[sc] += 1
        scen_mass[sc] += v
    end

    # Does the `max_stops` cap actually BIND? A cap that bites shows up as a persistent
    # FINAL LP/IP gap (the structural gap of
    # notes/2026-07-25_lp_ip_gap_structural_vs_pool_completeness_final.md, which raising
    # max_stops fixed), so a run ending at 0% is already evidence it does not. This records
    # it directly instead of inferring it: `column.route` is the physical stop sequence, so
    # `n_at_cap` counts pool columns sitting exactly on the cap. 0 means the cap is
    # slack and cannot be causing anything observed here.
    stops = Int[length(c.route) for c in values(columns_dict)]
    col_max_stops = isempty(stops) ? 0 : maximum(stops)
    col_median_stops = isempty(stops) ? 0.0 : median(stops)
    col_n_at_cap = count(==(max_stops), stops)

    ip_obj, ip_status, ip_sec = (NaN, "skipped", 0.0)
    if ip_every > 0 && (iteration == 1 || iteration % ip_every == 0)
        try
            ip_obj, ip_status, ip_sec = _ip_snapshot()
        catch err
            ip_status = "error: " * sprint(showerror, err)
        end
    end

    row = (; IDENTITY...,
        iteration=iteration,
        elapsed_sec=round(time() - STARTED[]; digits=1),
        lp_objective=lp_obj,
        n_columns=length(theta_dict),
        _binary_stats(yv, :y)...,
        _binary_stats(wv, :walk)...,
        _binary_stats(tv, :theta)...,
        col_max_stops=col_max_stops,
        col_median_stops=col_median_stops,
        col_n_at_cap=col_n_at_cap,
        worst_dist=worst,
        all_integral_tight=(worst <= TOL_TIGHT),
        all_integral_loose=(worst <= TOL_LOOSE),
        theta_scen_support=join(scen_support, "|"),
        theta_scen_mass=join([@sprintf("%.4f", x) for x in scen_mass], "|"),
        ip_objective=ip_obj,
        ip_status=ip_status,
        ip_sec=round(ip_sec; digits=2),
        ip_gap_pct=(isnan(ip_obj) || ip_obj == 0.0) ? NaN :
                    100.0 * (ip_obj - lp_obj) / abs(ip_obj),
    )
    push!(SNAPSHOTS, row)

    # Streamed after every iteration: a preempted task still leaves its per-iteration
    # history behind rather than being unobservable while it runs and unrecoverable if it
    # dies.
    fresh = !isfile(iter_path)
    CSV.write(iter_path, DataFrame([row]); append=!fresh, writeheader=fresh)
    yrows = [(; IDENTITY..., iteration=iteration, station_idx=j, y_value=yv[j])
             for j in eachindex(yv)]
    yfresh = !isfile(yval_path)
    CSV.write(yval_path, DataFrame(yrows); append=!yfresh, writeheader=yfresh)
    CSV.write(progress_path, DataFrame([(; IDENTITY..., status="in_progress",
        iterations_done=iteration, elapsed_sec=round(time() - STARTED[]; digits=1),
        lp_objective=lp_obj, n_columns=length(theta_dict), worst_dist=worst,
        total_limit_sec=total_limit)]))

    @printf("it %4d  lp=%12.4f  cols=%5d  y_frac=%2d/%2d  walk_frac=%3d  theta_frac=%4d/%4d  worst=%.6f  ip_gap=%s\n",
            iteration, lp_obj, length(theta_dict),
            row.y_n_frac, row.y_n, row.walk_n_frac, row.theta_n_frac, row.theta_n_support,
            worst, isnan(row.ip_gap_pct) ? "-" : @sprintf("%.4f%%", row.ip_gap_pct))
    flush(stdout)
    return nothing
end

solver = benchmark_cg_solver(pricing_limit;
    recover_integer_solution=true, threads=1,
    certifying_pricing_time_limit_sec=certifying_limit,
    total_time_limit_sec=total_limit,
    parallel_scenario_pricing=true, pricing=pricing,
    dual_callback=on_master_solved)

BR[] = build_model(problem, formulation, solver)
reason = StationSelection.check_feasibility(problem, formulation, solver)
isnothing(reason) || error("instance refuted before solve: $reason")

STARTED[] = time()
# `run_opt` = build + gate + optimize, and this study needs the model object BETWEEN the
# build and the solve so the callback can read it -- hence the three steps by hand.
# Only `run_opt`/`build_model` are exported; `optimize_model` and `check_feasibility`
# are package-internal and must be qualified.
result = StationSelection.optimize_model(BR[], solver)

md = result.metadata
lp_obj = get(md, "cg_lp_objective_value", missing)
frac_shares = [r.theta_frac_share for r in SNAPSHOTS if !isnan(r.theta_frac_share)]
ip_rows = [r for r in SNAPSHOTS if !isnan(r.ip_gap_pct)]
row = (; IDENTITY...,
    pricing_limit_sec=pricing_limit, certifying_limit_sec=certifying_limit,
    total_limit_sec=total_limit, ip_every=ip_every, ip_time_limit_sec=ip_time_limit,
    iterations=get(md, "cg_iterations", missing),
    stop_reason=get(md, "cg_stop_reason", missing),
    converged=get(md, "cg_converged", missing),
    certified_by_relaxation=get(md, "cg_certified_by_relaxation", missing),
    optimality_scope=get(md, "cg_optimality_scope", missing),
    termination_status=string(result.termination_status),
    lp_objective=lp_obj,
    ip_objective=result.objective_value,
    final_gap_pct=(ismissing(lp_obj) || result.objective_value == 0) ? missing :
        100.0 * (result.objective_value - lp_obj) / abs(result.objective_value),
    runtime_sec=round(result.runtime_sec; digits=1),
    # Wall spent inside the probe's own MIP snapshots. It comes OUT of the CG loop's
    # `total_time_limit_sec` (the callback runs inside the loop), so this column is what
    # makes a Study 11 runtime comparable to a Study 9 one -- see the README.
    ip_snapshot_sec=round(sum(Float64[r.ip_sec for r in SNAPSHOTS]; init=0.0); digits=1),
    snapshots=length(SNAPSHOTS),
    pool_max_stops=isempty(SNAPSHOTS) ? missing : maximum(r.col_max_stops for r in SNAPSHOTS),
    pool_n_at_cap=isempty(SNAPSHOTS) ? missing : maximum(r.col_n_at_cap for r in SNAPSHOTS),
    # How often the master LP came out integral on its own, over the whole run.
    integral_iterations_tight=count(r -> r.all_integral_tight, SNAPSHOTS),
    integral_iterations_loose=count(r -> r.all_integral_loose, SNAPSHOTS),
    mean_y_frac=isempty(SNAPSHOTS) ? missing : mean(Float64(r.y_n_frac) for r in SNAPSHOTS),
    max_y_frac=isempty(SNAPSHOTS) ? missing : maximum(r.y_n_frac for r in SNAPSHOTS),
    mean_theta_frac_share=isempty(frac_shares) ? missing : mean(frac_shares),
    mean_ip_gap_pct=isempty(ip_rows) ? missing : mean(r.ip_gap_pct for r in ip_rows),
    max_ip_gap_pct=isempty(ip_rows) ? missing : maximum(r.ip_gap_pct for r in ip_rows),
    final_ip_gap_pct=isempty(ip_rows) ? missing : ip_rows[end].ip_gap_pct,
)
CSV.write(result_path, DataFrame([row]))
isfile(progress_path) && rm(progress_path)

println("\n=== job $job_id : $stem ===")
for (k, v) in pairs(row)
    println(rpad(string(k), 28), v)
end
println("\nwrote:\n  $result_path\n  $iter_path\n  $yval_path")
