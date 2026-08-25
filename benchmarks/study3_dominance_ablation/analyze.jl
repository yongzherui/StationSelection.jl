"""Aggregate and pair Study 3 dominance-ablation jobs."""

using CSV
using DataFrames
using Dates
using Statistics

# LaTeX command names cannot contain digits, so n_stations is spelled out when it is
# baked into a row-macro name. Falls back to digit-by-digit for values not listed.
const LATEX_INT_WORDS = Dict(10 => "Ten", 15 => "Fifteen", 20 => "Twenty",
    25 => "TwentyFive", 30 => "Thirty", 35 => "ThirtyFive", 40 => "Forty")
const LATEX_DIGIT_WORDS = ("Zero", "One", "Two", "Three", "Four", "Five", "Six",
    "Seven", "Eight", "Nine")
latex_int_word(n::Integer) = get(LATEX_INT_WORDS, Int(n),
    join(LATEX_DIGIT_WORDS[d - '0' + 1] for d in string(n)))

1 <= length(ARGS) <= 2 || error("usage: analyze.jl RAW_DIR [RESULTS_DIR]")
raw_dir = abspath(ARGS[1])
project_root = normpath(joinpath(@__DIR__, "..", ".."))
results_dir = length(ARGS) == 2 ? abspath(ARGS[2]) :
    joinpath(project_root, "benchmarks", "results", "$(Dates.today())_study3_dominance_ablation")

files = sort(filter(path -> endswith(path, ".csv"), readdir(raw_dir; join=true)))
isempty(files) && error("no CSV result files found in $raw_dir")
cases = reduce(vcat, (CSV.read(path, DataFrame) for path in files))
bad_sizes = cases[
    cases.n_pairs_actual .!= cases.n_pairs .* cases.n_scenarios,
    [:job_id, :instance_id, :n_pairs, :n_scenarios, :n_pairs_actual, :pairs_per_scenario],
]
isempty(bad_sizes) || error("generated workload differs from configuration:\n$(sprint(show, bad_sizes))")
sort!(cases, [:instance_id, :compensated_dominance]; rev=[false, true])
mkpath(results_dir)
CSV.write(joinpath(results_dir, "case_results.csv"), cases)

keys = [:instance_id, :n_stations, :n_pairs, :n_scenarios, :seed, :k, :max_stops]
metrics = [:z_lp, :z_ip, :gap, :runtime_sec, :cg_iterations, :cg_converged,
    :cg_pricing_exhausted, :n_columns, :pricing_searches, :labels_generated,
    :labels_rejected_by_dominance, :labels_removed_by_dominance,
    :max_frontier_size, :max_live_labels]
function arm_frame(enabled::Bool, prefix::String)
    frame = select(cases[cases.compensated_dominance .== enabled, :], keys..., metrics...)
    rename!(frame, Dict(metric => Symbol(prefix, "_", metric) for metric in metrics))
    return frame
end

paired = innerjoin(arm_frame(true, "compensated"), arm_frame(false, "plain"); on=keys)
paired.both_certified = paired.compensated_cg_converged .&
    paired.compensated_cg_pricing_exhausted .& paired.plain_cg_converged .&
    paired.plain_cg_pricing_exhausted
# `z_lp` is the equivalence invariant: both dominance rules are exact, so run to
# exhaustion they must reach the same LP optimum. `z_ip` is the restricted-master
# heuristic over whichever pool each arm discovered, so a mismatch there is a real
# observation about pool quality, not a correctness failure -- recorded, never fatal.
# Tolerance: `atol` alone would set `rtol=0` in Julia (a pure absolute check). These
# objectives run to 1e3-1e4, so 1e-6 absolute demands ~1e-10 relative agreement --
# tighter than two independent solves over different column pools reliably deliver.
# `rtol=1e-9` adds proportional slack while `atol` still covers the near-zero case.
paired.lp_objective_delta = paired.compensated_z_lp .- paired.plain_z_lp
paired.lp_objective_match = [row.both_certified &&
    !ismissing(row.compensated_z_lp) && !ismissing(row.plain_z_lp) &&
    isapprox(row.compensated_z_lp, row.plain_z_lp; atol=1e-6, rtol=1e-9)
    for row in eachrow(paired)]
paired.ip_objective_delta = paired.compensated_z_ip .- paired.plain_z_ip
paired.ip_objective_match = [row.both_certified &&
    !ismissing(row.compensated_z_ip) && !ismissing(row.plain_z_ip) &&
    isapprox(row.compensated_z_ip, row.plain_z_ip; atol=1e-6, rtol=1e-9)
    for row in eachrow(paired)]
paired.runtime_ratio_plain_over_compensated =
    paired.plain_runtime_sec ./ paired.compensated_runtime_sec
paired.labels_ratio_plain_over_compensated =
    paired.plain_labels_generated ./ paired.compensated_labels_generated
paired.max_live_ratio_plain_over_compensated =
    paired.plain_max_live_labels ./ paired.compensated_max_live_labels
sort!(paired, :instance_id)
CSV.write(joinpath(results_dir, "paired_comparison.csv"), paired)

# Summaries are per (n_stations, dominance arm): the grid sweeps n, so pooling every
# size into one mean would average runtimes and label counts across instances of
# different difficulty.
summary_rows = NamedTuple[]
for n in sort(unique(cases.n_stations)), enabled in (true, false)
    sub = cases[(cases.n_stations .== n) .& (cases.compensated_dominance .== enabled), :]
    certified = sub[sub.cg_converged .& sub.cg_pricing_exhausted, :]
    push!(summary_rows, (
        n_stations=n, compensated_dominance=enabled, n_jobs=nrow(sub),
        n_certified=nrow(certified),
        mean_runtime_sec=isempty(certified) ? missing : mean(certified.runtime_sec),
        median_runtime_sec=isempty(certified) ? missing : median(certified.runtime_sec),
        mean_labels_generated=isempty(certified) ? missing : mean(certified.labels_generated),
        mean_max_live_labels=isempty(certified) ? missing : mean(certified.max_live_labels),
        mean_cg_iterations=isempty(certified) ? missing : mean(certified.cg_iterations),
        mean_n_columns=isempty(certified) ? missing : mean(certified.n_columns),
        mean_gap=isempty(certified) ? missing : mean(skipmissing(certified.gap)),
        max_gap=isempty(certified) ? missing : maximum(skipmissing(certified.gap)),
    ))
end
summary = DataFrame(summary_rows)
CSV.write(joinpath(results_dir, "variant_summary.csv"), summary)

certified_pairs = paired[paired.both_certified, :]
if any(.!certified_pairs.lp_objective_match)
    bad = certified_pairs[.!certified_pairs.lp_objective_match,
        [:instance_id, :compensated_z_lp, :plain_z_lp, :lp_objective_delta]]
    error("certified dominance arms disagree on the LP bound -- both rules are exact, " *
        "so this is a correctness failure:\n$(sprint(show, bad))")
end
if any(.!certified_pairs.ip_objective_match)
    @warn "certified arms agree on z_lp but differ on z_ip (expected: recovery is " *
        "pool-dependent, see benchmark_lp_ip)" n_pairs_differing=sum(.!certified_pairs.ip_objective_match)
end

open(joinpath(results_dir, "slides_results.tex"), "w") do io
    println(io, "% Generated by Study 3 analyze.jl")
    println(io, "% One macro per (dominance arm, n_stations) cell; means pool seeds only.")
    for row in eachrow(summary)
        arm = row.compensated_dominance ? "Compensated" : "Plain"
        name = "StudyThree$(arm)RowN$(latex_int_word(row.n_stations))"
        runtime = ismissing(row.mean_runtime_sec) ? "--" : string(round(row.mean_runtime_sec; digits=2))
        labels = ismissing(row.mean_labels_generated) ? "--" : string(round(row.mean_labels_generated; digits=0))
        live = ismissing(row.mean_max_live_labels) ? "--" : string(round(row.mean_max_live_labels; digits=0))
        gap = ismissing(row.mean_gap) ? "--" : string(round(100 * row.mean_gap; digits=2))
        println(io, "\\newcommand{\\$name}{$(row.n_stations) & $(row.n_certified) & $runtime & $labels & $live & $gap}")
    end
end

# Per-iteration logs live in a subdirectory so the summary glob above stays one row per
# job. Aggregating them is best-effort: a study run before iteration logging existed
# simply has no `iterations/` directory.
iterations_dir = joinpath(raw_dir, "iterations")
if isdir(iterations_dir)
    iter_files = sort(filter(p -> endswith(p, ".csv"), readdir(iterations_dir; join=true)))
    if !isempty(iter_files)
        iteration_log = reduce(vcat, (CSV.read(p, DataFrame) for p in iter_files))
        sort!(iteration_log, [:instance_id, :compensated_dominance, :iteration])
        CSV.write(joinpath(results_dir, "iteration_log.csv"), iteration_log)

        by_iter = combine(
            groupby(iteration_log, [:n_stations, :compensated_dominance, :iteration]),
            :columns_added => mean => :mean_columns_added,
            :master_sec => mean => :mean_master_sec,
            :pricing_sec => mean => :mean_pricing_sec,
            :add_columns_sec => mean => :mean_add_columns_sec,
            nrow => :n_jobs,
        )
        sort!(by_iter, [:n_stations, :compensated_dominance, :iteration])
        CSV.write(joinpath(results_dir, "iteration_profile.csv"), by_iter)
        println("Aggregated $(nrow(iteration_log)) iteration rows from $(length(iter_files)) jobs")
    end
end

println("Read $(length(files)) jobs and paired $(nrow(paired)) instances; wrote $results_dir")
