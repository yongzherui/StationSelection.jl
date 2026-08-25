"""Aggregate Study 2 job CSVs.

Usage: `julia --project=. analyze.jl RAW_DIR [RESULTS_DIR]`
"""

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
    joinpath(project_root, "benchmarks", "results", "$(Dates.today())_study2_passenger_max_ablation")
mkpath(results_dir)

files = sort(filter(path -> endswith(path, ".csv"), readdir(raw_dir; join=true)))
isempty(files) && error("no CSV result files found in $raw_dir")
case_results = reduce(vcat, (CSV.read(path, DataFrame) for path in files))
bad_sizes = case_results[
    case_results.n_pairs_actual .!= case_results.n_pairs .* case_results.n_scenarios,
    [:job_id, :instance_id, :n_pairs, :n_scenarios, :n_pairs_actual, :pairs_per_scenario],
]
isempty(bad_sizes) || error("generated workload differs from configuration:\n$(sprint(show, bad_sizes))")
sort!(case_results, [:instance_id, :pricing_mode])
CSV.write(joinpath(results_dir, "case_results.csv"), case_results)

keys = [:instance_id, :n_stations, :n_pairs, :n_scenarios, :seed, :k, :max_stops]
function mode_frame(mode, prefix)
    frame = select(case_results[case_results.pricing_mode .== mode, :], keys...,
        :z_lp, :z_ip, :gap, :runtime_sec, :cg_iterations, :cg_converged,
        :cg_pricing_exhausted, :n_columns)
    rename!(frame,
        :z_lp => Symbol(prefix, "_z_lp"),
        :z_ip => Symbol(prefix, "_z_ip"),
        :gap => Symbol(prefix, "_gap"),
        :runtime_sec => Symbol(prefix, "_runtime_sec"),
        :cg_iterations => Symbol(prefix, "_cg_iterations"),
        :cg_converged => Symbol(prefix, "_converged"),
        :cg_pricing_exhausted => Symbol(prefix, "_pricing_exhausted"),
        :n_columns => Symbol(prefix, "_n_columns"))
    return frame
end

paired = innerjoin(mode_frame("exact", "exact"), mode_frame("darp", "darp"); on=keys)
paired.both_certified = paired.exact_converged .& paired.exact_pricing_exhausted .&
    paired.darp_converged .& paired.darp_pricing_exhausted

# `z_lp` is the equivalence invariant: run to exhaustion, both pricers must reach the
# same LP optimum. `z_ip` is the restricted-master heuristic over whichever pool each
# pricer discovered, so a mismatch there is a real observation about pool quality, not
# a correctness failure -- recorded, never fatal.
# Tolerance: `atol` alone would set `rtol=0` in Julia (a pure absolute check). These
# objectives run to 1e3-1e4, so 1e-6 absolute demands ~1e-10 relative agreement --
# tighter than two independent solves over different column pools reliably deliver.
# `rtol=1e-9` adds proportional slack while `atol` still covers the near-zero case.
paired.lp_objective_delta = paired.darp_z_lp .- paired.exact_z_lp
paired.lp_objective_match = [row.both_certified &&
    !ismissing(row.exact_z_lp) && !ismissing(row.darp_z_lp) &&
    isapprox(row.exact_z_lp, row.darp_z_lp; atol=1e-6, rtol=1e-9) for row in eachrow(paired)]
paired.ip_objective_delta = paired.darp_z_ip .- paired.exact_z_ip
paired.ip_objective_match = [row.both_certified &&
    !ismissing(row.exact_z_ip) && !ismissing(row.darp_z_ip) &&
    isapprox(row.exact_z_ip, row.darp_z_ip; atol=1e-6, rtol=1e-9) for row in eachrow(paired)]
paired.runtime_ratio_darp_over_exact = paired.darp_runtime_sec ./ paired.exact_runtime_sec
sort!(paired, :instance_id)
CSV.write(joinpath(results_dir, "paired_comparison.csv"), paired)

# Summaries are per (n_stations, pricing_mode): the grid sweeps n, so pooling every
# size into one mean would average runtimes across instances of different difficulty.
summary_rows = NamedTuple[]
for n in sort(unique(case_results.n_stations)), mode in ("exact", "darp")
    sub = case_results[(case_results.n_stations .== n) .& (case_results.pricing_mode .== mode), :]
    certified = sub[sub.cg_converged .& sub.cg_pricing_exhausted, :]
    push!(summary_rows, (n_stations=n, pricing_mode=mode, n_jobs=nrow(sub),
        n_certified=nrow(certified),
        mean_runtime_sec=nrow(certified) == 0 ? missing : mean(certified.runtime_sec),
        median_runtime_sec=nrow(certified) == 0 ? missing : median(certified.runtime_sec),
        mean_cg_iterations=nrow(certified) == 0 ? missing : mean(certified.cg_iterations),
        mean_n_columns=nrow(certified) == 0 ? missing : mean(certified.n_columns),
        mean_gap=nrow(certified) == 0 ? missing : mean(skipmissing(certified.gap)),
        max_gap=nrow(certified) == 0 ? missing : maximum(skipmissing(certified.gap))))
end
variant_summary = DataFrame(summary_rows)
CSV.write(joinpath(results_dir, "variant_summary.csv"), variant_summary)

open(joinpath(results_dir, "slides_results.tex"), "w") do io
    println(io, "% Generated by Study 2 analyze.jl")
    println(io, "% One macro per (pricing mode, n_stations) cell; means pool seeds only.")
    for row in eachrow(variant_summary)
        arm = row.pricing_mode == "exact" ? "Exact" : "Darp"
        name = "StudyTwo$(arm)RowN$(latex_int_word(row.n_stations))"
        runtime = ismissing(row.mean_runtime_sec) ? "--" : string(round(row.mean_runtime_sec; digits=2))
        iterations = ismissing(row.mean_cg_iterations) ? "--" : string(round(row.mean_cg_iterations; digits=2))
        columns = ismissing(row.mean_n_columns) ? "--" : string(round(row.mean_n_columns; digits=2))
        gap = ismissing(row.mean_gap) ? "--" : string(round(100 * row.mean_gap; digits=2))
        println(io, "\\newcommand{\\$name}{$(row.n_stations) & $(row.n_certified) & $runtime & $iterations & $columns & $gap}")
    end
end

lp_mismatch = paired[paired.both_certified .& .!paired.lp_objective_match, :]
isempty(lp_mismatch) || error(
    "certified pricing modes disagree on the LP bound -- both are exact, so this is a " *
    "correctness failure:\n$(sprint(show, lp_mismatch[:, [:instance_id, :exact_z_lp, :darp_z_lp, :lp_objective_delta]]))"
)
ip_mismatch = paired[paired.both_certified .& .!paired.ip_objective_match, :]
isempty(ip_mismatch) || @warn "certified modes agree on z_lp but differ on z_ip " *
    "(expected: recovery is pool-dependent, see benchmark_lp_ip)" n_pairs_differing=nrow(ip_mismatch)

# Per-iteration logs live in a subdirectory so the summary glob above stays one row per
# job. Aggregating them is best-effort: a study run before iteration logging existed
# simply has no `iterations/` directory.
iterations_dir = joinpath(raw_dir, "iterations")
if isdir(iterations_dir)
    iter_files = sort(filter(p -> endswith(p, ".csv"), readdir(iterations_dir; join=true)))
    if !isempty(iter_files)
        iteration_log = reduce(vcat, (CSV.read(p, DataFrame) for p in iter_files))
        sort!(iteration_log, [:instance_id, :pricing_mode, :iteration])
        CSV.write(joinpath(results_dir, "iteration_log.csv"), iteration_log)

        by_iter = combine(
            groupby(iteration_log, [:n_stations, :pricing_mode, :iteration]),
            :columns_added => mean => :mean_columns_added,
            :master_sec => mean => :mean_master_sec,
            :pricing_sec => mean => :mean_pricing_sec,
            :add_columns_sec => mean => :mean_add_columns_sec,
            nrow => :n_jobs,
        )
        sort!(by_iter, [:n_stations, :pricing_mode, :iteration])
        CSV.write(joinpath(results_dir, "iteration_profile.csv"), by_iter)
        println("Aggregated $(nrow(iteration_log)) iteration rows from $(length(iter_files)) jobs")
    end
end

println("Read $(length(files)) job files; wrote summaries to $results_dir")
