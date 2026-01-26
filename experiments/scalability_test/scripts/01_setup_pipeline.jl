#!/usr/bin/env julia
"""
Setup pipeline experiment for station selection scalability.

This script:
1. Reads base_selection.toml and sweep_selection.toml
2. Generates parameter combinations
3. Creates job lists and per-job configs
4. Initializes progress tracking

Usage (from project root):
    julia experiments/<study_name>/scripts/01_setup_pipeline.jl
"""

using TOML, Dates, CSV, DataFrames, JSON

STUDY_DIR = abspath(joinpath(@__DIR__, ".."))
PROJECT_ROOT = abspath(joinpath(STUDY_DIR, "../../.."))

println("="^80)
println("Pipeline Experiment Setup")
println("="^80)
println("Study directory: $STUDY_DIR")
println("Project root: $PROJECT_ROOT")
println()

base_selection_path = joinpath(STUDY_DIR, "base_selection.toml")
sweep_selection_path = joinpath(STUDY_DIR, "sweep_selection.toml")

if !isfile(base_selection_path)
    error("base_selection.toml not found at: $base_selection_path")
end
if !isfile(sweep_selection_path)
    error("sweep_selection.toml not found at: $sweep_selection_path")
end

base_selection = TOML.parsefile(base_selection_path)
sweep_selection = TOML.parsefile(sweep_selection_path)

println("Loaded configurations:")
println("  Base selection: $base_selection_path")
println("  Sweep selection: $sweep_selection_path")
println()

function generate_combinations(sweep_dict)
    keys_vec = collect(keys(sweep_dict))
    vals_vec = collect(values(sweep_dict))
    combinations = vec(collect(Iterators.product(vals_vec...)))
    return [Dict(zip(keys_vec, combo)) for combo in combinations]
end

param_combos = generate_combinations(sweep_selection)
println("Generated $(length(param_combos)) raw parameter combinations")

# Optional filter for k/l if present
if haskey(sweep_selection, "k") && haskey(sweep_selection, "l")
    valid_combos = []
    invalid_count = 0
    for combo in param_combos
        k_val = combo["k"]
        l_val = combo["l"]
        if l_val >= k_val
            push!(valid_combos, combo)
        else
            invalid_count += 1
        end
    end
    param_combos = valid_combos
    println("Filtered out $invalid_count invalid combinations (l < k)")
    if isempty(param_combos)
        error("No valid parameter combinations remaining after filtering. Ensure l >= k for all combinations.")
    end
end

println("Valid combinations: $(length(param_combos))")
println()

# Create directories
config_dir = joinpath(STUDY_DIR, "config")
selection_config_dir = joinpath(config_dir, "selection")
progress_dir = joinpath(STUDY_DIR, "progress")

mkpath(selection_config_dir)
mkpath(progress_dir)

# Create job list entries
selection_jobs = []
for (i, params) in enumerate(param_combos)
    job = Dict(
        "job_id" => i,
        "name" => get(base_selection, "name", "selection")
    )
    merge!(job, params)
    push!(selection_jobs, job)
end

# Save selection job list
selection_df = DataFrame(selection_jobs)
select!(selection_df, :job_id, :)
selection_job_list_path = joinpath(config_dir, "selection_job_list.csv")
CSV.write(selection_job_list_path, selection_df)
println("✓ Saved selection job list: $selection_job_list_path")

# Write per-job selection configs
println("\nWriting selection config files...")
for (i, params) in enumerate(param_combos)
    selection_config = deepcopy(base_selection)
    base_params = get(selection_config, "parameters", Dict{String, Any}())
    merge!(base_params, params)
    selection_config["parameters"] = base_params

    config_file = joinpath(selection_config_dir, "job_$(i).toml")
    open(config_file, "w") do f
        TOML.print(f, selection_config)
    end
end
println("✓ Saved $(length(param_combos)) selection config files to config/selection/")

# Write simple job list (just IDs)
selection_jobs_txt_path = joinpath(config_dir, "selection_jobs.txt")
open(selection_jobs_txt_path, "w") do f
    for i in 1:length(param_combos)
        println(f, i)
    end
end
println("✓ Saved selection job list: $selection_jobs_txt_path")

# Save metadata
metadata = Dict(
    "created" => string(now()),
    "total_jobs" => length(selection_jobs),
    "selection_sweep_parameters" => collect(keys(sweep_selection)),
    "base_selection_config" => base_selection_path,
    "sweep_selection_config" => sweep_selection_path,
)

metadata_path = joinpath(STUDY_DIR, "setup_metadata.json")
open(metadata_path, "w") do f
    JSON.print(f, metadata, 2)
end
println("✓ Saved setup metadata: $metadata_path")

# Initialize progress tracking
println("\nInitializing progress tracking...")
for i in 1:length(selection_jobs)
    progress_file = joinpath(progress_dir, "job_$(i)_progress.json")
    progress_data = Dict(
        "job_id" => i,
        "created" => string(now()),
        "selection" => Dict("status" => "pending")
    )
    open(progress_file, "w") do f
        JSON.print(f, progress_data, 2)
    end
end
println("✓ Initialized $(length(selection_jobs)) progress tracking files")

# Summary
println()
println("="^80)
println("Setup Summary")
println("="^80)
println("Total selection jobs: $(length(selection_jobs))")
println()
println("Selection sweep parameters:")
for (param, values) in sweep_selection
    println("  $param: $values ($(length(values)) values)")
end
println()
println("Next steps:")
println("1. Review job list in config/")
println("2. Update SLURM array size in scripts/02_submit_selection.sh")
println("3. Submit: sbatch scripts/02_submit_selection.sh")
println()
