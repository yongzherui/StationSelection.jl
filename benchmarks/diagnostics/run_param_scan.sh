#!/bin/bash
# Guide-recovery probe, one job per K, safe to submit CONCURRENTLY.
#
# Unlike run_guide_recovery.sh this gives each job its OWN Julia depot. Several
# concurrent jobs sharing ~/.julia race on the precompile cache and can sit for
# ten-plus minutes before emitting a single line; the copy costs one full
# precompile (~3 min) and removes the race. Set CS_COPY_DEPOT=0 to opt out.
#SBATCH --job-name=param_scan
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=8G
#SBATCH --time=00:25:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/two-tier-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/two-tier-%j.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_JOB_ID}"
# slurm_array_task_env.sh names the per-job depot from the ARRAY vars, which a plain
# (non-array) sbatch never sets -- and `set -u` makes that fatal. Alias them to this job.
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-0}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/two_tier_param_scan.jl
