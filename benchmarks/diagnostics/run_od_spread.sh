#!/bin/bash
# OD-spread trend probe: does the spatial spread of the sampled OD pairs predict the
# per-round cost of the relaxed cluster sweep? Pure data generation, no solve.
#SBATCH --job-name=od_spread
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=8G
#SBATCH --time=00:40:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/od-spread-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/od-spread-%j.err
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
    benchmarks/diagnostics/od_spread_vs_round_cost.jl
