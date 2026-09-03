#!/bin/bash
#SBATCH --job-name=ss_profile
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=24G
#SBATCH --time=02:00:00
set -euo pipefail
export JULIA_NUM_THREADS=1   # no idle poptask samples to filter out
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK=profile
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
export CS_COPY_DEPOT=0
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/profile_station_simple_vs_exact.jl "${PROF_P:-16}" "${PROF_SEED:-51}" "${PROF_ITERS:-6}"
