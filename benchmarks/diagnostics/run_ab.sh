#!/bin/bash
#SBATCH --job-name=ss_ab
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=24G
#SBATCH --time=03:00:00
set -euo pipefail
export JULIA_NUM_THREADS=1
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK=ab
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
export CS_COPY_DEPOT=0
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/ab_station_simple_dominance.jl "${AB_P:-16}" "${AB_SEED:-51}" "${AB_REPS:-7}" "${AB_ITERS:-6}"
