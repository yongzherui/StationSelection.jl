#!/bin/bash
#SBATCH --job-name=study8_analyze
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=8G
#SBATCH --time=00:30:00
set -euo pipefail
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK=analyze8
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
export CS_COPY_DEPOT=0
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    benchmarks/study8_warm_start_speedup/analyze.jl
