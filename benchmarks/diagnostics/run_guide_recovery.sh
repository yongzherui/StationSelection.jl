#!/bin/bash
#SBATCH --job-name=guide_rec
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=8G
#SBATCH --time=02:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/guide-rec-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/guide-rec-%j.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/relaxed_cluster_guide_recovery.jl
