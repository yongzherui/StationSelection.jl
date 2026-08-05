#!/usr/bin/env bash
#SBATCH --job-name=cluster_50_100
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=06:00:00
#SBATCH --array=0-17
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-50-100-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-50-100-%A_%a.err

set -euo pipefail
ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
NS=(50 60 70 80 90 100); SEEDS=(42 314 2718)
N_INDEX=$((SLURM_ARRAY_TASK_ID / 3)); SEED_INDEX=$((SLURM_ARRAY_TASK_ID % 3))
N=${NS[$N_INDEX]}; SEED=${SEEDS[$SEED_INDEX]}
OUT="$ROOT/results/cluster_scaling_50_100/job_${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID}_n${N}_s${SEED}.txt"

module load julia/1.12.6
cd "$ROOT"
mkdir -p results/cluster_scaling_50_100
julia --project=. scripts/diag_station_cluster_scaling_50_100.jl "$N" "$SEED" "$OUT"
