#!/usr/bin/env bash
#SBATCH --job-name=cluster_cg
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=06:00:00
#SBATCH --array=0-17
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-cg-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-cg-%A_%a.err

set -euo pipefail
ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
NS=(10 15 20); SEEDS=(42 314 2718); MODES=(baseline cluster)
MODE_INDEX=$((SLURM_ARRAY_TASK_ID % 2))
SEED_INDEX=$(((SLURM_ARRAY_TASK_ID / 2) % 3))
N_INDEX=$((SLURM_ARRAY_TASK_ID / 6))
N=${NS[$N_INDEX]}; SEED=${SEEDS[$SEED_INDEX]}; MODE=${MODES[$MODE_INDEX]}
OUT="$ROOT/results/cluster_full_cg/job_${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID}_n${N}_s${SEED}_${MODE}.txt"

module load julia/1.12.6
cd "$ROOT"
mkdir -p results/cluster_full_cg
julia --project=. scripts/diag_station_cluster_full_cg.jl "$N" "$SEED" "$MODE" "$OUT"
