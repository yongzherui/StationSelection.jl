#!/usr/bin/env bash
#SBATCH --job-name=clust_n25_40
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=06:00:00
#SBATCH --array=0-47
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-n25-40-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-n25-40-%A_%a.err

set -euo pipefail
ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
NS=(25 30 35 40)
SEEDS=(42 314 2718)
MODES=(baseline cluster)
STOPS=(10 0)

MODE_INDEX=$((SLURM_ARRAY_TASK_ID % 2))
SEED_INDEX=$(((SLURM_ARRAY_TASK_ID / 2) % 3))
N_INDEX=$(((SLURM_ARRAY_TASK_ID / 6) % 4))
STOP_INDEX=$((SLURM_ARRAY_TASK_ID / 24))

N=${NS[$N_INDEX]}
SEED=${SEEDS[$SEED_INDEX]}
MODE=${MODES[$MODE_INDEX]}
MAX_STOPS=${STOPS[$STOP_INDEX]}
STOP_LABEL=$([[ "$MAX_STOPS" -eq 0 ]] && echo uncapped || echo ms10)
OUT="$ROOT/results/cluster_stop_limits/job_${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID}_n${N}_s${SEED}_${MODE}_${STOP_LABEL}.txt"

module load julia/1.12.6
cd "$ROOT"
mkdir -p results/cluster_stop_limits
export PFA_DIAG_MAX_STOPS="$MAX_STOPS"
export PFA_DIAG_MAX_VISITS=0
export PFA_DIAG_N_CANDIDATES=20
export PFA_DIAG_TOTAL_TIME=18000
export PFA_DIAG_CERT_TIME=3600
export PFA_DIAG_CLUSTER_TIME=3600
export PFA_DIAG_IP_TIME=1800
julia --project=. scripts/diag_station_cluster_full_cg.jl "$N" "$SEED" "$MODE" "$OUT"
