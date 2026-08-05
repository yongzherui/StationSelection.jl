#!/usr/bin/env bash
#SBATCH --job-name=cg_add100_n20
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=02:00:00
#SBATCH --array=0-2
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cg-add100-n20-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cg-add100-n20-%A_%a.err

set -euo pipefail
ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
SEEDS=(42 314 2718)
SEED=${SEEDS[$SLURM_ARRAY_TASK_ID]}
OUT="$ROOT/results/cluster_stop_limits/add100_${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID}_n20_s${SEED}.txt"

module load julia/1.12.6
cd "$ROOT"
mkdir -p results/cluster_stop_limits
export PFA_DIAG_MAX_STOPS=10
export PFA_DIAG_MAX_VISITS=0
export PFA_DIAG_N_CANDIDATES=100
export PFA_DIAG_MAX_NEW_COLUMNS=100
export PFA_DIAG_TOTAL_TIME=5400
export PFA_DIAG_CERT_TIME=1800
export PFA_DIAG_CLUSTER_TIME=1800
export PFA_DIAG_IP_TIME=900
julia --project=. scripts/diag_station_cluster_full_cg.jl 20 "$SEED" baseline "$OUT"
