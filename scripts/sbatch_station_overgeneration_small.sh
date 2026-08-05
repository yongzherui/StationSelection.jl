#!/usr/bin/env bash
#SBATCH --job-name=cg_overgen
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=02:00:00
#SBATCH --array=0-17
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cg-overgen-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cg-overgen-%A_%a.err

set -euo pipefail
ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
NS=(10 15 20)
SEEDS=(42 314 2718)
CANDIDATES=(20 100)

CANDIDATE_INDEX=$((SLURM_ARRAY_TASK_ID % 2))
SEED_INDEX=$(((SLURM_ARRAY_TASK_ID / 2) % 3))
N_INDEX=$((SLURM_ARRAY_TASK_ID / 6))
N=${NS[$N_INDEX]}
SEED=${SEEDS[$SEED_INDEX]}
N_CANDIDATES=${CANDIDATES[$CANDIDATE_INDEX]}
OUT="$ROOT/results/cluster_stop_limits/overgen_${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID}_n${N}_s${SEED}_c${N_CANDIDATES}.txt"

module load julia/1.12.6
cd "$ROOT"
mkdir -p results/cluster_stop_limits
export PFA_DIAG_MAX_STOPS=10
export PFA_DIAG_MAX_VISITS=0
export PFA_DIAG_N_CANDIDATES="$N_CANDIDATES"
export PFA_DIAG_TOTAL_TIME=5400
export PFA_DIAG_CERT_TIME=1800
export PFA_DIAG_CLUSTER_TIME=1800
export PFA_DIAG_IP_TIME=900
julia --project=. scripts/diag_station_cluster_full_cg.jl "$N" "$SEED" baseline "$OUT"
