#!/usr/bin/env bash
#SBATCH --job-name=cluster_price
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=24G
#SBATCH --time=02:00:00
#SBATCH --array=0-17
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-price-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-price-%A_%a.err

set -euo pipefail

PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
STATION_COUNTS=(10 15)
SEEDS=(42 314 2718)

# Layout: scenario varies fastest, then seed, then station count.
SCENARIO=$((SLURM_ARRAY_TASK_ID % 3 + 1))
SEED_INDEX=$(((SLURM_ARRAY_TASK_ID / 3) % 3))
N_INDEX=$((SLURM_ARRAY_TASK_ID / 9))
SEED=${SEEDS[$SEED_INDEX]}
N_STATIONS=${STATION_COUNTS[$N_INDEX]}

module load julia/1.12.6
cd "$PROJECT_ROOT"

julia --project=. scripts/diag_station_cluster_pricing.jl "$N_STATIONS" "$SEED" "$SCENARIO"
