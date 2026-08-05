#!/usr/bin/env bash
#SBATCH --job-name=cluster_direct
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=08:00:00
#SBATCH --array=0-20
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-direct-ms3-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/cluster-direct-ms3-%A_%a.err

set -euo pipefail
ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
NS=(10 15 20 25 30 35 40); SEEDS=(42 314 2718)
N_INDEX=$((SLURM_ARRAY_TASK_ID / 3)); SEED_INDEX=$((SLURM_ARRAY_TASK_ID % 3))
N=${NS[$N_INDEX]}; SEED=${SEEDS[$SEED_INDEX]}

module load julia/1.12.6
cd "$ROOT"
export PFAC_SEED="$SEED"
export PFAC_N_SCENARIOS=3
export PFAC_CLUSTER_CERT=1
julia --project=. scripts/diag_passenger_cg_vs_aggregate_direct.jl "$N" 16 3
