#!/bin/bash
#SBATCH --job-name=pfa_subset
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=24G
#SBATCH --time=01:15:00
#SBATCH --array=0-2

set -euo pipefail
NVALS=(10 15 20)
N="${NVALS[${SLURM_ARRAY_TASK_ID:?submit with --array=0-2}]}"
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFASS_OUTDIR:?set PFASS_OUTDIR}"
mkdir -p "$OUTDIR"
module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_station_subset_pricing.jl" "$N" "$OUTDIR/n${N}.csv"
