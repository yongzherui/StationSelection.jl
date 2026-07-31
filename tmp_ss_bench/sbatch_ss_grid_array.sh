#!/bin/bash
#SBATCH --job-name=ss_grid
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=8G
#SBATCH --time=04:00:00

set -uo pipefail

# One array task per station count so the breakpoints run concurrently on
# separate nodes. Submit with:  sbatch --array=0-2 sbatch_ss_grid_array.sh
read -ra NVALS <<< "${PFA_GRID_NVALS:-10 15 20}"
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-<#n minus 1>}"
N="${NVALS[$TASK]}"
MAX_STOPS="${PFA_GRID_MAX_STOPS:-5}"

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6
julia --version

# Per-task depot copy so concurrent tasks never race on precompilation.
JULIA_VERSION=$(julia --startup-file=no -e 'print(VERSION)')
if [ -n "${SLURM_TMPDIR:-}" ]; then
    export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot_v${JULIA_VERSION}"
else
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${TASK}"
fi
mkdir -p "$JULIA_DEPOT_PATH"
rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"

cd "$PROJECT_ROOT"
# p=16, 3 scenarios come from the benchmark's own N_PAIRS/N_SCENARIOS defaults.
# `MAX_STOPS=0` is the benchmark's uncapped sentinel. A 900s per-search limit
# prevents an uncapped case from consuming the whole allocation.
export PFA_TIME_LIMIT=900
echo "===== station-simple grid: n=${N}, ms=${MAX_STOPS}, p=16, 3 scenarios ====="
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/bench_passenger_free_assignment_labels.jl" \
    --cases "${N}:${MAX_STOPS}"
