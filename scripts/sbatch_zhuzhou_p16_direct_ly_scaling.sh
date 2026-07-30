#!/bin/bash
#SBATCH --job-name=zz_direct_ly_scaling
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=03:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# Array task runner for scripts/zhuzhou_p16_direct_ly_scaling.jl -- one n_stations value
# per task, task IDs 1..5 map to n_stations in {20,30,40,50,60}.
#
# Usage:
#   sbatch --array=1-5 --output=... --error=... scripts/sbatch_zhuzhou_p16_direct_ly_scaling.sh <outdir>

N_STATIONS_LIST=(20 30 40 50 60)
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"
OUTDIR="${1:-$PROJECT_ROOT/experiments/zhuzhou_p16_direct_ly_scaling}"

if [ -z "$TASK" ]; then
    echo "ERROR: SLURM_ARRAY_TASK_ID is not set; submit this script with --array=1-5."
    exit 1
fi
N_STATIONS="${N_STATIONS_LIST[$((TASK - 1))]}"

echo "=========================================="
echo "zhuzhou p16 :direct_ly scaling - array task"
echo "Array job:  ${SLURM_ARRAY_JOB_ID}  task: ${TASK}  n_stations: ${N_STATIONS}"
echo "Node:       ${SLURM_NODELIST}"
echo "Started:    $(date)"
echo "=========================================="
echo ""

JULIA_MODULE="${CS_JULIA_MODULE:-julia/1.12.6}"
GUROBI_MODULE="${CS_GUROBI_MODULE:-}"
module load "$JULIA_MODULE"
[ -n "$GUROBI_MODULE" ] && module load "$GUROBI_MODULE"

JULIA_VERSION=$(julia --startup-file=no -e 'print(VERSION)')
COPY_DEPOT="${CS_COPY_DEPOT:-1}"
if [ "$COPY_DEPOT" = "0" ]; then
    export JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-$HOME/.julia}"
else
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${TASK}"
    mkdir -p "$JULIA_DEPOT_PATH"
    rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
fi

cd "$PROJECT_ROOT"
set +e
stdbuf -o0 -e0 julia --startup-file=no --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/zhuzhou_p16_direct_ly_scaling.jl" "$N_STATIONS" "$OUTDIR"
EXIT_CODE=$?
set -e

echo ""
echo "Finished: $(date)  exit=$EXIT_CODE"
exit $EXIT_CODE
