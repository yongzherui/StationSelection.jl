#!/bin/bash
#SBATCH --job-name=zz_rwramp_task
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=04:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# zhuzhou (larger-n) route_regularization_weight_schedule ramp-vs-direct comparison --
# SLURM array task runner. Each array task reads ONE line from the job list
# (one (n_stations, n_pairs, seed, variant, schedule, target_weight,
# max_stops_mode) combo) and runs it via zhuzhou_route_weight_ramp_experiment.jl,
# exporting RAMP_* env vars from the parsed job-file field (not via sbatch
# --export, which splits on commas even inside a quoted value).
#
# Usage:
#   sbatch --array=1-N --output=... --error=... \
#          scripts/sbatch_zhuzhou_route_weight_ramp_task.sh <jobs_file> <base_outdir> <data_dir>

JOBS_FILE="${1:-}"
BASE_OUTDIR="${2:-}"
DATA_DIR="${3:-}"
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ] || [ -z "$DATA_DIR" ]; then
    echo "ERROR: Usage: sbatch_zhuzhou_route_weight_ramp_task.sh <jobs_file> <base_outdir> <data_dir>"
    exit 1
fi
if [ -z "$TASK" ]; then
    echo "ERROR: SLURM_ARRAY_TASK_ID is not set; submit this script with --array."
    exit 1
fi

JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
if [ -z "$JOB_LINE" ]; then
    echo "ERROR: No job found for task $TASK in $JOBS_FILE"
    exit 1
fi

N_STATIONS=$(echo      "$JOB_LINE" | cut -f1)
N_PAIRS=$(echo         "$JOB_LINE" | cut -f2)
SEED=$(echo            "$JOB_LINE" | cut -f3)
VARIANT=$(echo         "$JOB_LINE" | cut -f4)
SCHEDULE=$(echo        "$JOB_LINE" | cut -f5)
TARGET_WEIGHT=$(echo   "$JOB_LINE" | cut -f6)
MAX_STOPS_MODE=$(echo  "$JOB_LINE" | cut -f7)
OUTDIR="$BASE_OUTDIR/n${N_STATIONS}"

export RAMP_N_STATIONS="$N_STATIONS"
export RAMP_N_PAIRS="$N_PAIRS"
export RAMP_SEED="$SEED"
export RAMP_SCHEDULE="$SCHEDULE"
export RAMP_TARGET_WEIGHT="$TARGET_WEIGHT"
export RAMP_MAX_STOPS_MODE="$MAX_STOPS_MODE"
export RAMP_DATA_DIR="$DATA_DIR"

echo "=========================================="
echo "zhuzhou route_weight ramp vs direct - array task"
echo "Array job:      ${SLURM_ARRAY_JOB_ID}  task: ${TASK}"
echo "n_stations:     ${N_STATIONS}   n_pairs: ${N_PAIRS}   seed: ${SEED}   variant: ${VARIANT}"
echo "schedule:       ${SCHEDULE}   target_weight: ${TARGET_WEIGHT}   max_stops_mode: ${MAX_STOPS_MODE}"
echo "Node:           ${SLURM_NODELIST}"
echo "Started:        $(date)"
echo "=========================================="
echo ""

echo "===== Loading modules ====="
JULIA_MODULE="${CS_JULIA_MODULE:-julia/1.12.6}"
GUROBI_MODULE="${CS_GUROBI_MODULE:-}"
module load "$JULIA_MODULE"
if [ -n "$GUROBI_MODULE" ]; then
    module load "$GUROBI_MODULE"
fi
julia --version
echo ""

echo "===== Setting up Julia depot ====="
JULIA_VERSION=$(julia --startup-file=no -e 'print(VERSION)')
COPY_DEPOT="${CS_COPY_DEPOT:-1}"
if [ "$COPY_DEPOT" = "0" ]; then
    export JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-$HOME/.julia}"
    echo "Using existing depot: $JULIA_DEPOT_PATH"
else
    if [ -n "${SLURM_TMPDIR:-}" ]; then
        export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot_v${JULIA_VERSION}"
    else
        export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${TASK}"
    fi
    mkdir -p "$JULIA_DEPOT_PATH"
    rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
    echo "Depot ready: $JULIA_DEPOT_PATH"
fi
echo ""

cd "$PROJECT_ROOT"

echo "===== Running ====="
set +e
stdbuf -o0 -e0 julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/zhuzhou_route_weight_ramp_experiment.jl" \
      "$OUTDIR" "$VARIANT"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
