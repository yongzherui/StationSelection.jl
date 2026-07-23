#!/bin/bash
#SBATCH --job-name=aor_method_compare
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=03:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# AggregateODRouteModel method comparison - SLURM array task runner.
# Each array task reads ONE line from the job list -- one (instance, method)
# pair (Direct solve / plain CG / Benders Y,YZ,YZH variant) -- and runs it via
# run_method_compare_task.jl. Submitted in batches of one n_stations value at
# a time (450 jobs each, under the cluster's 500-job submission cap) by
# submit_method_compare.sh -- do not call this script directly with a raw
# --array range unless you've computed it against batch_manifest.txt yourself.
#
# Usage (via submit_method_compare.sh):
#   sbatch --array=<start>-<end> --output=... --error=... \
#          scripts/sbatch_method_compare.sh <jobs_file> <base_outdir> <data_dir>

JOBS_FILE="${1:-}"
BASE_OUTDIR="${2:-}"
DATA_DIR="${3:-}"
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ] || [ -z "$DATA_DIR" ]; then
    echo "ERROR: Usage: sbatch_method_compare.sh <jobs_file> <base_outdir> <data_dir>"
    exit 1
fi
if [ -z "$TASK" ]; then
    echo "ERROR: SLURM_ARRAY_TASK_ID is not set; submit this script with --array."
    exit 1
fi

# Task IDs are absolute 1-indexed data rows in JOBS_FILE (header is row 0), so
# a batch submitted as --array=451-900 reads rows 451-900 directly -- no
# per-batch offset bookkeeping needed.
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
if [ -z "$JOB_LINE" ]; then
    echo "ERROR: No job found for task $TASK in $JOBS_FILE"
    exit 1
fi

FAMILY=$(echo     "$JOB_LINE" | cut -f1)
N_STATIONS=$(echo "$JOB_LINE" | cut -f2)
L=$(echo          "$JOB_LINE" | cut -f3)
N_PAIRS=$(echo    "$JOB_LINE" | cut -f4)
SEED=$(echo       "$JOB_LINE" | cut -f5)
METHOD=$(echo     "$JOB_LINE" | cut -f6)

INST="${FAMILY}_n${N_STATIONS}_p${N_PAIRS}_s${SEED}"

echo "=========================================="
echo "AggregateODRouteModel Method Comparison"
echo "Array job:  ${SLURM_ARRAY_JOB_ID}  task: ${TASK}"
echo "Instance:   ${INST}"
echo "Method:     ${METHOD}"
echo "Node:       ${SLURM_NODELIST}"
echo "Started:    $(date)"
echo "Project:    ${PROJECT_ROOT}"
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
      "$PROJECT_ROOT/scripts/run_method_compare_task.jl" \
      "$BASE_OUTDIR" "$DATA_DIR" "$FAMILY" "$N_STATIONS" "$L" "$N_PAIRS" "$SEED" "$METHOD"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
