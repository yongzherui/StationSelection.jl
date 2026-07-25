#!/bin/bash
#SBATCH --job-name=sample09_task
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# sample_09 MW-vs-Direct -- SLURM array task runner. Each array task reads ONE
# line from the job list (one (n_stations, method) pair) and runs it via
# sample09_task.jl, so every combination lands on its own node instead of
# running sequentially in one job. Submitted via submit_sample09_task.sh.
#
# Usage (via submit_sample09_task.sh):
#   sbatch --array=1-N --output=... --error=... \
#          scripts/sbatch_sample09_task.sh <jobs_file> <base_outdir>

JOBS_FILE="${1:-}"
BASE_OUTDIR="${2:-}"
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ]; then
    echo "ERROR: Usage: sbatch_sample09_task.sh <jobs_file> <base_outdir>"
    exit 1
fi
if [ -z "$TASK" ]; then
    echo "ERROR: SLURM_ARRAY_TASK_ID is not set; submit this script with --array."
    exit 1
fi

# Task IDs are absolute 1-indexed data rows in JOBS_FILE (header is row 0).
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
if [ -z "$JOB_LINE" ]; then
    echo "ERROR: No job found for task $TASK in $JOBS_FILE"
    exit 1
fi

N_STATIONS=$(echo "$JOB_LINE" | cut -f1)
METHOD=$(echo     "$JOB_LINE" | cut -f2)

echo "=========================================="
echo "sample_09 MW vs Direct - array task"
echo "Array job:  ${SLURM_ARRAY_JOB_ID}  task: ${TASK}"
echo "n_stations: ${N_STATIONS}   method: ${METHOD}"
echo "Node:       ${SLURM_NODELIST}"
echo "Started:    $(date)"
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
      "$PROJECT_ROOT/scripts/sample09_task.jl" \
      "$BASE_OUTDIR" "$N_STATIONS" "$METHOD"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
