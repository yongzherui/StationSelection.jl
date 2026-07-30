#!/bin/bash
#SBATCH --job-name=sample09_rwramp_task
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# sample_09 route_regularization_weight_schedule ramp-vs-direct comparison --
# SLURM array task runner. Each array task reads ONE line from the job list
# (one (n_stations, decomposition, variant, schedule, target_weight) combo)
# and runs it via sample09_route_weight_ramp_experiment.jl, exporting
# RAMP_N_STATIONS / RAMP_SCHEDULE / RAMP_DECOMPOSITION / SAMPLE09_ROUTE_WEIGHT
# as env vars (set from the parsed job-file field, not via sbatch --export,
# which splits on commas even inside a quoted value -- see
# sample09_route_weight_ramp_experiment.jl's own note on this).
#
# Usage:
#   sbatch --array=1-N --output=... --error=... \
#          scripts/sbatch_sample09_route_weight_ramp_task.sh <jobs_file> <base_outdir>

JOBS_FILE="${1:-}"
BASE_OUTDIR="${2:-}"
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ]; then
    echo "ERROR: Usage: sbatch_sample09_route_weight_ramp_task.sh <jobs_file> <base_outdir>"
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

N_STATIONS=$(echo     "$JOB_LINE" | cut -f1)
DECOMPOSITION=$(echo  "$JOB_LINE" | cut -f2)
VARIANT=$(echo        "$JOB_LINE" | cut -f3)
SCHEDULE=$(echo       "$JOB_LINE" | cut -f4)
TARGET_WEIGHT=$(echo  "$JOB_LINE" | cut -f5)
CUT_DERIVATION=$(echo "$JOB_LINE" | cut -f6)
CONFIG_NAME=$(echo    "$JOB_LINE" | cut -f7)
# CONFIG_NAME namespaces different schedule *shapes* that all use variant=ramp for the same
# n_stations -- without it every shape would collide on the same OUTDIR/results/ramp.csv.
OUTDIR="$BASE_OUTDIR/${CONFIG_NAME:-default}/$N_STATIONS"

export RAMP_N_STATIONS="$N_STATIONS"
export RAMP_DECOMPOSITION="$DECOMPOSITION"
export RAMP_SCHEDULE="$SCHEDULE"
export SAMPLE09_ROUTE_WEIGHT="$TARGET_WEIGHT"
export RAMP_CUT_DERIVATION="${CUT_DERIVATION:-restricted_mw_fixed_pi}"

echo "=========================================="
echo "sample_09 route_weight ramp vs direct - array task"
echo "Array job:      ${SLURM_ARRAY_JOB_ID}  task: ${TASK}"
echo "n_stations:     ${N_STATIONS}   decomposition: ${DECOMPOSITION}   variant: ${VARIANT}   config: ${CONFIG_NAME}"
echo "schedule:       ${SCHEDULE}   target_weight: ${TARGET_WEIGHT}   cut_derivation: ${RAMP_CUT_DERIVATION}"
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
      "$PROJECT_ROOT/scripts/sample09_route_weight_ramp_experiment.jl" \
      "$OUTDIR" "$VARIANT"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
