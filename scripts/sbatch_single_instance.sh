#!/bin/bash
#SBATCH --job-name=cs_scaling
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=04:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# AggregateODRouteModel scaling experiment — SLURM array runner.
# Each task reads one line from the job list and solves one synthetic instance.
#
# Usage:
#   sbatch --array=1-<N> \
#          --output=<exp_dir>/slurm_logs/%A_%a.out \
#          --error=<exp_dir>/slurm_logs/%A_%a.err \
#          scripts/sbatch_single_instance.sh <jobs_file> <base_outdir>

JOBS_FILE="${1:-}"
BASE_OUTDIR="${2:-}"
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ]; then
    echo "ERROR: Usage: sbatch_single_instance.sh <jobs_file> <base_outdir>"
    exit 1
fi
if [ -z "$TASK" ]; then
    echo "ERROR: SLURM_ARRAY_TASK_ID is not set; submit this script with --array."
    exit 1
fi

# Skip the header. Task IDs start at 1 and map to data line TASK+1.
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
if [ -z "$JOB_LINE" ]; then
    echo "ERROR: No job found for task $TASK in $JOBS_FILE"
    exit 1
fi

NX=$(echo "$JOB_LINE" | cut -f1)
NY=$(echo "$JOB_LINE" | cut -f2)
N_REQUESTS=$(echo "$JOB_LINE" | cut -f3)
SEED=$(echo "$JOB_LINE" | cut -f4)

echo "=========================================="
echo "AggregateODRouteModel Scaling Experiment"
echo "Array job:  ${SLURM_ARRAY_JOB_ID}  task: ${TASK}"
echo "Instance:   g${NX}x${NY}_r${N_REQUESTS}_s${SEED}"
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

export CS_TIME_LIMIT="${CS_TIME_LIMIT:-10800}"
export CS_PRICING_TIME="${CS_PRICING_TIME:-300}"
export CS_MAX_CG_ITERS="${CS_MAX_CG_ITERS:-10000}"
export CS_MAX_NEW_COLS="${CS_MAX_NEW_COLS:-20}"
export CS_IP_TIME_LIMIT="${CS_IP_TIME_LIMIT:-1200}"
export CS_MIP_GAP="${CS_MIP_GAP:-1e-4}"
export CS_WALK_SCALE="${CS_WALK_SCALE:-600.0}"
export CS_ROUTE_SCALE="${CS_ROUTE_SCALE:-450.0}"
export CS_MAX_WALKING_DISTANCE="${CS_MAX_WALKING_DISTANCE:-}"
export CS_ROUTE_REGULARIZATION_WEIGHT="${CS_ROUTE_REGULARIZATION_WEIGHT:-1.0}"
export CS_REPOSITIONING_TIME="${CS_REPOSITIONING_TIME:-20.0}"

echo "===== Settings ====="
echo "  CS_TIME_LIMIT    = ${CS_TIME_LIMIT}s"
echo "  CS_PRICING_TIME  = ${CS_PRICING_TIME}s"
echo "  CS_MAX_CG_ITERS  = ${CS_MAX_CG_ITERS}"
echo "  CS_MAX_NEW_COLS  = ${CS_MAX_NEW_COLS}"
echo "  CS_IP_TIME_LIMIT = ${CS_IP_TIME_LIMIT}s"
echo "  CS_MIP_GAP       = ${CS_MIP_GAP}"
echo "  CS_WALK_SCALE    = ${CS_WALK_SCALE}"
echo "  CS_ROUTE_SCALE   = ${CS_ROUTE_SCALE}"
echo "  CS_MAX_WALKING_DISTANCE = ${CS_MAX_WALKING_DISTANCE:-grid diameter}"
echo "  CS_ROUTE_REGULARIZATION_WEIGHT = ${CS_ROUTE_REGULARIZATION_WEIGHT}"
echo "  CS_REPOSITIONING_TIME = ${CS_REPOSITIONING_TIME}"
echo ""

echo "===== Running ====="
set +e
julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/run_single_instance.jl" \
      "$BASE_OUTDIR" "$NX" "$NY" "$N_REQUESTS" "$SEED"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
