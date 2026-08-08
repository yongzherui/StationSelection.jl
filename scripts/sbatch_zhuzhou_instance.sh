#!/bin/bash
#SBATCH --job-name=zz_instance
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=04:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# Zhuzhou AggregateODRouteModel experiment -- SLURM array runner. Each task
# reads one line from the job list and solves one Zhuzhou instance. Generic
# across every Zhuzhou sweep (scenario count, route-weight, etc. are all
# CS_* env overrides below) -- see submit_zhuzhou.sh.
#
# Usage (via submit_zhuzhou.sh -- do not call directly):
#   sbatch --array=1-<N> --job-name=<name> \
#          --output=<exp_dir>/slurm_logs/%A_%a.out \
#          --error=<exp_dir>/slurm_logs/%A_%a.err \
#          scripts/sbatch_zhuzhou_instance.sh <jobs_file> <base_outdir> <data_dir>

JOBS_FILE="${1:-}"
BASE_OUTDIR="${2:-}"
DATA_DIR="${3:-}"
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ] || [ -z "$DATA_DIR" ]; then
    echo "ERROR: Usage: sbatch_zhuzhou_instance.sh <jobs_file> <base_outdir> <data_dir>"
    exit 1
fi
if [ -z "$TASK" ]; then
    echo "ERROR: SLURM_ARRAY_TASK_ID is not set; submit this script with --array."
    exit 1
fi

# Skip the header line. Task IDs start at 1 and map to data line TASK+1.
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
if [ -z "$JOB_LINE" ]; then
    echo "ERROR: No job found for task $TASK in $JOBS_FILE"
    exit 1
fi

N_STATIONS=$(echo "$JOB_LINE" | cut -f1)
L=$(echo          "$JOB_LINE" | cut -f2)
N_PAIRS=$(echo    "$JOB_LINE" | cut -f3)
OV=$(echo         "$JOB_LINE" | cut -f4)
SEED=$(echo       "$JOB_LINE" | cut -f5)

OV_STR="${OV//./_}"
INST="zz_n${N_STATIONS}_l${L}_p${N_PAIRS}_ov${OV_STR}_s${SEED}"

echo "=========================================="
echo "AggregateODRouteModel — Zhuzhou"
echo "Array job:  ${SLURM_ARRAY_JOB_ID}  task: ${TASK}"
echo "Instance:   ${INST}"
echo "Node:       ${SLURM_NODELIST}"
echo "Started:    $(date)"
echo "Project:    ${PROJECT_ROOT}"
echo "=========================================="
echo ""

source "$(dirname "${BASH_SOURCE[0]}")/lib/slurm_array_task_env.sh"

cd "$PROJECT_ROOT"

export CS_TIME_LIMIT="${CS_TIME_LIMIT:-10800}"
export CS_PRICING_TIME="${CS_PRICING_TIME:-300}"
export CS_MAX_CG_ITERS="${CS_MAX_CG_ITERS:-10000}"
export CS_MAX_NEW_COLS="${CS_MAX_NEW_COLS:-20}"
export CS_IP_TIME_LIMIT="${CS_IP_TIME_LIMIT:-1200}"
export CS_MIP_GAP="${CS_MIP_GAP:-1e-4}"
export CS_N_SCENARIOS="${CS_N_SCENARIOS:-3}"
export CS_MAX_WALKING_DISTANCE="${CS_MAX_WALKING_DISTANCE:-600}"
export CS_MAX_WAIT_TIME="${CS_MAX_WAIT_TIME:-900}"
export CS_DETOUR_FACTOR="${CS_DETOUR_FACTOR:-2.0}"
export CS_MAX_STOPS="${CS_MAX_STOPS:-}"
export CS_ROUTE_REG_WEIGHT="${CS_ROUTE_REG_WEIGHT:-1.0}"
export CS_REPOSITIONING_TIME="${CS_REPOSITIONING_TIME:-20.0}"

echo "===== Settings ====="
echo "  Instance                 = ${INST}"
echo "  DATA_DIR                 = ${DATA_DIR}"
echo "  CS_TIME_LIMIT            = ${CS_TIME_LIMIT}s"
echo "  CS_PRICING_TIME          = ${CS_PRICING_TIME}s"
echo "  CS_MAX_CG_ITERS          = ${CS_MAX_CG_ITERS}"
echo "  CS_MAX_NEW_COLS          = ${CS_MAX_NEW_COLS}"
echo "  CS_IP_TIME_LIMIT         = ${CS_IP_TIME_LIMIT}s"
echo "  CS_MIP_GAP               = ${CS_MIP_GAP}"
echo "  CS_N_SCENARIOS           = ${CS_N_SCENARIOS}"
echo "  CS_MAX_WALKING_DISTANCE  = ${CS_MAX_WALKING_DISTANCE}s"
echo "  CS_MAX_WAIT_TIME         = ${CS_MAX_WAIT_TIME}s"
echo "  CS_DETOUR_FACTOR         = ${CS_DETOUR_FACTOR}"
echo "  CS_MAX_STOPS             = ${CS_MAX_STOPS}"
echo "  CS_ROUTE_REG_WEIGHT      = ${CS_ROUTE_REG_WEIGHT}"
echo "  CS_REPOSITIONING_TIME    = ${CS_REPOSITIONING_TIME}s"
echo ""

echo "===== Running ====="
set +e
julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/run_zhuzhou_instance.jl" \
      "$BASE_OUTDIR" "$DATA_DIR" "$N_STATIONS" "$L" "$N_PAIRS" "$OV" "$SEED"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
