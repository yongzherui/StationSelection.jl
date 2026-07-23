#!/bin/bash
#SBATCH --job-name=benders_compare
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# BendersY / BendersYZ / BendersYZH convergence comparison - SLURM array runner.
# Each task reads one line from the job list (one Zhuzhou instance) and runs exactly ONE
# decomposition on it (reprice_subproblem=true, for correctness). Keyed by (instance,
# decomposition) rather than bundling all three decompositions into one task: BendersY's
# repricing can hit dual degeneracy and run far longer than BendersYZ/BendersYZH on the same
# instance, so bundling them let a slow BendersY block or starve the fast ones of their own
# walltime budget. submit_benders_compare.sh submits this script three times (once per
# decomposition, each with its own --time budget) against the same instance grid.
#
# Usage (via submit_benders_compare.sh - do not call directly):
#   sbatch --array=1-<N> --time=<budget> --job-name=benders_<decomp> \
#          --output=<exp_dir>/slurm_logs/%x_%A_%a.out \
#          --error=<exp_dir>/slurm_logs/%x_%A_%a.err \
#          scripts/sbatch_benders_compare.sh <jobs_file> <base_outdir> <data_dir> <decomposition>

JOBS_FILE="${1:-}"
BASE_OUTDIR="${2:-}"
DATA_DIR="${3:-}"
DECOMPOSITION="${4:-}"
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ] || [ -z "$DATA_DIR" ] || [ -z "$DECOMPOSITION" ]; then
    echo "ERROR: Usage: sbatch_benders_compare.sh <jobs_file> <base_outdir> <data_dir> <decomposition>"
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

OV_STR="${OV//./p}"
INST="zz_n${N_STATIONS}_l${L}_p${N_PAIRS}_ov${OV_STR}_s${SEED}"

echo "=========================================="
echo "Benders Decomposition Comparison - Zhuzhou"
echo "Array job:     ${SLURM_ARRAY_JOB_ID}  task: ${TASK}"
echo "Instance:      ${INST}"
echo "Decomposition: ${DECOMPOSITION}"
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

export CS_REPRICE_SUBPROBLEM="${CS_REPRICE_SUBPROBLEM:-true}"
export CS_BENDERS_MAX_ITERS="${CS_BENDERS_MAX_ITERS:-100}"
export CS_BENDERS_TIME_LIMIT="${CS_BENDERS_TIME_LIMIT:-6000}"
export CS_INNER_CG_MAX_ITERS="${CS_INNER_CG_MAX_ITERS:-200}"
export CS_INNER_PRICING_TIME="${CS_INNER_PRICING_TIME:-30}"
export CS_INNER_IP_TIME_LIMIT="${CS_INNER_IP_TIME_LIMIT:-30}"
export CS_MAX_REPRICE_ROUNDS="${CS_MAX_REPRICE_ROUNDS:-10000}"
export CS_N_SCENARIOS="${CS_N_SCENARIOS:-3}"
export CS_MAX_WALKING_DISTANCE="${CS_MAX_WALKING_DISTANCE:-600}"
export CS_MAX_WAIT_TIME="${CS_MAX_WAIT_TIME:-900}"
export CS_DETOUR_FACTOR="${CS_DETOUR_FACTOR:-2.0}"
export CS_MAX_STOPS="${CS_MAX_STOPS:-}"
export CS_ROUTE_REG_WEIGHT="${CS_ROUTE_REG_WEIGHT:-1.0}"
export CS_REPOSITIONING_TIME="${CS_REPOSITIONING_TIME:-20.0}"

echo "===== Settings ====="
echo "  Instance                 = ${INST}"
echo "  Decomposition            = ${DECOMPOSITION}"
echo "  CS_REPRICE_SUBPROBLEM    = ${CS_REPRICE_SUBPROBLEM}"
echo "  DATA_DIR                 = ${DATA_DIR}"
echo "  CS_BENDERS_MAX_ITERS     = ${CS_BENDERS_MAX_ITERS}"
echo "  CS_BENDERS_TIME_LIMIT    = ${CS_BENDERS_TIME_LIMIT}s"
echo "  CS_INNER_CG_MAX_ITERS    = ${CS_INNER_CG_MAX_ITERS}"
echo "  CS_INNER_PRICING_TIME    = ${CS_INNER_PRICING_TIME}s"
echo "  CS_INNER_IP_TIME_LIMIT   = ${CS_INNER_IP_TIME_LIMIT}s"
echo "  CS_MAX_REPRICE_ROUNDS    = ${CS_MAX_REPRICE_ROUNDS}"
echo "  CS_N_SCENARIOS           = ${CS_N_SCENARIOS}"
echo "  CS_MAX_WALKING_DISTANCE  = ${CS_MAX_WALKING_DISTANCE}s"
echo "  CS_MAX_WAIT_TIME         = ${CS_MAX_WAIT_TIME}s"
echo "  CS_DETOUR_FACTOR         = ${CS_DETOUR_FACTOR}"
echo "  CS_MAX_STOPS             = ${CS_MAX_STOPS}"
echo "  CS_ROUTE_REG_WEIGHT      = ${CS_ROUTE_REG_WEIGHT}"
echo "  CS_REPOSITIONING_TIME    = ${CS_REPOSITIONING_TIME}"
echo ""

echo "===== Running ====="
set +e
julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/compare_benders_decompositions.jl" \
      "$BASE_OUTDIR" "$DATA_DIR" "$N_STATIONS" "$L" "$N_PAIRS" "$OV" "$SEED" "$DECOMPOSITION"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
