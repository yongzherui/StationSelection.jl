#!/bin/bash
#SBATCH --job-name=diag_sched_yhat
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# Single (non-array) job: runs scripts/diag_schedule_same_yhat_gap.jl, which isolates whether
# seeding the CG-priming route-covering solve with a large historical column pool (as a multi-stage
# route_regularization_weight_schedule run does) can make it converge to a worse-than-fresh
# objective for the SAME fixed y_hat -- see that script's docstring for the full context.
#
# Usage:
#   sbatch --output=... --error=... scripts/sbatch_diag_schedule_same_yhat_gap.sh [outdir]

OUTDIR="${1:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

echo "=========================================="
echo "Diagnostic: schedule same-y_hat objective gap"
echo "Job:        ${SLURM_JOB_ID}"
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
        export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_JOB_ID}"
    fi
    mkdir -p "$JULIA_DEPOT_PATH"
    rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
    echo "Depot ready: $JULIA_DEPOT_PATH"
fi
echo ""

cd "$PROJECT_ROOT"
export SAMPLE09_ROUTE_WEIGHT="1.0"

echo "===== Running ====="
set +e
stdbuf -o0 -e0 julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/diag_schedule_same_yhat_gap.jl" \
      "$OUTDIR"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
