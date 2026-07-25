#!/bin/bash
#SBATCH --job-name=sample09_route_weight
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# Single (non-array) job: runs scripts/sample09_route_weight_sensitivity.jl end to
# end -- BendersY(mw) only, route_reg_weight in {0.0,0.1,1.0,10.0} x n_stations in
# {10,15} by default (8 solves total). See that script's docstring for why.
#
# Usage:
#   sbatch --output=... --error=... scripts/sbatch_sample09_route_weight_sensitivity.sh [outdir]

OUTDIR="${1:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

echo "=========================================="
echo "sample_09 route_reg_weight sensitivity (BendersY mw)"
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

echo "===== Running ====="
set +e
stdbuf -o0 -e0 julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/sample09_route_weight_sensitivity.jl" \
      "$OUTDIR"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
