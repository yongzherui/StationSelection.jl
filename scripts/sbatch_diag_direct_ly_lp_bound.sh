#!/bin/bash
#SBATCH --job-name=diag_direct_ly_lp_bound
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# Single (non-array) job: runs scripts/diag_direct_ly_lp_bound.jl end to end (3 solves
# on one zhuzhou_n20_p16_s123/ms4 instance). Standalone job, no concurrent submissions,
# so CS_COPY_DEPOT=0 (shared depot) is fine unless overridden.
#
# Usage:
#   sbatch --output=... --error=... scripts/sbatch_diag_direct_ly_lp_bound.sh

PROJECT_ROOT="$SLURM_SUBMIT_DIR"

echo "=========================================="
echo ":direct_ly LP bound diagnostic"
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
COPY_DEPOT="${CS_COPY_DEPOT:-0}"
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

echo "===== Step 1: correctness test (test/opt/test_aggregate_od_route_direct_ly.jl) ====="
set +e
stdbuf -o0 -e0 julia --startup-file=no --project="$PROJECT_ROOT" -e '
    using Test, StationSelection, DataFrames, Dates, JuMP, Gurobi
    const MOI = JuMP.MOI
    include("test/opt/test_aggregate_od_route_direct_ly.jl")
'
TEST_EXIT_CODE=$?
set -e

if [ $TEST_EXIT_CODE -ne 0 ]; then
    echo ""
    echo "Correctness test FAILED (exit=$TEST_EXIT_CODE) -- skipping the n=20 LP-bound diagnostic."
    exit $TEST_EXIT_CODE
fi

echo ""
echo "===== Step 2: zhuzhou_n20_p16_s123/ms4 LP-bound diagnostic ====="
set +e
stdbuf -o0 -e0 julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/diag_direct_ly_lp_bound.jl"
EXIT_CODE=$?
set -e

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
