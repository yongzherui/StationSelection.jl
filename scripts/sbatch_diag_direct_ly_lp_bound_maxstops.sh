#!/bin/bash
#SBATCH --job-name=diag_direct_ly_ms
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

# Runs scripts/diag_direct_ly_lp_bound.jl at a given max_stops (arg 1: an integer, or
# "uncapped") -- correctness test already passing (33/33), not re-run here.
#
# Usage:
#   sbatch --output=... --error=... scripts/sbatch_diag_direct_ly_lp_bound_maxstops.sh <max_stops>

MAX_STOPS_ARG="${1:-uncapped}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

echo "Job: ${SLURM_JOB_ID}  Node: ${SLURM_NODELIST}  Started: $(date)  max_stops=${MAX_STOPS_ARG}"

JULIA_MODULE="${CS_JULIA_MODULE:-julia/1.12.6}"
GUROBI_MODULE="${CS_GUROBI_MODULE:-}"
module load "$JULIA_MODULE"
[ -n "$GUROBI_MODULE" ] && module load "$GUROBI_MODULE"
export JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-$HOME/.julia}"

cd "$PROJECT_ROOT"
set +e
stdbuf -o0 -e0 julia --startup-file=no --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/diag_direct_ly_lp_bound.jl" "$MAX_STOPS_ARG"
EXIT_CODE=$?
set -e
echo "Finished: $(date)  exit=$EXIT_CODE"
exit $EXIT_CODE
