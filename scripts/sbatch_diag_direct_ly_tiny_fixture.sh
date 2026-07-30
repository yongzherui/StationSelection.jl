#!/bin/bash
#SBATCH --job-name=diag_direct_ly_tiny
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=8G
#SBATCH --time=00:15:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail
PROJECT_ROOT="$SLURM_SUBMIT_DIR"
echo "Job: ${SLURM_JOB_ID}  Node: ${SLURM_NODELIST}  Started: $(date)"

JULIA_MODULE="${CS_JULIA_MODULE:-julia/1.12.6}"
GUROBI_MODULE="${CS_GUROBI_MODULE:-}"
module load "$JULIA_MODULE"
[ -n "$GUROBI_MODULE" ] && module load "$GUROBI_MODULE"
export JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-$HOME/.julia}"

cd "$PROJECT_ROOT"
set +e
stdbuf -o0 -e0 julia --startup-file=no --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/diag_direct_ly_tiny_fixture.jl"
EXIT_CODE=$?
set -e
echo "Finished: $(date)  exit=$EXIT_CODE"
exit $EXIT_CODE
