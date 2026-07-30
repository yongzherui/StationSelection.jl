#!/bin/bash
#SBATCH --job-name=zz_bb_n10
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=32G
#SBATCH --time=12:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

JOBS_FILE="${1:-}"
BASE_OUTDIR="${2:-}"
TASK="${SLURM_ARRAY_TASK_ID:-}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ] || [ -z "$TASK" ]; then
    echo "Usage: sbatch --array=1-N $0 <jobs_file> <base_outdir>"
    exit 1
fi

JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
if [ -z "$JOB_LINE" ]; then
    echo "No job found for array task $TASK in $JOBS_FILE"
    exit 1
fi

SEED=$(echo "$JOB_LINE" | cut -f1)
MAX_STOPS_MODE=$(echo "$JOB_LINE" | cut -f2)
DECOMPOSITION=$(echo "$JOB_LINE" | cut -f3)
DECOMPOSITION="${DECOMPOSITION:-yz}"

echo "Branch-and-Benders n=${BRANCH_BENDERS_N_STATIONS:-10}: job=${SLURM_ARRAY_JOB_ID}_${TASK} seed=$SEED mode=$MAX_STOPS_MODE decomposition=$DECOMPOSITION"
echo "Node: $SLURM_NODELIST  Started: $(date)"

module load "${CS_JULIA_MODULE:-julia/1.12.6}"
if [ -n "${CS_GUROBI_MODULE:-}" ]; then
    module load "$CS_GUROBI_MODULE"
fi

# Keep both the depot and every persistent output on the shared home filesystem.
export JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-$HOME/.julia}"
# Experiment tasks should consume a cache prepared by a separate compute-node
# precompile job, not have every array task contend while updating the shared depot.
export JULIA_PKG_PRECOMPILE_AUTO=0
export JULIA_NUM_PRECOMPILE_TASKS=1
cd "$PROJECT_ROOT"

# Line buffering gives readable incremental SLURM logs while preserving complete
# lines from Julia, Gurobi, and Logging's stderr output.
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/zhuzhou_p16_branch_benders_n10_task.jl" \
    "$BASE_OUTDIR" "$SEED" "$MAX_STOPS_MODE" "$DECOMPOSITION"

echo "Finished: $(date)"
