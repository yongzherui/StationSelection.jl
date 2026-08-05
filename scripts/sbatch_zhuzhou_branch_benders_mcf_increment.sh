#!/bin/bash
#SBATCH --job-name=zz_bb_mcf_inc
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

set -euo pipefail

JOBS_FILE="$1"
OUTDIR="$2"
DATA_DIR="$3"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID is required}"
PROJECT_ROOT="$SLURM_SUBMIT_DIR"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
[ -n "$JOB_LINE" ] || { echo "No job row for task $TASK" >&2; exit 2; }

N=$(echo "$JOB_LINE" | cut -f1)
P=$(echo "$JOB_LINE" | cut -f2)
SEED=$(echo "$JOB_LINE" | cut -f3)
Q=$(echo "$JOB_LINE" | cut -f4)
TIME_CLASS=$(echo "$JOB_LINE" | cut -f5)
MCF_VARIANT=$(echo "$JOB_LINE" | cut -f6)

module load "${CS_JULIA_MODULE:-julia/1.12.6}"
JULIA_VERSION=$(julia --startup-file=no -e 'print(VERSION)')
if [ -n "${SLURM_TMPDIR:-}" ]; then
    export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot_v${JULIA_VERSION}"
else
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${TASK}"
fi
mkdir -p "$JULIA_DEPOT_PATH"
rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
export JULIA_PKG_PRECOMPILE_AUTO=0
export JULIA_NUM_PRECOMPILE_TASKS=1

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/zhuzhou_branch_benders_yz_mw_no_mcf_task.jl" \
    "$OUTDIR" "$DATA_DIR" "$N" "$P" "$SEED" "$Q" "$TIME_CLASS" "$MCF_VARIANT"
