#!/bin/bash
#SBATCH --job-name=bb_obj_audit
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=03:00:00

set -euo pipefail
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="$PROJECT_ROOT/results/branch_benders_objective_audit_n15_p16_s123_q3"
DATA_DIR="$PROJECT_ROOT/../Data/base_data"

module load "${CS_JULIA_MODULE:-julia/1.12.6}"
JULIA_VERSION=$(julia --startup-file=no -e 'print(VERSION)')
if [ -n "${SLURM_TMPDIR:-}" ]; then
    export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot_v${JULIA_VERSION}"
else
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_JOB_ID}"
fi
mkdir -p "$JULIA_DEPOT_PATH" "$OUTDIR"
rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
export JULIA_PKG_PRECOMPILE_AUTO=0
export JULIA_NUM_PRECOMPILE_TASKS=1

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/audit_branch_benders_objective_discrepancy.jl" "$OUTDIR" "$DATA_DIR"
