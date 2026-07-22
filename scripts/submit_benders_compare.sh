#!/bin/bash
set -euo pipefail

# Submits BendersY / BendersYZ / BendersYZH convergence comparison jobs, keyed by
# (instance, decomposition): one independent SLURM array per decomposition against the
# same instance grid, each with its own --time budget. BendersY's repricing can hit real
# dual degeneracy and run far longer than BendersYZ/BendersYZH on the same instance
# (observed: n=10 took 88s for BendersY vs 5s/0.7s for BendersYZ/BendersYZH; n=20 didn't
# finish BendersY's repricing within 20+ minutes), so bundling all three into one task
# let a slow BendersY block or starve the fast ones -- this keeps them fully independent.

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXP_DIR="${1:-$PROJECT_ROOT/experiments/benders_decomposition_compare}"
DATA_DIR="${ZZ_DATA_DIR:-$PROJECT_ROOT/../Data/base_data}"
JOBS_FILE="$EXP_DIR/jobs.txt"
LOG_DIR="$EXP_DIR/slurm_logs"

mkdir -p "$EXP_DIR" "$LOG_DIR"

echo "Project root : $PROJECT_ROOT"
echo "Data dir     : $DATA_DIR"
echo "Experiment   : $EXP_DIR"
echo ""

julia --project="$PROJECT_ROOT" "$PROJECT_ROOT/scripts/generate_benders_compare_job_list.jl" "$JOBS_FILE"
N_JOBS=$(($(wc -l < "$JOBS_FILE") - 1))

if [ "$N_JOBS" -le 0 ]; then
    echo "ERROR: no jobs were generated into $JOBS_FILE"
    exit 1
fi

# decomposition:time budget -- BendersY needs the long leash (repricing can hit dual
# degeneracy); BendersYZ/BendersYZH have consistently converged in seconds on small
# fixtures, so a much shorter budget is plenty while keeping their queue footprint small.
DECOMP_TIME_BUDGETS=(
    "BendersY:02:00:00"
    "BendersYZ:00:30:00"
    "BendersYZH:00:15:00"
)

echo ""
echo "Submitting $N_JOBS instances x ${#DECOMP_TIME_BUDGETS[@]} decompositions"
for entry in "${DECOMP_TIME_BUDGETS[@]}"; do
    DECOMP="${entry%%:*}"
    BUDGET="${entry#*:}"
    echo "  -> $DECOMP  (time=$BUDGET, array=1-$N_JOBS)"
    sbatch \
        --array=1-"$N_JOBS" \
        --time="$BUDGET" \
        --job-name="benders_${DECOMP}" \
        --output="$LOG_DIR/%x_%A_%a.out" \
        --error="$LOG_DIR/%x_%A_%a.err" \
        "$PROJECT_ROOT/scripts/sbatch_benders_compare.sh" \
        "$JOBS_FILE" \
        "$EXP_DIR" \
        "$DATA_DIR" \
        "$DECOMP"
done
