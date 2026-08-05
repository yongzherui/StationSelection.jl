#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTDIR="${1:-$PROJECT_ROOT/experiments/zhuzhou_branch_benders_yz_mw_no_mcf_ms5}"
DATA_DIR="${2:-$PROJECT_ROOT/../Data/base_data}"
JOBS="$OUTDIR/jobs.tsv"
LOGS="$OUTDIR/slurm_logs"
VALIDATION_JOB_ID="${VALIDATION_JOB_ID:-}"
mkdir -p "$OUTDIR" "$LOGS"
julia --project="$PROJECT_ROOT" "$PROJECT_ROOT/scripts/generate_zhuzhou_branch_benders_yz_mw_no_mcf_jobs.jl" "$JOBS"

DEPENDENCY_ARGS=()
if [ -n "$VALIDATION_JOB_ID" ]; then
    DEPENDENCY_ARGS=(--dependency="afterok:$VALIDATION_JOB_ID")
fi

# Rows 1-12 are n=10; rows 13-36 are n=15,20.
sbatch --array=1-12 --time=01:30:00 --mem=16G \
    "${DEPENDENCY_ARGS[@]}" \
    --output="$LOGS/%x_%A_%a.out" --error="$LOGS/%x_%A_%a.err" \
    "$PROJECT_ROOT/scripts/sbatch_zhuzhou_branch_benders_yz_mw_no_mcf.sh" "$JOBS" "$OUTDIR" "$DATA_DIR"
sbatch --array=13-36 --time=03:00:00 --mem=16G \
    "${DEPENDENCY_ARGS[@]}" \
    --output="$LOGS/%x_%A_%a.out" --error="$LOGS/%x_%A_%a.err" \
    "$PROJECT_ROOT/scripts/sbatch_zhuzhou_branch_benders_yz_mw_no_mcf.sh" "$JOBS" "$OUTDIR" "$DATA_DIR"
