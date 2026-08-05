#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTDIR="${1:-$PROJECT_ROOT/experiments/zhuzhou_branch_benders_mcf_increment_ms5}"
DATA_DIR="${2:-$PROJECT_ROOT/../Data/base_data}"
JOBS="$OUTDIR/jobs.tsv"
LOGS="$OUTDIR/slurm_logs"
mkdir -p "$OUTDIR" "$LOGS"
julia --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/generate_zhuzhou_branch_benders_mcf_increment_jobs.jl" "$JOBS"

# 24 rows per n. Run n=10 first as a smoke/benchmark gate.
if [ "${SMALL_ONLY:-0}" = "1" ]; then
    sbatch --array=1-24 --time=01:30:00 --mem=16G \
        --output="$LOGS/%x_%A_%a.out" --error="$LOGS/%x_%A_%a.err" \
        "$PROJECT_ROOT/scripts/sbatch_zhuzhou_branch_benders_mcf_increment.sh" "$JOBS" "$OUTDIR" "$DATA_DIR"
else
    sbatch --array=25-72 --time=03:00:00 --mem=16G \
        --output="$LOGS/%x_%A_%a.out" --error="$LOGS/%x_%A_%a.err" \
        "$PROJECT_ROOT/scripts/sbatch_zhuzhou_branch_benders_mcf_increment.sh" "$JOBS" "$OUTDIR" "$DATA_DIR"
fi
