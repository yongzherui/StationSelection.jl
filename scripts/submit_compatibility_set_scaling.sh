#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXP_DIR="${1:-$PROJECT_ROOT/experiments/compatibility_set_scaling}"
JOBS_FILE="$EXP_DIR/jobs.txt"
LOG_DIR="$EXP_DIR/slurm_logs"

mkdir -p "$EXP_DIR" "$LOG_DIR"

julia --project="$PROJECT_ROOT" "$PROJECT_ROOT/scripts/generate_job_list.jl" "$JOBS_FILE"
N_JOBS=$(($(wc -l < "$JOBS_FILE") - 1))

if [ "$N_JOBS" -le 0 ]; then
    echo "ERROR: no jobs were generated into $JOBS_FILE"
    exit 1
fi

echo "Submitting $N_JOBS compatibility-set jobs"
sbatch \
    --array=1-"$N_JOBS" \
    --output="$LOG_DIR/%A_%a.out" \
    --error="$LOG_DIR/%A_%a.err" \
    "$PROJECT_ROOT/scripts/sbatch_single_instance.sh" \
    "$JOBS_FILE" \
    "$EXP_DIR"
