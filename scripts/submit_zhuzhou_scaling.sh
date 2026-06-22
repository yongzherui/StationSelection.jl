#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXP_DIR="${1:-$PROJECT_ROOT/experiments/zhuzhou_scaling}"
DATA_DIR="${ZZ_DATA_DIR:-$PROJECT_ROOT/../Data/base_data}"
JOBS_FILE="$EXP_DIR/jobs.txt"
LOG_DIR="$EXP_DIR/slurm_logs"

mkdir -p "$EXP_DIR" "$LOG_DIR"

echo "Project root : $PROJECT_ROOT"
echo "Data dir     : $DATA_DIR"
echo "Experiment   : $EXP_DIR"
echo ""

julia --project="$PROJECT_ROOT" "$PROJECT_ROOT/scripts/generate_zhuzhou_job_list.jl" "$JOBS_FILE"
N_JOBS=$(($(wc -l < "$JOBS_FILE") - 1))

if [ "$N_JOBS" -le 0 ]; then
    echo "ERROR: no jobs were generated into $JOBS_FILE"
    exit 1
fi

echo ""
echo "Submitting $N_JOBS Zhuzhou scaling jobs"
sbatch \
    --array=1-"$N_JOBS" \
    --output="$LOG_DIR/%A_%a.out" \
    --error="$LOG_DIR/%A_%a.err" \
    "$PROJECT_ROOT/scripts/sbatch_zhuzhou_instance.sh" \
    "$JOBS_FILE" \
    "$EXP_DIR" \
    "$DATA_DIR"
