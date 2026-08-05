#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXP_DIR="${1:-$PROJECT_ROOT/experiments/zhuzhou_benders_cut_scaling_ms5}"
DATA_DIR="${2:-$PROJECT_ROOT/../Data/base_data}"
MODE="${3:-n10}"
LOG_DIR="$EXP_DIR/slurm_logs"

mkdir -p "$EXP_DIR" "$LOG_DIR"
julia --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/generate_zhuzhou_benders_cut_scaling_jobs.jl" "$EXP_DIR"

submit_group() {
    local group="$1"
    local jobs_file="$2"
    local wall_limit="$3"
    local direct_limit="$4"
    local n_jobs
    n_jobs=$(($(wc -l < "$jobs_file") - 1))
    sbatch \
        --array="1-$n_jobs" \
        --job-name="zz_cut_${group}" \
        --time="$wall_limit" \
        --mem=16G \
        --export="ALL,CS_DIRECT_TIME_LIMIT=$direct_limit,CS_REQUIRE_ZERO_FEASIBILITY_CUTS=true" \
        --output="$LOG_DIR/%x_%A_%a.out" \
        --error="$LOG_DIR/%x_%A_%a.err" \
        "$PROJECT_ROOT/scripts/sbatch_method_compare.sh" \
        "$jobs_file" "$EXP_DIR" "$DATA_DIR"
}

# Leave five minutes of wall headroom for startup, final solve, and CSV export.
case "$MODE" in
    n10)
        submit_group "n10" "$EXP_DIR/jobs_n10.tsv" "01:30:00" "5100"
        ;;
    full)
        submit_group "small" "$EXP_DIR/jobs_n_le_20.tsv" "01:30:00" "5100"
        submit_group "large" "$EXP_DIR/jobs_n_gt_20.tsv" "03:00:00" "10500"
        ;;
    remaining)
        submit_group "n15_n20" "$EXP_DIR/jobs_n15_n20.tsv" "01:30:00" "5100"
        submit_group "large" "$EXP_DIR/jobs_n_gt_20.tsv" "03:00:00" "10500"
        ;;
    *)
        echo "ERROR: mode must be n10, remaining, or full (got $MODE)" >&2
        exit 2
        ;;
esac
