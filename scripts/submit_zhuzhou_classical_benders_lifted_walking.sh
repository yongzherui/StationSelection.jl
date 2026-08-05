#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXP_DIR="${1:-$PROJECT_ROOT/experiments/zhuzhou_classical_benders_lifted_walking_ms5}"
DATA_DIR="${2:-$PROJECT_ROOT/../Data/base_data}"
JOBS="$EXP_DIR/jobs.tsv"
LOGS="$EXP_DIR/slurm_logs"
mkdir -p "$EXP_DIR" "$LOGS"
julia --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/generate_zhuzhou_classical_benders_lifted_walking_jobs.jl" "$JOBS"

COMMON_EXPORT="ALL,CS_REQUIRE_ZERO_FEASIBILITY_CUTS=true,CS_BENDERS_MAX_ITERS=2000"
sbatch --array=1-12 --time=01:30:00 --mem=16G --export="$COMMON_EXPORT" \
    --job-name="zz_liftwalk_n10" \
    --output="$LOGS/%x_%A_%a.out" --error="$LOGS/%x_%A_%a.err" \
    "$PROJECT_ROOT/scripts/sbatch_method_compare.sh" "$JOBS" "$EXP_DIR" "$DATA_DIR"
sbatch --array=13-24 --time=03:00:00 --mem=16G --export="$COMMON_EXPORT" \
    --job-name="zz_liftwalk_n15" \
    --output="$LOGS/%x_%A_%a.out" --error="$LOGS/%x_%A_%a.err" \
    "$PROJECT_ROOT/scripts/sbatch_method_compare.sh" "$JOBS" "$EXP_DIR" "$DATA_DIR"
