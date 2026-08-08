#!/bin/bash
set -euo pipefail

# Submit a Zhuzhou AggregateODRouteModel SLURM array job via
# sbatch_zhuzhou_instance.sh. Replaces the three near-identical
# submit_zhuzhou_{assignment,large_rrw,scaling}.sh scripts -- what varied
# between them (exp_dir, CS_N_SCENARIOS, CS_ROUTE_REG_WEIGHT, whether jobs.txt
# needs generating) are now explicit arguments/env vars.
#
# Usage:
#   scripts/submit_zhuzhou.sh <exp_dir> [--generate]
#
# Examples (reproducing the old scripts):
#   CS_N_SCENARIOS=1 scripts/submit_zhuzhou.sh experiments/zhuzhou_set_aggregate_od_route
#       # was: submit_zhuzhou_assignment.sh
#   CS_N_SCENARIOS=1 CS_ROUTE_REG_WEIGHT=100.0 scripts/submit_zhuzhou.sh experiments/zz_large_rrw
#       # was: submit_zhuzhou_large_rrw.sh
#   scripts/submit_zhuzhou.sh experiments/zhuzhou_scaling --generate
#       # was: submit_zhuzhou_scaling.sh (auto-generates jobs.txt via generate_zhuzhou_job_list.jl)
#
# Any CS_* env var sbatch_zhuzhou_instance.sh reads (CS_N_SCENARIOS,
# CS_ROUTE_REG_WEIGHT, CS_MAX_STOPS, ...) can be exported before calling this
# script; sbatch's default --export=ALL passes them through to the job.

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXP_DIR="${1:?usage: submit_zhuzhou.sh <exp_dir> [--generate]}"
GENERATE=0
[ "${2:-}" = "--generate" ] && GENERATE=1
DATA_DIR="${ZZ_DATA_DIR:-$PROJECT_ROOT/../Data/base_data}"
JOBS_FILE="$EXP_DIR/jobs.txt"
LOG_DIR="$EXP_DIR/slurm_logs"

mkdir -p "$EXP_DIR" "$LOG_DIR"

echo "Project root : $PROJECT_ROOT"
echo "Data dir     : $DATA_DIR"
echo "Experiment   : $EXP_DIR"
echo ""

if [ "$GENERATE" = 1 ]; then
    julia --project="$PROJECT_ROOT" "$PROJECT_ROOT/scripts/generate_zhuzhou_job_list.jl" "$JOBS_FILE"
elif [ ! -f "$JOBS_FILE" ]; then
    echo "ERROR: jobs file not found at $JOBS_FILE (pass --generate to create one via generate_zhuzhou_job_list.jl)"
    exit 1
fi

N_JOBS=$(($(wc -l < "$JOBS_FILE") - 1))
if [ "$N_JOBS" -le 0 ]; then
    echo "ERROR: no jobs found in $JOBS_FILE"
    exit 1
fi

echo "Submitting $N_JOBS AggregateODRouteModel jobs" \
     "${CS_N_SCENARIOS:+(CS_N_SCENARIOS=$CS_N_SCENARIOS)}" \
     "${CS_ROUTE_REG_WEIGHT:+(CS_ROUTE_REG_WEIGHT=$CS_ROUTE_REG_WEIGHT)}"
sbatch \
    --array=1-"$N_JOBS" \
    --output="$LOG_DIR/%A_%a.out" \
    --error="$LOG_DIR/%A_%a.err" \
    "$PROJECT_ROOT/scripts/sbatch_zhuzhou_instance.sh" \
    "$JOBS_FILE" \
    "$EXP_DIR" \
    "$DATA_DIR"
