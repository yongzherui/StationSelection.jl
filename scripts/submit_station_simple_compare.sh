#!/bin/bash
set -euo pipefail

# Submit ONE n_stations batch of the station-simple vs normal pricer comparison
# (BendersYZ restricted_mw_fixed_pi cut at max_stops=5, n_pairs=16, station-simple
# pricer on vs off -- see generate_station_simple_compare_job_list.jl) as a
# single SLURM array (one array task per (instance, method) pair). Batches are
# sized at 12 jobs (family x n_pairs x seed x method = 2 x 1 x 3 x 2), well
# under the cluster's 448-job-in-queue cap, so every n_stations value could in
# principle be submitted together -- this still rolls out one n_stations at a
# time (mirrors submit_method_compare.sh) so a correctness/timing problem at
# n_stations=10 is caught before burning queue time on n=60.
#
# Reuses sbatch_method_compare.sh / run_method_compare_task.jl unchanged --
# only the job list and experiment directory differ from the full method-grid
# comparison.
#
# Usage:
#   scripts/submit_station_simple_compare.sh list                 # show batches, submit nothing
#   scripts/submit_station_simple_compare.sh 10                    # submit the n_stations=10 batch
#   scripts/submit_station_simple_compare.sh 15                    # ...only once n=10 has finished
#   scripts/submit_station_simple_compare.sh 10 "" 1-8              # smoke-test: first 8 rows of the n=10 batch only
#
# Recommended rollout -- wait for each batch to fully finish (squeue -u $USER
# shows nothing for job-name ssc_n<N>) before submitting the next:
#   scripts/submit_station_simple_compare.sh 10
#   # wait; run scripts/analyze_method_compare.jl experiments/aggregate_od_route_station_simple_compare
#   scripts/submit_station_simple_compare.sh 15
#   scripts/submit_station_simple_compare.sh 20
#   scripts/submit_station_simple_compare.sh 30
#   scripts/submit_station_simple_compare.sh 40
#   scripts/submit_station_simple_compare.sh 50
#   scripts/submit_station_simple_compare.sh 60
#
# Override the queue cap (e.g. if the real limit differs) with QUEUE_CAP=N.

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARG1="${1:-}"
EXP_DIR="${2:-$PROJECT_ROOT/experiments/aggregate_od_route_station_simple_compare}"
[ -z "$EXP_DIR" ] && EXP_DIR="$PROJECT_ROOT/experiments/aggregate_od_route_station_simple_compare"
ARRAY_OVERRIDE="${3:-}"
DATA_DIR="${ZZ_DATA_DIR:-$PROJECT_ROOT/../Data/base_data}"
JOBS_FILE="$EXP_DIR/jobs.txt"
MANIFEST_FILE="$EXP_DIR/batch_manifest.txt"
LOG_DIR="$EXP_DIR/slurm_logs"

if [ -z "$ARG1" ]; then
    echo "ERROR: Usage: submit_station_simple_compare.sh <n_stations|list> [exp_dir] [array_override]"
    exit 1
fi

mkdir -p "$EXP_DIR" "$LOG_DIR"

if [ ! -f "$JOBS_FILE" ] || [ ! -f "$MANIFEST_FILE" ]; then
    echo "Generating job list + batch manifest..."
    julia --project="$PROJECT_ROOT" "$PROJECT_ROOT/scripts/generate_station_simple_compare_job_list.jl" "$JOBS_FILE"
fi

if [ "$ARG1" = "list" ]; then
    echo "Batches (from $MANIFEST_FILE):"
    column -t -s$'\t' "$MANIFEST_FILE"
    exit 0
fi

N_STATIONS="$ARG1"
BATCH_LINE=$(awk -F'\t' -v n="$N_STATIONS" 'NR>1 && $1==n {print; found=1} END{if(!found) exit 1}' "$MANIFEST_FILE") || {
    echo "ERROR: n_stations=$N_STATIONS not found in $MANIFEST_FILE. Available:"
    column -t -s$'\t' "$MANIFEST_FILE"
    exit 1
}
START=$(echo "$BATCH_LINE" | cut -f2)
END=$(echo "$BATCH_LINE" | cut -f3)
N_JOBS=$(echo "$BATCH_LINE" | cut -f4)

ARRAY_RANGE="$START-$END"
if [ -n "$ARRAY_OVERRIDE" ]; then
    # array_override is relative to the batch (e.g. "1-8" = first 8 rows of this batch)
    OVR_START=$(echo "$ARRAY_OVERRIDE" | cut -d- -f1)
    OVR_END=$(echo "$ARRAY_OVERRIDE" | cut -d- -f2)
    ARRAY_RANGE="$((START + OVR_START - 1))-$((START + OVR_END - 1))"
    echo "Smoke-test override: relative $ARRAY_OVERRIDE -> absolute rows $ARRAY_RANGE"
fi

echo "Project root : $PROJECT_ROOT"
echo "Data dir     : $DATA_DIR"
echo "Experiment   : $EXP_DIR"
echo "n_stations   : $N_STATIONS  (batch is $N_JOBS jobs, rows $START-$END)"
echo "Submitting   : --array=$ARRAY_RANGE"
echo ""

# Pre-flight queue check -- same 448 QOS cap as submit_method_compare.sh
# (sacctmgr show qos mit_preemptable: MaxSubmitPU=448).
QUEUE_CAP="${QUEUE_CAP:-448}"
N_NEW_JOBS=$((END - START + 1))
[ -n "$ARRAY_OVERRIDE" ] && N_NEW_JOBS=$((OVR_END - OVR_START + 1))
CURRENT_QUEUED=$(squeue -u "$USER" -h -r 2>/dev/null | wc -l | tr -d ' ')
CURRENT_QUEUED="${CURRENT_QUEUED:-0}"
PROJECTED=$((CURRENT_QUEUED + N_NEW_JOBS))

echo "Queue check  : $CURRENT_QUEUED currently pending/running for $USER; this batch adds $N_NEW_JOBS -> $PROJECTED (cap $QUEUE_CAP)"
if [ "$PROJECTED" -gt "$QUEUE_CAP" ]; then
    echo ""
    echo "ERROR: submitting this batch would put $PROJECTED jobs in queue, over the $QUEUE_CAP cap."
    echo "Wait for currently running/pending jobs to finish (squeue -u $USER) before submitting more,"
    echo "or override with QUEUE_CAP=<n> if the real limit is different."
    exit 1
fi
echo ""

sbatch \
    --array="$ARRAY_RANGE" \
    --job-name="ssc_n${N_STATIONS}" \
    --output="$LOG_DIR/%x_%A_%a.out" \
    --error="$LOG_DIR/%x_%A_%a.err" \
    "$PROJECT_ROOT/scripts/sbatch_method_compare.sh" \
    "$JOBS_FILE" \
    "$EXP_DIR" \
    "$DATA_DIR"
