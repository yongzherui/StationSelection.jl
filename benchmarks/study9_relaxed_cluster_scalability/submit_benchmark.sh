#!/bin/bash
# One table row per node/process. Submit with a pinned STUDY9_RUN_ID and matching array.
# Example: STUDY9_RUN_ID=2026-09-08_rc_scale sbatch --array=1-30 submit_benchmark.sh validation.tsv
#SBATCH --job-name=study9_relaxed_cluster_scale
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=32G
#SBATCH --time=06:30:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
STUDY_DIR="${SLURM_SUBMIT_DIR:?submit from the Study 9 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"
TABLE="${1:?usage: submit_benchmark.sh <validation.tsv|nNN.tsv>}"
case "$TABLE" in
    smoke.tsv|validation.tsv|n20.tsv|n25.tsv|n30.tsv|n35.tsv|n40.tsv|n45.tsv|n50.tsv|n55.tsv|n60.tsv|n65.tsv|n70.tsv|n75.tsv|n80.tsv|n84.tsv) ;;
    *) echo "invalid job table: $TABLE" >&2; exit 2 ;;
esac
RUN_ID="${STUDY9_RUN_ID:?set STUDY9_RUN_ID in the submitting shell}"
export STUDY9_OUTPUT_DIR="${STUDY9_OUTPUT_DIR:-$PROJECT_ROOT/benchmarks/experiments/${RUN_ID}_study9_relaxed_cluster_scalability}"
mkdir -p "$STUDY9_OUTPUT_DIR"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/$TABLE"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
[ -n "$JOB_LINE" ] || { echo "no task $TASK in $JOBS_FILE" >&2; exit 2; }
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
