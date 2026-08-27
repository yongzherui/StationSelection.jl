#!/bin/bash
# Study 5 -- single-threaded exact-CG scaling, one job per instance.
# kind of scaling sweep (see ../../scripts/sbatch_zhuzhou_instance.sh) -- 16G/4h per
# task, matched to the memory-censoring protocol in
# notes/2026-08-05_free_assignment_cg_direct_ms5_comparison.md so OOM outcomes stay
# comparable to that prior sweep.
#
# Usage: sbatch --array=1-<n_jobs> submit_benchmark.sh <substudy>
#   substudy = stations, passengers, or scenarios
#   <n_jobs> = number of data rows in config/jobs.tsv (i.e. lines - 1 for the header).
#   -o/-e need slurm_logs/ to exist before first submit: mkdir -p slurm_logs
#
#SBATCH --job-name=study5_scaling_exact_cg
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=16G
#SBATCH --time=01:00:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS=1

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 5 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

SUBSTUDY="${1:?usage: submit_benchmark.sh <stations|passengers|scenarios>}"
case "$SUBSTUDY" in
    stations|passengers|scenarios) ;;
    *) echo "invalid Study 5 sub-study: $SUBSTUDY" >&2; exit 2 ;;
esac
JOBS_FILE="$STUDY_DIR/config/${SUBSTUDY}_jobs.tsv"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header; task N -> data row N+1

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
