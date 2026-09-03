#!/bin/bash
# Study 8 -- station-simple warm-start speedup, warm_start arm only.
# The `exact` baseline is Study 7's completed runs on the identical grid.
#
# Usage: sbatch --array=1-30 submit_benchmark.sh
#   -o/-e need slurm_logs/ to exist before the first submit: mkdir -p slurm_logs
#
#SBATCH --job-name=study8_warm_start_speedup
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
# Identical to Study 7's allocation, deliberately: a wall-clock comparison across the two
# studies is only meaningful if CPUs, memory and threading match. One CPU per scenario
# (s=3); Gurobi stays pinned to 1 thread so scenario pricing is the only parallelism.
#SBATCH --cpus-per-task=3
#SBATCH --mem=24G
#SBATCH --time=04:30:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 8 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/jobs.tsv"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
