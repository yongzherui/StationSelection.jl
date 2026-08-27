#!/bin/bash
# Study 2 -- exact running-max vs. explicit DARP-style pricing. Each array
# task is one (instance, pricing_mode) row, so modes run in separate processes.
#
# Usage: sbatch --array=1-<n_jobs> submit_benchmark.sh
#   <n_jobs> = number of data rows in config/jobs.tsv (i.e. lines - 1 for the header).
#   -o/-e need slurm_logs/ to exist before first submit: mkdir -p slurm_logs
#
#SBATCH --job-name=study2_passenger_max_ablation
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=00:30:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 2 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/jobs.tsv"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header; task N -> data row N+1

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
