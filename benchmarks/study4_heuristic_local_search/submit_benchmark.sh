#!/bin/bash
# Study 4 -- exact versus two warm-start pricers, one independent node/process per row.
#
# Usage: sbatch --array=1-<n_jobs> submit_benchmark.sh
#   <n_jobs> = number of data rows in config/jobs.tsv (i.e. lines - 1 for the header).
#   -o/-e need slurm_logs/ to exist before first submit: mkdir -p slurm_logs
#
#SBATCH --job-name=study4_heuristic_local_search
#SBATCH --partition=mit_preemptable
# mit_preemptable, not mit_normal: mit_normal queues behind whatever else this
# account is running (a short job was estimated 29 h out during a large array),
# while preemptable nodes start in minutes and preemption is rare in practice.
# A preempted job shows as CANCELLED/PREEMPTED with truncated output -- check the
# sacct state before reading missing results as a failure.
#SBATCH --nodes=1
#SBATCH --ntasks=1
# One CPU per scenario, matching Studies 5 and 8. Gurobi is pinned to one thread by the
# runner, so scenario pricing is the only parallel work being compared.
#SBATCH --cpus-per-task=3
#SBATCH --mem=24G
#SBATCH --time=04:30:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 4 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/jobs.tsv"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header; task N -> data row N+1

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
