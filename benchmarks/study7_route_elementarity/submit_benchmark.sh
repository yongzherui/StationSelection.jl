#!/bin/bash
# Study 7 -- certified-optimal CG at n=20, exporting the selected route columns.
#
# Usage: sbatch --array=1-30 submit_benchmark.sh
#   <n_jobs> = data rows in config/jobs.tsv (lines - 1 for the header).
#   -o/-e need slurm_logs/ to exist before the first submit: mkdir -p slurm_logs
#
#SBATCH --job-name=study7_route_elementarity
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
# One CPU per scenario: scenario pricing is what parallelizes (Gurobi stays at 1 thread).
# Every job in this study is s=3, so this is fixed rather than per-submission as in Study 5.
#SBATCH --cpus-per-task=3
# 24G matches Study 5's allocation for the same n=20/s=3 cells, where the parallel arm
# peaked well inside it. Same figure so GC behaviour (Julia sizes its heap against the
# cgroup limit) stays comparable to the runs these budgets were derived from.
#SBATCH --mem=24G
# 4 h CG cap + integer recovery and a final master re-solve at up to 300 s each.
#SBATCH --time=04:30:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 7 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/jobs.tsv"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header; task N -> data row N+1

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
