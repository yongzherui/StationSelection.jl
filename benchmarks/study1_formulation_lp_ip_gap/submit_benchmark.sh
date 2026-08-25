#!/bin/bash
# Study 1 -- Base vs. Joint LP/IP gap, one job per (instance, formulation, setting) cell.
#
# Usage: sbatch --array=1-<n_jobs> submit_benchmark.sh
#   <n_jobs> = number of data rows in config/jobs.tsv (i.e. lines - 1 for the header),
#   produced by generate_jobs.jl. -o/-e need slurm_logs/ to exist before first submit:
#   mkdir -p slurm_logs
#
#SBATCH --job-name=study1_lp_ip_gap
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=01:00:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail

STUDY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/jobs.tsv"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header; task N -> data row N+1

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
