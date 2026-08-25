#!/bin/bash
# Study 1 -- four LP/IP-gap comparisons, one independent job per instance/variant cell.
#
# Usage: sbatch --array=1-<n_jobs> submit_benchmark.sh <substudy>
#   substudy = formulation, max_stops, max_wait_time, or detour_factor.
#   -o/-e need slurm_logs/ to exist before first submit:
#   mkdir -p slurm_logs
#
#SBATCH --job-name=study1_lp_ip_gap
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=04:00:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail

STUDY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

SUBSTUDY="${1:?usage: submit_benchmark.sh <formulation|max_stops|max_wait_time|detour_factor>}"
case "$SUBSTUDY" in
    formulation|max_stops|max_wait_time|detour_factor) ;;
    *) echo "invalid Study 1 sub-study: $SUBSTUDY" >&2; exit 2 ;;
esac
JOBS_FILE="$STUDY_DIR/config/${SUBSTUDY}_jobs.tsv"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header; task N -> data row N+1

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
