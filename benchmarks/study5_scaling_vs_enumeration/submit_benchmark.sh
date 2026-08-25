#!/bin/bash
# Study 5 -- CG/exact vs. darp/ vs. raw enumeration scaling, one job per
# (|P|, |J|, |S|, max_stops) cell. mem/time follow the established convention for this
# kind of scaling sweep (see ../../scripts/sbatch_zhuzhou_instance.sh) -- 16G/4h per
# task, matched to the memory-censoring protocol in
# notes/2026-08-05_free_assignment_cg_direct_ms5_comparison.md so OOM outcomes stay
# comparable to that prior sweep.
#
# Usage: sbatch --array=1-<n_jobs> submit_benchmark.sh
#   <n_jobs> = number of data rows in config/jobs.tsv (i.e. lines - 1 for the header).
#   -o/-e need slurm_logs/ to exist before first submit: mkdir -p slurm_logs
#
#SBATCH --job-name=study5_scaling_vs_enumeration
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=04:00:00
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
